/**
 * @file    rtos_tasks.c
 * @brief   FreeRTOS task implementations and system coordinator for the Syringe Pump.
 * @author  eyazici
 * @date    2026
 *
 * @details Implements the syringe pump FreeRTOS task coordination:
 *
 *          **TaskInput** (Priority 3, 50 Hz polling):
 *          Reads the rotary encoder counter (TIM1 hardware encoder mode) and
 *          three push buttons via GPIO polling. Detects direction changes,
 *          button press/release edges, and long-press events. Pushes
 *          InputMessage_t events into a FreeRTOS queue.
 *
 *          **TaskUI** (Priority 2, event-driven):
 *          Blocks on the InputQueue waiting for events and delegates LCD menu
 *          page navigation and rendering to the ui_menu module.
 *
 *          **TaskMotor** (Priority 4, highest):
 *          Converts user-specified volume, flow rate, and syringe dimensions into
 *          physical motor parameters (steps, frequency). Launches the stepper
 *          driver and tracks live progress for the UI's pumping view.
 *
 *          **TaskEncoder** (Priority 4, 100 Hz):
 *          Reads the AS5600 magnetic rotary encoder over I2C2 and updates
 *          cumulative linear travel distance.
 *
 *          **TaskUartGatekeeper** (UartTask, Priority 4):
 *          Serializes TMC2208 single-wire UART DMA register transactions on
 *          USART1 and asynchronous diagnostic output on USART2 (VCP).
 */

#include "printf.h"
#include "rtos_tasks.h"
#include "ui_menu.h"
#include "main.h"
#include "pump_kinematics.h"
#include "stepper.h"
#include "flash_storage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "as5600.h"
#include "closed_loop.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/** @brief TIM1 handle — used in encoder mode for rotary encoder input */
extern TIM_HandleTypeDef htim1;
/** @brief TIM2 handle — used for stepper motor PWM generation */
extern TIM_HandleTypeDef htim2;

/* ========================================================================== */
/*                         Constants and Macros                               */
/* ========================================================================== */

/** @defgroup RTOS_Tasks_Config Application Configuration Constants
 *  @{
 */
#define MOTOR_SUPERVISORY_PERIOD_MS     20U     ///< Closed-loop supervisory loop period (50 Hz)
#define SCREEN_REMAINING_VOL_UPDATE     1000    ///< Pumping screen refresh interval in ms
#define DEFAULT_MICROSTEPPING           128      ///< Default driver microstepping divisor
#define ENCODER_BIT_SHIFT				2       ///< Right-shift for TIM1 CNT to get detent-aligned encoder ticks
#define MANUAL_MODE_UPDATE_DELAY		50      ///< Input polling period is ms while manual motor drive
/** @} */

/* ========================================================================== */
/*                           Global RTOS Objects                              */
/* ========================================================================== */

/** @brief FreeRTOS queue for passing input events from TaskInput to TaskUI */
QueueHandle_t InputQueue = NULL;
/** @brief FreeRTOS mutex to protect I2C2 bus between LCD and AS5600 */
SemaphoreHandle_t I2C2_Mutex = NULL;

/** @brief Global stepper control struct (shared between tasks and TIM2 ISR) */
extern volatile StepperControl_t gStepperControl;
extern TaskHandle_t TaskInputHandle;

/* ========================================================================== */
/*                         Global Application State                           */
/* ========================================================================== */

/** @brief Live-updated pumped volume in mL (written by TaskMotor, read by TaskUI) */
volatile float gLivePumpedVolume_ml = 0.0f;
/** @brief Current top-level system state (idle, pumping, paused) */
volatile SystemState_e gSysState = SYS_IDLE;

/**
 * @brief  Hardware specifications for the pump mechanism.
 *
 * @details These values define the physical characteristics of the lead screw
 *          and stepper motor. They are used by TaskMotor to convert flow rate
 *          (mL/min) into step frequency (Hz).
 */
PumpHardwareSpecs_t gPumpHw = {
    .screwLead_mm_per_rev = 2.0f,       ///< Lead screw travel per revolution (2mm for T8 lead screw)
    .motorStepsPerRev = 200,             ///< 1.8° stepper = 200 full steps/rev
    .microstepping = DEFAULT_MICROSTEPPING, ///< Must match CHOPCONF MRES set by stepper_init() over UART
    .maxFlowRate_ul_min = 100000.0f,     ///< Theoretical max flow in µL/min
    .maxFlowRate_ml_min = 100.0f         ///< Theoretical max flow in mL/min
};

/**
 * @brief  User-configured pump job parameters (set via the menu UI).
 *
 * @details Defaults are overwritten at init time from the flash database
 *          if at least one syringe profile is saved.
 */
PumpJobParams_t gPumpJob = {
    .targetVolume = 1.0f,
    .volumeUnit = VOL_UNIT_ML,
    .targetFlowRate = 10.0f,
    .flowUnit = FLOW_UNIT_ML_MIN,
    .syringeDiameter_mm = 0.0f,
    .syringeName = "DEFAULT"
};

/** @brief Previous encoder tick count for direction detection */
static uint16_t lastEncCount = 0;

/** @brief Flag signaling TaskMotor to resynchronize closed-loop controller on resume */
static volatile bool sStallResumeResync = false;
static float sJobBaselineDistance_mm = 0.0f;
static float sJobTargetDist_mm       = 0.0f;
static float sJobStepsPerMm          = 0.0f;

/* ========================================================================== */
/*                     Hardware Interrupt Management                          */
/* ========================================================================== */

/* Mask/unmask EXTI line specifically at peripheral level without disabling NVIC EXTI9_5 */
static inline void EnableSelectButtonISR(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(buttonSelect_Pin);
    EXTI->IMR |= buttonSelect_Pin;
}

static inline void DisableSelectButtonISR(void) {
    EXTI->IMR &= ~buttonSelect_Pin;
}
/**
 * @brief  Initialize the RTOS menu system and load saved syringe data.
 *
 * @details Performs the following setup:
 *          1. Creates the FreeRTOS InputQueue (10 deep, InputMessage_t sized).
 *          2. Initializes the flash storage subsystem.
 *          3. If saved syringe profiles exist, loads the first one as default.
 *          4. Captures the initial encoder counter value for delta detection.
 *
 * @note   Must be called after peripheral init and before vTaskStartScheduler().
 */
void RtosTasks_Init(void) {
	DisableSelectButtonISR();

    /* Initialize flash database */
    FlashStorage_Init();

    /* Initialize UI menu subsystem & load active syringe profile */
    UiMenu_Init();

    /* Initialize closed-loop motion supervisor with default thresholds */
    ClosedLoop_Init(NULL);
    
    /*
     * Capture initial encoder counter for delta-based direction detection.
     * TIM1 is in encoder mode: CNT counts up/down with each quadrature edge.
     * Right-shift by ENCODER_BIT_SHIFT to get detent-aligned tick count.
     */
    lastEncCount = TIM1->CNT >> ENCODER_BIT_SHIFT;

    /* Create the inter-task message queue (InputTask → UITask) */
    InputQueue = xQueueCreate(10, sizeof(InputMessage_t));
    I2C2_Mutex = xSemaphoreCreateMutex();

    /* Initialize UART Gatekeeper queue */
    stepper_uart_gatekeeper_init();
}

/* ========================================================================== */
/*                        TaskInput — Input Polling                          */
/* ========================================================================== */

/**
 * @brief  Lightweight input polling task (runs at 50 Hz = 20 ms period).
 * @param  argument Unused (required by FreeRTOS task signature).
 *
 * @details Polls all physical inputs and generates InputMessage_t events:
 *
 *          **Rotary Encoder Rotation**:
 *          Reads TIM1->CNT (hardware encoder mode), computes delta from last
 *          sample. Positive delta → EVENT_ENC_CW, negative → EVENT_ENC_CCW.
 *          The large negative threshold (-10000) handles 16-bit counter
 *          wrap-around when rotating CW past 0xFFFF.
 *
 *          **Button Press Detection** (edge-triggered):
 *          Samples GPIO pins each cycle and fires events on falling edges
 *          (transition from released to pressed, active-low).
 *
 *          **Long Press Detection** (Select button only):
 *          Tracks how long the Select button has been continuously held.
 *          If held > 600 ms, fires EVENT_BTN_SELECT_LONG_PRESS once.
 *          On release, fires EVENT_BTN_SELECT_PRESS only if no long press
 *          was already fired.
 *
 * @note   Runs at priority 3 (above UI, below Motor) with a 128-word stack.
 */
void RtosTasks_TaskInput(void *argument) {
    InputMessage_t msg;
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Capture initial button states for edge detection */
    bool lastButtonCursorLeft = (HAL_GPIO_ReadPin(buttonCursorLeft_GPIO_Port, buttonCursorLeft_Pin) == GPIO_PIN_SET);
    bool lastButtonSelect = (HAL_GPIO_ReadPin(buttonSelect_GPIO_Port, buttonSelect_Pin) == GPIO_PIN_SET);
    bool lastButtonCursorRight = (HAL_GPIO_ReadPin(buttonCursorRight_GPIO_Port, buttonCursorRight_Pin) == GPIO_PIN_SET);

    uint32_t buttonSelect_press_tick = 0;
    bool buttonSelect_long_fired = false;

    for (;;) {
        /* Suspend polling during pumping; re-seed pin states on resume to prevent false edge clicks */
        if (gSysState == SYS_PUMPING || gSysState == SYS_PUMPING_CALIBRATION) {
            vTaskSuspend(NULL);
            
            /* Re-seed the edge detection state to prevent false clicks upon resuming */
            lastButtonCursorLeft = (HAL_GPIO_ReadPin(buttonCursorLeft_GPIO_Port, buttonCursorLeft_Pin) == GPIO_PIN_SET);
            bool btnSelectHeld = (HAL_GPIO_ReadPin(buttonSelect_GPIO_Port, buttonSelect_Pin) == GPIO_PIN_RESET);
            lastButtonSelect = !btnSelectHeld;
            if (btnSelectHeld) {
                buttonSelect_long_fired = true; /* Treat current physical press as already consumed by EXTI pause */
            } else {
                buttonSelect_long_fired = false;
            }
            lastButtonCursorRight = (HAL_GPIO_ReadPin(buttonCursorRight_GPIO_Port, buttonCursorRight_Pin) == GPIO_PIN_SET);
            lastEncCount = TIM1->CNT >> ENCODER_BIT_SHIFT;
            continue;
        }

        /* --- Sample all GPIO inputs --- */
        bool buttonCursorLeft = (HAL_GPIO_ReadPin(buttonCursorLeft_GPIO_Port, buttonCursorLeft_Pin) == GPIO_PIN_SET);
        bool buttonSelect = (HAL_GPIO_ReadPin(buttonSelect_GPIO_Port, buttonSelect_Pin) == GPIO_PIN_SET);
        bool buttonCursorRight = (HAL_GPIO_ReadPin(buttonCursorRight_GPIO_Port, buttonCursorRight_Pin) == GPIO_PIN_SET);

        /*
         * --- Left and Right buttons action detection ---
         * Fires for once after transition detected.
         * This logic allows user to drive stepper motor manually.
         */
        if(!buttonCursorLeft){
        	if(lastButtonCursorLeft){
        		msg.event = EVENT_BTN_CURSOR_LEFT_PRESS;
        		msg.timestamp = xTaskGetTickCount();
        		xQueueSend(InputQueue, &msg, 0);
        	}
        }
        else{
        	if(!lastButtonCursorLeft){
        	msg.event = EVENT_BTN_CURSOR_LEFT_RELEASE;
        	msg.timestamp = xTaskGetTickCount();
        	xQueueSend(InputQueue, &msg, 0);
        	}
        }

        if(!buttonCursorRight){
        	if(lastButtonCursorRight){
        		msg.event = EVENT_BTN_CURSOR_RIGHT_PRESS;
        		msg.timestamp = xTaskGetTickCount();
        		xQueueSend(InputQueue, &msg, 0);
        	}
        }
        else{
        	if(!lastButtonCursorRight){
        	msg.event = EVENT_BTN_CURSOR_RIGHT_RELEASE;
        	msg.timestamp = xTaskGetTickCount();
        	xQueueSend(InputQueue, &msg, 0);
        	}
        }

        /*
         * --- Select button: short press + long press detection ---
         *
         * State machine for the Select button:
         * - On press (falling edge): record the tick, clear long-press flag.
         * - While held: if duration > 600 ms, fire long-press event (once).
         * - On release (rising edge): if no long-press was fired, fire short press.
         *
         * Note: The Select button is active-LOW (pressed = GPIO low).
         */
        if (!buttonSelect) {
            /* Button is currently pressed (active-low) */
            if (!lastButtonSelect) {
                /* Still held — check for long press threshold */
                if (!buttonSelect_long_fired && (xTaskGetTickCount() - buttonSelect_press_tick > pdMS_TO_TICKS(600))) {
                    msg.event = EVENT_BTN_SELECT_LONG_PRESS;
                    msg.timestamp = xTaskGetTickCount();
                    xQueueSend(InputQueue, &msg, 0);
                    buttonSelect_long_fired = true;  /* Prevent repeat fires */
                }
            }
            else {
                /* Falling edge: button just pressed — start timing */
                buttonSelect_press_tick = xTaskGetTickCount();
                buttonSelect_long_fired = false;
            }
        }
        else {
            /* Button is released (high) */
            if (!lastButtonSelect) {
                /* Rising edge: button just released */
                if (!buttonSelect_long_fired) {
                    /* No long-press fired → treat as a short press */
                    msg.event = EVENT_BTN_SELECT_PRESS;
                    msg.timestamp = xTaskGetTickCount();
                    xQueueSend(InputQueue, &msg, 0);
                }
            }
        }

        /* Store current states for next cycle's edge detection */
        lastButtonCursorLeft = buttonCursorLeft;
        lastButtonSelect = buttonSelect;
        lastButtonCursorRight = buttonCursorRight;

        /*
         * --- Rotary Encoder: delta-based direction detection ---
         *
         * TIM1 is configured in encoder mode: CNT increments/decrements
         * on each quadrature edge. We right-shift by ENCODER_BIT_SHIFT
         * to convert edge counts to detent counts.
         *
         * A large negative diff (< -10000) indicates the 16-bit counter
         * wrapped around 0→65535, which actually means CW rotation.
         */
        uint16_t currentEnc = TIM1->CNT >> ENCODER_BIT_SHIFT;
        if (currentEnc != lastEncCount) {
            msg.timestamp = xTaskGetTickCount();
            int16_t diff = (int16_t)(currentEnc - lastEncCount);
            if (diff > 0 || diff < -10000) { 
                /* Clockwise rotation (or CW wrap-around) */
                msg.event = EVENT_ENC_CW;
                xQueueSend(InputQueue, &msg, 0);
            }
            else {
                /* Counter-clockwise rotation */
                msg.event = EVENT_ENC_CCW;
                xQueueSend(InputQueue, &msg, 0);
            }
            lastEncCount = currentEnc;
        }

        /* 20 ms polling period → 50 Hz input sampling rate */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ========================================================================== */
/*                     Hardware Bridge Helper Functions                       */
/* ========================================================================== */

static void RtosTasks_StartPump(void) {
    if (UiMenu_GetDirection() == DIR_FORWARD) {
        HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_FORWARD);
    } else {
        HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_BACKWARD);
    }
    gSysState = SYS_PUMPING;
    EnableSelectButtonISR();
}

static void RtosTasks_StartCalibrationPump(void) {
    HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_FORWARD);
    gSysState = SYS_PUMPING_CALIBRATION;
    EnableSelectButtonISR();
}

static void RtosTasks_CancelPump(void) {
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    ClosedLoop_Reset();

    sJobStepsPerMm = 0.0f;
    sStallResumeResync = false;
    gStepperControl.totalStepsRemaining = 0;
    /* Note: preserve partial delivered volume in gLivePumpedVolume_ml (do NOT zero it out) */
    gStepperControl.MotorState = MOTOR_STATE_IDLE;

    gSysState = SYS_IDLE;
    DisableSelectButtonISR();
    stepper_set_spreadcycle(false);
    if (TaskInputHandle != NULL) {
        vTaskResume(TaskInputHandle);
    }
}

static void RtosTasks_ResumePump(void) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /* If resuming an active job, re-anchor remaining steps to ground-truth physical travel */
    if (sJobStepsPerMm > 0.0f) {
        AS5600_Snapshot_t snap = {0};
        AS5600_GetSnapshot(&snap);
        if (snap.isValid) {
            float actualTravel_mm = pump_calc_relative_travel_mm(
                snap.travelledDistance_mm,
                sJobBaselineDistance_mm,
                (UiMenu_GetDirection() == DIR_FORWARD));

            gStepperControl.totalStepsRemaining = pump_calc_remaining_steps(
                sJobTargetDist_mm,
                actualTravel_mm,
                sJobStepsPerMm);
        }
    }

    if (gStepperControl.totalStepsRemaining == 0) {
        /* The pump reached target distance right as we paused/stalled */
        sJobStepsPerMm = 0.0f;
        gSysState = SYS_IDLE;
        DisableSelectButtonISR();
        stepper_set_spreadcycle(false);
        UiMenu_OnMotorComplete();
        if (TaskInputHandle != NULL) {
            vTaskResume(TaskInputHandle);
        }
    } else {
        gSysState = SYS_PUMPING;
        EnableSelectButtonISR();

        /* Ensure TMC2208 is verified in SpreadCycle and MS128 before resuming pulses */
        stepper_set_spreadcycle(true);

        /* Reset closed-loop controller and arm 250 ms acceleration blanking window on resume */
        ClosedLoop_Reset();
        ClosedLoop_ArmBlanking(CL_DEFAULT_BLANKING_MS);

        /* Re-clamp acceleration steps if we paused very close to the end */
        if (gStepperControl.totalStepsRemaining < (2 * gStepperControl.accelSteps)) {
            gStepperControl.accelSteps = gStepperControl.totalStepsRemaining / 2;
        }

        /* Restart timer with acceleration from current state */
        gStepperControl.rampStepCounter = 0;
        gStepperControl.MotorState = MOTOR_STATE_ACCEL;
        gStepperControl.ARRCurrent = (uint32_t)((float)TIMER_CLOCK / ((PRESCALER + 1) * gStepperControl.startFreqHz)) - 1;

        __HAL_TIM_SET_AUTORELOAD(&htim2, gStepperControl.ARRCurrent);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, gStepperControl.ARRCurrent / 2);
        TIM2->EGR |= TIM_EGR_UG;

        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }
}

static void RtosTasks_HandleManualMove(InputEvent_e initialEvent) {
    InputMessage_t msg;
    msg.event = initialEvent;

    if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
        /* Wait for any ongoing deceleration to finish */
        while (gStepperControl.MotorState != MOTOR_STATE_IDLE) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_BACKWARD);
        gStepperControl.MotorMode = MOTOR_MODE_MANUAL;
        stepper_start((StepperControl_t*)&gStepperControl, MANUAL_CRUISE_FREQ, MANUAL_TOTAL_STEPS, MANUAL_ACCEL_STEPS);
        while (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
            xQueueReceive(InputQueue, &msg, pdMS_TO_TICKS(MANUAL_MODE_UPDATE_DELAY));
            if (msg.event == EVENT_BTN_CURSOR_LEFT_RELEASE) {
                if (gStepperControl.MotorState == MOTOR_STATE_ACCEL || gStepperControl.MotorState == MOTOR_STATE_CRUISE) {
                    gStepperControl.MotorState = MOTOR_STATE_DECEL;
                }
            }
        }
    }
    else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
        /* Wait for any ongoing deceleration to finish */
        while (gStepperControl.MotorState != MOTOR_STATE_IDLE) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_FORWARD);
        gStepperControl.MotorMode = MOTOR_MODE_MANUAL;
        stepper_start((StepperControl_t*)&gStepperControl, MANUAL_CRUISE_FREQ, MANUAL_TOTAL_STEPS, MANUAL_ACCEL_STEPS);
        while (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
            xQueueReceive(InputQueue, &msg, pdMS_TO_TICKS(MANUAL_MODE_UPDATE_DELAY));
            if (msg.event == EVENT_BTN_CURSOR_RIGHT_RELEASE) {
                if (gStepperControl.MotorState == MOTOR_STATE_ACCEL || gStepperControl.MotorState == MOTOR_STATE_CRUISE) {
                    gStepperControl.MotorState = MOTOR_STATE_DECEL;
                }
            }
        }
    }
}

/* ========================================================================== */
/*                     TaskUI — Menu State Machine                           */
/* ========================================================================== */

/**
 * @brief  UI state machine task (event-driven, blocks on InputQueue).
 * @param  argument Unused (required by FreeRTOS task signature).
 *
 * @details Delegates LCD screen page processing to the ui_menu module.
 *          Handles system-level actions (pumping start, pause, resume, cancel)
 *          and async progress updates.
 *
 * @note   Stack size: 512 words. Priority: 2 (below Input and Motor tasks).
 */
void RtosTasks_TaskUI(void *argument) {
    (void)argument;
    InputMessage_t msg;
    UiMenu_Init();
    TickType_t lastDrawTick = xTaskGetTickCount();

    for (;;) {
        TickType_t waitTime = portMAX_DELAY;
        TickType_t now = xTaskGetTickCount();
        if (UiMenu_IsPumpingView()) {
            if ((now - lastDrawTick) >= pdMS_TO_TICKS(SCREEN_REMAINING_VOL_UPDATE)) waitTime = 0;
            else waitTime = pdMS_TO_TICKS(SCREEN_REMAINING_VOL_UPDATE) - (now - lastDrawTick);
        }

        if (xQueueReceive(InputQueue, &msg, waitTime) == pdTRUE) {
            /* Pumping Priority Handler: intercept pause and motor completion */
            if (gSysState == SYS_PUMPING || gSysState == SYS_PUMPING_CALIBRATION) {
                if (msg.event == EVENT_MOTOR_COMPLETE) {
                    gSysState = SYS_IDLE;
                    DisableSelectButtonISR();
                    if (TaskInputHandle != NULL) vTaskResume(TaskInputHandle);
                    if (UiMenu_GetState() == MENU_ADD_SYRINGE_AUTO_PUMPING) {
                        UiMenu_OnCalibrationMotorComplete();
                    } else {
                        gStepperControl.totalStepsRemaining = 0;
                        UiMenu_OnMotorComplete();
                    }
                }
                else if (msg.event == EVENT_BTN_SELECT_PRESS && (gSysState == SYS_PUMPING || gSysState == SYS_PUMPING_CALIBRATION)) {
                    gSysState = SYS_PAUSED;
                    DisableSelectButtonISR();
                    gStepperControl.MotorState = MOTOR_STATE_DECEL;
                    if (TaskInputHandle != NULL) vTaskResume(TaskInputHandle);
                    UiMenu_OnPauseRequested();
                }
                else if (msg.event == EVENT_STALL_DETECTED) {
                    UiMenu_ProcessEvent(msg.event);
                }
                continue;
            }

            /* Manual Free Move Intercept */
            if (UiMenu_GetState() == MENU_FREE_MOVE_VIEW &&
                (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS || msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS)) {
                RtosTasks_HandleManualMove(msg.event);
                continue;
            }

            /* Standard Menu Page Event Dispatch */
            UiMenuAction_e action = UiMenu_ProcessEvent(msg.event);
            switch (action) {
                case UI_ACTION_START_PUMP:
                    RtosTasks_StartPump();
                    break;
                case UI_ACTION_START_CALIBRATION_PUMP:
                    RtosTasks_StartCalibrationPump();
                    break;
                case UI_ACTION_RESUME_PUMP:
                    sStallResumeResync = true;
                    RtosTasks_ResumePump();
                    break;
                case UI_ACTION_CANCEL_PUMP:
                    RtosTasks_CancelPump();
                    break;
                case UI_ACTION_RESUME_STALL_PUMP:
                    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                    sStallResumeResync = true;
                    RtosTasks_ResumePump();
                    break;
                case UI_ACTION_ABORT_STALL_PUMP:
                    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                    RtosTasks_CancelPump();
                    stepper_set_spreadcycle(false);
                    gSysState = SYS_IDLE;
                    break;
                case UI_ACTION_NONE:
                default:
                    break;
            }
        }

        /* Periodic Async Screen Refresh during Pumping */
        if (UiMenu_IsPumpingView() && gSysState == SYS_PUMPING) {
            TickType_t now = xTaskGetTickCount();
            if ((now - lastDrawTick) >= pdMS_TO_TICKS(SCREEN_REMAINING_VOL_UPDATE)) {
                lastDrawTick = now;
                UiMenu_UpdatePumpingView(gLivePumpedVolume_ml);
            }
        }
    }
}

/* ========================================================================== */
/*                    TaskMotor — Motor Supervisor                           */
/* ========================================================================== */

/**
 * @brief  Motor supervisor task — computes kinematics and monitors pump progress.
 * @param  argument Unused (required by FreeRTOS task signature).
 *
 * @details This task performs the complete kinematics pipeline when gSysState
 *          transitions to SYS_PUMPING:
 *
 *          **Step 1 — Volume Conversion**:
 *          Convert user target volume to mm³ (internal unit for volumetric math).
 *
 *          **Step 2 — Flow Rate Conversion**:
 *          Convert user flow rate to mm³/sec through the unit cascade:
 *            user_unit → mm³/min → mm³/sec
 *
 *          **Step 3 — Syringe Cross-Sectional Area**:
 *          A = π * (diameter/2)²
 *
 *          **Step 4 — Linear Kinematics**:
 *          - Linear travel (mm) = volume (mm³) / area (mm²)
 *          - Linear speed (mm/s) = flow (mm³/s) / area (mm²)
 *
 *          **Step 5 — Motor Step Calculation**:
 *          - stepsPerMm = (motorStepsPerRev / screwLead_mm) * microstepping
 *          - totalSteps = linearTravel * stepsPerMm
 *          - cruiseHz = linearSpeed * stepsPerMm
 *          - accelSteps = (cruiseHz^2) / (2 * TARGET_ACCEL_RATE)
 *          - Constraints: accelSteps is clamped between MIN_ACCEL_STEPS and MAX_ACCEL_STEPS.
 *            If the total move is short (totalSteps < 2 * accelSteps), accelSteps = totalSteps / 2.
 *
 *          After launching the stepper, the task enters a 50 Hz (20 ms)
 *          closed-loop supervisory loop that updates gLivePumpedVolume_ml from
 *          AS5600 encoder odometry, checks for motor stalls, and executes
 *          post-move creep correction if needed.
 *
 * @note   Priority 4 (highest). Stack: 512 words. Polls gSysState every 100 ms
 *         when idle, and runs the 50 Hz (20 ms) closed-loop supervisory loop
 *         while pumping.
 */
void RtosTasks_TaskMotor(void *argument) {
    uint32_t ulNotifiedValue;
    for (;;) {
        /* Wait for task notification or timeout (100 ms polling interval) */
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, pdMS_TO_TICKS(100));
        
        if (gSysState == SYS_PUMPING || gSysState == SYS_PUMPING_CALIBRATION) {
            if (gSysState == SYS_PUMPING_CALIBRATION) {
                /* ---- Calibration Pumping: Fixed distance ---- */
                PumpHwConfig_t calibHw = {
                    .screwLead_mm_per_rev = gPumpHw.screwLead_mm_per_rev,
                    .motorStepsPerRev = gPumpHw.motorStepsPerRev,
                    .microstepping = DEFAULT_MICROSTEPPING
                };
                float stepsPerMm = pump_calc_steps_per_mm(&calibHw);
                uint32_t totalSteps = (uint32_t)(CALIBRATION_PUMP_DISTANCE_MM * stepsPerMm);
                float cruiseHz = 2.0f * stepsPerMm;
                uint32_t accel = pump_calc_accel_steps(cruiseHz, TARGET_ACCEL_RATE, MIN_ACCEL_STEPS, MAX_ACCEL_STEPS, totalSteps);
                
                stepper_set_spreadcycle(true);
                stepper_start((StepperControl_t*)&gStepperControl, cruiseHz, totalSteps, accel);
                
                while (gSysState == SYS_PUMPING_CALIBRATION) {
                    if (gStepperControl.MotorState == MOTOR_STATE_IDLE) {
                        stepper_set_spreadcycle(false);
                        InputMessage_t msg = { .event = EVENT_MOTOR_COMPLETE };
                        xQueueSend(InputQueue, &msg, 0);
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
            else {
                /* ---- Normal Pumping ---- */
                PumpHwConfig_t hwConfig = {
                    .screwLead_mm_per_rev = gPumpHw.screwLead_mm_per_rev,
                    .motorStepsPerRev = gPumpHw.motorStepsPerRev,
                    .microstepping = DEFAULT_MICROSTEPPING
                };
                gPumpHw.microstepping = DEFAULT_MICROSTEPPING;

                PumpKinematics_t k;
                pump_calc_kinematics(gPumpJob.targetVolume, (PumpVolumeUnit_e)gPumpJob.volumeUnit,
                                     gPumpJob.targetFlowRate, (PumpFlowUnit_e)gPumpJob.flowUnit,
                                     gPumpJob.syringeDiameter_mm, &hwConfig,
                                     TARGET_ACCEL_RATE, MIN_ACCEL_STEPS, MAX_ACCEL_STEPS,
                                     &k);

                float area = k.area_mm2;
                float stepsPerMm = k.stepsPerMm;
                uint32_t totalSteps = k.totalSteps;
                float cruiseHz = k.cruiseFreqHz;
                uint32_t accel = k.accelSteps;
                float commandedVel_mm_s = k.linearSpeed_mm_sec;
                float targetDist_mm = k.linearTravel_mm;

                /* Closed-loop supervisor state */
                sStallResumeResync = false;
                TickType_t xLastWakeTime = xTaskGetTickCount();
                uint32_t lastTick_ms = (uint32_t)xLastWakeTime;

                /* Initialize Closed-Loop controller and arm 250 ms acceleration blanking window */
                ClosedLoop_Init(NULL);

                /* Capture initial encoder odometry baseline at job start */
                AS5600_Snapshot_t startSnap = {0};
                AS5600_GetSnapshot(&startSnap);
                float baselineDistance_mm = startSnap.isValid ? startSnap.travelledDistance_mm : 0.0f;
                sJobBaselineDistance_mm = baselineDistance_mm;
                sJobTargetDist_mm       = targetDist_mm;
                sJobStepsPerMm          = stepsPerMm;

                /* Reset live progress tracker and launch the stepper */
                gLivePumpedVolume_ml = 0.0f;
                stepper_set_spreadcycle(true);
                stepper_start((StepperControl_t*)&gStepperControl, cruiseHz, totalSteps, accel);

                /* 50 Hz Supervisory Loop */
                while (gSysState == SYS_PUMPING || gSysState == SYS_PAUSED) {
                    /* If paused, hold supervisory updates */
                    if (gSysState == SYS_PAUSED) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                        xLastWakeTime = xTaskGetTickCount();
                        lastTick_ms = (uint32_t)xLastWakeTime;
                        continue;
                    }

                    /* Check if primary move has completed */
                    if (gStepperControl.MotorState == MOTOR_STATE_IDLE) {
                        break;
                    }

                    /* 1. Calculate timing delta */
                    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
                    uint32_t dt_ms = now_ms - lastTick_ms;
                    if (dt_ms == 0U) {
                        dt_ms = MOTOR_SUPERVISORY_PERIOD_MS;
                    }
                    lastTick_ms = now_ms;

                    /* 2. Sample current AS5600 encoder snapshot */
                    AS5600_Snapshot_t snapshot = {0};
                    AS5600_GetSnapshot(&snapshot);

                    /* 3. Track commanded progress */
                    uint32_t remaining = gStepperControl.totalStepsRemaining;
                    uint32_t stepsDone = (totalSteps > remaining) ? (totalSteps - remaining) : 0U;
                    float commandedDist_mm = (stepsPerMm > 0.0f) ? ((float)stepsDone / stepsPerMm) : 0.0f;

                    /* 4. Resynchronize closed-loop controller on resume */
                    if (sStallResumeResync) {
                        sStallResumeResync = false;
                        ClosedLoop_Reset();
                        ClosedLoop_ArmBlanking(CL_DEFAULT_BLANKING_MS);
                    }

                    /* 5. Compute relative displacement from baseline */
                    float actualDist_mm = 0.0f;
                    if (snapshot.isValid) {
                        actualDist_mm = pump_calc_relative_travel_mm(
                            snapshot.travelledDistance_mm,
                            baselineDistance_mm,
                            (UiMenu_GetDirection() == DIR_FORWARD));
                    }
                    float actualVel_mm_s = snapshot.isValid ? fabsf(snapshot.currentVelocity_mm_s) : 0.0f;

                    /* Calculate live pumped volume strictly from ground-truth encoder distance */
                    if (snapshot.isValid) {
                        gLivePumpedVolume_ml = (actualDist_mm * area) / 1000.0f;
                    } else {
                        gLivePumpedVolume_ml = pump_calc_pumped_volume_ml(stepsDone, stepsPerMm, area);
                    }

                    /* 6. Closed-Loop Supervisory update */
                    ClosedLoopOutput_t clOut = ClosedLoop_Update(
                        commandedDist_mm,
                        actualDist_mm,
                        commandedVel_mm_s,
                        actualVel_mm_s,
                        dt_ms,
                        (uint32_t)lroundf(stepsPerMm)
                    );

                    /* 7. Process Closed-Loop decisions */
                    if (clOut.status == CL_STATUS_STALL_DETECTED) {
                        /* Immediate Motor Halt on Stall Confirmation */
                        HAL_TIM_Base_Stop_IT(&htim2);
                        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                        gStepperControl.MotorState = MOTOR_STATE_PAUSED;
                        gSysState = SYS_PAUSED;
                        DisableSelectButtonISR();
                        if (TaskInputHandle != NULL) {
                            vTaskResume(TaskInputHandle);
                        }
                        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
                        InputMessage_t msg = {
                            .event = EVENT_STALL_DETECTED,
                            .timestamp = xTaskGetTickCount()
                        };
                        xQueueSendToFront(InputQueue, &msg, 0);
                    }

#if CLOSED_LOOP_TEST
                    gClosedLoopDebug.targetDist_mm       = targetDist_mm;
                    gClosedLoopDebug.baselineDist_mm     = baselineDistance_mm;
                    gClosedLoopDebug.commandedDist_mm    = commandedDist_mm;
                    gClosedLoopDebug.actualDist_mm       = actualDist_mm;
                    gClosedLoopDebug.commandedVel_mm_s   = commandedVel_mm_s;
                    gClosedLoopDebug.actualVel_mm_s      = actualVel_mm_s;
                    gClosedLoopDebug.positionLag_mm      = clOut.positionLag_mm;
                    gClosedLoopDebug.velocityRatio       = clOut.velocityRatio;
                    gClosedLoopDebug.status              = clOut.status;
                    gClosedLoopDebug.stallTimer_ms       = ClosedLoop_GetStallTimer();
                    gClosedLoopDebug.blankingTimer_ms    = ClosedLoop_GetBlankingTimer();
                    gClosedLoopDebug.blankingActive      = clOut.blankingActive;
                    gClosedLoopDebug.isStalled           = ClosedLoop_IsStalled();
                    gClosedLoopDebug.compensationSteps   = clOut.compensationSteps;
                    gClosedLoopDebug.stepsDone           = stepsDone;
                    gClosedLoopDebug.stepsRemaining      = gStepperControl.totalStepsRemaining;
                    gClosedLoopDebug.motorState          = (uint8_t)gStepperControl.MotorState;
                    gClosedLoopDebug.encoderValid        = snapshot.isValid;
                    gClosedLoopDebug.updateCount++;
#endif

                    /* 50 Hz rate control */
                    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_SUPERVISORY_PERIOD_MS));
                }

                /* ---- Move Completion & Feature 7: Post-Move Creep Step Correction ---- */
                if (gStepperControl.MotorState == MOTOR_STATE_IDLE && gSysState == SYS_PUMPING) {
                    /* Settling delay for encoder EMA dynamic-alpha */
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_SUPERVISORY_PERIOD_MS));

                    AS5600_Snapshot_t finalSnap = {0};
                    AS5600_GetSnapshot(&finalSnap);
                    float finalActualDist_mm = 0.0f;
                    if (finalSnap.isValid) {
                        finalActualDist_mm = pump_calc_relative_travel_mm(
                            finalSnap.travelledDistance_mm,
                            baselineDistance_mm,
                            (UiMenu_GetDirection() == DIR_FORWARD));
                    }

                    uint32_t creepSteps = ClosedLoop_CalcCreepSteps(targetDist_mm, finalActualDist_mm,
                                                                    CL_DEFAULT_CREEP_LAG_MM,
                                                                    CL_DEFAULT_CREEP_MAX_STEPS,
                                                                    (uint32_t)lroundf(stepsPerMm));
#if CLOSED_LOOP_TEST
                    gClosedLoopDebug.creepStepsLast = creepSteps;
                    gClosedLoopDebug.actualDist_mm  = finalActualDist_mm;
#endif
                    if (creepSteps >= 2U) {
                        uint32_t creepAccel = (creepSteps >= 2U * MIN_ACCEL_STEPS) ? MIN_ACCEL_STEPS : (creepSteps / 2U);
                        if (creepAccel == 0U) {
                            creepAccel = 1U;
                        }

                        /* Launch low-speed creep correction move at START_FREQ_HZ (maximum torque) */
                        stepper_set_spreadcycle(true);
                        stepper_start((StepperControl_t*)&gStepperControl, (float)START_FREQ_HZ, creepSteps, creepAccel);

                        while (gStepperControl.MotorState != MOTOR_STATE_IDLE && gSysState == SYS_PUMPING) {
                            vTaskDelay(pdMS_TO_TICKS(MOTOR_SUPERVISORY_PERIOD_MS));

                            /* Update live volume during creep */
                            AS5600_Snapshot_t creepSnap = {0};
                            AS5600_GetSnapshot(&creepSnap);
                            if (creepSnap.isValid) {
                                float creepDist = pump_calc_relative_travel_mm(
                                    creepSnap.travelledDistance_mm,
                                    baselineDistance_mm,
                                    (UiMenu_GetDirection() == DIR_FORWARD));
                                if (creepDist > targetDist_mm) {
                                    creepDist = targetDist_mm;
                                }
                                uint32_t creepDoneSteps = (uint32_t)lroundf(creepDist * stepsPerMm);
                                (void)creepDoneSteps;
                                gLivePumpedVolume_ml = (creepDist * area) / 1000.0f;
                            }
                        }

                        /* Settling delay for post-creep encoder snap before final reading */
                        vTaskDelay(pdMS_TO_TICKS(MOTOR_SUPERVISORY_PERIOD_MS));
                    }

                    /* Pumping successfully finished */
                    if (gSysState == SYS_PUMPING) {
                        sJobStepsPerMm = 0.0f;
                        stepper_set_spreadcycle(false);

                        AS5600_Snapshot_t endSnap = {0};
                        AS5600_GetSnapshot(&endSnap);
                        if (endSnap.isValid) {
                            float finalDist = pump_calc_relative_travel_mm(
                                endSnap.travelledDistance_mm,
                                baselineDistance_mm,
                                (UiMenu_GetDirection() == DIR_FORWARD));
                            gLivePumpedVolume_ml = (finalDist * area) / 1000.0f;
#if CLOSED_LOOP_TEST
                            gClosedLoopDebug.actualDist_mm  = finalDist;
                            gClosedLoopDebug.positionLag_mm = targetDist_mm - finalDist;
                            gClosedLoopDebug.motorState     = (uint8_t)gStepperControl.MotorState;
#endif
                        } else {
                            gLivePumpedVolume_ml = pump_calc_pumped_volume_ml(totalSteps, stepsPerMm, area);
                        }

                        InputMessage_t msg = { .event = EVENT_MOTOR_COMPLETE };
                        xQueueSend(InputQueue, &msg, 0);
                    }
                }
            } /* End of else (normal pumping) */
        }
    }
}

void RtosTasks_TaskEncoder(void* argument){
    /* Wait for system to initialize */
    vTaskDelay(pdMS_TO_TICKS(100));
    if( I2C2_Mutex != NULL && xSemaphoreTake(I2C2_Mutex, pdMS_TO_TICKS(50)) == pdTRUE){
		AS5600_Check_Magnet_Status();
		xSemaphoreGive(I2C2_Mutex);
    }
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {
        if (I2C2_Mutex != NULL) {
            if (xSemaphoreTake(I2C2_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                /* Perform blocking I2C angle read (5 ms timeout) */
                HAL_StatusTypeDef status = AS5600_Read_Angle();
                
                if (status == HAL_OK) {
                        AS5600_Process_I2C_Buffer();
                    }
                else if(status == HAL_ERROR) {

                	AS5600_I2C_Error_Handler();
                }

                /* Release mutex after I2C read and angle processing */
                xSemaphoreGive(I2C2_Mutex);
            }
        }

        /* Strict 100 Hz sampling period (10 ms) immune to execution drift for accurate odometry */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vApplicationIdleHook(void) {

}
