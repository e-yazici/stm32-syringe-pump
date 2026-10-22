/**
 * @file    rtos_menu.c
 * @brief   FreeRTOS task implementations and UI state machine for the Syringe Pump.
 * @author  eyazici
 * @date    2026
 *
 * @details Implements the complete syringe pump application logic using a
 *          3-task FreeRTOS architecture:
 *
 *          **TaskInput** (Priority 3, 50 Hz polling):
 *          Reads the rotary encoder counter (TIM1 hardware encoder mode) and
 *          three push buttons via GPIO polling. Detects direction changes,
 *          button press/release edges, and long-press events. Pushes
 *          InputMessage_t events into a FreeRTOS queue.
 *
 *          **TaskUI** (Priority 2, event-driven):
 *          Blocks on the InputQueue waiting for events. Implements a hierarchical
 *          menu state machine that drives the 16×2 LCD. Menu states include:
 *          volume/flow rate editing with digit-level cursor, syringe profile
 *          selection and creation, pump operation screens, and warning dialogs.
 *
 *          **TaskMotor** (Priority 4, highest):
 *          Converts user-specified volume, flow rate, and syringe dimensions into
 *          physical motor parameters (steps, frequency). Launches the stepper
 *          driver and tracks live progress for the UI's pumping view.
 *
 * @note    The motor task polls gSysState to detect SYS_PUMPING transitions
 *          rather than using explicit task notifications. This simplifies the
 *          UI→Motor interface at the cost of a 100 ms polling latency.
 */

#include "printf.h"
#include "rtos_menu.h"
#include "main.h"
#include "liquidcrystal_i2c.h"
#include "stepper.h"
#include "flash_storage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "as5600.h"
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

/** @defgroup RTOS_Menu_Config Application Configuration Constants
 *  @{
 */
#define SCREEN_REMAINING_VOL_UPDATE     1000    ///< Pumping screen refresh interval in ms
#define MATH_PI                         3.14159265f  ///< Pi constant for syringe area calculation
#define VOL_ML_TO_MM3                   1000.0f ///< Conversion factor: 1 mL = 1000 mm³
#define TIME_MIN_TO_SEC                 60.0f   ///< Conversion factor: 1 min = 60 sec
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

extern AS5600_Params_t gEncoderParams;

/** @brief Global stepper control struct (shared between tasks and TIM2 ISR) */
extern volatile StepperControl_t gStepperControl;
extern TaskHandle_t TaskInputHandle;
/* ========================================================================== */
/*                          Hardware Abstraction                              */
/* ========================================================================== */

/**
 * @brief  Configure the stepper driver microstepping pins.
 * @param  ms Microstepping divisor (e.g., 1, 2, 4, 8, 16, 32).
 *
 * @note   Placeholder implementation. Wire this to the actual MS1/MS2 GPIO
 *         pins for your specific stepper driver (e.g., A4988, DRV8825, TMC2209).
 *
 * @todo   Implement hardware GPIO writes for microstepping configuration.
 */
static void configMicrostepping(uint8_t ms) {
    /** 
     * @todo Implement based on your stepper driver's MS pin truth table.
     */
}

/* ========================================================================== */
/*                         Global Application State                           */
/* ========================================================================== */

/** @brief Live-updated pumped volume in mL (written by TaskMotor, read by TaskUI) */
volatile float gLivePumpedVolume_ml = 0.0f;
/** @brief Current top-level system state (idle, pumping, paused, fault) */
SystemState_e gSysState = SYS_IDLE;

/** @brief Current menu sub-state within the UI state machine */
MenuState_e gMenuState = MENU_OVERVIEW;

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

/* ========================================================================== */
/*                    Numeric Cursor Editing Subsystem                        */
/* ========================================================================== */

/**
 * @brief  Multiplier table for digit-level numeric editing.
 *
 * @details Maps the digit_cursor position to the value increment/decrement
 *          applied when the encoder is rotated:
 *          Index:  0          1      2     3     4        5         6
 *          Place:  0.01       0.1    1.0   10.0  100.0    1000.0    10000.0
 *                  hundredths tenths ones  tens  hundreds thousands ten-thousands
 */
static const float mults[] = {0.01f, 0.1f, 1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};
static const float weight_mults[] = {0.00001f, 0.0001f, 0.001f, 0.01f, 0.1f, 1.0f, 10.0f, 100.0f};

/** @brief Current cursor position index into mults[] (default: 2 = ones digit) */
static uint8_t digit_cursor = 2;

/* ========================================================================== */
/*                           Initialization                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                     Hardware Interrupt Management                          */
/* ========================================================================== */

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
void RtosMenu_Init(void) {
	DisableSelectButtonISR();
    /* Create the inter-task message queue (InputTask → UITask) */
    InputQueue = xQueueCreate(10, sizeof(InputMessage_t));
    I2C2_Mutex = xSemaphoreCreateMutex();

    /* Initialize flash database and load saved syringe profiles */
    FlashStorage_Init();
    SyringeDatabase_t* db = FlashStorage_GetDB();
    if (db->count > 0) {
        /* Load the first saved syringe as the active profile */
        gPumpJob.syringeDiameter_mm = db->syringes[0].innerDiameter_mm;
        gPumpJob.syringeMaxVolume_ml = db->syringes[0].maxVolume_ml;
        strcpy(gPumpJob.syringeName, db->syringes[0].name);
    }
    else {
        /* No saved profiles — set safe defaults to prevent division by zero */
        gPumpJob.syringeDiameter_mm = 0.0f;
        gPumpJob.syringeMaxVolume_ml = 0.0f;
        strcpy(gPumpJob.syringeName, "NO SYRINGE");
    }
    
    /* Ensure critical parameters are valid (in case .data initialization failed) */
    if (gPumpJob.targetVolume < 0.001f || gPumpJob.targetVolume > 1000.0f) {
        gPumpJob.targetVolume = 1.0f;
        gPumpJob.volumeUnit = VOL_UNIT_ML;
    }
    if (gPumpJob.targetFlowRate < 0.001f || gPumpJob.targetFlowRate > 1000.0f) {
        gPumpJob.targetFlowRate = 2.0f;
        gPumpJob.flowUnit = FLOW_UNIT_ML_MIN;
    }
    
    /*
     * Capture initial encoder counter for delta-based direction detection.
     * TIM1 is in encoder mode: CNT counts up/down with each quadrature edge.
     * Right-shift by ENCODER_BIT_SHIFT to get detent-aligned tick count.
     */
    lastEncCount = TIM1->CNT >> ENCODER_BIT_SHIFT;
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
 *          Samples GPIO pins each cycle and fires events on rising edges
 *          (transition from released to pressed). Active-low logic assumed.
 *
 *          **Long Press Detection** (Select button only):
 *          Tracks how long the Select button has been continuously held.
 *          If held > 600 ms, fires EVENT_BTN_SELECT_LONG_PRESS once.
 *          On release, fires EVENT_BTN_SELECT_PRESS only if no long press
 *          was already fired.
 *
 * @note   Runs at priority 3 (above UI, below Motor) with a 128-word stack.
 */
void RtosMenu_TaskInput(void *argument) {
    InputMessage_t msg;

    /* Delay to let GPIO pull-ups stabilize and avoid spurious edge detection at boot */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Capture initial button states for edge detection */
    bool lastButtonCursorLeft = (HAL_GPIO_ReadPin(buttonCursorLeft_GPIO_Port, buttonCursorLeft_Pin) == GPIO_PIN_SET);
    bool lastButtonSelect = (HAL_GPIO_ReadPin(buttonSelect_GPIO_Port, buttonSelect_Pin) == GPIO_PIN_SET);
    bool lastButtonCursorRight = (HAL_GPIO_ReadPin(buttonCursorRight_GPIO_Port, buttonCursorRight_Pin) == GPIO_PIN_SET);

    uint32_t buttonSelect_press_tick = 0;   ///< Tick count when Select was first pressed
    bool buttonSelect_long_fired = false;    ///< True if long-press event already fired for this press

    for (;;) {
        if (gSysState == SYS_PUMPING || gSysState == SYS_PUMPING_CALIBRATION) {
            vTaskSuspend(NULL); // Suspend myself
            
            /* Re-seed the edge detection state to prevent false clicks upon resuming */
            lastButtonCursorLeft = (HAL_GPIO_ReadPin(buttonCursorLeft_GPIO_Port, buttonCursorLeft_Pin) == GPIO_PIN_SET);
            lastButtonSelect = (HAL_GPIO_ReadPin(buttonSelect_GPIO_Port, buttonSelect_Pin) == GPIO_PIN_SET);
            lastButtonCursorRight = (HAL_GPIO_ReadPin(buttonCursorRight_GPIO_Port, buttonCursorRight_Pin) == GPIO_PIN_SET);
            lastEncCount = TIM1->CNT >> ENCODER_BIT_SHIFT;
            buttonSelect_long_fired = false;
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
/*                     Data Handling Helper Functions                         */
/* ========================================================================== */

/**
 * @brief  Clamp a volume value to valid range for the given unit.
 * @param  val  Volume value to clamp.
 * @param  unit Volume unit (determines max range: mL → 60, µL → 1000).
 * @retval Clamped volume value (>= 0).
 * @todo   Optimize max. values according to physical limits.
 */
static float ClampVolume(float val, VolumeUnit_e unit) {
    if (val < 0.0f) val = 0.0f;
    float max_val = (unit == VOL_UNIT_ML) ? 60.0f : 1000.0f;
    if (val > max_val) val = max_val;
    return val;
}

/**
 * @brief  Clamp a flow rate value to valid range for the given unit.
 * @param  val  Flow rate value to clamp.
 * @param  unit Flow rate unit (determines max range per unit type).
 * @retval Clamped flow rate value (>= 0).
 * @todo   Optimize max. values for stepper motor and driver
 */
static float ClampFlowRate(float val, FlowUnit_e unit) {
    if (val < 0.0f) val = 0.0f;
    float max_val = 1000.0f;
    switch(unit) {
        case FLOW_UNIT_ML_MIN: max_val = gPumpHw.maxFlowRate_ml_min; break;
        case FLOW_UNIT_UL_MIN: max_val = gPumpHw.maxFlowRate_ul_min; break;
        case FLOW_UNIT_ML_HR: max_val = 999.99f; break;
        case FLOW_UNIT_UL_HR: max_val = 9999.99f; break;
    }
    if (val > max_val) val = max_val;
    return val;
}

/**
 * @brief  Get the display string for a volume unit enum.
 * @param  unit Volume unit enum value.
 * @retval Static string: "mL" or "uL".
 */
static const char* GetVolumeUnitStr(VolumeUnit_e unit) {
    return (unit == VOL_UNIT_ML) ? "mL" : "uL";
}

/**
 * @brief  Get the display string for a flow rate unit enum.
 * @param  unit Flow rate unit enum value.
 * @retval Static string: "mL/m", "uL/m", "mL/h", or "uL/h".
 */
static const char* GetFlowUnitStr(FlowUnit_e unit) {
    switch(unit) {
        case FLOW_UNIT_ML_MIN: return "mL/m";
        case FLOW_UNIT_UL_MIN: return "uL/m";
        case FLOW_UNIT_ML_HR: return "mL/h";
        case FLOW_UNIT_UL_HR: return "uL/h";
    }
    return "";
}

/* ========================================================================== */
/*                         UI Drawing Helpers                                 */
/* ========================================================================== */

static const char* MainMenuStrings[MAIN_MENU_MAX] = {
    "> Select Syringe",
    "> Volume",
    "> Flowrate",
    "> Add Syringe",
    "> Set Direction",
    "> Pump",
    "> Manual Mode"
};

static uint8_t mainMenuCursor = 0;
static VolumeUnit_e tempVolUnit;
static FlowUnit_e tempFlowUnit;
static float tempVolume;
static float tempFlowRate;
static SyringeProfile_t tempAddSyringe;
static uint8_t list_cursor = 0;
static bool isModifyingSyringe = false;
static float tempWeight_g = 1.0f;
static MotorDirection_e tempDirection = DIR_FORWARD;
static MotorDirection_e currentDirection = DIR_FORWARD;

/** @brief Current selection in the ACTION menu (0=MODIFY, 1=DELETE) */
static uint8_t actionSelection = 0;
/** @brief Current selection in the DELETE confirm menu (0=YES, 1=NO) */
static uint8_t deleteConfirmSelection = 1;
/** @brief Current selection in DIA MODE menu (0=MANUAL, 1=AUTO) */
static uint8_t diaModeSelection = 0;

/**
 * @brief  Character set for syringe name entry.
 *
 * @details The encoder scrolls through this array to select each character
 *          position in the syringe name. Includes space, A–Z, and 0–9 (37 chars).
 */
static const char *const syringeNameCharset[37] = {
    " ", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L",
    "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y",
    "Z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"
};

/**
 * @brief  Render the main menu overview screen on the LCD.
 *
 * @details Draws a two-line display showing the currently selected menu item
 *          (prefixed with ">") and context-dependent information on the second
 *          line. The menu acts as a virtual scrollable list.
 */
static void DrawMainMenu(void) {
    char buf[17];
    HD44780_NoBlink();
    HD44780_NoCursor();
    HD44780_Clear();
    HD44780_SetCursor(0, 0);

    switch ((MainMenuItem_e)mainMenuCursor) {
        case MAIN_MENU_SELECT_SYRINGE:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_SELECT_SYRINGE]);
            HD44780_SetCursor(0, 1);
            snprintf(buf, sizeof(buf), "  %s", gPumpJob.syringeName);
            HD44780_PrintStr(buf);
            break;

        case MAIN_MENU_VOLUME:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_VOLUME]);
            HD44780_SetCursor(0, 1);
            snprintf(buf, sizeof(buf), "  %.2f %s", gPumpJob.targetVolume, GetVolumeUnitStr(gPumpJob.volumeUnit));
            HD44780_PrintStr(buf);
            break;
            
        case MAIN_MENU_FLOWRATE:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_FLOWRATE]);
            HD44780_SetCursor(0, 1);
            snprintf(buf, sizeof(buf), "  %.2f %s", gPumpJob.targetFlowRate, GetFlowUnitStr(gPumpJob.flowUnit));
            HD44780_PrintStr(buf);
            break;

        case MAIN_MENU_ADD_SYRINGE:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_ADD_SYRINGE]);
            HD44780_SetCursor(0, 1);
            HD44780_PrintStr("  Set Direction");
            break;

        case MAIN_MENU_DIRECTION:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_DIRECTION]);
            HD44780_SetCursor(0, 1);
            HD44780_PrintStr(currentDirection == DIR_FORWARD ? "  Forward" : "  Backward");
            break;

        case MAIN_MENU_PUMP:
            HD44780_PrintStr(" Set Direction");
            HD44780_SetCursor(0, 1);
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_PUMP]);
            break;
        case MAIN_MENU_FREE_MOVE:
        	HD44780_PrintStr("  Pump");
            HD44780_SetCursor(0, 1);
        	HD44780_PrintStr(MainMenuStrings[MAIN_MENU_FREE_MOVE]);
        	break;
        default:
            break;
    }
}

/**
 * @brief  Display a numeric value on line 2 with cursor at the active digit.
 * @param  val    The numeric value to display.
 * @param  prefix String printed before the number (e.g., "").
 * @param  suffix Unit string printed after the number (e.g., "mL").
 *
 * @details Formats the value as "prefix XX.XX suffix" on LCD line 2, then
 *          positions the hardware blinking cursor on the digit corresponding
 *          to the current digit_cursor index. The cursor position is calculated
 *          relative to the decimal point position in the formatted string.
 */
static void PrintNumberWithCursor(float val, const char* prefix, const char* suffix, int precision) {
    char buf[20];
    char fmt[16];
    /* Format the complete string for display */
    snprintf(fmt, sizeof(fmt), "%%s%%.%df %%s", precision);
    snprintf(buf, sizeof(buf), fmt, prefix, val, suffix);
    
    /* Clear and redraw line 2 */
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("                ");  /* Blank the entire line (16 chars) */
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(buf);

    /*
     * Calculate the LCD column for the hardware cursor based on digit_cursor.
     * Find the decimal point position, then offset from it:
     */
    char *dot = strchr(buf, '.');
    if (dot != NULL) {
        int dot_idx = dot - buf;
        int cur_x;
        if (digit_cursor < precision) {
            /* Fractional part */
            cur_x = dot_idx + (precision - digit_cursor);
        }
        else {
            /* Integer part */
            cur_x = dot_idx - (digit_cursor - precision + 1);
        }
        
        /* Clamp to display bounds */
        if (cur_x < 0) cur_x = 0;
        if (cur_x > 15) cur_x = 15;

        /* Position and enable the blinking cursor */
        HD44780_SetCursor((uint8_t)cur_x, 1);
        HD44780_Cursor();
        HD44780_NoBlink();
    }
}

/**
 * @brief  Display a unit selection screen with a title and current unit.
 * @param  title   Title string for LCD line 1 (e.g., "Select Unit").
 * @param  unitStr Current unit string for LCD line 2 (e.g., "mL/m").
 */
static void PrintUnitSelect(const char* title, const char* unitStr) {
    char buf[17];
    HD44780_NoBlink();
    HD44780_NoCursor();
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(title);
    
    snprintf(buf, sizeof(buf), "> Unit: %s", unitStr);
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(buf);
}

/* ========================================================================== */
/*                     TaskUI — Menu State Machine                           */
/* ========================================================================== */

/**
 * @brief  UI state machine task (event-driven, blocks on InputQueue).
 * @param  argument Unused (required by FreeRTOS task signature).
 *
 * @details Implements a two-layer state machine:
 *
 *          **Layer 1 — System State (gSysState)**:
 *          - SYS_PUMPING: Only accepts motor-complete and long-press (pause) events.
 *            All other inputs are discarded to prevent accidental menu changes.
 *          - SYS_IDLE/SYS_PAUSED: Full menu interaction is allowed.
 *
 *          **Layer 2 — Menu State (gMenuState)**:
 *          Each menu state handles a specific subset of events to navigate,
 *          edit values, or confirm actions. See MenuState_e for the full list.
 *
 *          The pumping view uses a hybrid event/timer model: it wakes up every
 *          500 ms to refresh the live progress display with alternating
 *          flow rate and remaining time animations.
 *
 * @note   Stack size: 256 words. Priority: 2 (below Input and Motor tasks).
 */
void RtosMenu_TaskUI(void *argument) {
    InputMessage_t msg;
    DrawMainMenu();

    static uint8_t anim_idx = 0;	    ///< Animation frame index for pumping view
    static uint8_t cancelSelection = 1; ///< Pause confirm dialog: 0 = YES, 1 = NO
    TickType_t lastDrawTick = xTaskGetTickCount();
    
    for (;;) {
        /*
         * Dynamic wait time: during pumping, wake up every 500 ms for screen refresh
         * even if no input events arrive. Otherwise, block indefinitely.
         */
        TickType_t waitTime = portMAX_DELAY;
        TickType_t now = xTaskGetTickCount();
        if (gMenuState == MENU_PUMPING_VIEW) {
            if ((now - lastDrawTick) >= pdMS_TO_TICKS(SCREEN_REMAINING_VOL_UPDATE)) waitTime = 0;
            else waitTime = pdMS_TO_TICKS(SCREEN_REMAINING_VOL_UPDATE) - (now - lastDrawTick);
        }
        
        if (xQueueReceive(InputQueue, &msg, waitTime) == pdTRUE) {
            
            /* =============================================================
             * PUMPING PRIORITY HANDLER
             * When the system is actively pumping, only two events are
             * processed: motor completion and long-press (pause). All other
             * inputs are silently discarded to prevent accidental changes.
             * ============================================================= */
            if (gSysState == SYS_PUMPING) {
                if (msg.event == EVENT_MOTOR_COMPLETE) {
                    /* --- Natural pump completion --- */
                    gSysState = SYS_IDLE;
                    DisableSelectButtonISR();
                    gMenuState = MENU_PUMP_COMPLETED;
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Pump Completed!");
                    HD44780_SetCursor(0, 1);
                    HD44780_PrintStr("Press any button");
                    
                    gStepperControl.totalStepsRemaining = 0;
                    if (TaskInputHandle != NULL) vTaskResume(TaskInputHandle);
                }
                else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                    /* --- User requested pause --- */
                    gSysState = SYS_PAUSED;
                    DisableSelectButtonISR();
                    /* Start decelerating the stepper motor */
                    gStepperControl.MotorState = MOTOR_STATE_DECEL;
                    
                    gMenuState = MENU_PUMP_PAUSE_CONFIRM;
                    cancelSelection = 1; /* Default selection: NO (don't cancel) */
                    
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Cancel Pump?");
                    HD44780_SetCursor(0, 1);
                    HD44780_PrintStr("  YES    > NO");
                    if (TaskInputHandle != NULL) vTaskResume(TaskInputHandle);
                }
                continue; /* Discard all other events while pumping */
            }

            /* =============================================================
             * MENU STATE MACHINE
             * Each case handles the input events relevant to that menu screen.
             * ============================================================= */
            switch (gMenuState) {

                /* ----- MAIN MENU OVERVIEW ----- */
                case MENU_OVERVIEW:
                    if (msg.event == EVENT_ENC_CW) {
                        if (mainMenuCursor < MAIN_MENU_MAX - 1) {
                            mainMenuCursor++;
                            DrawMainMenu();
                        }
                    } 
                    else if (msg.event == EVENT_ENC_CCW) {
                        if (mainMenuCursor > 0) {
                            mainMenuCursor--;
                            DrawMainMenu();
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        switch ((MainMenuItem_e)mainMenuCursor) {
                            case MAIN_MENU_SELECT_SYRINGE:
                                if (FlashStorage_GetDB()->count == 0) {
                                    break;  /* Silently ignore if no syringes saved */
                                }
                                gMenuState = MENU_SELECT_SYRINGE_LIST;
                                list_cursor = 0;
                                HD44780_Clear();
                                HD44780_PrintStr("> ");
                                HD44780_PrintStr(FlashStorage_GetDB()->syringes[list_cursor].name);
                                HD44780_SetCursor(0, 1);
                                char sbuf[17];
                                snprintf(sbuf, sizeof(sbuf), "D:%.1fmm, %.1fml",
                                        FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm,
                                        FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml);
                                HD44780_PrintStr(sbuf);
                                break;
                            case MAIN_MENU_VOLUME:
                                /* Enter volume unit selection first */
                                gMenuState = MENU_EDIT_VOLUME_UNIT;
                                tempVolUnit = gPumpJob.volumeUnit;
                                PrintUnitSelect("Select Unit", GetVolumeUnitStr(tempVolUnit));
                                break;
                            case MAIN_MENU_FLOWRATE:
                                /* Enter flow rate unit selection first */
                                gMenuState = MENU_EDIT_FLOWRATE_UNIT;
                                tempFlowUnit = gPumpJob.flowUnit;
                                PrintUnitSelect("Select Unit", GetFlowUnitStr(tempFlowUnit));
                                break;
                            case MAIN_MENU_ADD_SYRINGE:
                                /* Initialize a blank syringe profile for the Add flow */
                                gMenuState = MENU_ADD_SYRINGE_DIA_MODE;
                                isModifyingSyringe = false;
                                tempAddSyringe.maxVolume_ml = 0.0f;
                                tempAddSyringe.innerDiameter_mm = 0.0f;
                                memset(tempAddSyringe.name, ' ', SYRINGE_NAME_LEN - 1);
                                tempAddSyringe.name[SYRINGE_NAME_LEN - 1] = '\0';
                                diaModeSelection = 0; /* Default to MANUAL */
                                HD44780_Clear();
                                HD44780_SetCursor(0, 0);
                                HD44780_PrintStr("Dia Method:");
                                HD44780_SetCursor(0, 1);
                                HD44780_PrintStr(">MANUAL   AUTO");
                                break;
                            case MAIN_MENU_DIRECTION:
                                gMenuState = MENU_EDIT_DIRECTION;
                                tempDirection = currentDirection;
                                HD44780_Clear();
                                HD44780_SetCursor(0, 0);
                                HD44780_PrintStr("Direction:");
                                HD44780_SetCursor(0, 1);
                                if (tempDirection == DIR_FORWARD) {
                                    HD44780_PrintStr(">Forward   BackW");
                                } else {
                                    HD44780_PrintStr(" Forward  >BackW");
                                }
                                break;
                            case MAIN_MENU_PUMP:
                            {
                                /* --- Pre-flight checks before starting pump --- */
                                if (FlashStorage_GetDB()->count == 0) {
                                    /* No syringe in database — show warning */
                                    gMenuState = MENU_WARNING_NO_SYRINGE;
                                    HD44780_Clear();
                                    HD44780_SetCursor(0, 0);
                                    HD44780_PrintStr("! No Syringe !");
                                    HD44780_SetCursor(0, 1);
                                    HD44780_PrintStr("  DB is Empty ");
                                    break;
                                }
                                
                                /* Convert target volume to mL for comparison against syringe max */
                                float vol_ml = (gPumpJob.volumeUnit == VOL_UNIT_UL) ? (gPumpJob.targetVolume / 1000.0f) : gPumpJob.targetVolume;
                                
                                /* Check if requested volume exceeds syringe capacity */
                                if (gPumpJob.syringeMaxVolume_ml > 0.001f && vol_ml > gPumpJob.syringeMaxVolume_ml) {
                                    gMenuState = MENU_WARNING_VOLUME;
                                    HD44780_Clear();
                                    HD44780_SetCursor(0, 0);
                                    HD44780_PrintStr("! HIGH VOLUME !");
                                    HD44780_SetCursor(0, 1);
                                    HD44780_PrintStr("Exceeds Syringe");
                                    break;
                                }
                                
                                /*
                                 * All checks passed — transition to pumping state.
                                 * Update direction pin state.
                                 * TaskMotor polls gSysState and will pick up SYS_PUMPING
                                 * within its 100 ms polling interval.
                                 */
                                if (currentDirection == DIR_FORWARD) {
                                    HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_FORWARD);
                                }
                                else {
                                    HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_BACKWARD);
                                }
                                gSysState = SYS_PUMPING;
                                EnableSelectButtonISR();
                                gMenuState = MENU_PUMPING_VIEW;
                                HD44780_Clear();
                                HD44780_SetCursor(0, 0);
                                HD44780_PrintStr("Starting...");
                                break;
                            }
                            case MAIN_MENU_FREE_MOVE:
                            	gMenuState = MENU_FREE_MOVE_VIEW;
                            	HD44780_Clear();
                            	HD44780_SetCursor(1, 0);
                            	HD44780_PrintStr("*MANUEL MODE*");
                            	HD44780_SetCursor(0, 1);
                            	HD44780_PrintStr("USE L/R BUTTONS");
                            	break;
                            default:
                                break;
                        }
                    }
                    break;

                /* ----- VOLUME UNIT SELECTION ----- */
                case MENU_EDIT_VOLUME_UNIT:
                    if (msg.event == EVENT_ENC_CW || msg.event == EVENT_ENC_CCW) {
                        /* Toggle between mL and µL */
                        tempVolUnit = (tempVolUnit == VOL_UNIT_ML) ? VOL_UNIT_UL : VOL_UNIT_ML;
                        PrintUnitSelect("Select Unit", GetVolumeUnitStr(tempVolUnit));
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Confirm unit selection → proceed to numeric editing */
                        gPumpJob.volumeUnit = tempVolUnit;
                        tempVolume = ClampVolume(gPumpJob.targetVolume, gPumpJob.volumeUnit);
                        gMenuState = MENU_EDIT_VOLUME;
                        digit_cursor = 2; /* Default to ones digit */
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Edit Volume");
                        PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        /* Cancel — return to main menu without saving */
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;
                    
                /* ----- VOLUME NUMERIC EDITING ----- */
                case MENU_EDIT_VOLUME:
                    if (msg.event == EVENT_ENC_CW) {
                        tempVolume += mults[digit_cursor];
                        tempVolume = ClampVolume(tempVolume, gPumpJob.volumeUnit);
                        PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
                    }
                    else if (msg.event == EVENT_ENC_CCW) {
                        tempVolume -= mults[digit_cursor];
                        tempVolume = ClampVolume(tempVolume, gPumpJob.volumeUnit);
                        PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                        if (digit_cursor < 6) digit_cursor++;  /* Move cursor to higher digit */
                        PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                        if (digit_cursor > 0) digit_cursor--;  /* Move cursor to lower digit */
                        PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Confirm — save edited volume and return to menu */
                        gPumpJob.targetVolume = tempVolume;
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        /* Abort — discard changes and return to menu */
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;

                /* ----- FLOW RATE UNIT SELECTION ----- */
                case MENU_EDIT_FLOWRATE_UNIT:
                    if (msg.event == EVENT_ENC_CW) {
                        /* Cycle forward through 4 flow units */
                        tempFlowUnit = (FlowUnit_e)((tempFlowUnit + 1) % 4);
                        PrintUnitSelect("Select Unit", GetFlowUnitStr(tempFlowUnit));
                    }
                    else if (msg.event == EVENT_ENC_CCW) {
                        /* Cycle backward through 4 flow units */
                        tempFlowUnit = (FlowUnit_e)((tempFlowUnit == 0) ? 3 : tempFlowUnit - 1);
                        PrintUnitSelect("Select Unit", GetFlowUnitStr(tempFlowUnit));
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Confirm unit → proceed to numeric editing */
                        gPumpJob.flowUnit = tempFlowUnit;
                        tempFlowRate = ClampFlowRate(gPumpJob.targetFlowRate, gPumpJob.flowUnit);
                        gMenuState = MENU_EDIT_FLOWRATE;
                        digit_cursor = 2;
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Edit Flow Rate");
                        PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;

                /* ----- FLOW RATE NUMERIC EDITING ----- */
                case MENU_EDIT_FLOWRATE:
                    if (msg.event == EVENT_ENC_CW) {
                        tempFlowRate += mults[digit_cursor];
                        tempFlowRate = ClampFlowRate(tempFlowRate, gPumpJob.flowUnit);
                        PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
                    }
                    else if (msg.event == EVENT_ENC_CCW) {
                        tempFlowRate -= mults[digit_cursor];
                        tempFlowRate = ClampFlowRate(tempFlowRate, gPumpJob.flowUnit);
                        PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                        if (digit_cursor < 6) digit_cursor++;
                        PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                        if (digit_cursor > 0) digit_cursor--;
                        PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        gPumpJob.targetFlowRate = tempFlowRate;
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;
                
                /* ----- ADD SYRINGE: DIA MODE SELECTION ----- */
                case MENU_ADD_SYRINGE_DIA_MODE:
                    if (msg.event == EVENT_ENC_CW || msg.event == EVENT_ENC_CCW) {
                        diaModeSelection = !diaModeSelection;
                        HD44780_SetCursor(0, 1);
                        if (diaModeSelection == 0) {
                            HD44780_PrintStr(">MANUAL   AUTO");
                        } else {
                            HD44780_PrintStr(" MANUAL  >AUTO");
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        if (diaModeSelection == 0) { /* MANUAL */
                            /* Advance to diameter entry */
                            gMenuState = MENU_ADD_SYRINGE_DIA;
                            digit_cursor = 2;
                            HD44780_Clear();
                            HD44780_SetCursor(0, 0);
                            HD44780_PrintStr("Add Syringe Dia");
                            PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
                        }
                        else { /* AUTO */
                            gMenuState = MENU_ADD_SYRINGE_AUTO_PROMPT;
                            HD44780_Clear();
                            HD44780_SetCursor(0, 0);
                            HD44780_PrintStr("Fill w/ Water");
                            HD44780_SetCursor(0, 1);
                            HD44780_PrintStr("Press Select ->");
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;

                /* ----- ADD SYRINGE: AUTO PROMPT ----- */
                case MENU_ADD_SYRINGE_AUTO_PROMPT:
                    if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        gSysState = SYS_PUMPING_CALIBRATION;
                        EnableSelectButtonISR();
                        gMenuState = MENU_ADD_SYRINGE_AUTO_PUMPING;
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Pumping 10mm...");
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;

                /* ----- ADD SYRINGE: AUTO PUMPING ----- */
                case MENU_ADD_SYRINGE_AUTO_PUMPING:
                    /* Wait for EVENT_MOTOR_COMPLETE */
                    if (msg.event == EVENT_MOTOR_COMPLETE) {
                        gSysState = SYS_IDLE;
                        DisableSelectButtonISR();
                        gMenuState = MENU_ADD_SYRINGE_AUTO_WEIGHT;
                        tempWeight_g = 1.0f;
                        digit_cursor = 5; /* Ones digit of fraction*/
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Weight (g):");
                        PrintNumberWithCursor(tempWeight_g, "", "g", 5);
                        if (TaskInputHandle != NULL) vTaskResume(TaskInputHandle);
                    }
                    break;

                /* ----- ADD SYRINGE: AUTO WEIGHT ----- */
                case MENU_ADD_SYRINGE_AUTO_WEIGHT:
                    if (msg.event == EVENT_ENC_CW) {
                        tempWeight_g += weight_mults[digit_cursor];
                        if (tempWeight_g > 100.0f) tempWeight_g = 100.0f;
                        PrintNumberWithCursor(tempWeight_g, "", "g", 5);
                    }
                    else if (msg.event == EVENT_ENC_CCW) {
                        tempWeight_g -= weight_mults[digit_cursor];
                        if (tempWeight_g < 0.0f) tempWeight_g = 0.0f;
                        PrintNumberWithCursor(tempWeight_g, "", "g", 5);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                        if (digit_cursor < 7) digit_cursor++;
                        PrintNumberWithCursor(tempWeight_g, "", "g", 5);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                        if (digit_cursor > 0) digit_cursor--;
                        PrintNumberWithCursor(tempWeight_g, "", "g", 5);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Calculate diameter and go to DIA confirmation */
                        /* diameter = 2 * sqrt((weight * 1000) / (distance * PI)) */
                        float area = (tempWeight_g * 1000.0f) / CALIBRATION_PUMP_DISTANCE_MM;
                        float diameter = 2.0f * sqrt(area / MATH_PI);
                        tempAddSyringe.innerDiameter_mm = diameter;
                        
                        gMenuState = MENU_ADD_SYRINGE_DIA;
                        digit_cursor = 2;
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Add Syringe Dia");
                        PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;

                /* ----- ADD SYRINGE: DIAMETER ENTRY ----- */
                case MENU_ADD_SYRINGE_DIA:
                    if (msg.event == EVENT_ENC_CW) {
                        tempAddSyringe.innerDiameter_mm += mults[digit_cursor];
                        if (tempAddSyringe.innerDiameter_mm > 50.0f) tempAddSyringe.innerDiameter_mm = 50.0f;
                        PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
                    }
                    else if (msg.event == EVENT_ENC_CCW) {
                        tempAddSyringe.innerDiameter_mm -= mults[digit_cursor];
                        if (tempAddSyringe.innerDiameter_mm < 0.0f) tempAddSyringe.innerDiameter_mm = 0.0f;
                        PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                        if (digit_cursor < 3) digit_cursor++;
                        PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                        if (digit_cursor > 0) digit_cursor--;
                        PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Advance to volume entry */
                        gMenuState = MENU_ADD_SYRINGE_VOL;
                        digit_cursor = 2; /* Start at ones digit */
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Syringe Volume");
                        PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;

                /* ----- ADD SYRINGE: VOLUME ENTRY ----- */
                case MENU_ADD_SYRINGE_VOL:
                    if (msg.event == EVENT_ENC_CW) {
                        tempAddSyringe.maxVolume_ml += mults[digit_cursor];
                        if (tempAddSyringe.maxVolume_ml > 60.0f) tempAddSyringe.maxVolume_ml = 60.0f;
                        PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
                    }
                    else if (msg.event == EVENT_ENC_CCW) {
                        tempAddSyringe.maxVolume_ml -= mults[digit_cursor];
                        if (tempAddSyringe.maxVolume_ml < 0.0f) tempAddSyringe.maxVolume_ml = 0.0f;
                        PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                        if (digit_cursor < 3) digit_cursor++;
                        /* Skip hundredths position for volume (only xx.x precision) */
                        if (digit_cursor == 1) digit_cursor++;
                        PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                        if (digit_cursor > 1) digit_cursor--;
                        if (digit_cursor == 1) digit_cursor--;
                        PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Advance to name entry */
                        gMenuState = MENU_ADD_SYRINGE_NAME;
                        digit_cursor = 0; /* REPURPOSED: now indexes character position (0–14) */
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Add Name:");
                        HD44780_SetCursor(0, 1);
                        HD44780_PrintStr(tempAddSyringe.name);
                        HD44780_Cursor(); /* Enable hardware underline cursor */
                        HD44780_SetCursor(digit_cursor, 1);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        /* Cancel the entire add flow */
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;



                /* ----- ADD SYRINGE: NAME ENTRY ----- */
                case MENU_ADD_SYRINGE_NAME:
                    if (msg.event == EVENT_ENC_CW || msg.event == EVENT_ENC_CCW) {
                        /*
                         * Scroll through the character set for the current position.
                         * Find the current character's index in syringeNameCharset[],
                         * then advance or retreat by one.
                         */
                        int charsetIdx = 0;
                        for(int i=0; i<37; i++){
                            if(tempAddSyringe.name[digit_cursor] == syringeNameCharset[i][0]) {
                                charsetIdx = i; break;
                            }
                        }
                        if (msg.event == EVENT_ENC_CW) {
                            charsetIdx = (charsetIdx + 1) % 37;
                        }
                        else {
                            charsetIdx = (charsetIdx == 0) ? 36 : charsetIdx - 1;
                        }
                        tempAddSyringe.name[digit_cursor] = syringeNameCharset[charsetIdx][0];
                        
                        /* Briefly hide cursor to prevent visual jitter during character update */
                        HD44780_NoCursor();
                        HD44780_SetCursor(digit_cursor, 1);
                        HD44780_PrintStr(syringeNameCharset[charsetIdx]);
                        HD44780_Cursor();
                        HD44780_SetCursor(digit_cursor, 1);
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                        if (digit_cursor > 0) {
                            digit_cursor--;
                            HD44780_SetCursor(digit_cursor, 1);
                        }
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                        if (digit_cursor < SYRINGE_NAME_LEN - 2) { 
                            digit_cursor++;
                            HD44780_SetCursor(digit_cursor, 1);
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Confirm: save the new syringe profile to flash */
                        HD44780_NoCursor();
                        if (isModifyingSyringe) {
                            FlashStorage_ModifySyringe(list_cursor, &tempAddSyringe);
                        }
                        else {
                            FlashStorage_AddSyringe(&tempAddSyringe);
                        }
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        /* Cancel: discard the new syringe and return to menu */
                        HD44780_NoCursor();
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;

                /* ----- SYRINGE SELECTION LIST ----- */
                case MENU_SELECT_SYRINGE_LIST:
                    if (msg.event == EVENT_ENC_CW) {
                        /* Scroll forward with wrap-around */
                        if (list_cursor < FlashStorage_GetDB()->count - 1) list_cursor++;
                        else list_cursor = 0;
                        
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(FlashStorage_GetDB()->syringes[list_cursor].name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm,
                                FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                    }
                    else if (msg.event == EVENT_ENC_CCW) {
                        /* Scroll backward with wrap-around */
                        if (list_cursor > 0) list_cursor--;
                        else list_cursor = FlashStorage_GetDB()->count - 1;
                        
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(FlashStorage_GetDB()->syringes[list_cursor].name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm,
                                FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        /* Confirm: load selected syringe as active profile */
                        gPumpJob.syringeDiameter_mm = FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm;
                        gPumpJob.syringeMaxVolume_ml = FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml;
                        strcpy(gPumpJob.syringeName, FlashStorage_GetDB()->syringes[list_cursor].name);
                        
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_SYRINGE_ACTION_SELECT;
                        actionSelection = 0; /* Default to MODIFY */
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Select Action:");
                        HD44780_SetCursor(0, 1);
                        HD44780_PrintStr(">MODIFY   DELETE");
                    }
                    else if(msg.event == EVENT_BTN_CURSOR_LEFT_PRESS){
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;
                
                /* ----- SYRINGE ACTION SELECT ----- */
                case MENU_SYRINGE_ACTION_SELECT:
                    if (msg.event == EVENT_ENC_CW || msg.event == EVENT_ENC_CCW) {
                        actionSelection = !actionSelection;
                        HD44780_SetCursor(0, 1);
                        if (actionSelection == 0) {
                            HD44780_PrintStr(">MODIFY   DELETE");
                        }
                        else {
                            HD44780_PrintStr(" MODIFY  >DELETE");
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        if (actionSelection == 0) { /* MODIFY */
                            isModifyingSyringe = true;
                            /* Pre-fill tempAddSyringe with the current profile */
                            tempAddSyringe = FlashStorage_GetDB()->syringes[list_cursor];
                            
                            /* Jump to the dia mode select state */
                            gMenuState = MENU_ADD_SYRINGE_DIA_MODE;
                            diaModeSelection = 0;
                            HD44780_Clear();
                            HD44780_SetCursor(0, 0);
                            HD44780_PrintStr("Diameter Method:");
                            HD44780_SetCursor(0, 1);
                            HD44780_PrintStr(">MANUAL   AUTO");
                        }
                        else { /* DELETE */
                            gMenuState = MENU_DELETE_SYRINGE_CONFIRM;
                            deleteConfirmSelection = 1; /* NO by default */
                            HD44780_Clear();
                            HD44780_SetCursor(0, 0);
                            HD44780_PrintStr("Delete Syringe?");
                            HD44780_SetCursor(0, 1);
                            HD44780_PrintStr("  YES    > NO");
                        }
                    }
                    else if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                        gMenuState = MENU_SELECT_SYRINGE_LIST;
                        /* Redraw syringe list */
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(FlashStorage_GetDB()->syringes[list_cursor].name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.1fmm, %.1fmL",
                                FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm,
                                FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                    }
                    break;

                /* ----- DELETE SYRINGE CONFIRM ----- */
                case MENU_DELETE_SYRINGE_CONFIRM:
                    if (msg.event == EVENT_ENC_CW || msg.event == EVENT_ENC_CCW) {
                        deleteConfirmSelection = !deleteConfirmSelection;
                        HD44780_SetCursor(0, 1);
                        if (deleteConfirmSelection == 0) {
                            HD44780_PrintStr("> YES      NO");
                        }
                        else {
                            HD44780_PrintStr("  YES    > NO");
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        if (deleteConfirmSelection == 0) { /* YES */
                            FlashStorage_DeleteSyringe(list_cursor);
                            if (FlashStorage_GetDB()->count == 0) {
                                gPumpJob.syringeDiameter_mm = 0.0f;
                                gPumpJob.syringeMaxVolume_ml = 0.0f;
                                strcpy(gPumpJob.syringeName, "NO SYRINGE");
                                gMenuState = MENU_OVERVIEW;
                                DrawMainMenu();
                            }
                            else {
                                if (list_cursor >= FlashStorage_GetDB()->count) {
                                    list_cursor = FlashStorage_GetDB()->count - 1;
                                }
                                gMenuState = MENU_SELECT_SYRINGE_LIST;
                                /* Redraw syringe list */
                                HD44780_Clear();
                                HD44780_PrintStr("> ");
                                HD44780_PrintStr(FlashStorage_GetDB()->syringes[list_cursor].name);
                                HD44780_SetCursor(0, 1);
                                char sbuf[17];
                                snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                        FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm,
                                        FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml);
                                HD44780_PrintStr(sbuf);
                            }
                        }
                        else { /* NO */
                            gMenuState = MENU_SELECT_SYRINGE_LIST;
                            /* Redraw syringe list */
                            HD44780_Clear();
                            HD44780_PrintStr("> ");
                            HD44780_PrintStr(FlashStorage_GetDB()->syringes[list_cursor].name);
                            HD44780_SetCursor(0, 1);
                            char sbuf[17];
                            snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                    FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm,
                                    FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml);
                            HD44780_PrintStr(sbuf);
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_SELECT_SYRINGE_LIST;
                        /* Redraw syringe list */
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(FlashStorage_GetDB()->syringes[list_cursor].name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.1fmm, %.1fmL",
                                FlashStorage_GetDB()->syringes[list_cursor].innerDiameter_mm,
                                FlashStorage_GetDB()->syringes[list_cursor].maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                    }
                    break;
                
                /* ----- WARNING DIALOGS (dismiss on any button) ----- */
                case MENU_WARNING_VOLUME:
                case MENU_WARNING_NO_SYRINGE:
                    if (msg.event == EVENT_BTN_SELECT_PRESS || msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;
                
                /* ----- PUMPING VIEW (async refresh handled below) ----- */
                case MENU_PUMPING_VIEW:
                    /* Input events are consumed but ignored — screen is timer-driven */
                    break;
                
                /* ----- PAUSE CONFIRMATION DIALOG ----- */
                case MENU_PUMP_PAUSE_CONFIRM:
                    if (msg.event == EVENT_ENC_CW || msg.event == EVENT_ENC_CCW) {
                        /* Toggle between YES and NO */
                        cancelSelection = !cancelSelection;
                        HD44780_SetCursor(0, 1);
                        if (cancelSelection == 0) HD44780_PrintStr("> YES      NO");
                        else HD44780_PrintStr("  YES    > NO");
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        if (cancelSelection == 0) {
                            /* --- User confirmed: CANCEL the pump --- */
                            /* Ensure motor is stopped if still decelerating */
                            HAL_TIM_Base_Stop_IT(&htim2);
                            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

                            gStepperControl.totalStepsRemaining = 0;
                            gLivePumpedVolume_ml = 0;
                            gStepperControl.MotorState = MOTOR_STATE_IDLE;
                            
                            gSysState = SYS_IDLE;
                            DisableSelectButtonISR();
                            gMenuState = MENU_PUMP_CANCELLED;
                            stepper_set_spreadcycle(false);
                            HD44780_Clear();
                            HD44780_SetCursor(0, 0);
                            HD44780_PrintStr("Pump Cancelled!");
                            HD44780_SetCursor(0, 1);
                            HD44780_PrintStr("Press any button");
                            if (TaskInputHandle != NULL) vTaskResume(TaskInputHandle);
                        }
                        else {
                            /* --- User selected NO: RESUME the pump --- */
                            if (gStepperControl.totalStepsRemaining == 0) {
                                /* The pump actually finished naturally right as we paused it! */
                                gSysState = SYS_IDLE;
                                DisableSelectButtonISR();
                                gMenuState = MENU_PUMP_COMPLETED;
                                stepper_set_spreadcycle(false);
                                HD44780_Clear();
                                HD44780_SetCursor(0, 0);
                                HD44780_PrintStr("Pump Completed!");
                                HD44780_SetCursor(0, 1);
                                HD44780_PrintStr("Press any button");
                                if (TaskInputHandle != NULL) vTaskResume(TaskInputHandle);
                            } else {
                                gSysState = SYS_PUMPING;
                                EnableSelectButtonISR();
                                gMenuState = MENU_PUMPING_VIEW;
                                HD44780_Clear();
                                
                                /* Re-clamp acceleration steps if we paused very close to the end.
                                 * This prevents the deceleration logic from getting out of sync. */
                                if (gStepperControl.totalStepsRemaining < (2 * gStepperControl.accelSteps)) {
                                    gStepperControl.accelSteps = gStepperControl.totalStepsRemaining / 2;
                                }
                                
                                /* Restart the timer with acceleration from current state */
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
                    }
                    break;

                /* ----- POST-PUMP RESULT SCREENS (any button returns to menu) ----- */
                case MENU_PUMP_COMPLETED:
                case MENU_PUMP_CANCELLED:
                    if (msg.event != EVENT_NONE) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;
                case MENU_EDIT_DIRECTION:
                    if (msg.event == EVENT_ENC_CW || msg.event == EVENT_ENC_CCW) {
                        tempDirection = (tempDirection == DIR_FORWARD) ? DIR_BACKWARD : DIR_FORWARD;
                        HD44780_SetCursor(0, 1);
                        if (tempDirection == DIR_FORWARD) {
                            HD44780_PrintStr(">Forward   BackW");
                        }
                        else {
                            HD44780_PrintStr(" Forward  >BackW");
                        }
                    }
                    else if (msg.event == EVENT_BTN_SELECT_PRESS) {
                        currentDirection = tempDirection;
                        if (currentDirection == DIR_FORWARD) {
                            HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_FORWARD);
                        }
                        else {
                            HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_BACKWARD);
                        }
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    else if (msg.event == EVENT_BTN_SELECT_LONG_PRESS) {
                        gMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    }
                    break;
                case MENU_FREE_MOVE_VIEW:
                	if (msg.event == EVENT_BTN_CURSOR_LEFT_PRESS){
                        /* Wait for any ongoing deceleration to finish */
                        while (gStepperControl.MotorState != MOTOR_STATE_IDLE) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
						HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_BACKWARD);
						gStepperControl.MotorMode = MOTOR_MODE_MANUAL;
						stepper_start((StepperControl_t*) &gStepperControl, MANUAL_CRUISE_FREQ, MANUAL_TOTAL_STEPS, MANUAL_ACCEL_STEPS);
						while(msg.event == EVENT_BTN_CURSOR_LEFT_PRESS){
							xQueueReceive(InputQueue, &msg, pdMS_TO_TICKS(MANUAL_MODE_UPDATE_DELAY));
							if (msg.event == EVENT_BTN_CURSOR_LEFT_RELEASE){
                                if (gStepperControl.MotorState == MOTOR_STATE_ACCEL || gStepperControl.MotorState == MOTOR_STATE_CRUISE) {
								    gStepperControl.MotorState = MOTOR_STATE_DECEL;
                                }
							}
						}
					}
					else if (msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS){
                        /* Wait for any ongoing deceleration to finish */
                        while (gStepperControl.MotorState != MOTOR_STATE_IDLE) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
						HAL_GPIO_WritePin(stepperDir_GPIO_Port, stepperDir_Pin, STEPPER_DIR_FORWARD);
						gStepperControl.MotorMode = MOTOR_MODE_MANUAL;
						stepper_start((StepperControl_t*) &gStepperControl, MANUAL_CRUISE_FREQ, MANUAL_TOTAL_STEPS, MANUAL_ACCEL_STEPS);
						while(msg.event == EVENT_BTN_CURSOR_RIGHT_PRESS){
							xQueueReceive(InputQueue, &msg, pdMS_TO_TICKS(MANUAL_MODE_UPDATE_DELAY));
							if (msg.event == EVENT_BTN_CURSOR_RIGHT_RELEASE){
                                if (gStepperControl.MotorState == MOTOR_STATE_ACCEL || gStepperControl.MotorState == MOTOR_STATE_CRUISE) {
								    gStepperControl.MotorState = MOTOR_STATE_DECEL;
                                }
							}
						}
                	}
					else if (msg.event == EVENT_BTN_SELECT_PRESS || msg.event == EVENT_BTN_SELECT_LONG_PRESS){
                		gMenuState = MENU_OVERVIEW;
						DrawMainMenu();
					}
					break;
                default:
                    /* Safety fallback: unknown state → reset to main menu */
                    gMenuState = MENU_OVERVIEW;
                    DrawMainMenu();
                    break;
            }
        }
        
        /* =================================================================
         * ASYNC PUMPING SCREEN REFRESH
         *
         * When the pump is running, this block updates the LCD every
         * SCREEN_REMAINING_VOL_UPDATE ms with:
         * - Line 1: Alternates between flow rate and remaining time (HH:MM:SS)
         * - Line 2: Pumped volume / total volume in mL
         * ================================================================= */
        if (gMenuState == MENU_PUMPING_VIEW && gSysState == SYS_PUMPING) {
            TickType_t now = xTaskGetTickCount();
            if ((now - lastDrawTick) >= pdMS_TO_TICKS(SCREEN_REMAINING_VOL_UPDATE)) {
                lastDrawTick = now;
                anim_idx++;
                
                /* Convert target volume to mL for uniform calculation */
                float target_ml = (gPumpJob.volumeUnit == VOL_UNIT_UL) ? (gPumpJob.targetVolume / 1000.0f) : gPumpJob.targetVolume;
                float remaining_ml = target_ml - gLivePumpedVolume_ml;
                if (remaining_ml < 0.0f) remaining_ml = 0.0f;
                
                /* Convert flow rate to mL/min for time-remaining calculation */
                float flow_ml_min = 0;
                switch(gPumpJob.flowUnit) {
                    case FLOW_UNIT_ML_MIN: flow_ml_min = gPumpJob.targetFlowRate; break;
                    case FLOW_UNIT_UL_MIN: flow_ml_min = gPumpJob.targetFlowRate / 1000.0f; break;
                    case FLOW_UNIT_ML_HR:  flow_ml_min = gPumpJob.targetFlowRate / 60.0f; break;
                    case FLOW_UNIT_UL_HR:  flow_ml_min = (gPumpJob.targetFlowRate / 1000.0f) / 60.0f; break;
                }
                
                /* Calculate estimated remaining time in seconds */
                uint32_t remaining_sec = 0;
                if (flow_ml_min > 0.001f) {
                    remaining_sec = (uint32_t)((remaining_ml / flow_ml_min) * 60.0f);
                }
                
                /* Decompose into hours, minutes, seconds */
                uint32_t h = remaining_sec / 3600;
                uint32_t m = (remaining_sec % 3600) / 60;
                uint32_t s = remaining_sec % 60;
                
                /*
                 * Line 1 animation: alternate between two views every 2 frames.
                 * Frames 0–1: Show current flow rate
                 * Frames 2–3: Show remaining time (HH:MM:SS)
                 */
                char row1[17];
                if ((anim_idx % 4) < 2) {
                    snprintf(row1, sizeof(row1), "%.1f%s         ", gPumpJob.targetFlowRate, GetFlowUnitStr(gPumpJob.flowUnit));
                }
                else {
                    snprintf(row1, sizeof(row1), "        %02lu:%02lu:%02lu", h, m, s);
                }
                
                /* Line 2: Current pumped volume / target volume */
                char row2[17];
                snprintf(row2, sizeof(row2), "%.2f/%.2fmL", gLivePumpedVolume_ml, target_ml);
                
                HD44780_NoCursor();
                HD44780_SetCursor(0,0);
                HD44780_PrintStr(row1);
                HD44780_SetCursor(0,1);
                HD44780_PrintStr(row2);
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
 *          After launching the stepper, the task enters a polling loop that
 *          updates gLivePumpedVolume_ml from the stepper's remaining step count,
 *          enabling the UI's live progress display.
 *
 * @note   Priority 4 (highest). Stack: 256 words. Polls gSysState every 100 ms
 *         when idle, and updates volume every SCREEN_REMAINING_VOL_UPDATE ms
 *         while pumping.
 */
void RtosMenu_TaskMotor(void *argument) {
    uint32_t ulNotifiedValue;
    for (;;) {
        /* Wait for task notification or timeout (100 ms polling interval) */
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, pdMS_TO_TICKS(100));
        
        if (gSysState == SYS_PUMPING || gSysState == SYS_PUMPING_CALIBRATION) {
            if (gSysState == SYS_PUMPING_CALIBRATION) {
                /* ---- Calibration Pumping: Fixed distance ---- */
                float stepsPerMm = ((float)gPumpHw.motorStepsPerRev / gPumpHw.screwLead_mm_per_rev) * (float)DEFAULT_MICROSTEPPING;
                uint32_t totalSteps = (uint32_t)(CALIBRATION_PUMP_DISTANCE_MM * stepsPerMm);
                float cruiseHz = 2.0f * stepsPerMm;
                
                /* Kinematic acceleration step calculation: S = V^2 / 2A */
                uint32_t accel = (uint32_t)((cruiseHz * cruiseHz) / (2.0f * TARGET_ACCEL_RATE));
                if (accel > MAX_ACCEL_STEPS) accel = MAX_ACCEL_STEPS;
                if (accel < MIN_ACCEL_STEPS) accel = MIN_ACCEL_STEPS;
                if (totalSteps < 2 * accel) accel = totalSteps / 2;
                
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
                /* ---- Step 1: Convert target volume to mm³ ---- */
            float vol_mm3 = (gPumpJob.volumeUnit == VOL_UNIT_ML) ? (gPumpJob.targetVolume * VOL_ML_TO_MM3) : gPumpJob.targetVolume;
            
            /* ---- Step 2: Convert flow rate to mm³/min, then to mm³/sec ---- */
            float flow_mm3_min = 0;
            switch(gPumpJob.flowUnit) {
                case FLOW_UNIT_ML_MIN: flow_mm3_min = gPumpJob.targetFlowRate * VOL_ML_TO_MM3; break;
                case FLOW_UNIT_UL_MIN: flow_mm3_min = gPumpJob.targetFlowRate; break;         /* µL ≡ mm³ */
                case FLOW_UNIT_ML_HR:  flow_mm3_min = (gPumpJob.targetFlowRate * VOL_ML_TO_MM3) / TIME_MIN_TO_SEC; break;
                case FLOW_UNIT_UL_HR:  flow_mm3_min = gPumpJob.targetFlowRate / TIME_MIN_TO_SEC; break;
            }
            float flow_mm3_sec = flow_mm3_min / TIME_MIN_TO_SEC;
            
            /* ---- Step 3: Syringe cross-sectional area (mm²) ---- */
            float r = gPumpJob.syringeDiameter_mm / 2.0f;
            float area = MATH_PI * r * r;  /* A = π r² */
            
            /* ---- Step 4: Linear travel and speed ---- */
            float linearTravel_mm = vol_mm3 / area;         /* Distance plunger must travel */
            float linearSpeed_mm_sec = flow_mm3_sec / area;  /* Speed of plunger travel */
            
            /* ---- Step 5: Convert linear motion to stepper parameters ---- */
            float baseStepsPerMm = (float)gPumpHw.motorStepsPerRev / gPumpHw.screwLead_mm_per_rev;
            
            /* Configure microstepping (currently fixed at DEFAULT_MICROSTEPPING) */
            uint8_t optimalMicrostepping = DEFAULT_MICROSTEPPING;
            gPumpHw.microstepping = optimalMicrostepping;
            configMicrostepping(optimalMicrostepping);
            
            /* Final step calculations with microstepping applied */
            float stepsPerMm = baseStepsPerMm * (float)optimalMicrostepping;
            uint32_t totalSteps = (uint32_t)(linearTravel_mm * stepsPerMm);
            float cruiseHz = linearSpeed_mm_sec * stepsPerMm;
            
            /* Kinematic acceleration step calculation: S = V^2 / 2A */
            uint32_t accel = (uint32_t)((cruiseHz * cruiseHz) / (2.0f * TARGET_ACCEL_RATE));
            if (accel > MAX_ACCEL_STEPS) accel = MAX_ACCEL_STEPS;
            if (accel < MIN_ACCEL_STEPS) accel = MIN_ACCEL_STEPS;
            if (totalSteps < 2 * accel) accel = totalSteps / 2;

            /* Reset live progress tracker and launch the stepper */
            gLivePumpedVolume_ml = 0.0f;
            stepper_set_spreadcycle(true);
            stepper_start((StepperControl_t*)&gStepperControl, cruiseHz, totalSteps, accel);
            
            while (gSysState == SYS_PUMPING || gSysState == SYS_PAUSED) {
                if (gStepperControl.MotorState == MOTOR_STATE_IDLE && gSysState != SYS_PAUSED) {
                    stepper_set_spreadcycle(false);
                    mainMenuCursor = 0;
                    InputMessage_t msg = { .event = EVENT_MOTOR_COMPLETE };
                    xQueueSend(InputQueue, &msg, 0);
                    break;
                }
                
                /* Reverse-calculate pumped volume from completed steps */
                uint32_t stepsDone = totalSteps - gStepperControl.totalStepsRemaining;
                float pumped_mm = (float)stepsDone / stepsPerMm;
                gLivePumpedVolume_ml = (pumped_mm * area) / VOL_ML_TO_MM3;

                vTaskDelay(pdMS_TO_TICKS(SCREEN_REMAINING_VOL_UPDATE));
            }
            } /* End of else (normal pumping) */
        }
    }
}

void RtosMenu_TaskEncoder(void* argument){
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
                /* Start non-blocking DMA read */
                HAL_StatusTypeDef status = AS5600_Read_Angle();
                
                if (status == HAL_OK) {
                        AS5600_Process_I2C_Buffer();
                    }
                else if(status == HAL_ERROR) {

                	AS5600_I2C_Error_Handler();
                }

                /* Give back the mutex as soon as DMA is initiated/finished */
                xSemaphoreGive(I2C2_Mutex);
            }
        }

        /* Poll at an absolute 100 Hz (10 ms period), immune to execution time drift */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vApplicationIdleHook(void) {

}
