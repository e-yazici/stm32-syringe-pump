/**
 * @file    rtos_tasks.h
 * @brief   Public interface and data structures for the Syringe Pump RTOS tasks.
 * @author  eyazici
 * @date    2026
 *
 * @details This file defines the system state components, input events, and data
 *          structures used by the Input, UI, Motor, Encoder, and UART FreeRTOS tasks.
 *          It serves as the single source of truth for shared system structures
 *          to avoid coupling between modules.
 *
 *          The system uses a 5-task FreeRTOS architecture:
 *          - **InputTask** (50 Hz): Polls encoder + buttons, emits InputMessage_t to queue
 *          - **UITask**: Blocks on InputQueue, delegates LCD menu pages to ui_menu
 *          - **MotorTask**: Computes kinematics and supervises stepper execution
 *          - **EncoderTask** (100 Hz): Reads AS5600 magnetic encoder over I2C2
 *          - **UartTask**: Serializes USART1 TMC2208 DMA and USART2 VCP DMA transactions
 */

#ifndef LIBS_INC_RTOS_TASKS_H_
#define LIBS_INC_RTOS_TASKS_H_

#include <stdint.h>
#include <stdbool.h>
#include "flash_storage.h"


/**
 * @brief Physical capabilities of the syringe pump hardware
 *        These values are largely set once and used mathematically.
 */
typedef struct {
    float screwLead_mm_per_rev;    ///< Distance output shaft travels per revolution in mm (e.g. 2.0 for T8 lead screw)
    uint16_t motorStepsPerRev;     ///< Full steps per revolution of the motor (e.g. 200 for 1.8° stepper)
    uint16_t microstepping;        ///< Active driver microstep setting (e.g. 128)
    float maxFlowRate_ul_min;      ///< Theoretical maximum flow rate in µL/min
    float maxFlowRate_ml_min;      ///< Theoretical maximum flow rate in mL/min
} PumpHardwareSpecs_t;

/** @brief Volume unit selection for user-facing display and calculations */
typedef enum {
    VOL_UNIT_UL = 0,
    VOL_UNIT_ML
} VolumeUnit_e;

/** @brief Flow rate unit selection for user-facing display and motor calculations */
typedef enum {
    FLOW_UNIT_UL_MIN = 0,
    FLOW_UNIT_ML_MIN,
    FLOW_UNIT_UL_HR,
    FLOW_UNIT_ML_HR
} FlowUnit_e;

/**
 * @brief User-defined target parameters for pumping
 *        These are configured via the LCD menu.
 */
typedef struct {
    float targetVolume;            ///< Target dispense volume (unit determined by volumeUnit)
    VolumeUnit_e volumeUnit;       ///< Unit of targetVolume (mL or µL)
    float targetFlowRate;          ///< Target flow rate (unit determined by flowUnit)
    FlowUnit_e flowUnit;           ///< Unit of targetFlowRate (mL/min, µL/min, mL/hr, µL/hr)
    float syringeDiameter_mm;      ///< Inner barrel diameter of the loaded syringe in mm
    float syringeMaxVolume_ml;     ///< Maximum capacity of the loaded syringe in mL
    char syringeName[16];          ///< Name of the selected syringe profile (null-terminated)
} PumpJobParams_t;

/** @brief Stepper motor rotation direction */
typedef enum {
    DIR_FORWARD = 0,
    DIR_BACKWARD = 1
} MotorDirection_e;

/**
 * @brief Main system machine states
 */
typedef enum {
    SYS_IDLE,                     ///< User is configuring menu, motor idle
    SYS_PUMPING,                  ///< Motor is currently running
    SYS_PUMPING_CALIBRATION,      ///< Motor is pumping fixed distance for calibration (diameter finder)
    SYS_PAUSED                    ///< Motor halted by user, can be resumed

} SystemState_e;

/**
 * @brief Menu sub-states for the UI
 */
typedef enum {
    MENU_OVERVIEW = 0,                ///< Display current Target and Flow Rate
    MENU_EDIT_VOLUME,                 ///< Adjusting numerical Volume
    MENU_EDIT_VOLUME_UNIT,            ///< Adjusting Volume Unit (mL/uL)
    MENU_EDIT_FLOWRATE,               ///< Adjusting numerical Flow Rate
    MENU_EDIT_FLOWRATE_UNIT,          ///< Adjusting Flow Rate Unit (mL/uL)
    MENU_SELECT_SYRINGE_LIST,         ///< Pick pre-saved syringe
    MENU_SYRINGE_ACTION_SELECT,       ///< Select MODIFY or DELETE
    MENU_DELETE_SYRINGE_CONFIRM,      ///< Confirm deletion of syringe
    MENU_ADD_SYRINGE_DIA_MODE,        ///< Select Manual or Auto diameter
    MENU_ADD_SYRINGE_AUTO_PROMPT,     ///< Prompt user to fill syringe
    MENU_ADD_SYRINGE_AUTO_PUMPING,    ///< Motor running calibration distance
    MENU_ADD_SYRINGE_AUTO_WEIGHT,     ///< Enter weight of pumped water
    MENU_ADD_SYRINGE_AUTO_Z_FACTOR,   ///< Enter ISO 8655 Z-factor for gravimetric calibration
    MENU_ADD_SYRINGE_DIA,             ///< Add new syringe - diameter setup
    MENU_ADD_SYRINGE_VOL,             ///< Add new syringe - Volume
    MENU_ADD_SYRINGE_NAME,            ///< Add new syringe - name setup
    MENU_WARNING_VOLUME,              ///< Warn user about exceeded volume
    MENU_WARNING_NO_SYRINGE,          ///< Warn user to add a syringe first
    MENU_PUMPING_VIEW,                ///< Display pumping progress info
    MENU_PUMP_PAUSE_CONFIRM,          ///< Cancel pump confirmation YES/NO
    MENU_PUMP_COMPLETED,              ///< Static info screen on natural finish
    MENU_PUMP_CANCELLED,              ///< Static info screen on manual abort
    MENU_FREE_MOVE_VIEW,              ///< Manual motor control screen
    MENU_EDIT_DIRECTION,              ///< Select motor direction
    MENU_STALL_ALERT                  ///< Motor stall alert dialog (Continue / Abort)
} MenuState_e;

/**
 * @brief Items for the scrollable main menu
 */
typedef enum {
    MAIN_MENU_SELECT_SYRINGE = 0,
    MAIN_MENU_VOLUME,
    MAIN_MENU_FLOWRATE,
    MAIN_MENU_ADD_SYRINGE,
    MAIN_MENU_DIRECTION,
    MAIN_MENU_PUMP,
    MAIN_MENU_FREE_MOVE,
    MAIN_MENU_MAX
} MainMenuItem_e;

/**
 * @brief Input events for the InputQueue
 */
typedef enum {
    EVENT_NONE = 0,
    EVENT_ENC_CW,
    EVENT_ENC_CCW,
    EVENT_BTN_CURSOR_LEFT_PRESS,
    EVENT_BTN_CURSOR_LEFT_RELEASE,
    EVENT_BTN_SELECT_PRESS,
    EVENT_BTN_SELECT_LONG_PRESS,
    EVENT_BTN_CURSOR_RIGHT_PRESS,
    EVENT_BTN_CURSOR_RIGHT_RELEASE,
    EVENT_LIMIT_SW_TRIGGERED,          ///< @todo Implement this state in the state machine
    EVENT_MOTOR_COMPLETE,
    EVENT_STALL_DETECTED              ///< Closed-loop supervisor confirmed motor stall
} InputEvent_e;

/**
 * @brief Input structure sent over queue
 */
typedef struct {
    InputEvent_e event;
    uint32_t timestamp;           ///< FreeRTOS tick count for  debounce filtering
} InputMessage_t;


extern PumpHardwareSpecs_t gPumpHw;
extern PumpJobParams_t gPumpJob;
extern volatile SystemState_e gSysState;
extern volatile float gLivePumpedVolume_ml;

/**
 * @brief Initialize local parameters (Must be called before OS starts)
 */
void RtosTasks_Init(void);

/**
 * @brief Lightweight polling task (50Hz) to read hardware encoder & switches.
 *        Pushes `InputMessage_t` to the InputQueue.
 */
void RtosTasks_TaskInput(void *argument);

/**
 * @brief Heavyweight UI state machine task.
 *        Sleeps perpetually until an Input Event arrives via InputQueue.
 */
void RtosTasks_TaskUI(void *argument);

/**
 * @brief Motor Supervisor Task.
 *        Listens for start/stop commands and computes acceleration physics.
 */
void RtosTasks_TaskMotor(void *argument);

/**
 * @brief AS5600 Encoder Task (100 Hz).
 *        Reads the AS5600 angle over I2C2, filters angle and velocity, and updates the odometry snapshot.
 */
void RtosTasks_TaskEncoder(void* argument);

/**
 * @brief UART Gatekeeper Task (USART1 TMC2208 DMA & USART2 VCP DMA).
 */
void RtosTasks_TaskUartGatekeeper(void *argument);

/**
 * @brief Enqueue an asynchronous diagnostic buffer for transmission over USART2 (VCP) via DMA.
 * @param data Pointer to bytes to transmit.
 * @param len  Number of bytes to transmit (clamped to internal VCP payload size).
 * @return true if successfully queued to the UART Gatekeeper task.
 */
bool RtosTasks_UartSendVcpAsync(const uint8_t *data, uint16_t len);

#endif /* LIBS_INC_RTOS_TASKS_H_ */
