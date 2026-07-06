/**
 * @file    stepper.h
 * @brief   Stepper motor driver with S-curve acceleration profile support.
 * @author  eyazici
 * @date    2026
 *
 * @details Provides an S-curve motion profile engine for driving a
 *          stepper motor via a hardware timer (TIM2) in PWM mode. The driver
 *          pre-computes an acceleration lookup table using a cosine easing
 *          function and feeds it into the timer ISR for real-time ARR updates.
 *
 *          The state machine runs entirely inside the TIM2 PeriodElapsed ISR
 *          (implemented in main.c), while this module handles initialization,
 *          profile generation, and hardware setup.
 *
 * @note    Hardware assumptions:
 *          - Timer clock: 72 MHz (STM32F303RE @ PLL max)
 *          - TIM2 Channel 1 drives the stepper STEP pin
 *          - Prescaler is fixed at compile time
 */

#ifndef LIBS_INC_STEPPER_H_
#define LIBS_INC_STEPPER_H_

#include <stdint.h>
#include "math.h"
#include "stm32f3xx_hal.h"

/** @defgroup Stepper_Config Stepper Configuration Constants
 *  @brief   Compile-time constants for the stepper acceleration engine.
 *  @{
 */
#define TIMER_CLOCK 72000000       ///< Timer input clock frequency in Hz (APB1 = 72 MHz for STM32F303RE)
#define PRESCALER 71               ///< TIM2 prescaler value; effective tick = TIMER_CLOCK/(PRESCALER+1) = 1 MHz
#define START_ARR ((TIMER_CLOCK / ((PRESCALER + 1) * START_FREQ_HZ)) - 1) ///< ARR value corresponding to START_FREQ_HZ
#define START_FREQ_HZ 3000          ///< Initial step frequency (Hz) at the start of acceleration ramp
#define MAX_ACCEL_STEPS 15000       ///< Maximum number of steps in the acceleration/deceleration ramp
#define MIN_ACCEL_STEPS 100        ///< Minimum number of acceleration steps for low speeds
#define TARGET_ACCEL_RATE 1000.0f ///< Target constant acceleration rate in steps/sec^2
#define CALIBRATION_PUMP_DISTANCE_MM  10.0f   ///< Distance to pump for auto diameter calibration
#define MANUAL_CRUISE_FREQ 50000    ///< Stepper manual control constant speed, in Hz
#define MANUAL_TOTAL_STEPS 4000000  ///< Dummy total steps, manual drive controlled via buttons
#define MANUAL_ACCEL_STEPS 3000     ///< Constant acceleration steps for manual drive
#define STEPPER_DIR_FORWARD 1      ///< Used in HAL_GPIO_WritePin(), modify according to stepper driver direction pin
#define STEPPER_DIR_BACKWARD 0     ///< Used in HAL_GPIO_WritePin(), modify according to stepper driver direction pin
/** @} */

/**
 * @brief Motor state machine states.
 *
 * Tracks the current phase of motion profile. Transitions are
 * driven by the TIM2 PeriodElapsed ISR in main.c.
 */
typedef enum{
	MOTOR_STATE_IDLE,    	///< Motor is stopped; no active move
	MOTOR_STATE_ACCEL,   	///< Accelerating: stepping through lookup table forward
	MOTOR_STATE_CRUISE,  	///< Cruising: ARR held constant at target frequency
	MOTOR_STATE_DECEL,   	///< Decelerating: stepping through lookup table in reverse
	MOTOR_STATE_PAUSED      ///< Motor is paused but movement not complete
}MotorState_e;

/**
 * @brief Motor state machine modes.
 *
 * Tracks the current button states. Transitions are
 * driven by the TIM2 PeriodElapsed ISR in main.c and TaskUI.
 */
typedef enum{
	MOTOR_MODE_NONE,        ///< Motor is stopped
	MOTOR_MODE_PUMP,        ///< Programmed pump mode
	MOTOR_MODE_MANUAL       ///< Free motor control using buttons
}MotorMode_e;

/**
 * @brief Runtime control structure for a single stepper motor axis.
 *
 * @details Contains all state needed by both the setup code (stepper_start) and
 *          the ISR (HAL_TIM_PeriodElapsedCallback) to execute a complete motion
 *          profile. This struct should be declared volatile when shared between
 *          ISR and task contexts.
 */
typedef struct{
	uint32_t ARRCurrent;              ///< Current Auto-Reload Register value being applied to TIM2
	uint32_t totalStepsRemaining;     ///< Steps left until move completes (decremented in ISR)
	uint32_t accelSteps;              ///< Number of steps allocated for acceleration (and deceleration)
	uint32_t decelStartStep;          ///< Step count threshold at which deceleration begins
	uint32_t rampStepCounter;         ///< Current index into the accelLookupTable during ramp phases
	uint16_t accelLookupTable[MAX_ACCEL_STEPS]; ///< Pre-computed ARR values for the S-curve ramp profile

	MotorState_e MotorState;          ///< Current state of the motor state machine
	MotorMode_e  MotorMode;           ///< Current mode of the motor state machine, programmed or manual action
}StepperControl_t;

/**
 * @brief  Initialize all stepper control fields to safe defaults and configure
 *         the TMC2208 driver over UART.
 * @param  stepper Pointer to the StepperControl_t instance to initialize.
 * @note   Must be called before any other stepper function, and before the
 *         FreeRTOS scheduler starts.
 */
void stepper_init(StepperControl_t* stepper);

/**
 * @brief  Pre-compute the S-curve acceleration lookup table.
 * @param  stepper      Pointer to the StepperControl_t whose lookup table will be populated.
 * @param  startFreqHz  Starting step frequency in Hz (lowest speed).
 * @param  cruiseFreqHz Target cruise step frequency in Hz (highest speed).
 * @param  accelSteps   Number of steps over which to ramp from start to cruise.
 *
 * @details Uses a cosine easing function: scale = 0.5*(1 - cos(i*PI/(N-1)))
 *          to produce a smooth S-shaped velocity transition. The resulting
 *          frequency for each step is converted to an ARR value and stored
 *          in accelLookupTable[]. The same table is read in reverse for decel.
 *
 * @note   If accelSteps exceeds MAX_ACCEL_STEPS, the function returns early
 *         without modifying the table.
 */
void stepper_generate_accel_curve(StepperControl_t* stepper, float startFreqHz, float cruiseFreqHz, uint32_t accelSteps);

/**
 * @brief  Compute the motion profile and start the stepper motor.
 * @param  stepper      Pointer to the StepperControl_t instance.
 * @param  cruiseFreqHz Desired cruise (maximum) step frequency in Hz.
 * @param  totalSteps   Total number of steps for the entire move.
 * @param  accelSteps   Desired number of acceleration steps (will be clamped
 *                      to totalSteps/2 if the move is too short for full ramp).
 *
 * @details This function:
 *          1. Validates the motor is idle (returns immediately if not).
 *          2. Clamps accelSteps for short moves (triangular profile).
 *          3. Generates the S-curve lookup table.
 *          4. Configures TIM2 prescaler, ARR, and compare registers.
 *          5. Arms the state machine to MOTOR_STATE_ACCEL.
 *          6. Starts TIM2 in interrupt + PWM mode.
 *
 * @note   This function is safe to call from any thread context, but the motor
 *         must be in MOTOR_STATE_IDLE. The ISR in main.c will drive the state
 *         machine from this point.
 */
void stepper_start(StepperControl_t* stepper, float cruiseFreqHz, uint32_t totalSteps, uint32_t accelSteps);

/**
 * @brief  Switch the TMC2208 between SpreadCycle and StealthChop at runtime.
 * @param  enable  true  → SpreadCycle (active pumping / manual drive)
 *                 false → StealthChop (idle — silent and cooler)
 */
void stepper_set_spreadcycle(uint8_t enable);


#endif // LIBS_INC_STEPPER_H_
