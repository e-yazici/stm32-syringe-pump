/**
 * @file    stepper_profile.h
 * @brief   S-curve motion profile calculation and real-time ISR step tick engine.
 * @author  eyazici
 * @date    2026
 *
 * @details Computes normalized S-curve lookup tables, ARR register values,
 *          and provides the real-time step tick engine designed to be inlined
 *          directly inside the timer ISR (e.g. TIM2) with zero call overhead.
 */

#ifndef LIBS_INC_STEPPER_PROFILE_H_
#define LIBS_INC_STEPPER_PROFILE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Force inlining in timer ISR context to eliminate function call overhead and register spilling */
#if defined(__GNUC__) || defined(__clang__)
#define STEPPER_ALWAYS_INLINE static inline __attribute__((always_inline))
#else
#define STEPPER_ALWAYS_INLINE static inline
#endif

#define S_CURVE_TABLE_SIZE 256

/** @brief Motion states for the stepper state machine */
typedef enum {
    MOTOR_STATE_IDLE = 0,    ///< Motor is stopped; no active move
    MOTOR_STATE_ACCEL,       ///< Accelerating: stepping through lookup table forward
    MOTOR_STATE_CRUISE,      ///< Cruising: ARR held constant at target frequency
    MOTOR_STATE_DECEL,       ///< Decelerating: stepping through lookup table in reverse
    MOTOR_STATE_PAUSED       ///< Motor is paused but movement not complete
} StepperMotorState_e;

/** @brief Motion modes */
typedef enum {
    MOTOR_MODE_NONE = 0,     ///< Motor is stopped
    MOTOR_MODE_PUMP,         ///< Programmed pump mode
    MOTOR_MODE_MANUAL        ///< Free motor control using buttons
} StepperMotorMode_e;

/** @brief Action returned by the step tick calculation to the ISR */
typedef enum {
    STEP_ACTION_CONTINUE = 0,  ///< Move in progress; apply new ARR to timer
    STEP_ACTION_STOP_IDLE,     ///< Move completed (or manual stop); stop PWM and go idle
    STEP_ACTION_STOP_PAUSED    ///< Move paused; stop PWM and hold paused state
} StepperStepAction_e;

/**
 * @brief Runtime control structure for stepper motion profile.
 *        Compatible with StepperControl_t.
 */
typedef struct {
    uint32_t ARRCurrent;              ///< Current Auto-Reload Register value being applied to timer
    uint32_t totalStepsRemaining;     ///< Steps left until move completes
    uint32_t accelSteps;              ///< Number of steps allocated for acceleration (and deceleration)
    uint32_t decelStartStep;          ///< Step count threshold at which deceleration begins
    uint32_t rampStepCounter;         ///< Current step index during acceleration/deceleration
    float startFreqHz;                ///< Starting step frequency (Hz)
    float freqDiff;                   ///< Target cruise frequency minus start frequency (Hz)

    StepperMotorState_e MotorState;   ///< Current state of the state machine
    StepperMotorMode_e  MotorMode;    ///< Programmed pump or manual mode
} StepperProfileState_t;

/**
 * @brief  Populate the 256-element normalized cosine S-curve table (0.0 to 1.0).
 * @param  table Array of at least 256 float elements.
 */
void StepperProfile_GenerateSCurveTable(float *table);

/**
 * @brief  Prepare stepper profile state before starting a move.
 * @param  state        Pointer to StepperProfileState_t.
 * @param  cruiseFreqHz Target cruise frequency in Hz.
 * @param  startFreqHz  Starting frequency in Hz.
 * @param  totalSteps   Total steps for the entire move.
 * @param  accelSteps   Desired acceleration steps.
 * @param  timerClock   Timer clock in Hz.
 * @param  prescaler    Timer prescaler.
 * @param  mode         MOTOR_MODE_PUMP or MOTOR_MODE_MANUAL.
 */
void StepperProfile_PrepareMove(StepperProfileState_t *state,
                                float cruiseFreqHz, float startFreqHz,
                                uint32_t totalSteps, uint32_t accelSteps,
                                uint32_t timerClock, uint32_t prescaler,
                                StepperMotorMode_e mode);

/**
 * @brief  Calculate timer ARR value for a given step frequency (inlined for zero ISR overhead).
 * @param  timerClock Input clock frequency to the timer in Hz (e.g. 72 MHz).
 * @param  prescaler  Timer prescaler value (e.g. 71 for 1 MHz timer tick).
 * @param  freqHz     Desired step frequency in Hz.
 * @retval Calculated ARR value.
 */
STEPPER_ALWAYS_INLINE uint32_t StepperProfile_CalcFreqToARR(uint32_t timerClock,
                                                            uint32_t prescaler,
                                                            float freqHz) {
    /* Guard against division by zero; max ARR yields the longest possible timer period */
    if (freqHz <= 0.0f) {
        return 0xFFFFFFFFU;
    }
    return (uint32_t)((float)timerClock / ((float)(prescaler + 1U) * freqHz)) - 1U;
}

/**
 * @brief  Real-time step calculation function inlined directly inside the timer ISR
 *         (e.g. TIM2 PeriodElapsedCallback) with zero function-call latency.
 *
 * @param  state      Pointer to StepperProfileState_t (may be volatile).
 * @param  s_curve    Pointer to normalized 256-element S-curve table.
 * @param  timerClock Timer input clock frequency (e.g. 72000000).
 * @param  prescaler  Timer prescaler (e.g. 71).
 * @retval StepperStepAction_e indicating whether to continue or stop PWM.
 */
STEPPER_ALWAYS_INLINE StepperStepAction_e StepperProfile_StepTick(volatile StepperProfileState_t *state,
                                                                  const float *s_curve,
                                                                  uint32_t timerClock,
                                                                  uint32_t prescaler) {
    state->totalStepsRemaining--;

    switch (state->MotorState) {
        case MOTOR_STATE_ACCEL:
            state->rampStepCounter++;
            if (state->rampStepCounter >= state->accelSteps) {
                /* Acceleration complete — max frequency */
                float currentFreq = state->startFreqHz + state->freqDiff;
                state->ARRCurrent = (uint32_t)((float)timerClock / ((float)(prescaler + 1U) * currentFreq)) - 1U;
                state->MotorState = MOTOR_STATE_CRUISE;
            } else {
                /* Still accelerating — interpolate from s_curve_norm table */
                float exact_index = ((float)state->rampStepCounter * 255.0f) / (float)state->accelSteps;
                uint32_t index_int = (uint32_t)exact_index;
                float fraction = exact_index - (float)index_int;

                if (index_int >= 255U) {
                    index_int = 254U;
                    fraction = 1.0f;
                }

                /* Linear interpolation across 256-point table avoids expensive runtime cosf() inside ISR */
                float scale = s_curve[index_int] + ((s_curve[index_int + 1U] - s_curve[index_int]) * fraction);
                float currentFreq = state->startFreqHz + (state->freqDiff * scale);

                state->ARRCurrent = (uint32_t)((float)timerClock / ((float)(prescaler + 1U) * currentFreq)) - 1U;
            }

            /* Early decel check: handles triangular profile (no cruise phase) */
            if (state->totalStepsRemaining <= state->accelSteps) {
                state->MotorState = MOTOR_STATE_DECEL;
            }
            break;

        case MOTOR_STATE_CRUISE:
            if ((state->totalStepsRemaining <= state->accelSteps) && (state->MotorMode != MOTOR_MODE_MANUAL)) {
                state->MotorState = MOTOR_STATE_DECEL;
            }
            /* ARR remains constant — no update needed */
            break;

        case MOTOR_STATE_DECEL:
            /* Decrement step counter, stopping at 0 to prevent unsigned underflow */
            if (state->rampStepCounter > 0U) {
                state->rampStepCounter--;
            }

            /* Interpolate the corresponding ARR value from the lookup table */
            if (state->rampStepCounter < state->accelSteps) {
                float exact_index = ((float)state->rampStepCounter * 255.0f) / (float)state->accelSteps;
                uint32_t index_int = (uint32_t)exact_index;
                float fraction = exact_index - (float)index_int;

                if (index_int >= 255U) {
                    index_int = 254U;
                    fraction = 1.0f;
                }

                float scale = s_curve[index_int] + ((s_curve[index_int + 1U] - s_curve[index_int]) * fraction);
                float currentFreq = state->startFreqHz + (state->freqDiff * scale);

                state->ARRCurrent = (uint32_t)((float)timerClock / ((float)(prescaler + 1U) * currentFreq)) - 1U;
            }
            break;

        default:
            /* Safety fallback: unknown state → stop motor */
            state->MotorState = MOTOR_STATE_IDLE;
            state->MotorMode = MOTOR_MODE_NONE;
            break;
    }

    /* --- Move completion check --- */
    if (state->totalStepsRemaining == 0U) {
        /* All steps executed — stop timer and PWM, return to idle */
        state->MotorState = MOTOR_STATE_IDLE;
        state->MotorMode = MOTOR_MODE_NONE;
        return STEP_ACTION_STOP_IDLE;
    } else if (state->rampStepCounter < 1U && state->MotorState == MOTOR_STATE_DECEL) {
        if (state->MotorMode == MOTOR_MODE_MANUAL) {
            /* Manual move finished */
            state->totalStepsRemaining = 0U;
            state->MotorState = MOTOR_STATE_IDLE;
            state->MotorMode = MOTOR_MODE_NONE;
            return STEP_ACTION_STOP_IDLE;
        } else {
            /* Motor decelerated to a stop, but steps remain (paused) */
            state->MotorState = MOTOR_STATE_PAUSED;
            return STEP_ACTION_STOP_PAUSED;
        }
    }

    return STEP_ACTION_CONTINUE;
}

#ifdef __cplusplus
}
#endif

#endif /* LIBS_INC_STEPPER_PROFILE_H_ */
