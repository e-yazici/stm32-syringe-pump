/**
 * @file    stepper_profile.c
 * @brief   Implementation of S-curve motion profile setup and lookup table generation.
 * @author  eyazici
 * @date    2026
 */

#include "stepper_profile.h"
#include <stddef.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void StepperProfile_GenerateSCurveTable(float *table) {
    if (table == NULL) {
        return;
    }
    for (uint32_t i = 0; i < S_CURVE_TABLE_SIZE; i++) {
        table[i] = 0.5f * (1.0f - cosf((float)i * (float)M_PI / 255.0f));
    }
}

void StepperProfile_PrepareMove(StepperProfileState_t *state,
                                float cruiseFreqHz, float startFreqHz,
                                uint32_t totalSteps, uint32_t accelSteps,
                                uint32_t timerClock, uint32_t prescaler,
                                StepperMotorMode_e mode) {
    if (state == NULL) {
        return;
    }

    /* Clamp to symmetric triangular profile when total travel is too short for a cruise phase */
    if (totalSteps < (2U * accelSteps) && mode != MOTOR_MODE_MANUAL) {
        accelSteps = totalSteps / 2U;
    }

    state->totalStepsRemaining = totalSteps;
    state->accelSteps = accelSteps;
    state->decelStartStep = totalSteps - accelSteps;
    state->rampStepCounter = 0U;

    float startFreq = startFreqHz;
    if (cruiseFreqHz < startFreq) {
        startFreq = cruiseFreqHz;
    }

    state->startFreqHz = startFreq;
    state->freqDiff = cruiseFreqHz - startFreq;
    state->MotorMode = mode;

    /* Start at initial frequency ARR */
    state->ARRCurrent = StepperProfile_CalcFreqToARR(timerClock, prescaler, startFreq);
    state->MotorState = MOTOR_STATE_ACCEL;
}
