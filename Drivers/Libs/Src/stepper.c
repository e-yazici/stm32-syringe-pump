/**
 * @file    stepper.c
 * @brief   Stepper motor S-curve acceleration engine implementation.
 * @author  eyazici
 * @date    2026
 *
 * @details Implements the stepper motor acceleration profile generator and
 *          motion start sequence. The acceleration profile uses a cosine
 *          easing function to produce a smooth S-shaped velocity ramp, which
 *          reduces mechanical vibration, motor stall and improves accuracy at high speeds.
 *
 *          The motion is executed as a trapezoidal profile with three phases:
 *          1. **Acceleration**: Step frequency increases from START_FREQ_HZ
 *             to cruiseFreqHz following the S-curve lookup table.
 *          2. **Cruise**: Constant frequency at cruiseFreqHz.
 *          3. **Deceleration**: Mirror of acceleration (table read in reverse).
 *
 *          If the total move is too short for a full trapezoid, the cruise
 *          phase is eliminated resulting in a triangular profile.
 *
 * @note    The actual step-by-step state machine execution happens in the
 *          HAL_TIM_PeriodElapsedCallback() ISR in main.c. This module only
 *          sets up the profile and starts the timer.
 */

#include "stepper.h"

/** @brief TIM2 handle (defined in tim.c by CubeMX). Used for stepper PWM output. */
extern TIM_HandleTypeDef htim2;

/**
 * @brief  Initialize all stepper control fields to safe defaults.
 * @param  stepper Pointer to the StepperControl_t instance to initialize.
 */
void stepper_init(StepperControl_t *stepper)
{
    stepper->ARRCurrent = 0;

    stepper->totalStepsRemaining = 0;
    stepper->accelSteps = 0;
    stepper->decelStartStep = 0;
    stepper->rampStepCounter = 0;

    stepper->MotorState = MOTOR_STATE_IDLE;
    stepper->MotorMode = MOTOR_MODE_NONE;
}

/**
 * @brief  Generate an S-curve acceleration lookup table using cosine easing.
 * @param  stepper      Pointer to the StepperControl_t whose table will be populated.
 * @param  startFreqHz  Step frequency at the start of the ramp (Hz).
 * @param  cruiseFreqHz Step frequency at the end of the ramp (Hz).
 * @param  accelSteps   Number of entries (steps) in the ramp.
 *
 * @details The S-curve is generated using a cosine interpolation formula:
 *
 *          scale(i) = 0.5 * (1 - cos(i * PI / (N - 1)))
 *
 *          This maps each step index i (0..N-1) to a value between 0.0 and 1.0
 *          with smooth acceleration at both ends (zero jerk at start/finish).
 *          The frequency at each step is then:
 *
 *          freq(i) = startFreq + (cruiseFreq - startFreq) * scale(i)
 *
 *          Each frequency is converted to a timer ARR value:
 *
 *          ARR(i) = TIMER_CLOCK / ((PRESCALER + 1) * freq(i)) - 1
 *
 * @note   If accelSteps == 0, only a single entry at cruiseFreqHz is stored.
 *         If accelSteps > MAX_ACCEL_STEPS, the function returns without action.
 */
static void generate_s_curve_profile(StepperControl_t *stepper,
        float startFreqHz, float cruiseFreqHz, uint32_t accelSteps)
{
    /* Guard against buffer overflow */
    if (accelSteps > MAX_ACCEL_STEPS) {
        return;
    }
    
    /* Special case: zero-length ramp — just store the cruise ARR */
    if (accelSteps == 0) {
        uint32_t arr = (uint32_t) ((float) TIMER_CLOCK / ((PRESCALER + 1) * cruiseFreqHz)) - 1;
        stepper->accelLookupTable[0] = (uint16_t) arr;
        return;
    }

    float freqChange = cruiseFreqHz - startFreqHz;

    for (uint32_t i = 0; i < accelSteps; i++) {
        /*
         * Cosine easing: maps i from [0, N-1] to a scale factor [0.0, 1.0]
         * in an S-curve shape. At i=0 scale≈0 (start), at i=N-1 scale=1.0 (cruise).
         * The cosine provides zero-derivative at both endpoints (smooth start/stop).
         */
        float scale = 0.5f * (1.0f - cosf((float) i * M_PI / (accelSteps - 1)));

        /* Interpolate between start and cruise frequencies */
        float currentFreq = startFreqHz + (freqChange * scale);

        /*
         * Convert frequency (Hz) to timer Auto-Reload Register value:
         *   ARR = (TimerClock / ((PSC + 1) * freq)) - 1
         * The effective timer tick rate is TIMER_CLOCK / (PSC + 1) = 1 MHz,
         * so ARR = 1,000,000 / freq - 1.
         */
        uint32_t arr = (uint32_t) ((float) TIMER_CLOCK / ((PRESCALER + 1) * currentFreq)) - 1;

        stepper->accelLookupTable[i] = (uint16_t) arr;
    }
}

/**
 * @brief  Compute the motion profile and start the stepper motor.
 * @param  stepper      Pointer to the StepperControl_t instance.
 * @param  cruiseFreqHz Desired cruise step frequency (Hz).
 * @param  totalSteps   Total steps for the move.
 * @param  accelSteps   Desired number of acceleration ramp steps.
 *
 * @details Sequence of operations:
 *          1. Reject if motor is already running.
 *          2. Clamp accelSteps to totalSteps/2 for short moves (triangular profile).
 *          3. Generate the S-curve lookup table.
 *          4. Stop any prior timer activity (defensive).
 *          5. Configure TIM2: prescaler, initial ARR, 50% duty compare value.
 *          6. Force an update event (UG bit) to load shadow registers immediately.
 *          7. Arm the state machine to MOTOR_STATE_ACCEL.
 *          8. Start TIM2 in interrupt + PWM mode.
 */
void stepper_start(StepperControl_t *stepper, float cruiseFreqHz,
        uint32_t totalSteps, uint32_t accelSteps)
{
    /* Reject if a move is already in progress */
    if (stepper->MotorState != MOTOR_STATE_IDLE) {
        return;
    }

    /*
     * Clamp acceleration steps for short moves: if total steps < 2*accel,
     * there's no room for both accel and decel ramps. Fall back to a
     * triangular profile by halving.
     */
    if (totalSteps < (2 * accelSteps) && stepper->MotorMode != MOTOR_MODE_MANUAL) {
        accelSteps = totalSteps / 2;
    }

    /* Store motion parameters for the ISR state machine */
    stepper->totalStepsRemaining = totalSteps;
    stepper->accelSteps = accelSteps;
    stepper->decelStartStep = totalSteps - accelSteps;

    /*
     * Ensure start frequency doesn't exceed cruise (e.g., for very slow moves).
     * If cruise is slower than the default start, we start directly at cruise.
     */
    float startFreq = (float) START_FREQ_HZ;
    if (cruiseFreqHz < startFreq) {
        startFreq = cruiseFreqHz;
    }

    /* Pre-compute the entire acceleration ramp into the lookup table */
    generate_s_curve_profile(stepper, startFreq, cruiseFreqHz,
            stepper->accelSteps);

    /* --- Configure and Start Timer --- */

    /* Defensive stop: ensure timer is fully halted before reconfiguring */
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    /* Set the prescaler (effective tick = TIMER_CLOCK / (PSC + 1) = 1 MHz) */
    __HAL_TIM_SET_PRESCALER(&htim2, PRESCALER);

    /* Load the first ARR value from the lookup table (slowest speed) */
    stepper->ARRCurrent = stepper->accelLookupTable[0];
    __HAL_TIM_SET_AUTORELOAD(&htim2, stepper->ARRCurrent);
    /* 50% duty cycle: compare = ARR/2 produces a symmetric step pulse */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, stepper->ARRCurrent / 2);

    /*
     * Force an Update Generation event (UG bit in EGR register).
     * This immediately loads the new PSC and ARR values from the shadow
     * registers into the active registers, so the very first period is correct.
     */
    TIM2->EGR |= TIM_EGR_UG;
    
    /* 
     * Generating an update event also sets the Update Interrupt Flag (UIF).
     * If we don't clear it here, the timer ISR will fire INSTANTLY upon 
     * calling HAL_TIM_Base_Start_IT, consuming the first step prematurely.
     */
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

    /* ---- Arm the state machine ---- */
    stepper->rampStepCounter = 0;
    stepper->MotorState = MOTOR_STATE_ACCEL;

    /* ---- Start the Hardware ---- */
    /* Start base timer interrupt (triggers HAL_TIM_PeriodElapsedCallback each period) */
    HAL_TIM_Base_Start_IT(&htim2);
    /* Start PWM output on Channel 1 (drives the stepper STEP pin) */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
