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
#include "usart.h"

/*
 * Expand the IC cache to support all 4 possible UART addresses (0-3).
 * The TMC2208 UART address is set by the MS1/MS2 pins: if they are not
 * explicitly tied to GND the address may not be 0.  With IC_CACHE_COUNT=4
 * shadow registers work correctly regardless of which address is used.
 * Must be defined BEFORE including TMC2208.h because of the #ifndef guard.
 */
#ifndef TMC2208_IC_CACHE_COUNT
#define TMC2208_IC_CACHE_COUNT 4
#endif

#include "TMC2208.h"

/** @brief TIM2 handle (defined in tim.c by CubeMX). Used for stepper PWM output. */
extern TIM_HandleTypeDef htim2;
uint16_t node_id = 0xFFFF;
float s_curve_norm[256];


/**
 * @brief  Initialize all stepper control fields to safe defaults and configure
 *         the TMC2208 over UART.
 * @param  stepper Pointer to the StepperControl_t instance to initialize.
 * @retval true  TMC2208 found and configured over UART.
 * @retval false TMC2208 did not respond within 2 s; motor state is IDLE,
 *               chip is running with standalone hardware defaults.
 */
void stepper_init(StepperControl_t *stepper)
{
    stepper->ARRCurrent = 0;

    stepper->totalStepsRemaining = 0;
    stepper->accelSteps = 0;
    stepper->decelStartStep = 0;
    stepper->rampStepCounter = 0;

    stepper->MotorState    = MOTOR_STATE_IDLE;
    stepper->MotorMode     = MOTOR_MODE_NONE;
    
    /* Pre-compute normalized S-curve table (0.0 to 1.0) for interpolation */
    for (uint32_t i = 0; i < 256; i++) {
        s_curve_norm[i] = 0.5f * (1.0f - cosf((float)i * M_PI / 255.0f));
    }


    uint32_t ioin = 0;

    while (node_id == 0xFFFF) {
        /* Scan all 4 possible UART addresses */
        for (uint16_t addr = 0; addr < 4; addr++) {
            ioin = (uint32_t)tmc2208_readRegister(addr, TMC2208_IOIN);
            if (((ioin >> 24) & 0xFF) == 0x20) {
                node_id = addr;
                break;
            }
        }
    }

    // Pre-seed shadow cache with hardware reset defaults
    uint32_t chopconf_default = 0x10000053U;
    uint32_t pwmconf_default  = 0xC10D0024U;
    tmc2208_cache(node_id, TMC2208_CACHE_FILL_DEFAULT, TMC2208_CHOPCONF, &chopconf_default);
    tmc2208_cache(node_id, TMC2208_CACHE_FILL_DEFAULT, TMC2208_PWMCONF,  &pwmconf_default);

    uint32_t gconf = (uint32_t)tmc2208_readRegister(node_id, TMC2208_GCONF);
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_PDN_DISABLE_FIELD,      1); // Enable UART control
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_MSTEP_REG_SELECT_FIELD, 1); // MRES from register
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_EN_SPREADCYCLE_FIELD,   0); // StealthChop at idle
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_MULTISTEP_FILT_FIELD,   0); // No step filter
    tmc2208_writeRegister(node_id, TMC2208_GCONF, (int32_t)gconf);

    uint32_t chopconf = (uint32_t)tmc2208_readRegister(node_id, TMC2208_CHOPCONF);
    chopconf = tmc2208_fieldUpdate(chopconf, TMC2208_MRES_FIELD,   1); // 128 microsteps
    chopconf = tmc2208_fieldUpdate(chopconf, TMC2208_INTPOL_FIELD, 0); // No interpolation
    tmc2208_writeRegister(node_id, TMC2208_CHOPCONF, (int32_t)chopconf);

    tmc2208_writeRegister(node_id, TMC2208_TPWMTHRS, 0);

}

/**
 * @brief  Switch the TMC2208 chopper mode at runtime.
 *
 * @param  enable  true  → SpreadCycle (use when motor is actively driving load)
 *                 false → StealthChop (use at idle — silent, lower temperature)
 *
 * @details Does a live GCONF read-modify-write over UART.  Safe to call from
 *          any thread context as long as UART is not simultaneously used by
 *          another task.  Has no effect if stepper_init() did not succeed
 */
void stepper_set_spreadcycle(uint8_t enable)
{
    uint32_t gconf = (uint32_t)tmc2208_readRegister(node_id, TMC2208_GCONF);
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_EN_SPREADCYCLE_FIELD, enable ? 1U : 0U);
    tmc2208_writeRegister(node_id, TMC2208_GCONF, (int32_t)gconf);
}

uint8_t tmc2208_getNodeAddress(uint16_t icID)
{
    // icID directly encodes the UART node address (0–3), set by MS1/MS2 pins.
    return (uint8_t)icID;
}

bool tmc2208_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{

     // USART1 is configured in HAL half-duplex mode

    HAL_HalfDuplex_EnableTransmitter(&huart1);

    if (HAL_UART_Transmit(&huart1, data, (uint16_t)writeLength, 100) != HAL_OK) {
        return false;
    }

    if (readLength > 0) {
        uint32_t tickstart = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
            if ((HAL_GetTick() - tickstart) > 50U) {
                return false;
            }
        }

        __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                        UART_CLEAR_PEF | UART_CLEAR_FEF);

        HAL_HalfDuplex_EnableReceiver(&huart1);

        if (HAL_UART_Receive(&huart1, data, (uint16_t)readLength, 100) != HAL_OK) {
            return false;
        }
    }

    return true;
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

    stepper->startFreqHz = startFreq;
    stepper->freqDiff = cruiseFreqHz - startFreq;

    /* --- Configure and Start Timer --- */

    /* Defensive stop: ensure timer is fully halted before reconfiguring */
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    /* Set the prescaler (effective tick = TIMER_CLOCK / (PSC + 1) = 1 MHz) */
    __HAL_TIM_SET_PRESCALER(&htim2, PRESCALER);

    /* Calculate the initial ARR value (slowest speed) */
    stepper->ARRCurrent = (uint32_t)((float)TIMER_CLOCK / ((PRESCALER + 1) * startFreq)) - 1;
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
