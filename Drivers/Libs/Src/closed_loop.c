/**
 * @file    closed_loop.c
 * @brief   Implementation of closed-loop motion supervisor and stall detection.
 * @author  eyazici
 * @date    2026
 */

#include "closed_loop.h"
#include <math.h>
#include <string.h>

/* Module Static State Variables */
static ClosedLoopConfig_t s_config;
static uint32_t           s_stallTimer_ms       = 0;
static uint32_t           s_blankingTimer_ms    = 0;
static bool               s_isStalled           = false;
static float              s_lastActualDist_mm   = 0.0f;
static bool               s_lastActualDistValid = false;

#if CLOSED_LOOP_TEST
volatile ClosedLoopDebug_t gClosedLoopDebug = {0};
#endif

void ClosedLoop_Init(const ClosedLoopConfig_t *cfg)
{
    if (cfg != NULL) {
        s_config = *cfg;
        if (s_config.stallPersistTime_ms == 0U) {
            s_config.stallPersistTime_ms = CL_DEFAULT_STALL_PERSIST_MS;
        }
        if (s_config.maxCompensationSteps == 0U) {
            s_config.maxCompensationSteps = CL_DEFAULT_MAX_COMP_STEPS;
        }
    } else {
        s_config.positionLagThresh_mm   = CL_DEFAULT_DEADBAND_MM;
        s_config.stallLagThresh_mm      = CL_DEFAULT_STALL_LAG_MM;
        s_config.grossStallLagThresh_mm = CL_DEFAULT_GROSS_STALL_LAG_MM;
        s_config.stallVelRatioThresh    = CL_DEFAULT_STALL_VEL_RATIO;
        s_config.stallPersistTime_ms    = CL_DEFAULT_STALL_PERSIST_MS;
        s_config.blankingDuration_ms    = CL_DEFAULT_BLANKING_MS;
        s_config.maxCompensationSteps   = CL_DEFAULT_MAX_COMP_STEPS;
    }

    ClosedLoop_Reset();
    ClosedLoop_ArmBlanking(s_config.blankingDuration_ms);
}

void ClosedLoop_Reset(void)
{
    s_stallTimer_ms       = 0;
    s_blankingTimer_ms    = 0;
    s_isStalled           = false;
    s_lastActualDistValid = false;
    s_lastActualDist_mm   = 0.0f;
}

void ClosedLoop_ArmBlanking(uint32_t duration_ms)
{
    s_blankingTimer_ms    = duration_ms;
    s_stallTimer_ms       = 0;
    s_isStalled           = false;
    s_lastActualDistValid = false;
    s_lastActualDist_mm   = 0.0f;
}

void ClosedLoop_SetMaxCompensationSteps(uint32_t maxSteps)
{
    s_config.maxCompensationSteps = maxSteps;
}

const ClosedLoopConfig_t* ClosedLoop_GetConfig(void)
{
    return &s_config;
}

uint32_t ClosedLoop_GetStallTimer(void)
{
    return s_stallTimer_ms;
}

uint32_t ClosedLoop_GetBlankingTimer(void)
{
    return s_blankingTimer_ms;
}

bool ClosedLoop_IsStalled(void)
{
    return s_isStalled;
}

ClosedLoopOutput_t ClosedLoop_Update(float commandedDist_mm, float actualDist_mm,
                                    float commandedVel_mm_s, float actualVel_mm_s,
                                    uint32_t dt_ms, uint32_t stepsPerMm)
{
    ClosedLoopOutput_t out;
    memset(&out, 0, sizeof(out));

    /* 1. Calculate Position Lag: E_lag = X_cmd - X_act */
    float lag_mm = commandedDist_mm - actualDist_mm;
    out.positionLag_mm = lag_mm;

    /* 2. Calculate Velocity Ratio: V_act / V_cruise (protect against division by zero) */
    if (commandedVel_mm_s > 1e-6f) {
        out.velocityRatio = actualVel_mm_s / commandedVel_mm_s;
    } else {
        out.velocityRatio = 1.0f;
    }

    /* 2b. Instantaneous Cycle Velocity Ratio (immune to multi-sample EMA filter latency)
     * For commanded speeds >= 0.10 mm/s, compare cycle-to-cycle displacement.
     * When motor is physically blocked by an obstacle or thumb, actual travel drops to 0 immediately,
     * allowing instant stall detection rather than waiting 150-200 ms for the EMA filter to decay. */
    float cycleVelRatio = out.velocityRatio;
    if (s_lastActualDistValid && dt_ms > 0 && commandedVel_mm_s >= 0.10f) {
        float actualDelta_mm = actualDist_mm - s_lastActualDist_mm;
        if (actualDelta_mm < 0.0f) {
            actualDelta_mm = 0.0f;
        }
        float cycleVel_mm_s = (actualDelta_mm * 1000.0f) / (float)dt_ms;
        cycleVelRatio = cycleVel_mm_s / commandedVel_mm_s;
    }
    s_lastActualDist_mm = actualDist_mm;
    s_lastActualDistValid = true;

    /* Use effective velocity ratio: minimum of cycle instantaneous and filtered ratio */
    float effectiveVelRatio = (cycleVelRatio < out.velocityRatio) ? cycleVelRatio : out.velocityRatio;
    out.velocityRatio = effectiveVelRatio;

    /* 3. Handle Blanking Window Countdown & Calculate Unblanked Elapsed Time */
    uint32_t dt_unblanked = 0;
    if (s_blankingTimer_ms > 0) {
        if (dt_ms >= s_blankingTimer_ms) {
            dt_unblanked = dt_ms - s_blankingTimer_ms;
            s_blankingTimer_ms = 0;
        } else {
            s_blankingTimer_ms -= dt_ms;
            dt_unblanked = 0;
        }
    } else {
        dt_unblanked = dt_ms;
    }
    out.blankingActive = (s_blankingTimer_ms > 0);

    /* 4. Stall Detection Logic
     * Blockage condition: position lag exceeds minimum threshold (stallLagThresh_mm, default 0.030 mm) AND
     * velocity has dropped to near-zero (effectiveVelRatio < stallVelRatioThresh).
     * Optional gross lag cutoff is active only if configured (> 0.0f). */
    bool isStallCondition = ((lag_mm >= s_config.stallLagThresh_mm) &&
                             (effectiveVelRatio < s_config.stallVelRatioThresh)) ||
                            ((s_config.grossStallLagThresh_mm > 0.0f) &&
                             (lag_mm >= s_config.grossStallLagThresh_mm));

    /* Speed-aware stall persistence:
     * - High cruise speeds (>= 0.50 mm/s): tightens to 20 ms (1 cycle) for instant blockage trip.
     * - Moderate speeds (0.05 to 0.50 mm/s): uses baseline stallPersistTime_ms (40 ms = 2 cycles).
     * - Ultra-slow speeds (< 0.05 mm/s): adaptively scales persistence to ensure the
     *   AS5600 12-bit encoder (0.49 um LSB) has enough time to resolve physical travel. */
    uint32_t effectivePersist_ms = s_config.stallPersistTime_ms;
    if (s_config.stallPersistTime_ms == CL_DEFAULT_STALL_PERSIST_MS && commandedVel_mm_s >= 0.50f) {
        effectivePersist_ms = 20U; /* 1 cycle (20 ms) fast trip at high speeds */
    } else if (commandedVel_mm_s > 1e-4f && commandedVel_mm_s < 0.05f) {
        /* Time needed to traverse ~2.0 um (4 encoder counts) */
        uint32_t speedAdapted_ms = (uint32_t)(2.0f / commandedVel_mm_s);
        if (speedAdapted_ms > effectivePersist_ms) {
            effectivePersist_ms = (speedAdapted_ms > 1000U) ? 1000U : speedAdapted_ms;
        }
    }

    /* Accumulate stall timer ONLY using unblanked elapsed time */
    if (isStallCondition && dt_unblanked > 0) {
        s_stallTimer_ms += dt_unblanked;
        if (s_stallTimer_ms >= effectivePersist_ms) {
            s_isStalled = true;
        }
    } else if (effectiveVelRatio >= 0.50f || lag_mm <= s_config.positionLagThresh_mm) {
        /* Only decay stall timer when motor is genuinely moving at healthy speed (>= 50%)
         * or when lag is within deadband. This prevents tooth-snap vibration spikes
         * from wiping out stall accumulation while motor is struggling or blocked */
        if (s_stallTimer_ms > (dt_unblanked * 2)) {
            s_stallTimer_ms -= (dt_unblanked * 2);
        } else {
            s_stallTimer_ms = 0;
        }
    }

    /* If stall is confirmed and latched, halt compensation and return immediately */
    if (s_isStalled) {
        out.status = CL_STATUS_STALL_DETECTED;
        out.compensationSteps = 0;
        return out;
    }

    /* 5. Step Skip Detection and Compensation (Zone 2 & Severe Lag without Stall) */
    if (isfinite(lag_mm) && lag_mm > s_config.positionLagThresh_mm) {
        out.status = CL_STATUS_STEP_SKIP_DETECTED;

        if (stepsPerMm > 0) {
            float rawSteps = lag_mm * (float)stepsPerMm;
            uint32_t compSteps = 0U;
            if (!isfinite(rawSteps) || rawSteps >= (float)UINT32_MAX) {
                compSteps = (s_config.maxCompensationSteps > 0U) ? s_config.maxCompensationSteps : CL_DEFAULT_MAX_COMP_STEPS;
            } else if (rawSteps > 0.0f) {
                compSteps = (uint32_t)lroundf(rawSteps);
            }

            /* Apply safety clamp if configured */
            if (s_config.maxCompensationSteps > 0 && compSteps > s_config.maxCompensationSteps) {
                compSteps = s_config.maxCompensationSteps;
            }
            out.compensationSteps = compSteps;
        } else {
            out.compensationSteps = 0;
        }
    } else {
        /* Deadband (lag <= deadband) or negative lag (overshoot) or non-finite */
        out.status = CL_STATUS_NORMAL;
        out.compensationSteps = 0;
    }

    return out;
}

uint32_t ClosedLoop_CalcCreepSteps(float targetDist_mm, float actualDist_mm,
                                   float lagThresh_mm, uint32_t maxCreepSteps,
                                   uint32_t stepsPerMm)
{
    float residualLag_mm = targetDist_mm - actualDist_mm;

    /* Forward-only creep: only compensate positive lag (undershoot).
     * Do NOT retract plunger on negative lag (overshoot) to prevent patient fluid back-aspiration.
     * Epsilon 1e-5f (0.01 um) prevents float representation error at threshold boundaries. */
    if (isnan(residualLag_mm) || residualLag_mm <= (lagThresh_mm + 1e-5f) || stepsPerMm == 0U) {
        return 0U;
    }

    float rawSteps = residualLag_mm * (float)stepsPerMm;
    uint32_t compSteps = 0U;
    if (!isfinite(rawSteps) || rawSteps >= (float)UINT32_MAX) {
        compSteps = (maxCreepSteps > 0U) ? maxCreepSteps : CL_DEFAULT_CREEP_MAX_STEPS;
    } else if (rawSteps > 0.0f) {
        compSteps = (uint32_t)lroundf(rawSteps);
    }

    /* Apply safety clamp */
    if (maxCreepSteps > 0U && compSteps > maxCreepSteps) {
        compSteps = maxCreepSteps;
    }

    return compSteps;
}
