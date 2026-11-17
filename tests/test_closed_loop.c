/**
 * @file    test_closed_loop.c
 * @brief   Host unit tests for the syringe pump closed-loop tracking,
 *          step skipping compensation, and stall detection module.
 *          Structured in conventional Arrange-Act-Assert (Given-When-Then) style.
 */

#include "unity.h"
#include "closed_loop.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define TEST_MOTOR_STEPS_PER_REV 200U
#define TEST_MICROSTEPPING       128U
#define TEST_SCREW_LEAD_MM       2U
#define TEST_STEPS_PER_MM        ((TEST_MOTOR_STEPS_PER_REV * TEST_MICROSTEPPING) / TEST_SCREW_LEAD_MM)
#define TEST_DT_MS               20U     /* 50 Hz supervisory loop period */

void setUp(void) {
    ClosedLoop_Init(NULL);
}

void tearDown(void) {
    ClosedLoop_Reset();
}

/**
 * @brief Test Case 1: Normal tracking with zero lag and matching velocity.
 *        Verifies that when actual matches commanded, status remains NORMAL
 *        and zero compensation steps are calculated.
 */
void test_closed_loop_normal_tracking_zero_lag(void) {
    /* [GIVEN] Default configuration, no blanking active */
    ClosedLoopConfig_t cfg = {
        .positionLagThresh_mm = 0.05f,
        .stallLagThresh_mm    = 0.15f,
        .stallVelRatioThresh  = 0.20f,
        .stallPersistTime_ms  = 200U,
        .blankingDuration_ms  = 0U,
        .maxCompensationSteps = 1280U
    };
    ClosedLoop_Init(&cfg);

    float dist = 0.0f;
    const float cruiseVel = 2.0f; /* 2.0 mm/s */
    const float stepDist = cruiseVel * ((float)TEST_DT_MS / 1000.0f); /* 0.04 mm per 20 ms */

    /* [WHEN] Running 25 consecutive 50 Hz steps (500 ms) with perfect tracking */
    for (int i = 0; i < 25; i++) {
        dist += stepDist;
        ClosedLoopOutput_t out = ClosedLoop_Update(dist, dist, cruiseVel, cruiseVel, TEST_DT_MS, TEST_STEPS_PER_MM);

        /* [THEN] Status is NORMAL, compensation is 0, lag is 0.0, velocity ratio is 1.0 */
        TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, out.status);
        TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out.positionLag_mm);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, out.velocityRatio);
        TEST_ASSERT_FALSE(out.blankingActive);
    }
}

/**
 * @brief Test Case 2: Deadband filtering suppresses minor lags below threshold.
 *        Verifies that lag <= 0.025 mm produces 0 compensation steps and NORMAL status.
 */
void test_closed_loop_deadband_filtering_below_threshold(void) {
    /* [GIVEN] Deadband threshold is 0.025 mm (default). Minor lag of 0.015 mm (~1.5 full steps). */
    ClosedLoop_Init(NULL);
    const float cmdDist = 2.000f;
    const float actDist = 1.985f; /* Lag = 0.015 mm < 0.025 mm */
    const float vel = 1.5f;

    /* [WHEN] Evaluating closed loop within deadband */
    ClosedLoopOutput_t out = ClosedLoop_Update(cmdDist, actDist, vel, vel, TEST_DT_MS, TEST_STEPS_PER_MM);

    /* [THEN] Position lag is measured accurately, but no compensation steps are emitted */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.015f, out.positionLag_mm);
    TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, out.status);

    /* Boundary check: lag exactly at 0.025 mm threshold */
    ClosedLoopOutput_t outBoundary = ClosedLoop_Update(cmdDist, 1.975f, vel, vel, TEST_DT_MS, TEST_STEPS_PER_MM);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.025f, outBoundary.positionLag_mm);
    TEST_ASSERT_EQUAL_UINT32(0, outBoundary.compensationSteps);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, outBoundary.status);
}

/**
 * @brief Test Case 3: Step skip detection and exact compensation calculation.
 *        Lag = 0.08 mm -> round(0.08 * 12800) = 1024 microsteps (exactly 8 full steps).
 */
void test_closed_loop_step_skip_detection_and_exact_compensation(void) {
    /* [GIVEN] Lag of 0.08 mm (exceeds 0.025 mm deadband, above 0.030 mm stall lag).
     * Velocity is maintained at 90% cruise (1.8 / 2.0 mm/s >= 0.15 ratio). */
    ClosedLoop_Init(NULL);
    const float cmdDist = 5.00f;
    const float actDist = 4.92f; /* Lag = 0.08 mm */
    const float cmdVel = 2.0f;
    const float actVel = 1.8f;

    /* [WHEN] Updating closed loop */
    ClosedLoopOutput_t out = ClosedLoop_Update(cmdDist, actDist, cmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);

    /* [THEN] Step skip detected, exact 1024 microsteps calculated for compensation */
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STEP_SKIP_DETECTED, out.status);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.08f, out.positionLag_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.90f, out.velocityRatio);
    TEST_ASSERT_EQUAL_UINT32(1024, out.compensationSteps);
}

/**
 * @brief Test Case 4: Safety clamping on compensation steps.
 *        Verifies that compensation steps are capped at configured ceiling.
 */
void test_closed_loop_safety_clamping_on_compensation_steps(void) {
    /* [GIVEN] Custom config with a strict clamp of 512 microsteps (4 full steps) */
    ClosedLoopConfig_t cfg = {
        .positionLagThresh_mm = 0.05f,
        .stallLagThresh_mm    = 0.20f,
        .stallVelRatioThresh  = 0.20f,
        .stallPersistTime_ms  = 200U,
        .blankingDuration_ms  = 0U,
        .maxCompensationSteps = 512U
    };
    ClosedLoop_Init(&cfg);

    /* [WHEN] Lag is 0.10 mm -> raw steps = round(0.10 * 12800) = 1280 microsteps */
    ClosedLoopOutput_t out = ClosedLoop_Update(10.0f, 9.90f, 2.0f, 1.8f, TEST_DT_MS, TEST_STEPS_PER_MM);

    /* [THEN] Compensation steps clamped to maxCompensationSteps (512) */
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STEP_SKIP_DETECTED, out.status);
    TEST_ASSERT_EQUAL_UINT32(512, out.compensationSteps);
    TEST_ASSERT_TRUE(out.compensationSteps <= cfg.maxCompensationSteps);

    /* Test default clamp when maxCompensationSteps is 0:
     * Default clamp is CL_DEFAULT_MAX_COMP_STEPS (128000 microsteps = 10.0 mm) */
    ClosedLoop_Init(NULL);
    /* Lag 12.0 mm -> raw steps = round(12.0 * 12800) = 153600 microsteps */
    ClosedLoopOutput_t outDefault = ClosedLoop_Update(20.0f, 8.0f, 2.0f, 1.8f, TEST_DT_MS, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STEP_SKIP_DETECTED, outDefault.status);
    TEST_ASSERT_EQUAL_UINT32(CL_DEFAULT_MAX_COMP_STEPS, outDefault.compensationSteps);
}

/**
 * @brief Test Case 4b: Overshoot (negative lag) produces zero compensation steps.
 */
void test_closed_loop_overshoot_negative_lag_zero_compensation(void) {
    /* [GIVEN] Actual position leads commanded position (Lag = -0.05 mm) */
    ClosedLoop_Init(NULL);

    /* [WHEN] Updating closed loop with negative lag */
    ClosedLoopOutput_t out = ClosedLoop_Update(2.00f, 2.05f, 2.0f, 2.0f, TEST_DT_MS, TEST_STEPS_PER_MM);

    /* [THEN] Negative lag reported accurately, but status is NORMAL and compensation is 0 */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.05f, out.positionLag_mm);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, out.status);
    TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);
}

/**
 * @brief Test Case 5: Stall detection trips only after sustained persistence (>= 200 ms).
 *        Lag > 0.15 mm and Vel < 20% cruise.
 */
void test_closed_loop_stall_detection_trip_after_persistence(void) {
    /* [GIVEN] Default config: stallLag = 0.15 mm, stallVel = 20%, persist = 200 ms, blanking = 0 */
    ClosedLoopConfig_t cfg = {
        .positionLagThresh_mm = 0.05f,
        .stallLagThresh_mm    = 0.15f,
        .stallVelRatioThresh  = 0.20f,
        .stallPersistTime_ms  = 200U,
        .blankingDuration_ms  = 0U,
        .maxCompensationSteps = 1280U
    };
    ClosedLoop_Init(&cfg);

    float cmdDist = 1.20f;
    const float actDist = 1.00f; /* Jammed, initial lag = 0.20 mm > 0.15 mm stall threshold */
    const float cmdVel = 2.0f;
    const float actVel = 0.0f; /* 0% of cruise */

    /* Advance time in 20 ms steps from t = 20 ms to t = 180 ms (9 steps = 180 ms) */
    ClosedLoopOutput_t out;
    for (int t = 20; t <= 180; t += 20) {
        cmdDist += cmdVel * ((float)TEST_DT_MS / 1000.0f);
        out = ClosedLoop_Update(cmdDist, actDist, cmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);

        /* [THEN] At t < 200 ms, stall condition is active but stall has NOT tripped yet */
        TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, out.status);
    }

    /* At t = 200 ms (10th tick of 20 ms) -> persistence threshold reached! */
    cmdDist += cmdVel * ((float)TEST_DT_MS / 1000.0f);
    out = ClosedLoop_Update(cmdDist, actDist, cmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);

    /* [THEN] Stall confirmed */
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STALL_DETECTED, out.status);
    /* In stall, compensation must be 0 to avoid motor damage */
    TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);

    /* Verify latching: subsequent update remains STALL_DETECTED */
    cmdDist += cmdVel * ((float)TEST_DT_MS / 1000.0f);
    out = ClosedLoop_Update(cmdDist, actDist, cmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STALL_DETECTED, out.status);
    TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);
}

/**
 * @brief Test Case 6: Transient disturbance rejection (50 ms lag spike does NOT trigger stall).
 */
void test_closed_loop_transient_disturbance_rejection_no_stall(void) {
    /* [GIVEN] Stall persistence is 200 ms. Disturbance lasts only 50 ms. */
    ClosedLoopConfig_t cfg = {
        .positionLagThresh_mm = 0.05f,
        .stallLagThresh_mm    = 0.15f,
        .stallVelRatioThresh  = 0.20f,
        .stallPersistTime_ms  = 200U,
        .blankingDuration_ms  = 0U,
        .maxCompensationSteps = 1280U
    };
    ClosedLoop_Init(&cfg);
    const uint32_t dt_ms = 10; /* 10 ms resolution */

    float cmdDist = 2.0f;
    float actDist = 2.0f;
    const float cmdVel = 2.0f;

    /* [WHEN] Disturbance for 50 ms (5 ticks of 10 ms): motor temporarily stops */
    ClosedLoopOutput_t out;
    for (int i = 0; i < 5; i++) {
        cmdDist += 0.04f; /* Lag grows: 0.04, 0.08, 0.12, 0.16, 0.20 mm */
        out = ClosedLoop_Update(cmdDist, actDist, cmdVel, 0.0f, dt_ms, TEST_STEPS_PER_MM);
        /* Must NOT trip stall within 50 ms */
        TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, out.status);
    }

    /* Disturbance clears: motor catches up and resumes cruise velocity */
    actDist = cmdDist - 0.02f; /* Residual lag 0.02 mm (< 0.05 mm deadband) */
    for (int i = 0; i < 20; i++) { /* Run for another 200 ms */
        cmdDist += cmdVel * ((float)dt_ms / 1000.0f);
        actDist += cmdVel * ((float)dt_ms / 1000.0f);
        out = ClosedLoop_Update(cmdDist, actDist, cmdVel, cmdVel, dt_ms, TEST_STEPS_PER_MM);

        /* [THEN] Stall accumulator was cleared; status returns to NORMAL */
        TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, out.status);
        TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);
    }
}

/**
 * @brief Test Case 7: Blanking window functionality.
 *        Stall conditions are ignored during first 250 ms after start/resume.
 */
void test_closed_loop_blanking_window_suppresses_stall_on_start(void) {
    /* [GIVEN] Blanking duration configured for 250 ms upon pump start */
    ClosedLoopConfig_t cfg = {
        .positionLagThresh_mm = 0.05f,
        .stallLagThresh_mm    = 0.15f,
        .stallVelRatioThresh  = 0.20f,
        .stallPersistTime_ms  = 200U,
        .blankingDuration_ms  = 250U,
        .maxCompensationSteps = 1280U
    };
    ClosedLoop_Init(&cfg);
    const uint32_t dt_ms = 25; /* 10 ticks = 250 ms */

    /* During startup acceleration, actual velocity is 0 and lag accumulates. */
    float cmdDist = 0.0f;
    const float actDist = 0.0f;
    const float cmdVel = 2.0f;
    const float actVel = 0.0f;

    ClosedLoopOutput_t out;
    for (int t = 25; t < 250; t += 25) {
        cmdDist += cmdVel * ((float)dt_ms / 1000.0f);
        out = ClosedLoop_Update(cmdDist, actDist, cmdVel, actVel, dt_ms, TEST_STEPS_PER_MM);

        /* [THEN] Blanking window is active and stall detection is suppressed */
        TEST_ASSERT_TRUE(out.blankingActive);
        TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, out.status);
    }

    /* Final tick at 250 ms: blanking duration expires */
    cmdDist += cmdVel * ((float)dt_ms / 1000.0f);
    out = ClosedLoop_Update(cmdDist, actDist, cmdVel, actVel, dt_ms, TEST_STEPS_PER_MM);
    TEST_ASSERT_FALSE(out.blankingActive);

    /* After blanking expires, if motor reached cruise speed, no stall occurs */
    out = ClosedLoop_Update(cmdDist, cmdDist, cmdVel, cmdVel, dt_ms, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, out.status);
}

/**
 * @brief Test Case 8: Reset and re-sync behavior on resume and abort.
 */
void test_closed_loop_reset_and_resync_lifecycle(void) {
    /* [GIVEN] Controller tripped into STALL_DETECTED state */
    ClosedLoopConfig_t cfg = {
        .positionLagThresh_mm = 0.05f,
        .stallLagThresh_mm    = 0.15f,
        .stallVelRatioThresh  = 0.20f,
        .stallPersistTime_ms  = 200U,
        .blankingDuration_ms  = 0U,
        .maxCompensationSteps = 1280U
    };
    ClosedLoop_Init(&cfg);

    /* Force stall condition for 200 ms */
    for (int i = 0; i < 11; i++) {
        ClosedLoop_Update(1.0f + (float)i * 0.05f, 0.5f, 2.0f, 0.0f, 20, TEST_STEPS_PER_MM);
    }
    ClosedLoopOutput_t outStall = ClosedLoop_Update(1.5f, 0.5f, 2.0f, 0.0f, 20, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STALL_DETECTED, outStall.status);

    /* [WHEN] User selects "Continue" (Resume) -> Reset called and Blanking re-armed */
    ClosedLoop_Reset();
    ClosedLoop_ArmBlanking(CL_DEFAULT_BLANKING_MS);

    /* [THEN] Immediately after reset and re-syncing distance baseline (cmd == act = 0.5 mm) */
    ClosedLoopOutput_t outResumed = ClosedLoop_Update(0.5f, 0.5f, 2.0f, 0.0f, 20, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, outResumed.status);
    TEST_ASSERT_TRUE(outResumed.blankingActive);
    TEST_ASSERT_EQUAL_UINT32(0, outResumed.compensationSteps);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, outResumed.positionLag_mm);

    /* [WHEN] User selects "Abort" -> Reset clears all latched states */
    ClosedLoop_Reset();
    ClosedLoopOutput_t outIdle = ClosedLoop_Update(0.0f, 0.0f, 0.0f, 0.0f, 20, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, outIdle.status);
    TEST_ASSERT_FALSE(outIdle.blankingActive);
    TEST_ASSERT_EQUAL_UINT32(0, outIdle.compensationSteps);
}

/**
 * @brief Robustness Test 9: Initialization with custom config and NULL pointer fallback.
 */
void test_closed_loop_init_with_custom_config_and_null_fallback(void) {
    /* [WHEN] Initializing with custom config */
    ClosedLoopConfig_t custom = {
        .positionLagThresh_mm = 0.08f,
        .stallLagThresh_mm    = 0.30f,
        .stallVelRatioThresh  = 0.15f,
        .stallPersistTime_ms  = 300U,
        .blankingDuration_ms  = 500U,
        .maxCompensationSteps = 640U
    };
    ClosedLoop_Init(&custom);

    /* Verify custom deadband of 0.08 mm (0.06 mm lag should now be in deadband) */
    ClosedLoopOutput_t out = ClosedLoop_Update(1.06f, 1.00f, 2.0f, 2.0f, 20, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, out.status);
    TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);

    /* [WHEN] Calling Init(NULL) -> falls back to safe default constants */
    ClosedLoop_Init(NULL);
    /* In default config, deadband is 0.025 mm, so 0.06 mm lag triggers STEP_SKIP_DETECTED */
    ClosedLoopOutput_t outDef = ClosedLoop_Update(1.06f, 1.00f, 2.0f, 2.0f, 20, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STEP_SKIP_DETECTED, outDef.status);
    TEST_ASSERT_TRUE(outDef.compensationSteps > 0);
}

/**
 * @brief Robustness Test 10: Zero commanded velocity handles division by zero safely.
 */
void test_closed_loop_zero_velocity_division_by_zero_safety(void) {
    /* [GIVEN] Motor at standstill or zero commanded velocity */
    ClosedLoop_Init(NULL);

    /* [WHEN] Updating with commandedVel = 0.0f */
    ClosedLoopOutput_t out = ClosedLoop_Update(1.0f, 1.0f, 0.0f, 0.0f, TEST_DT_MS, TEST_STEPS_PER_MM);

    /* [THEN] Velocity ratio is safe, not NaN or Inf */
    TEST_ASSERT_FALSE(isnan(out.velocityRatio));
    TEST_ASSERT_FALSE(isinf(out.velocityRatio));
    TEST_ASSERT_EQUAL_INT(CL_STATUS_NORMAL, out.status);
}

/**
 * @brief Test Case 11: Blanking window countdown with unblanked time calculation.
 *        Verifies that when a single large dt (e.g. 250 ms or 300 ms) exhausts the
 *        blanking window, only time elapsed AFTER blanking expiration accumulates
 *        in the stall persistence timer, preventing premature stall on tick 1.
 */
void test_closed_loop_blanking_window_large_dt_no_leakage(void) {
    /* [GIVEN] Blanking duration 250 ms, stall persistence 200 ms */
    ClosedLoopConfig_t cfg = {
        .positionLagThresh_mm = 0.05f,
        .stallLagThresh_mm    = 0.15f,
        .stallVelRatioThresh  = 0.20f,
        .stallPersistTime_ms  = 200U,
        .blankingDuration_ms  = 250U,
        .maxCompensationSteps = 1280U
    };
    ClosedLoop_Init(&cfg);

    /* Scenario A: Single startup tick where dt == blankingDuration (250 ms).
     * Motor has severe stall condition (lag = 0.50 mm, velocity = 0.0 mm/s). */
    ClosedLoopOutput_t outA = ClosedLoop_Update(0.50f, 0.00f, 2.0f, 0.0f, 250U, TEST_STEPS_PER_MM);

    /* [THEN]
     * 1. Blanking duration is now expired (blankingActive == false, blankingTimer == 0).
     * 2. Unblanked elapsed time is 0 ms (250 - 250 = 0), so stall timer MUST be 0!
     * 3. Motor must NOT trip into stall on tick 1! */
    TEST_ASSERT_FALSE(outA.blankingActive);
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_GetBlankingTimer());
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_GetStallTimer());
    TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, outA.status);
    TEST_ASSERT_FALSE(ClosedLoop_IsStalled());

    /* Scenario B: Oversized dt (300 ms when blanking is 250 ms) -> only 50 ms accumulates */
    ClosedLoop_Reset();
    ClosedLoop_ArmBlanking(250U);
    ClosedLoopOutput_t outB = ClosedLoop_Update(0.60f, 0.00f, 2.0f, 0.0f, 300U, TEST_STEPS_PER_MM);

    /* 300 ms - 250 ms blanking = exactly 50 ms unblanked elapsed time */
    TEST_ASSERT_FALSE(outB.blankingActive);
    TEST_ASSERT_EQUAL_UINT32(50, ClosedLoop_GetStallTimer());
    TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, outB.status);
    TEST_ASSERT_FALSE(ClosedLoop_IsStalled());

    /* Advancing 140 ms (total unblanked = 190 ms < 200 ms persist threshold) -> still no stall */
    ClosedLoopOutput_t outC = ClosedLoop_Update(0.88f, 0.00f, 2.0f, 0.0f, 140U, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_UINT32(190, ClosedLoop_GetStallTimer());
    TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, outC.status);
    TEST_ASSERT_FALSE(ClosedLoop_IsStalled());

    /* Final 10 ms (total unblanked = 200 ms >= 200 ms persist threshold) -> confirms stall */
    ClosedLoopOutput_t outD = ClosedLoop_Update(0.90f, 0.00f, 2.0f, 0.0f, 10U, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_UINT32(200, ClosedLoop_GetStallTimer());
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STALL_DETECTED, outD.status);
    TEST_ASSERT_TRUE(ClosedLoop_IsStalled());
}

/**
 * @brief Test Case 12: Severe lag with normal velocity evaluates step skip compensation.
 *        Verifies that when position lag >= stallLagThresh_mm (e.g. 0.16 mm or 0.25 mm)
 *        but motor velocity is maintained (velocityRatio >= stallVelRatioThresh),
 *        the controller reports CL_STATUS_STEP_SKIP_DETECTED and emits clamped
 *        compensation steps, rather than falling through to CL_STATUS_NORMAL.
 */
void test_closed_loop_severe_lag_normal_velocity_step_skip_compensation(void) {
    /* [GIVEN] Default configuration */
    ClosedLoop_Init(NULL);

    /* Scenario A: Lag = 0.160 mm (> 0.030 mm stall threshold), cruise velocity = 2.0 mm/s.
     * Motor is moving at full speed (actVel = 2.0 mm/s, ratio = 1.0 >= 0.15). */
    const float cmdDistA = 5.000f;
    const float actDistA = 4.840f; /* Lag = 0.160 mm */
    const float cmdVel = 2.0f;
    const float actVelNorm = 2.0f;

    ClosedLoopOutput_t outA = ClosedLoop_Update(cmdDistA, actDistA, cmdVel, actVelNorm, TEST_DT_MS, TEST_STEPS_PER_MM);

    /* [THEN]
     * 1. Status MUST be CL_STATUS_STEP_SKIP_DETECTED (not NORMAL and not STALL).
     * 2. Position lag is 0.160 mm.
     * 3. Compensation steps = round(0.160 * 12800) = 2048 microsteps (exact within maxCompensationSteps).
     * 4. Stall timer must remain 0 and motor not stalled. */
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STEP_SKIP_DETECTED, outA.status);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.160f, outA.positionLag_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, outA.velocityRatio);
    TEST_ASSERT_EQUAL_UINT32(2048U, outA.compensationSteps);
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_GetStallTimer());
    TEST_ASSERT_FALSE(ClosedLoop_IsStalled());

    /* Scenario B: Lag = 0.250 mm (grossStallLag disabled = 0.0f), velocity is reduced but still above stall threshold
     * (e.g. 0.50 mm/s, ratio = 0.50 / 2.00 = 0.25 >= 0.15 stallVelRatioThresh).
     * Run for 15 cycles (300 ms). */
    float cmdDistB = 10.0f;
    float actDistB = 9.75f; /* Constant lag = 0.250 mm */
    const float partialVel = 0.50f;
    const float dtDist = partialVel * ((float)TEST_DT_MS / 1000.0f);

    for (int i = 0; i < 15; i++) {
        cmdDistB += dtDist;
        actDistB += dtDist;
        ClosedLoopOutput_t outB = ClosedLoop_Update(cmdDistB, actDistB, cmdVel, partialVel, TEST_DT_MS, TEST_STEPS_PER_MM);

        /* Because velocity ratio (0.25) >= 0.15 and grossStallLagThresh_mm is disabled (0.0f),
         * it is NOT a stall!
         * Status must remain STEP_SKIP_DETECTED and emit exact compensation steps (3200). */
        TEST_ASSERT_EQUAL_INT(CL_STATUS_STEP_SKIP_DETECTED, outB.status);
        TEST_ASSERT_EQUAL_UINT32(3200U, outB.compensationSteps);
        TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_GetStallTimer());
        TEST_ASSERT_FALSE(ClosedLoop_IsStalled());
    }
}

/**
 * @brief Test Case 12b: Moderate speed blockage stall detection (40 ms persistence).
 */
void test_closed_loop_tight_zero_velocity_stall_detection(void) {
    ClosedLoop_Init(NULL);
    ClosedLoop_ArmBlanking(0U); /* Disable blanking window for pure persistence timing */
    const float cmdVel = 0.20f; /* 0.20 mm/s moderate cruise speed -> 40 ms persist */
    const float actVel = 0.0f;

    /* [WHEN] Plunger hits bottom: lag accumulates, velocity is zero */
    /* At t = 20 ms, stall has not tripped yet (moderate persist = 40 ms) */
    ClosedLoopOutput_t out = ClosedLoop_Update(1.06f, 1.0f, cmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);
    TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, out.status);

    /* At t = 40 ms (2nd tick of 20 ms), blockage trip confirms stall! */
    out = ClosedLoop_Update(1.08f, 1.0f, cmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STALL_DETECTED, out.status);
    TEST_ASSERT_TRUE(ClosedLoop_IsStalled());
    TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);
}

/**
 * @brief Test Case 12d: High speed instant blockage stall detection (20 ms single-tick response).
 */
void test_closed_loop_high_speed_instant_stall_detection(void) {
    ClosedLoop_Init(NULL);
    ClosedLoop_ArmBlanking(0U);
    const float cmdVel = 1.66f; /* 1.66 mm/s high speed -> 20 ms single-tick trip */
    const float actVel = 0.0f;

    /* At t = 20 ms (tick 1), high-speed blockage confirms stall immediately to prevent tooth thumping */
    ClosedLoopOutput_t out = ClosedLoop_Update(1.06f, 1.0f, cmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STALL_DETECTED, out.status);
    TEST_ASSERT_TRUE(ClosedLoop_IsStalled());
    TEST_ASSERT_EQUAL_UINT32(0, out.compensationSteps);
}

/**
 * @brief Test Case 12c: Speed-adaptive stall persistence at ultra-slow microfluidic flow rates.
 *        Verifies that at 0.01 mm/s, persistence adapts to 200 ms to prevent false stalls
 *        due to encoder quantization.
 */
void test_closed_loop_speed_adaptive_stall_at_ultra_slow_speed(void) {
    ClosedLoop_Init(NULL);
    ClosedLoop_ArmBlanking(0U);
    const float slowCmdVel = 0.01f; /* 0.01 mm/s ultra-slow flow */
    const float actVel = 0.0f;
    const float actDist = 1.000f;
    float cmdDist = 1.060f; /* Initial lag = 0.060 mm > 0.030 mm stall lag threshold */

    /* For 0.01 mm/s, speedAdapted = 2.0 / 0.01 = 200 ms persistence.
     * At t = 80 ms (standard persist), it must NOT trip stall yet! */
    ClosedLoopOutput_t out;
    for (int t = 20; t < 200; t += 20) {
        cmdDist += slowCmdVel * ((float)TEST_DT_MS / 1000.0f);
        out = ClosedLoop_Update(cmdDist, actDist, slowCmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);
        TEST_ASSERT_NOT_EQUAL(CL_STATUS_STALL_DETECTED, out.status);
    }

    /* At t = 200 ms: confirmed blockage trips stall */
    cmdDist += slowCmdVel * ((float)TEST_DT_MS / 1000.0f);
    out = ClosedLoop_Update(cmdDist, actDist, slowCmdVel, actVel, TEST_DT_MS, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_INT(CL_STATUS_STALL_DETECTED, out.status);
    TEST_ASSERT_TRUE(ClosedLoop_IsStalled());
}

/**
 * @brief Test Case 13: Post-move creep step calculation within deadband returns zero.
 */
void test_closed_loop_creep_steps_within_deadband(void) {
    const float targetDist = 10.000f;
    const float actualDist = 9.985f; /* Residual lag = 0.015 mm <= 0.020 mm threshold */
    uint32_t creepSteps = ClosedLoop_CalcCreepSteps(targetDist, actualDist, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_UINT32(0, creepSteps);

    /* Boundary check: lag exactly at threshold (0.020 mm) */
    uint32_t creepBoundary = ClosedLoop_CalcCreepSteps(targetDist, 9.980f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_UINT32(0, creepBoundary);
}

/**
 * @brief Test Case 14: Post-move creep step calculation above deadband returns exact steps.
 */
void test_closed_loop_creep_steps_above_threshold_exact(void) {
    const float targetDist = 10.000f;
    const float actualDist = 9.960f; /* Residual lag = 0.040 mm > 0.020 mm threshold */
    /* 0.040 mm * 12800 steps/mm = exactly 512 microsteps */
    uint32_t creepSteps = ClosedLoop_CalcCreepSteps(targetDist, actualDist, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_UINT32(512, creepSteps);
}

/**
 * @brief Test Case 15: Post-move creep step calculation clamps to max ceiling.
 */
void test_closed_loop_creep_steps_clamping_to_max(void) {
    const float targetDist = 10.000f;
    const float actualDist = 9.800f; /* Residual lag = 0.200 mm -> 2560 steps */
    /* Clamp is 1280 steps */
    uint32_t creepSteps = ClosedLoop_CalcCreepSteps(targetDist, actualDist, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_UINT32(1280, creepSteps);
}

/**
 * @brief Test Case 16: Negative residual lag (overshoot) returns zero steps (forward only).
 */
void test_closed_loop_creep_steps_negative_lag_zero(void) {
    const float targetDist = 10.000f;
    const float actualDist = 10.030f; /* Overshoot by 0.030 mm */
    uint32_t creepSteps = ClosedLoop_CalcCreepSteps(targetDist, actualDist, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM);
    TEST_ASSERT_EQUAL_UINT32(0, creepSteps);
}

/**
 * @brief Test Case 17: Zero stepsPerMm returns zero steps safely.
 */
void test_closed_loop_creep_steps_zero_steps_per_mm(void) {
    const float targetDist = 10.000f;
    const float actualDist = 9.900f;
    uint32_t creepSteps = ClosedLoop_CalcCreepSteps(targetDist, actualDist, CL_DEFAULT_CREEP_LAG_MM, 1280U, 0U);
    TEST_ASSERT_EQUAL_UINT32(0, creepSteps);
}

/**
 * @brief Test Case 18: Extreme edge cases and adversarial boundary conditions for creep step calculation.
 *        Empirically verifies:
 *        - targetDist == actualDist (0.0f, 10.0f, 100.0f)
 *        - Tiny lag just at threshold (0.02000 mm and 0.02001 mm epsilon deadband)
 *        - Tiny lag just above threshold (0.02002 mm triggers 256 steps)
 *        - Huge lag (10.0 mm, 100.0 mm) clamped to maxCreepSteps
 *        - Zero stepsPerMm returns 0
 *        - Floating-point NaN, -INFINITY, +INFINITY, and overshoot safety
 */
void test_closed_loop_creep_steps_extreme_edge_cases(void) {
    /* 1. targetDist == actualDist (exact equality) */
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(0.0f, 0.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(10.0f, 10.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(100.0f, 100.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));

    /* 2. Tiny lag around threshold (0.02 mm deadband) */
    /* Exact deadband 0.02000 mm -> within threshold -> 0 */
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(10.0f, 9.98000f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    /* Boundary 0.02001 mm -> within epsilon (0.02 + 1e-5f) -> 0 */
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(10.0f, 9.97999f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    /* Just above epsilon 0.02002 mm -> 0.02002 * 12800 = 256.256 -> 256 microsteps */
    TEST_ASSERT_EQUAL_UINT32(256, ClosedLoop_CalcCreepSteps(10.0f, 9.97998f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));

    /* 3. Huge lag (10.0 mm and 100.0 mm) */
    /* 10.0 mm lag -> 128,000 steps -> clamped to maxCreepSteps (1280) */
    TEST_ASSERT_EQUAL_UINT32(1280, ClosedLoop_CalcCreepSteps(10.0f, 0.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    /* 100.0 mm lag -> 1,280,000 steps -> clamped to maxCreepSteps (1280) */
    TEST_ASSERT_EQUAL_UINT32(1280, ClosedLoop_CalcCreepSteps(100.0f, 0.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    /* Custom clamp ceiling: 256 steps */
    TEST_ASSERT_EQUAL_UINT32(256, ClosedLoop_CalcCreepSteps(10.0f, 0.0f, CL_DEFAULT_CREEP_LAG_MM, 256U, TEST_STEPS_PER_MM));

    /* 4. Zero stepsPerMm */
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(10.0f, 0.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, 0U));

    /* 5. Floating point rounding, NaN, and infinity edge cases */
#ifdef NAN
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(NAN, 5.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(5.0f, NAN, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
#endif
#ifdef INFINITY
    /* +INFINITY lag gets clamped to maxCreepSteps */
    TEST_ASSERT_EQUAL_UINT32(1280, ClosedLoop_CalcCreepSteps(INFINITY, 5.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    /* -INFINITY lag (massive overshoot) returns 0 */
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(-INFINITY, 5.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
#endif

    /* Overshoot (negative lag) must NEVER inject reverse creep */
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(5.0f, 5.001f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
    TEST_ASSERT_EQUAL_UINT32(0, ClosedLoop_CalcCreepSteps(5.0f, 15.0f, CL_DEFAULT_CREEP_LAG_MM, 1280U, TEST_STEPS_PER_MM));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_closed_loop_normal_tracking_zero_lag);
    RUN_TEST(test_closed_loop_deadband_filtering_below_threshold);
    RUN_TEST(test_closed_loop_step_skip_detection_and_exact_compensation);
    RUN_TEST(test_closed_loop_safety_clamping_on_compensation_steps);
    RUN_TEST(test_closed_loop_overshoot_negative_lag_zero_compensation);
    RUN_TEST(test_closed_loop_stall_detection_trip_after_persistence);
    RUN_TEST(test_closed_loop_transient_disturbance_rejection_no_stall);
    RUN_TEST(test_closed_loop_blanking_window_suppresses_stall_on_start);
    RUN_TEST(test_closed_loop_reset_and_resync_lifecycle);
    RUN_TEST(test_closed_loop_init_with_custom_config_and_null_fallback);
    RUN_TEST(test_closed_loop_zero_velocity_division_by_zero_safety);
    RUN_TEST(test_closed_loop_blanking_window_large_dt_no_leakage);
    RUN_TEST(test_closed_loop_severe_lag_normal_velocity_step_skip_compensation);
    RUN_TEST(test_closed_loop_tight_zero_velocity_stall_detection);
    RUN_TEST(test_closed_loop_high_speed_instant_stall_detection);
    RUN_TEST(test_closed_loop_speed_adaptive_stall_at_ultra_slow_speed);
    RUN_TEST(test_closed_loop_creep_steps_within_deadband);
    RUN_TEST(test_closed_loop_creep_steps_above_threshold_exact);
    RUN_TEST(test_closed_loop_creep_steps_clamping_to_max);
    RUN_TEST(test_closed_loop_creep_steps_negative_lag_zero);
    RUN_TEST(test_closed_loop_creep_steps_zero_steps_per_mm);
    RUN_TEST(test_closed_loop_creep_steps_extreme_edge_cases);
    return UNITY_END();
}
