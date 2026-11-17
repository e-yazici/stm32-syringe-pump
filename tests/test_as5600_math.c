/**
 * @file    test_as5600_math.c
 * @brief   Unit tests for AS5600 angle odometry, multi-turn tracking, and EMA filter.
 *          Structured in conventional Arrange-Act-Assert (Given-When-Then) style.
 */

#include "unity.h"
#include "as5600_math.h"

static AS5600_Params_t params;
static uint8_t isBootInitialized;
static uint8_t filterInitialized;

void setUp(void) {
    AS5600_Math_InitParams(&params);
    isBootInitialized = 0;
    filterInitialized = 0;
}

void tearDown(void) {}

void test_assemble_raw_angle(void) {
    /* [GIVEN] High byte (0x0C, bits [3:0] valid) and low byte (0x0D) from I2C RX buffer */
    const uint8_t input_high_byte = 0x05;
    const uint8_t input_low_byte  = 0xA3;
    const uint16_t expected_raw   = 0x05A3;

    /* [WHEN] Assembling 12-bit raw angle */
    uint16_t actual_raw = AS5600_Math_AssembleRawAngle(input_high_byte, input_low_byte);

    /* [THEN] Upper nibble of high byte is masked out and 12-bit value is formed */
    TEST_ASSERT_EQUAL_UINT16(expected_raw, actual_raw);
    TEST_ASSERT_EQUAL_UINT16(0x0FFF, AS5600_Math_AssembleRawAngle(0xFF, 0xFF));
}

void test_raw_to_degrees(void) {
    /* [GIVEN] 12-bit raw integer counts */
    /* [WHEN] Converting to degrees (raw * 360.0 / 4096.0) */
    /* [THEN] Check exact proportional degree values */

    /* 0 counts -> 0.0° */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, AS5600_Math_RawToDegrees(0));

    /* 1024 counts (1/4 circle) -> 90.0° */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, AS5600_Math_RawToDegrees(1024));

    /* 2048 counts (1/2 circle) -> 180.0° */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, AS5600_Math_RawToDegrees(2048));

    /* 3072 counts (3/4 circle) -> 270.0° */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 270.0f, AS5600_Math_RawToDegrees(3072));

    /* 4095 counts (max 12-bit count) -> 359.912° */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 359.912f, AS5600_Math_RawToDegrees(4095));
}

void test_angle_correction_and_wrapping(void) {
    /* [GIVEN] A calibrated zero reference angle of 50.0° */
    const float input_start_angle = 50.0f;

    /* Case 1: Raw angle ahead of start angle (100° - 50° = 50°) */
    const float input_raw_ahead = 100.0f;
    const float expected_corrected_ahead = 50.0f;
    float actual_corrected_ahead = AS5600_Math_CorrectAngle(input_raw_ahead, input_start_angle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_corrected_ahead, actual_corrected_ahead);

    /* Case 2: Raw angle behind start angle past 0° boundary (30° - 50° = -20° -> wraps to 340°) */
    const float input_raw_wrap = 30.0f;
    const float expected_corrected_wrap = 340.0f;
    float actual_corrected_wrap = AS5600_Math_CorrectAngle(input_raw_wrap, input_start_angle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_corrected_wrap, actual_corrected_wrap);

    /* Case 3: Raw angle equal to start angle (50° - 50° = 0°) */
    const float input_raw_zero = 50.0f;
    const float expected_corrected_zero = 0.0f;
    float actual_corrected_zero = AS5600_Math_CorrectAngle(input_raw_zero, input_start_angle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_corrected_zero, actual_corrected_zero);
}

void test_quadrant_detection(void) {
    /* [GIVEN] Angles in each of the 4 90° quadrants */
    /* [WHEN] Determining quadrant */
    /* [THEN] Correct quadrant enum returned */
    TEST_ASSERT_EQUAL_INT(Quadrant_1, AS5600_Math_GetQuadrant(10.0f));
    TEST_ASSERT_EQUAL_INT(Quadrant_2, AS5600_Math_GetQuadrant(95.0f));
    TEST_ASSERT_EQUAL_INT(Quadrant_3, AS5600_Math_GetQuadrant(190.0f));
    TEST_ASSERT_EQUAL_INT(Quadrant_4, AS5600_Math_GetQuadrant(280.0f));
}

void test_clockwise_full_revolution_tracking(void) {
    /* [GIVEN] Shaft starts in Q1 (10°) */
    params.correctedAngle = 10.0f;
    AS5600_Math_CheckQuadrant(&params);
    TEST_ASSERT_EQUAL_INT32(0, params.totalTurns);

    /* [WHEN] Rotating CW through Q2 -> Q3 -> Q4 */
    params.correctedAngle = 120.0f;
    AS5600_Math_CheckQuadrant(&params);
    params.correctedAngle = 200.0f;
    AS5600_Math_CheckQuadrant(&params);
    params.correctedAngle = 300.0f;
    AS5600_Math_CheckQuadrant(&params);
    TEST_ASSERT_EQUAL_INT32(0, params.totalTurns);

    /* [WHEN] Crossing 360°/0° boundary from Q4 back into Q1 (10°) */
    params.correctedAngle = 10.0f;
    AS5600_Math_CheckQuadrant(&params);

    /* [THEN] +1 complete revolution accumulated (totalAngle = 360 + 10 = 370°) */
    TEST_ASSERT_EQUAL_INT32(1, params.totalTurns);
    TEST_ASSERT_EQUAL_INT32(370, params.totalAngle);

    /* [WHEN] Completing a 2nd clockwise revolution */
    params.correctedAngle = 120.0f;
    AS5600_Math_CheckQuadrant(&params);
    params.correctedAngle = 200.0f;
    AS5600_Math_CheckQuadrant(&params);
    params.correctedAngle = 300.0f;
    AS5600_Math_CheckQuadrant(&params);
    params.correctedAngle = 10.0f;
    AS5600_Math_CheckQuadrant(&params);

    /* [THEN] Total turns = 2, total angle = 730° */
    TEST_ASSERT_EQUAL_INT32(2, params.totalTurns);
    TEST_ASSERT_EQUAL_INT32(730, params.totalAngle);
}

void test_counter_clockwise_full_revolution_tracking(void) {
    /* [GIVEN] Shaft starts in Q1 (10°) */
    params.correctedAngle = 10.0f;
    AS5600_Math_CheckQuadrant(&params);
    TEST_ASSERT_EQUAL_INT32(0, params.totalTurns);

    /* [WHEN] Crossing CCW backward past 0° boundary from Q1 into Q4 (350°) */
    params.correctedAngle = 350.0f;
    AS5600_Math_CheckQuadrant(&params);

    /* [THEN] -1 complete revolution detected (totalAngle = -360 + 350 = -10°) */
    TEST_ASSERT_EQUAL_INT32(-1, params.totalTurns);
    TEST_ASSERT_EQUAL_INT32(-10, params.totalAngle);
}

void test_hardware_pumping_5mm_900deg_cw(void) {
    /* [GIVEN] 2.0 mm/rev lead screw, initial arbitrary magnet angle at boot (e.g. 45.0°) */
    const float input_screw_lead_mm = 2.0f;
    const float expected_distance_mm = 5.0f;
    const int32_t expected_total_angle_deg = 900; /* 2.5 revolutions CW */

    uint16_t initial_raw = (uint16_t)((45.0f / 360.0f) * 4096.0f);
    AS5600_Math_ProcessSample(&params, initial_raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, params.travelledDistance_mm);
    TEST_ASSERT_EQUAL_INT32(0, params.totalAngle);

    /* [WHEN] Pumping 5.0 mm: 2.5 CW revolutions (900°) in 10° steps */
    float current_deg = 45.0f;
    for (int step = 0; step < 90; step++) {
        current_deg += 10.0f;
        if (current_deg >= 360.0f) {
            current_deg -= 360.0f;
        }
        uint16_t raw = (uint16_t)((current_deg / 360.0f) * 4096.0f);
        AS5600_Math_ProcessSample(&params, raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    }

    /* Allow EMA filter to settle at end position */
    uint16_t final_raw = (uint16_t)((current_deg / 360.0f) * 4096.0f);
    for (int i = 0; i < 20; i++) {
        AS5600_Math_ProcessSample(&params, final_raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    }

    /* [THEN] Total turns = 2, totalAngle = 900°, and travelledDistance_mm = 5.00 mm */
    TEST_ASSERT_EQUAL_INT32(2, params.totalTurns);
    TEST_ASSERT_INT_WITHIN(1, expected_total_angle_deg, params.totalAngle);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_distance_mm, params.travelledDistance_mm);
}

void test_linear_displacement_from_revolutions(void) {
    /* [GIVEN] 2.0 mm/rev lead screw pitch */
    const float input_screw_lead_mm = 2.0f;
    const float expected_linear_distance_mm = 10.0f; /* 5 revs * 2.0 mm/rev */

    /* [WHEN] Simulating 5 full 360° revolutions */
    for (int rev = 0; rev < 5; rev++) {
        for (int deg = 0; deg < 360; deg += 30) {
            uint16_t raw = (uint16_t)(((float)deg / 360.0f) * 4096.0f);
            AS5600_Math_ProcessSample(&params, raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
        }
    }
    /* Feed final 0° sample to close the 5th revolution */
    AS5600_Math_ProcessSample(&params, 0, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);

    /* Allow EMA filter to settle at steady state */
    for (int i = 0; i < 20; i++) {
        AS5600_Math_ProcessSample(&params, 0, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    }

    /* [THEN] Total linear displacement equals 10.00 mm */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_linear_distance_mm, params.travelledDistance_mm);
}

void test_boot_jitter_across_zero_reference_and_5mm_pump(void) {
    /* [GIVEN] Magnet boots at an arbitrary physical angle (273.5° -> raw = 3111)
     *         and experiences ±2 LSB sensor noise across the zero-offset boundary
     *         before the motor starts moving. */
    const float input_screw_lead_mm = 2.0f;
    const uint16_t boot_raw = 3111; /* ~273.43° */

    /* Boot sample locks startAngle */
    AS5600_Math_ProcessSample(&params, boot_raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    TEST_ASSERT_EQUAL_INT32(0, params.totalTurns);
    TEST_ASSERT_EQUAL_INT32(0, params.totalAngle);

    /* [WHEN] Idle ADC/magnetic jitter fluctuates -2, +1, -1, 0 LSB around boot_raw
     *        (crossing back and forth between 359.8° Q4 and 0.1° Q1) */
    AS5600_Math_ProcessSample(&params, boot_raw - 2, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    AS5600_Math_ProcessSample(&params, boot_raw + 1, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    AS5600_Math_ProcessSample(&params, boot_raw - 1, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    AS5600_Math_ProcessSample(&params, boot_raw,     &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);

    /* [THEN] Net turns and net angle at rest must still be 0 (no permanent false turn accumulation) */
    TEST_ASSERT_EQUAL_INT32(0, params.totalTurns);
    TEST_ASSERT_EQUAL_INT32(0, params.totalAngle);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, params.travelledDistance_mm);

    /* [WHEN] Now executing a 5.0 mm (900° = 2.5 revolutions = 10240 raw counts) CW pump from boot_raw */
    for (uint32_t step = 1; step <= 160; step++) {
        /* 10240 counts / 160 samples = 64 counts (5.625°) per 10ms sample */
        uint16_t raw = (uint16_t)((boot_raw + (step * 64U)) & 0x0FFFU);
        AS5600_Math_ProcessSample(&params, raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    }

    /* Settle EMA filter at final position (boot_raw + 2048 counts = +180° offset + 2 turns = 900°) */
    uint16_t final_raw = (uint16_t)((boot_raw + 2048U) & 0x0FFFU);
    for (int i = 0; i < 25; i++) {
        AS5600_Math_ProcessSample(&params, final_raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    }

    /* [THEN] Exact 2 turns, 900°, and 5.00 mm travelled distance */
    TEST_ASSERT_EQUAL_INT32(2, params.totalTurns);
    TEST_ASSERT_INT_WITHIN(1, 900, params.totalAngle);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 5.0f, params.travelledDistance_mm);
}

void test_reverse_5mm_900deg_ccw_pump(void) {
    /* [GIVEN] 2.0 mm/rev lead screw, starting at arbitrary raw angle (3000) */
    const float input_screw_lead_mm = 2.0f;
    const uint16_t boot_raw = 3000;

    AS5600_Math_ProcessSample(&params, boot_raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);

    /* [WHEN] Rotating 900° (2.5 revolutions = 10240 counts) CCW (backward) */
    for (uint32_t step = 1; step <= 160; step++) {
        uint16_t raw = (uint16_t)((boot_raw - (step * 64U)) & 0x0FFFU);
        AS5600_Math_ProcessSample(&params, raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    }

    uint16_t final_raw = (uint16_t)((boot_raw - 2048U) & 0x0FFFU);
    for (int i = 0; i < 25; i++) {
        AS5600_Math_ProcessSample(&params, final_raw, &isBootInitialized, &filterInitialized, input_screw_lead_mm, 0.01f);
    }

    /* [THEN] -3 turns + 180° correctedAngle = -900° -> -5.00 mm */
    TEST_ASSERT_EQUAL_INT32(-3, params.totalTurns);
    TEST_ASSERT_INT_WITHIN(1, -900, params.totalAngle);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -5.0f, params.travelledDistance_mm);
}

void test_init_params_and_null_safety(void) {
    /* [GIVEN] A params structure populated with non-zero values */
    params.rawAngle = 1234;
    params.degreeAngle = 45.0f;
    params.totalTurns = 5;
    params.travelledDistance_mm = 10.0f;

    /* [WHEN] Initializing params */
    AS5600_Math_InitParams(&params);

    /* [THEN] All fields are reset to zero */
    TEST_ASSERT_EQUAL_UINT16(0, params.rawAngle);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, params.degreeAngle);
    TEST_ASSERT_EQUAL_INT32(0, params.totalTurns);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, params.travelledDistance_mm);

    /* [WHEN/THEN] NULL pointer safety calls should not crash */
    AS5600_Math_InitParams(NULL);
    AS5600_Math_CheckQuadrant(NULL);
    AS5600_Math_FilterAndCalcDistance(NULL, NULL, 2.0f, 0.01f);
    AS5600_Math_ProcessSample(NULL, 0, NULL, NULL, 2.0f, 0.01f);
}

void test_default_constants_and_process_sample_with_defaults(void) {
    /* [GIVEN] Verifying defined default hardware constants */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, AS5600_DEFAULT_SCREW_LEAD_MM);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.01f, AS5600_SAMPLE_DT_SEC);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, AS5600_EMA_ALPHA);

    /* [WHEN] Running samples through pipeline using default constants */
    AS5600_Math_ProcessSample(&params, 0, &isBootInitialized, &filterInitialized,
                              AS5600_DEFAULT_SCREW_LEAD_MM, AS5600_SAMPLE_DT_SEC);
    TEST_ASSERT_EQUAL_UINT8(1, isBootInitialized);
    TEST_ASSERT_EQUAL_UINT8(1, filterInitialized);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, params.travelledDistance_mm);
}

void test_sub_degree_float_resolution_across_wraps(void) {
    /* [GIVEN] Boot at raw count 0 */
    AS5600_Math_ProcessSample(&params, 0, &isBootInitialized, &filterInitialized, 2.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, params.totalAngle);

    /* [WHEN] Increment by 1 raw count: 1 * (360 / 4096) = 0.087890625 deg */
    AS5600_Math_ProcessSample(&params, 1, &isBootInitialized, &filterInitialized, 2.0f, 0.01f);

    /* [THEN] Full float resolution is preserved without integer truncation */
    const float expected_lsb_deg = 360.0f / 4096.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, expected_lsb_deg, params.totalAngle);

    /* [WHEN] Advancing through Q2, Q3, Q4 and wrapping into turn 1 at count 1 */
    AS5600_Math_ProcessSample(&params, 1024, &isBootInitialized, &filterInitialized, 2.0f, 0.01f);
    AS5600_Math_ProcessSample(&params, 2048, &isBootInitialized, &filterInitialized, 2.0f, 0.01f);
    AS5600_Math_ProcessSample(&params, 4095, &isBootInitialized, &filterInitialized, 2.0f, 0.01f);
    AS5600_Math_ProcessSample(&params, 0, &isBootInitialized, &filterInitialized, 2.0f, 0.01f);
    AS5600_Math_ProcessSample(&params, 1, &isBootInitialized, &filterInitialized, 2.0f, 0.01f);

    /* [THEN] Multi-turn totalAngle preserves sub-degree precision (360.08789 deg) */
    TEST_ASSERT_EQUAL_INT32(1, params.totalTurns);
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 360.0f + expected_lsb_deg, params.totalAngle);
}

void test_dynamic_ema_alpha_small_and_large_error(void) {
    /* [GIVEN] Seeded EMA filter at 100.0 deg */
    filterInitialized = 1U;
    params.filteredTotalAngle = 100.0f;
    params.totalAngle = 100.04f; /* |error| = 0.04 < AS5600_DYNAMIC_EMA_THRESH_DEG (0.05) */

    /* [WHEN] Filtering with small error below dynamic threshold */
    AS5600_Math_FilterAndCalcDistance(&params, &filterInitialized, AS5600_DEFAULT_SCREW_LEAD_MM, AS5600_SAMPLE_DT_SEC);

    /* [THEN] dynamic_alpha snaps to 1.0f and filteredTotalAngle matches totalAngle immediately */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 100.04f, params.filteredTotalAngle);

    /* [GIVEN] Large error >= 0.05 deg (10.0 deg step from 100.0 to 110.0) */
    params.filteredTotalAngle = 100.0f;
    params.totalAngle = 110.0f;

    /* [WHEN] Filtering with large error */
    AS5600_Math_FilterAndCalcDistance(&params, &filterInitialized, AS5600_DEFAULT_SCREW_LEAD_MM, AS5600_SAMPLE_DT_SEC);

    /* [THEN] filteredTotalAngle steps by AS5600_EMA_ALPHA * 10.0f = 2.0f */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 102.0f, params.filteredTotalAngle);
}

void test_velocity_ema_step_response_and_decay(void) {
    /* [GIVEN] Seeded filter at 0.0 deg / 0.0 mm / 0.0 mm/s */
    filterInitialized = 1U;
    params.filteredTotalAngle = 0.0f;
    params.travelledDistance_mm = 0.0f;
    params.currentVelocity_mm_s = 0.0f;

    /* [WHEN] Applying a 360.0 deg step over dt = 0.01 s with 2.0 mm/rev screw lead */
    params.totalAngle = 360.0f;
    AS5600_Math_FilterAndCalcDistance(&params, &filterInitialized, AS5600_DEFAULT_SCREW_LEAD_MM, AS5600_SAMPLE_DT_SEC);

    /* filteredTotalAngle = 72.0 deg -> distance = 0.4 mm -> raw_vel = 40.0 mm/s
     * currentVelocity_mm_s = 0.33 * 40.0 = 13.2 mm/s */
    const float expected_vel_step = AS5600_VEL_EMA_ALPHA * 40.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_vel_step, params.currentVelocity_mm_s);

    /* [WHEN] Displacement stops (totalAngle equals filteredTotalAngle -> raw_vel = 0.0 mm/s) */
    params.totalAngle = params.filteredTotalAngle;
    AS5600_Math_FilterAndCalcDistance(&params, &filterInitialized, AS5600_DEFAULT_SCREW_LEAD_MM, AS5600_SAMPLE_DT_SEC);

    /* [THEN] Velocity decays by (1.0f - AS5600_VEL_EMA_ALPHA) */
    const float expected_vel_decay = (1.0f - AS5600_VEL_EMA_ALPHA) * expected_vel_step;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_vel_decay, params.currentVelocity_mm_s);
}

void test_stationary_snap_behavior(void) {
    /* [GIVEN] Motor is NOT stationary, with lag > 0.18 deg */
    params.totalAngle = 100.50f;
    params.filteredTotalAngle = 100.00f;
    params.travelledDistance_mm = (100.00f / 360.0f) * AS5600_DEFAULT_SCREW_LEAD_MM;
    params.currentVelocity_mm_s = 1.2f;

    /* [WHEN] Calling SnapIfStationary with motorStationary == false */
    bool snapped = AS5600_Math_SnapIfStationary(&params, false, AS5600_STATIONARY_SNAP_DEG, AS5600_DEFAULT_SCREW_LEAD_MM);

    /* [THEN] Returns false and leaves state untouched */
    TEST_ASSERT_FALSE(snapped);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 100.00f, params.filteredTotalAngle);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.2f, params.currentVelocity_mm_s);

    /* [GIVEN] Motor IS stationary, error <= 0.18 deg (0.15 deg and exact 0.18 deg boundary) */
    params.filteredTotalAngle = 0.0f;
    params.totalAngle = 0.15f;
    TEST_ASSERT_FALSE(AS5600_Math_SnapIfStationary(&params, true, AS5600_STATIONARY_SNAP_DEG, AS5600_DEFAULT_SCREW_LEAD_MM));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, params.filteredTotalAngle);

    params.totalAngle = AS5600_STATIONARY_SNAP_DEG;
    TEST_ASSERT_FALSE(AS5600_Math_SnapIfStationary(&params, true, AS5600_STATIONARY_SNAP_DEG, AS5600_DEFAULT_SCREW_LEAD_MM));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, params.filteredTotalAngle);

    /* [GIVEN] Motor IS stationary, error > 0.18 deg (0.25 deg) */
    params.filteredTotalAngle = 180.00f;
    params.totalAngle = 180.25f;
    params.currentVelocity_mm_s = 1.5f;

    /* [WHEN] Calling SnapIfStationary */
    snapped = AS5600_Math_SnapIfStationary(&params, true, AS5600_STATIONARY_SNAP_DEG, AS5600_DEFAULT_SCREW_LEAD_MM);

    /* [THEN] Snaps filteredTotalAngle, recalculates distance, zeroes velocity, returns true */
    TEST_ASSERT_TRUE(snapped);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 180.25f, params.filteredTotalAngle);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, (180.25f / 360.0f) * AS5600_DEFAULT_SCREW_LEAD_MM, params.travelledDistance_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, params.currentVelocity_mm_s);

    /* [THEN] NULL pointer safety */
    TEST_ASSERT_FALSE(AS5600_Math_SnapIfStationary(NULL, true, AS5600_STATIONARY_SNAP_DEG, AS5600_DEFAULT_SCREW_LEAD_MM));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_assemble_raw_angle);
    RUN_TEST(test_raw_to_degrees);
    RUN_TEST(test_angle_correction_and_wrapping);
    RUN_TEST(test_quadrant_detection);
    RUN_TEST(test_clockwise_full_revolution_tracking);
    RUN_TEST(test_counter_clockwise_full_revolution_tracking);
    RUN_TEST(test_hardware_pumping_5mm_900deg_cw);
    RUN_TEST(test_linear_displacement_from_revolutions);
    RUN_TEST(test_boot_jitter_across_zero_reference_and_5mm_pump);
    RUN_TEST(test_reverse_5mm_900deg_ccw_pump);
    RUN_TEST(test_init_params_and_null_safety);
    RUN_TEST(test_default_constants_and_process_sample_with_defaults);
    RUN_TEST(test_sub_degree_float_resolution_across_wraps);
    RUN_TEST(test_dynamic_ema_alpha_small_and_large_error);
    RUN_TEST(test_velocity_ema_step_response_and_decay);
    RUN_TEST(test_stationary_snap_behavior);
    return UNITY_END();
}
