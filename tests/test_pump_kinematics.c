/**
 * @file    test_pump_kinematics.c
 * @brief   Unit tests for pump kinematics, volumetric conversions, and calibration.
 *          Structured in conventional Arrange-Act-Assert (Given-When-Then) style.
 */

#include "unity.h"
#include "pump_kinematics.h"

static PumpHwConfig_t defaultHw = {
    .screwLead_mm_per_rev = 2.0f,
    .motorStepsPerRev = 200,
    .microstepping = 128
};

void setUp(void) {}
void tearDown(void) {}

void test_volume_conversion(void) {
    /* [GIVEN] Volume in mL */
    const float input_ml = 1.5f;
    const float expected_mm3_from_ml = 1500.0f;

    /* [WHEN] Converting mL to mm³ */
    float actual_mm3_from_ml = pump_vol_to_mm3(input_ml, PUMP_VOL_UNIT_ML);

    /* [THEN] 1.5 mL should equal 1500.0 mm³ */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected_mm3_from_ml, actual_mm3_from_ml);

    /* [GIVEN] Volume in µL */
    const float input_ul = 250.0f;
    const float expected_mm3_from_ul = 250.0f;

    /* [WHEN] Converting µL to mm³ */
    float actual_mm3_from_ul = pump_vol_to_mm3(input_ul, PUMP_VOL_UNIT_UL);

    /* [THEN] 250 µL should equal 250.0 mm³ */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected_mm3_from_ul, actual_mm3_from_ul);
}

void test_flow_rate_conversion(void) {
    /* Case 1: 60 mL/min -> 1000 mm³/s */
    const float input_ml_min = 60.0f;
    const float expected_from_ml_min = 1000.0f;
    float actual_from_ml_min = pump_flow_to_mm3_sec(input_ml_min, PUMP_FLOW_UNIT_ML_MIN);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_from_ml_min, actual_from_ml_min);

    /* Case 2: 60 µL/min -> 1 mm³/s */
    const float input_ul_min = 60.0f;
    const float expected_from_ul_min = 1.0f;
    float actual_from_ul_min = pump_flow_to_mm3_sec(input_ul_min, PUMP_FLOW_UNIT_UL_MIN);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_from_ul_min, actual_from_ul_min);

    /* Case 3: 3600 mL/hr -> 1000 mm³/s */
    const float input_ml_hr = 3600.0f;
    const float expected_from_ml_hr = 1000.0f;
    float actual_from_ml_hr = pump_flow_to_mm3_sec(input_ml_hr, PUMP_FLOW_UNIT_ML_HR);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_from_ml_hr, actual_from_ml_hr);

    /* Case 4: 3600 µL/hr -> 1 mm³/s */
    const float input_ul_hr = 3600.0f;
    const float expected_from_ul_hr = 1.0f;
    float actual_from_ul_hr = pump_flow_to_mm3_sec(input_ul_hr, PUMP_FLOW_UNIT_UL_HR);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_from_ul_hr, actual_from_ul_hr);
}

void test_flow_rate_to_ml_min(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, pump_flow_to_ml_min(5.0f, PUMP_FLOW_UNIT_ML_MIN));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pump_flow_to_ml_min(1000.0f, PUMP_FLOW_UNIT_UL_MIN));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pump_flow_to_ml_min(60.0f, PUMP_FLOW_UNIT_ML_HR));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pump_flow_to_ml_min(60000.0f, PUMP_FLOW_UNIT_UL_HR));
}

void test_syringe_cross_section_area(void) {
    /* [GIVEN] A 10.0 mm inner diameter syringe */
    const float input_diameter_mm = 10.0f;
    const float expected_area_mm2 = 78.5398f; /* A = π * (10/2)² */

    /* [WHEN] Calculating cross-sectional area */
    float actual_area_mm2 = pump_calc_cross_section_area(input_diameter_mm);

    /* [THEN] Area should match expected */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_area_mm2, actual_area_mm2);

    /* Edge cases: zero or negative diameter */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_cross_section_area(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_cross_section_area(-5.0f));
}

void test_steps_per_mm(void) {
    /* [GIVEN] 200 steps/rev, 128 microstepping, 2.0 mm screw pitch */
    const float expected_steps_per_mm = 12800.0f; /* (200 * 128) / 2.0 */

    /* [WHEN] Computing steps per mm */
    float actual_steps_per_mm = pump_calc_steps_per_mm(&defaultHw);

    /* [THEN] Should match expected */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_steps_per_mm, actual_steps_per_mm);

    /* Edge cases: NULL hardware config or zero lead */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_steps_per_mm(NULL));
    PumpHwConfig_t zeroLeadHw = { .screwLead_mm_per_rev = 0.0f, .motorStepsPerRev = 200, .microstepping = 128 };
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_steps_per_mm(&zeroLeadHw));
}

void test_linear_travel_and_speed(void) {
    /* 1500 mm³ in 150 mm² area -> 10.0 mm */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, pump_calc_linear_travel(1500.0f, 150.0f));
    /* Edge case: zero or negative area */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_linear_travel(1500.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_linear_travel(1500.0f, -10.0f));

    /* 75 mm³/s in 150 mm² area -> 0.5 mm/s */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pump_calc_linear_speed(75.0f, 150.0f));
    /* Edge case: zero or negative area */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_linear_speed(75.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_linear_speed(75.0f, -10.0f));
}

void test_full_kinematics_pipeline(void) {
    /* [GIVEN] Inputs for a 1.0 mL dispense at 5.0 mL/min with a 14.5 mm diameter syringe */
    const float input_volume = 1.0f;
    const PumpVolumeUnit_e input_vol_unit = PUMP_VOL_UNIT_ML;
    const float input_flow_rate = 5.0f;
    const PumpFlowUnit_e input_flow_unit = PUMP_FLOW_UNIT_ML_MIN;
    const float input_diameter_mm = 14.5f;

    /* [EXPECTED] */
    const float expected_area_mm2 = 165.13f;
    const float expected_linear_travel_mm = 6.0558f;
    const uint32_t expected_total_steps = 77514;
    const float expected_linear_speed_mm_s = 0.50465f;
    const float expected_cruise_hz = 6459.5f;
    const uint32_t expected_accel_steps = 15000; /* Clamped to max */

    PumpKinematics_t actual_kinematics;

    /* [WHEN] Running full kinematics pipeline */
    bool ok = pump_calc_kinematics(input_volume, input_vol_unit,
                                   input_flow_rate, input_flow_unit,
                                   input_diameter_mm, &defaultHw,
                                   1000.0f, 100, 15000,
                                   &actual_kinematics);

    /* [THEN] All calculated parameters match theoretical physical values */
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_area_mm2, actual_kinematics.area_mm2);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_linear_travel_mm, actual_kinematics.linearTravel_mm);
    TEST_ASSERT_UINT32_WITHIN(200, expected_total_steps, actual_kinematics.totalSteps);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_linear_speed_mm_s, actual_kinematics.linearSpeed_mm_sec);
    TEST_ASSERT_FLOAT_WITHIN(50.0f, expected_cruise_hz, actual_kinematics.cruiseFreqHz);
    TEST_ASSERT_EQUAL_UINT32(expected_accel_steps, actual_kinematics.accelSteps);
}

void test_triangular_profile_clamping(void) {
    /* [GIVEN] A short move where totalSteps (200) < 2 * requestedAccel (15000) */
    const uint32_t input_total_steps = 200;
    const uint32_t expected_clamped_accel = 100; /* totalSteps / 2 */

    /* [WHEN] Calculating acceleration steps */
    uint32_t actual_accel = pump_calc_accel_steps(5000.0f, 1000.0f, 100, 15000, input_total_steps);

    /* [THEN] Must clamp to triangular profile */
    TEST_ASSERT_EQUAL_UINT32(expected_clamped_accel, actual_accel);
}

void test_diameter_from_weight_calibration(void) {
    /* [GIVEN] 1.6513 grams of water delivered across 10.0 mm calibration travel with Z = 1.0 */
    const float input_weight_g = 1.6513f;
    const float input_z_factor = 1.0f;
    const float input_distance_mm = 10.0f;
    const float expected_diameter_mm = 14.50f;

    /* [WHEN] Deducing inner diameter */
    float actual_diameter_mm = pump_calc_diameter_from_weight(input_weight_g, input_z_factor, input_distance_mm);

    /* [THEN] Deduced diameter must match 14.5 mm */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_diameter_mm, actual_diameter_mm);

    /* [GIVEN] ISO 8655 Z-factor at 20 deg C (Z = 1.0029 uL/mg): 1.646525 g * 1.0029 = 1.6513 mL */
    const float iso_weight_g = 1.646525f;
    const float iso_z_factor = 1.0029f;
    float iso_diameter_mm = pump_calc_diameter_from_weight(iso_weight_g, iso_z_factor, input_distance_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_diameter_mm, iso_diameter_mm);

    /* Edge cases */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_diameter_from_weight(0.0f, 1.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_diameter_from_weight(1.0f, 0.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pump_calc_diameter_from_weight(1.0f, 1.0f, 0.0f));
}

void test_live_pumped_volume_and_remaining_time(void) {
    /* [GIVEN] 100 mm² area, 10,000 steps/mm, 5,000 steps executed */
    const uint32_t input_steps_done = 5000;
    const float expected_volume_ml = 0.05f;

    /* [WHEN] Calculating live pumped volume */
    float actual_volume_ml = pump_calc_pumped_volume_ml(input_steps_done, 10000.0f, 100.0f);

    /* [THEN] */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_volume_ml, actual_volume_ml);

    /* [GIVEN] 2.0 mL remaining at 2.0 mL/min */
    const float input_remaining_ml = 2.0f;
    const float input_flow_ml_min = 2.0f;
    const float expected_seconds = 60.0f;

    /* [WHEN] Calculating remaining time */
    float actual_seconds = pump_calc_remaining_time_sec(input_remaining_ml, input_flow_ml_min);

    /* [THEN] */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_seconds, actual_seconds);
}

void test_relative_travel_calculation(void) {
    /* [GIVEN] Forward direction: current > baseline */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.5f, pump_calc_relative_travel_mm(13.5f, 10.0f, true));
    /* Forward direction: negative relative travel (backlash/jitter) clamped to 0.0f */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pump_calc_relative_travel_mm(9.8f, 10.0f, true));

    /* [GIVEN] Backward direction: current < baseline */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 4.2f, pump_calc_relative_travel_mm(5.8f, 10.0f, false));
    /* Backward direction: negative relative travel (current > baseline) clamped to 0.0f */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pump_calc_relative_travel_mm(10.3f, 10.0f, false));
}

void test_remaining_steps_reanchoring(void) {
    const float stepsPerMm = 12800.0f;

    /* [GIVEN] Partial progress: 10.0 mm target, 2.5 mm actual -> 7.5 mm * 12800 = 96000 steps */
    TEST_ASSERT_EQUAL_UINT32(96000U, pump_calc_remaining_steps(10.0f, 2.5f, stepsPerMm));

    /* [GIVEN] Negative actualTravel_mm is clamped to 0 -> full 10.0 mm * 12800 = 128000 steps */
    TEST_ASSERT_EQUAL_UINT32(128000U, pump_calc_remaining_steps(10.0f, -1.0f, stepsPerMm));

    /* [GIVEN] Target exactly reached or exceeded -> clamped to 0 steps */
    TEST_ASSERT_EQUAL_UINT32(0U, pump_calc_remaining_steps(10.0f, 10.0f, stepsPerMm));
    TEST_ASSERT_EQUAL_UINT32(0U, pump_calc_remaining_steps(10.0f, 10.5f, stepsPerMm));

    /* [GIVEN] Invalid stepsPerMm <= 0 -> returns 0 safely */
    TEST_ASSERT_EQUAL_UINT32(0U, pump_calc_remaining_steps(10.0f, 2.0f, 0.0f));
    TEST_ASSERT_EQUAL_UINT32(0U, pump_calc_remaining_steps(10.0f, 2.0f, -100.0f));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_volume_conversion);
    RUN_TEST(test_flow_rate_conversion);
    RUN_TEST(test_flow_rate_to_ml_min);
    RUN_TEST(test_syringe_cross_section_area);
    RUN_TEST(test_steps_per_mm);
    RUN_TEST(test_linear_travel_and_speed);
    RUN_TEST(test_full_kinematics_pipeline);
    RUN_TEST(test_triangular_profile_clamping);
    RUN_TEST(test_diameter_from_weight_calibration);
    RUN_TEST(test_live_pumped_volume_and_remaining_time);
    RUN_TEST(test_relative_travel_calculation);
    RUN_TEST(test_remaining_steps_reanchoring);
    return UNITY_END();
}
