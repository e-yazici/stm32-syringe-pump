/**
 * @file    test_stepper_profile.c
 * @brief   Unit tests for S-curve table, ARR calculation, and ISR step tick state machine.
 *          Structured in conventional Arrange-Act-Assert (Given-When-Then) style.
 */

#include "unity.h"
#include "stepper_profile.h"

#define TEST_TIMER_CLOCK 72000000U
#define TEST_PRESCALER   71U

static float s_curve_table[S_CURVE_TABLE_SIZE];

void setUp(void) {
    StepperProfile_GenerateSCurveTable(s_curve_table);
}

void tearDown(void) {}

void test_s_curve_table_boundaries_and_monotonicity(void) {
    /* [GIVEN] A generated 256-element normalized cosine S-curve lookup table */
    const float expected_first_element = 0.0f;
    const float expected_last_element = 1.0f;

    /* [WHEN] Inspecting table boundaries */
    float actual_first_element = s_curve_table[0];
    float actual_last_element = s_curve_table[255];

    /* [THEN] First must be 0.0, last must be 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_first_element, actual_first_element);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_last_element, actual_last_element);

    /* [THEN] Every subsequent element must be >= previous (monotonicity) */
    for (uint32_t i = 1; i < S_CURVE_TABLE_SIZE; i++) {
        TEST_ASSERT_TRUE(s_curve_table[i] >= s_curve_table[i - 1]);
    }
}

void test_arr_calculation(void) {
    /*
     * Effective timer tick rate = 72,000,000 / (71 + 1) = 1,000,000 Hz (1 MHz).
     * ARR formula: (timer_tick / freq) - 1.
     */

    /* Case 1: 1,000 Hz step frequency */
    const float input_freq_1k = 1000.0f;
    const uint32_t expected_arr_1k = 999; /* (1,000,000 / 1000) - 1 */
    uint32_t actual_arr_1k = StepperProfile_CalcFreqToARR(TEST_TIMER_CLOCK, TEST_PRESCALER, input_freq_1k);
    TEST_ASSERT_EQUAL_UINT32(expected_arr_1k, actual_arr_1k);

    /* Case 2: 10,000 Hz step frequency */
    const float input_freq_10k = 10000.0f;
    const uint32_t expected_arr_10k = 99; /* (1,000,000 / 10,000) - 1 */
    uint32_t actual_arr_10k = StepperProfile_CalcFreqToARR(TEST_TIMER_CLOCK, TEST_PRESCALER, input_freq_10k);
    TEST_ASSERT_EQUAL_UINT32(expected_arr_10k, actual_arr_10k);

    /* Case 3: 50,000 Hz step frequency */
    const float input_freq_50k = 50000.0f;
    const uint32_t expected_arr_50k = 19; /* (1,000,000 / 50,000) - 1 */
    uint32_t actual_arr_50k = StepperProfile_CalcFreqToARR(TEST_TIMER_CLOCK, TEST_PRESCALER, input_freq_50k);
    TEST_ASSERT_EQUAL_UINT32(expected_arr_50k, actual_arr_50k);
}

void test_full_trapezoidal_move_simulation(void) {
    /* [GIVEN] A programmed move of 1000 steps with 200 accel/decel ramp steps */
    const uint32_t input_total_steps = 1000;
    const uint32_t input_accel_steps = 200;
    const float input_start_freq = 1000.0f;
    const float input_cruise_freq = 10000.0f;

    StepperProfileState_t state;
    StepperProfile_PrepareMove(&state, input_cruise_freq, input_start_freq,
                               input_total_steps, input_accel_steps,
                               TEST_TIMER_CLOCK, TEST_PRESCALER, MOTOR_MODE_PUMP);

    /* Initial state verification */
    TEST_ASSERT_EQUAL_INT(MOTOR_STATE_ACCEL, state.MotorState);
    TEST_ASSERT_EQUAL_UINT32(input_total_steps, state.totalStepsRemaining);
    TEST_ASSERT_EQUAL_UINT32(input_accel_steps, state.accelSteps);

    /* [WHEN] Simulating timer interrupt step ticks */
    uint32_t actual_steps_executed = 0;
    StepperStepAction_e action = STEP_ACTION_CONTINUE;
    bool reached_cruise = false;
    bool reached_decel = false;

    while (action == STEP_ACTION_CONTINUE && actual_steps_executed < 2000) {
        if (state.MotorState == MOTOR_STATE_CRUISE) {
            reached_cruise = true;
        }
        if (state.MotorState == MOTOR_STATE_DECEL) {
            reached_decel = true;
        }

        action = StepperProfile_StepTick(&state, s_curve_table, TEST_TIMER_CLOCK, TEST_PRESCALER);
        actual_steps_executed++;
    }

    /* [THEN] Move completed through all 3 phases and reached IDLE on exact step count */
    TEST_ASSERT_TRUE(reached_cruise);
    TEST_ASSERT_TRUE(reached_decel);
    TEST_ASSERT_EQUAL_INT(STEP_ACTION_STOP_IDLE, action);
    TEST_ASSERT_EQUAL_INT(MOTOR_STATE_IDLE, state.MotorState);
    TEST_ASSERT_EQUAL_UINT32(0, state.totalStepsRemaining);
    TEST_ASSERT_EQUAL_UINT32(input_total_steps, actual_steps_executed);
}

void test_short_triangular_move_simulation(void) {
    /* [GIVEN] A short move of 150 steps requesting 100 accel steps (100 + 100 = 200 > 150) */
    const uint32_t input_total_steps = 150;
    const uint32_t input_requested_accel = 100;
    const uint32_t expected_clamped_accel = 75; /* totalSteps / 2 */

    StepperProfileState_t state;
    StepperProfile_PrepareMove(&state, 5000.0f, 1000.0f,
                               input_total_steps, input_requested_accel,
                               TEST_TIMER_CLOCK, TEST_PRESCALER, MOTOR_MODE_PUMP);

    /* [THEN] Acceleration steps must be clamped to 75 to eliminate cruise phase */
    TEST_ASSERT_EQUAL_UINT32(expected_clamped_accel, state.accelSteps);

    /* [WHEN] Simulating execution of the triangular move */
    uint32_t actual_steps_executed = 0;
    StepperStepAction_e action = STEP_ACTION_CONTINUE;

    while (action == STEP_ACTION_CONTINUE && actual_steps_executed < 500) {
        action = StepperProfile_StepTick(&state, s_curve_table, TEST_TIMER_CLOCK, TEST_PRESCALER);
        actual_steps_executed++;
    }

    /* [THEN] Move completes cleanly without underflow */
    TEST_ASSERT_EQUAL_INT(STEP_ACTION_STOP_IDLE, action);
    TEST_ASSERT_EQUAL_UINT32(input_total_steps, actual_steps_executed);
    TEST_ASSERT_EQUAL_UINT32(0, state.totalStepsRemaining);
}

void test_pause_during_cruise(void) {
    /* [GIVEN] A move running at steady cruise speed */
    StepperProfileState_t state;
    StepperProfile_PrepareMove(&state, 10000.0f, 1000.0f, 2000, 200,
                               TEST_TIMER_CLOCK, TEST_PRESCALER, MOTOR_MODE_PUMP);

    /* Advance into cruise phase */
    while (state.MotorState == MOTOR_STATE_ACCEL) {
        StepperProfile_StepTick(&state, s_curve_table, TEST_TIMER_CLOCK, TEST_PRESCALER);
    }
    TEST_ASSERT_EQUAL_INT(MOTOR_STATE_CRUISE, state.MotorState);

    /* [WHEN] User triggers a pause command (transitions to DECEL) */
    state.MotorState = MOTOR_STATE_DECEL;

    StepperStepAction_e action = STEP_ACTION_CONTINUE;
    uint32_t actual_decel_steps = 0;
    while (action == STEP_ACTION_CONTINUE && actual_decel_steps < 500) {
        action = StepperProfile_StepTick(&state, s_curve_table, TEST_TIMER_CLOCK, TEST_PRESCALER);
        actual_decel_steps++;
    }

    /* [THEN] Motor decelerates to a stop, enters PAUSED state, and steps remaining are preserved */
    TEST_ASSERT_EQUAL_INT(STEP_ACTION_STOP_PAUSED, action);
    TEST_ASSERT_EQUAL_INT(MOTOR_STATE_PAUSED, state.MotorState);
    TEST_ASSERT_TRUE(state.totalStepsRemaining > 0);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_s_curve_table_boundaries_and_monotonicity);
    RUN_TEST(test_arr_calculation);
    RUN_TEST(test_full_trapezoidal_move_simulation);
    RUN_TEST(test_short_triangular_move_simulation);
    RUN_TEST(test_pause_during_cruise);
    return UNITY_END();
}
