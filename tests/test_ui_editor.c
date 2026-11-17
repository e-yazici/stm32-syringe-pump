/**
 * @file    test_ui_editor.c
 * @brief   Unit tests for UI numeric cursor editing and bounds clamping.
 *          Structured in conventional Arrange-Act-Assert (Given-When-Then) style.
 */

#include "unity.h"
#include "ui_editor.h"

void setUp(void) {}
void tearDown(void) {}

void test_cursor_navigation(void) {
    /* [GIVEN] Cursor initialized at index 2 (ones place) */
    uint8_t actual_cursor = 2;
    const uint8_t input_max_cursor = 4;
    const uint8_t input_min_cursor = 0;

    /* [WHEN] Moving cursor left within bounds */
    bool moved_left_1 = UIEditor_CursorLeft(&actual_cursor, input_max_cursor);
    TEST_ASSERT_TRUE(moved_left_1);
    TEST_ASSERT_EQUAL_UINT8(3, actual_cursor);

    bool moved_left_2 = UIEditor_CursorLeft(&actual_cursor, input_max_cursor);
    TEST_ASSERT_TRUE(moved_left_2);
    TEST_ASSERT_EQUAL_UINT8(4, actual_cursor);

    /* [WHEN] Attempting to move past left boundary */
    bool moved_left_overflow = UIEditor_CursorLeft(&actual_cursor, input_max_cursor);

    /* [THEN] Boundary respected, cursor held at max */
    TEST_ASSERT_FALSE(moved_left_overflow);
    TEST_ASSERT_EQUAL_UINT8(4, actual_cursor);

    /* [WHEN] Moving cursor right down to min boundary */
    TEST_ASSERT_TRUE(UIEditor_CursorRight(&actual_cursor, input_min_cursor)); /* -> 3 */
    TEST_ASSERT_TRUE(UIEditor_CursorRight(&actual_cursor, input_min_cursor)); /* -> 2 */
    TEST_ASSERT_TRUE(UIEditor_CursorRight(&actual_cursor, input_min_cursor)); /* -> 1 */
    TEST_ASSERT_TRUE(UIEditor_CursorRight(&actual_cursor, input_min_cursor)); /* -> 0 */
    TEST_ASSERT_EQUAL_UINT8(0, actual_cursor);

    /* [WHEN] Attempting to move past right boundary */
    bool moved_right_underflow = UIEditor_CursorRight(&actual_cursor, input_min_cursor);

    /* [THEN] Boundary respected, cursor held at min (0) */
    TEST_ASSERT_FALSE(moved_right_underflow);
    TEST_ASSERT_EQUAL_UINT8(0, actual_cursor);

    /* [WHEN] Passing NULL pointers */
    TEST_ASSERT_FALSE(UIEditor_CursorLeft(NULL, input_max_cursor));
    TEST_ASSERT_FALSE(UIEditor_CursorRight(NULL, input_min_cursor));
}

void test_value_increment_and_clamping(void) {
    /* [GIVEN] A numerical parameter starting at 5.00 with max clamp 10.00 */
    float actual_val = 5.0f;
    const float input_max_val = 10.0f;

    /* [WHEN] Incrementing at cursor index 2 (ones place: +1.00) */
    UIEditor_Increment(&actual_val, 2, gUIEditorMults, UI_EDITOR_MULTS_COUNT, input_max_val);
    /* [THEN] Value becomes 6.00 */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, actual_val);

    /* [WHEN] Incrementing at cursor index 1 (tenths place: +0.10) */
    UIEditor_Increment(&actual_val, 1, gUIEditorMults, UI_EDITOR_MULTS_COUNT, input_max_val);
    /* [THEN] Value becomes 6.10 */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.1f, actual_val);

    /* [WHEN] Incrementing at cursor index 0 (hundredths place: +0.01) */
    UIEditor_Increment(&actual_val, 0, gUIEditorMults, UI_EDITOR_MULTS_COUNT, input_max_val);
    /* [THEN] Value becomes 6.11 */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.11f, actual_val);

    /* [WHEN] Incrementing past the maximum clamp */
    actual_val = 9.5f;
    UIEditor_Increment(&actual_val, 2, gUIEditorMults, UI_EDITOR_MULTS_COUNT, input_max_val);

    /* [THEN] Clamped strictly to maximum limit (10.00) */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, actual_val);
}

void test_value_decrement_and_clamping(void) {
    /* [GIVEN] A numerical parameter starting at 2.00 with min clamp 0.00 */
    float actual_val = 2.0f;
    const float input_min_val = 0.0f;

    /* [WHEN] Decrementing at cursor index 2 (ones place: -1.00) */
    UIEditor_Decrement(&actual_val, 2, gUIEditorMults, UI_EDITOR_MULTS_COUNT, input_min_val);
    /* [THEN] Value becomes 1.00 */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, actual_val);

    /* [WHEN] Decrementing at cursor index 1 (tenths place: -0.10) */
    UIEditor_Decrement(&actual_val, 1, gUIEditorMults, UI_EDITOR_MULTS_COUNT, input_min_val);
    /* [THEN] Value becomes 0.90 */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.9f, actual_val);

    /* [WHEN] Decrementing past the minimum clamp */
    UIEditor_Decrement(&actual_val, 2, gUIEditorMults, UI_EDITOR_MULTS_COUNT, input_min_val);

    /* [THEN] Clamped strictly to minimum limit (0.00) */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, actual_val);
}

void test_weight_multipliers(void) {
    /* [GIVEN] Calibration weight starting at 1.00000 g */
    float actual_weight = 1.00000f;
    const float input_max_weight = 100.0f;

    /* [WHEN] Incrementing at cursor index 2 (+0.001 g) */
    UIEditor_Increment(&actual_weight, 2, gUIEditorWeightMults, UI_EDITOR_WEIGHT_MULTS_COUNT, input_max_weight);
    /* [THEN] Value becomes 1.00100 g */
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 1.001f, actual_weight);

    /* [WHEN] Incrementing at cursor index 0 (+0.00001 g) */
    UIEditor_Increment(&actual_weight, 0, gUIEditorWeightMults, UI_EDITOR_WEIGHT_MULTS_COUNT, input_max_weight);
    /* [THEN] Value becomes 1.00101 g */
    TEST_ASSERT_FLOAT_WITHIN(0.000005f, 1.00101f, actual_weight);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_cursor_navigation);
    RUN_TEST(test_value_increment_and_clamping);
    RUN_TEST(test_value_decrement_and_clamping);
    RUN_TEST(test_weight_multipliers);
    return UNITY_END();
}
