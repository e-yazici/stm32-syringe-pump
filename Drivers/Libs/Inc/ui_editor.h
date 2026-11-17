/**
 * @file    ui_editor.h
 * @brief   Hardware-agnostic numeric cursor editor math and boundary validation.
 * @author  eyazici
 * @date    2026
 *
 * @details Provides reusable digit-level cursor manipulation, multiplier tables,
 *          and value bounds clamping for parameters like volume, flow rate,
 *          syringe diameter, and calibration weight.
 */

#ifndef LIBS_INC_UI_EDITOR_H_
#define LIBS_INC_UI_EDITOR_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UI_EDITOR_MULTS_COUNT 7
#define UI_EDITOR_WEIGHT_MULTS_COUNT 8

extern const float gUIEditorMults[UI_EDITOR_MULTS_COUNT];
extern const float gUIEditorWeightMults[UI_EDITOR_WEIGHT_MULTS_COUNT];

/**
 * @brief  Increment a floating-point value at the specified digit cursor index.
 * @param  value     Pointer to value to increment.
 * @param  cursorIdx Index into multiplier table.
 * @param  table     Multiplier array (e.g. gUIEditorMults).
 * @param  tableSize Number of elements in table.
 * @param  maxVal    Maximum allowed value.
 * @retval New clamped value.
 */
float UIEditor_Increment(float *value, uint8_t cursorIdx, const float *table, uint8_t tableSize, float maxVal);

/**
 * @brief  Decrement a floating-point value at the specified digit cursor index.
 * @param  value     Pointer to value to decrement.
 * @param  cursorIdx Index into multiplier table.
 * @param  table     Multiplier array.
 * @param  tableSize Number of elements in table.
 * @param  minVal    Minimum allowed value.
 * @retval New clamped value.
 */
float UIEditor_Decrement(float *value, uint8_t cursorIdx, const float *table, uint8_t tableSize, float minVal);

/**
 * @brief  Move cursor left (toward higher place value).
 * @param  cursorIdx Pointer to current cursor index.
 * @param  maxCursor Maximum allowed cursor index.
 * @retval true if moved, false if already at boundary or NULL.
 */
/* Multiplier table index increases to the left (higher powers of ten) */
static inline bool UIEditor_CursorLeft(uint8_t *cursorIdx, uint8_t maxCursor) {
    if (cursorIdx == NULL) {
        return false;
    }
    if (*cursorIdx < maxCursor) {
        (*cursorIdx)++;
        return true;
    }
    return false;
}

/**
 * @brief  Move cursor right (toward lower place value).
 * @param  cursorIdx Pointer to current cursor index.
 * @param  minCursor Minimum allowed cursor index (usually 0).
 * @retval true if moved, false if already at boundary or NULL.
 */
static inline bool UIEditor_CursorRight(uint8_t *cursorIdx, uint8_t minCursor) {
    if (cursorIdx == NULL) {
        return false;
    }
    if (*cursorIdx > minCursor) {
        (*cursorIdx)--;
        return true;
    }
    return false;
}

#ifdef __cplusplus
}
#endif

#endif /* LIBS_INC_UI_EDITOR_H_ */
