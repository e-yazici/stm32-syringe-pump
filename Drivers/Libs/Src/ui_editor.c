/**
 * @file    ui_editor.c
 * @brief   Implementation of numeric cursor editor math and boundary validation.
 * @author  eyazici
 * @date    2026
 */

#include "ui_editor.h"
#include <stddef.h>


const float gUIEditorMults[UI_EDITOR_MULTS_COUNT] = {
    0.01f, 0.1f, 1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f
};

const float gUIEditorWeightMults[UI_EDITOR_WEIGHT_MULTS_COUNT] = {
    0.00001f, 0.0001f, 0.001f, 0.01f, 0.1f, 1.0f, 10.0f, 100.0f
};

float UIEditor_Increment(float *value, uint8_t cursorIdx, const float *table, uint8_t tableSize, float maxVal) {
    if (value == NULL || table == NULL || cursorIdx >= tableSize) {
        return (value != NULL) ? *value : 0.0f;
    }
    *value += table[cursorIdx];
    if (*value > maxVal) {
        *value = maxVal;
    }
    return *value;
}

float UIEditor_Decrement(float *value, uint8_t cursorIdx, const float *table, uint8_t tableSize, float minVal) {
    if (value == NULL || table == NULL || cursorIdx >= tableSize) {
        return (value != NULL) ? *value : 0.0f;
    }
    *value -= table[cursorIdx];
    if (*value < minVal) {
        *value = minVal;
    }
    return *value;
}

