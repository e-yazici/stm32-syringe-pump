/**
 * @file    test_ui_menu.c
 * @brief   Host unit tests for the LCD menu state machine (ui_menu.c).
 */

#include "unity.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

#include "syringe_db.h"
#include "flash_storage.h"
#include "rtos_tasks.h"
#include "liquidcrystal_i2c.h"

/* =========================================================================
 * MOCK HARDWARE & RTOS INFRASTRUCTURE
 * ========================================================================= */

void _putchar(char character) {
    (void)character;
}

static char gMockLcdLines[2][17];
static uint8_t gMockCursorX = 0;
static uint8_t gMockCursorY = 0;
static bool gMockCursorVisible = false;
static bool gMockBlink = false;
static uint32_t gMockClearCount = 0;

void HD44780_Init(uint8_t rows) {
    (void)rows;
}

void HD44780_Clear(void) {
    memset(gMockLcdLines[0], ' ', 16);
    gMockLcdLines[0][16] = '\0';
    memset(gMockLcdLines[1], ' ', 16);
    gMockLcdLines[1][16] = '\0';
    gMockCursorX = 0;
    gMockCursorY = 0;
    gMockClearCount++;
}

void HD44780_Home(void) {
    gMockCursorX = 0;
    gMockCursorY = 0;
}

void HD44780_SetCursor(uint8_t col, uint8_t row) {
    gMockCursorX = (col < 16) ? col : 15;
    gMockCursorY = (row < 2) ? row : 1;
}

void HD44780_PrintStr(const char *c) {
    if (!c) return;
    while (*c && gMockCursorX < 16) {
        gMockLcdLines[gMockCursorY][gMockCursorX++] = *c++;
    }
}

void HD44780_Cursor(void) {
    gMockCursorVisible = true;
}

void HD44780_NoCursor(void) {
    gMockCursorVisible = false;
}

void HD44780_Blink(void) {
    gMockBlink = true;
}

void HD44780_NoBlink(void) {
    gMockBlink = false;
}

/* Mock Flash Storage */
static SyringeDatabase_t gMockDb;

void FlashStorage_Init(void) {
    SyringeDB_Format(&gMockDb);
}

const SyringeDatabase_t* FlashStorage_GetDB(void) {
    return &gMockDb;
}

bool FlashStorage_AddSyringe(const SyringeProfile_t *profile) {
    return SyringeDB_Add(&gMockDb, profile);
}

bool FlashStorage_ModifySyringe(uint32_t index, const SyringeProfile_t *profile) {
    return SyringeDB_Modify(&gMockDb, index, profile);
}

bool FlashStorage_DeleteSyringe(uint32_t index) {
    return SyringeDB_Delete(&gMockDb, index);
}

/* Shared Pump & System Globals */
PumpHardwareSpecs_t gPumpHw = {
    .screwLead_mm_per_rev = 2.0f,
    .motorStepsPerRev = 200,
    .microstepping = 128,
    .maxFlowRate_ul_min = 100000.0f,
    .maxFlowRate_ml_min = 100.0f
};
PumpJobParams_t gPumpJob;
volatile SystemState_e gSysState = SYS_IDLE;
volatile float gLivePumpedVolume_ml = 0.0f;

/* Include ui_menu.c directly to test internal state transitions and helpers */
#include "../Drivers/Libs/Src/ui_menu.c"

void setUp(void) {
    FlashStorage_Init();
    memset(&gPumpJob, 0, sizeof(gPumpJob));
    gPumpJob.targetVolume = 1.0f;
    gPumpJob.volumeUnit = VOL_UNIT_ML;
    gPumpJob.targetFlowRate = 5.0f;
    gPumpJob.flowUnit = FLOW_UNIT_ML_MIN;
    gPumpJob.syringeDiameter_mm = 14.5f;
    gPumpJob.syringeMaxVolume_ml = 10.0f;
    strncpy(gPumpJob.syringeName, "TEST_SYR", sizeof(gPumpJob.syringeName) - 1);

    UiMenu_Init();
}

void tearDown(void) {}

/* =========================================================================
 * 1. NUMERIC BOUNDARY AND CLAMPING TESTS
 * ========================================================================= */

void test_clamp_volume_boundaries(void) {
    /* ClampVolume with mL unit (max 60.0f) */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampVolume(-100.0f, VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampVolume(-0.0001f, VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampVolume(0.0f, VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_FLOAT(15.5f, ClampVolume(15.5f, VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_FLOAT(60.0f, ClampVolume(60.0f, VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_FLOAT(60.0f, ClampVolume(60.0001f, VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_FLOAT(60.0f, ClampVolume(1000.0f, VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_FLOAT(60.0f, ClampVolume(FLT_MAX, VOL_UNIT_ML));

    /* ClampVolume with uL unit (max 1000.0f) */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampVolume(-5.0f, VOL_UNIT_UL));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampVolume(0.0f, VOL_UNIT_UL));
    TEST_ASSERT_EQUAL_FLOAT(500.0f, ClampVolume(500.0f, VOL_UNIT_UL));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, ClampVolume(1000.0f, VOL_UNIT_UL));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, ClampVolume(1000.01f, VOL_UNIT_UL));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, ClampVolume(99999.0f, VOL_UNIT_UL));
}

void test_clamp_flow_rate_boundaries(void) {
    /* FLOW_UNIT_ML_MIN: clamped to gPumpHw.maxFlowRate_ml_min (100.0f) */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampFlowRate(-1.0f, FLOW_UNIT_ML_MIN));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampFlowRate(0.0f, FLOW_UNIT_ML_MIN));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, ClampFlowRate(50.0f, FLOW_UNIT_ML_MIN));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, ClampFlowRate(100.0f, FLOW_UNIT_ML_MIN));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, ClampFlowRate(100.05f, FLOW_UNIT_ML_MIN));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, ClampFlowRate(5000.0f, FLOW_UNIT_ML_MIN));

    /* FLOW_UNIT_UL_MIN: clamped to gPumpHw.maxFlowRate_ul_min (100000.0f) */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampFlowRate(-10.0f, FLOW_UNIT_UL_MIN));
    TEST_ASSERT_EQUAL_FLOAT(50000.0f, ClampFlowRate(50000.0f, FLOW_UNIT_UL_MIN));
    TEST_ASSERT_EQUAL_FLOAT(100000.0f, ClampFlowRate(100000.0f, FLOW_UNIT_UL_MIN));
    TEST_ASSERT_EQUAL_FLOAT(100000.0f, ClampFlowRate(200000.0f, FLOW_UNIT_UL_MIN));

    /* FLOW_UNIT_ML_HR: clamped to 999.99f */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampFlowRate(-0.5f, FLOW_UNIT_ML_HR));
    TEST_ASSERT_EQUAL_FLOAT(500.0f, ClampFlowRate(500.0f, FLOW_UNIT_ML_HR));
    TEST_ASSERT_EQUAL_FLOAT(999.99f, ClampFlowRate(999.99f, FLOW_UNIT_ML_HR));
    TEST_ASSERT_EQUAL_FLOAT(999.99f, ClampFlowRate(1000.0f, FLOW_UNIT_ML_HR));

    /* FLOW_UNIT_UL_HR: clamped to 9999.99f */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ClampFlowRate(-50.0f, FLOW_UNIT_UL_HR));
    TEST_ASSERT_EQUAL_FLOAT(9999.99f, ClampFlowRate(9999.99f, FLOW_UNIT_UL_HR));
    TEST_ASSERT_EQUAL_FLOAT(9999.99f, ClampFlowRate(15000.0f, FLOW_UNIT_UL_HR));

    /* Invalid flow unit enum (fallback max 1000.0f) */
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, ClampFlowRate(1500.0f, (FlowUnit_e)99));
}

void test_unit_string_helpers(void) {
    TEST_ASSERT_EQUAL_STRING("mL", GetVolumeUnitStr(VOL_UNIT_ML));
    TEST_ASSERT_EQUAL_STRING("uL", GetVolumeUnitStr(VOL_UNIT_UL));

    TEST_ASSERT_EQUAL_STRING("mL/m", GetFlowUnitStr(FLOW_UNIT_ML_MIN));
    TEST_ASSERT_EQUAL_STRING("uL/m", GetFlowUnitStr(FLOW_UNIT_UL_MIN));
    TEST_ASSERT_EQUAL_STRING("mL/h", GetFlowUnitStr(FLOW_UNIT_ML_HR));
    TEST_ASSERT_EQUAL_STRING("uL/h", GetFlowUnitStr(FLOW_UNIT_UL_HR));
    TEST_ASSERT_EQUAL_STRING("", GetFlowUnitStr((FlowUnit_e)123));
}

/* =========================================================================
 * 2. MAIN MENU CURSOR NAVIGATION
 * ========================================================================= */

void test_main_menu_cursor_bounds(void) {
    UiMenu_Init();
    TEST_ASSERT_EQUAL_UINT8(0, mainMenuCursor);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);

    /* Turn CCW when at 0: must stay 0 */
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, UiMenu_ProcessEvent(EVENT_ENC_CCW));
        TEST_ASSERT_EQUAL_UINT8(0, mainMenuCursor);
    }

    /* Turn CW to scroll through all 7 items (0 to 6) */
    for (int i = 0; i < MAIN_MENU_MAX - 1; i++) {
        TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, UiMenu_ProcessEvent(EVENT_ENC_CW));
        TEST_ASSERT_EQUAL_UINT8(i + 1, mainMenuCursor);
    }
    TEST_ASSERT_EQUAL_UINT8(MAIN_MENU_MAX - 1, mainMenuCursor);

    /* Turn CW past the end: must remain clamped at MAIN_MENU_MAX - 1 */
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, UiMenu_ProcessEvent(EVENT_ENC_CW));
        TEST_ASSERT_EQUAL_UINT8(MAIN_MENU_MAX - 1, mainMenuCursor);
    }

    /* Turn CCW back to 0 */
    for (int i = MAIN_MENU_MAX - 1; i > 0; i--) {
        TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, UiMenu_ProcessEvent(EVENT_ENC_CCW));
        TEST_ASSERT_EQUAL_UINT8(i - 1, mainMenuCursor);
    }
    TEST_ASSERT_EQUAL_UINT8(0, mainMenuCursor);
}

/* =========================================================================
 * 3. VOLUME AND FLOW RATE EDITING WORKFLOWS
 * ========================================================================= */

void test_volume_edit_workflow_and_cursors(void) {
    UiMenu_Init();
    /* Navigate to Volume item (cursor 1) */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(MAIN_MENU_VOLUME, mainMenuCursor);

    /* Press Select -> enter MENU_EDIT_VOLUME_UNIT */
    TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS));
    TEST_ASSERT_EQUAL_INT(MENU_EDIT_VOLUME_UNIT, sMenuState);

    /* Toggle unit via encoder */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_INT(VOL_UNIT_UL, tempVolUnit);
    UiMenu_ProcessEvent(EVENT_ENC_CCW);
    TEST_ASSERT_EQUAL_INT(VOL_UNIT_ML, tempVolUnit);

    /* Select unit -> enter MENU_EDIT_VOLUME */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_EDIT_VOLUME, sMenuState);
    TEST_ASSERT_EQUAL_UINT8(2, digit_cursor); /* Default ones digit */

    /* Test cursor bounds in volume edit */
    for (int i = 0; i < 10; i++) {
        UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    }
    TEST_ASSERT_EQUAL_UINT8(0, digit_cursor);

    for (int i = 0; i < 10; i++) {
        UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS);
    }
    TEST_ASSERT_EQUAL_UINT8(6, digit_cursor);

    /* Increment value by 10,000 mL -> should clamp to 60.0f mL */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, tempVolume);

    /* Move cursor to ones place (2) and decrement -> should clamp to 0.0f */
    for (int i = 0; i < 4; i++) {
        UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    }
    TEST_ASSERT_EQUAL_UINT8(2, digit_cursor);
    for (int i = 0; i < 65; i++) {
        UiMenu_ProcessEvent(EVENT_ENC_CCW);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tempVolume);

    /* Set to 12.34 mL and commit */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS);
    TEST_ASSERT_EQUAL_UINT8(3, digit_cursor);
    UiMenu_ProcessEvent(EVENT_ENC_CW); /* +10 -> 10.00 */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    TEST_ASSERT_EQUAL_UINT8(2, digit_cursor);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW); /* +2 -> 12.00 */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    TEST_ASSERT_EQUAL_UINT8(1, digit_cursor);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW); /* +0.3 -> 12.30 */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    TEST_ASSERT_EQUAL_UINT8(0, digit_cursor);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW); /* +0.04 -> 12.34 */

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.34f, tempVolume);

    /* Commit */
    TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS));
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.34f, gPumpJob.targetVolume);
}

void test_flowrate_edit_workflow_and_cursors(void) {
    UiMenu_Init();
    /* Navigate to Flowrate (cursor 2) */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(MAIN_MENU_FLOWRATE, mainMenuCursor);

    /* Enter MENU_EDIT_FLOWRATE_UNIT */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_EDIT_FLOWRATE_UNIT, sMenuState);

    /* Cycle flow units: ML_MIN (1) -> UL_HR (2) -> ML_HR (3) -> UL_MIN (0) -> ML_MIN (1) */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_INT(FLOW_UNIT_UL_HR, tempFlowUnit);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_INT(FLOW_UNIT_ML_HR, tempFlowUnit);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_INT(FLOW_UNIT_UL_MIN, tempFlowUnit);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_INT(FLOW_UNIT_ML_MIN, tempFlowUnit);

    /* CCW from 1 gives 0 (FLOW_UNIT_UL_MIN) */
    UiMenu_ProcessEvent(EVENT_ENC_CCW);
    TEST_ASSERT_EQUAL_INT(FLOW_UNIT_UL_MIN, tempFlowUnit);
    /* CCW from 0 wraps to 3 (FLOW_UNIT_ML_HR) */
    UiMenu_ProcessEvent(EVENT_ENC_CCW);
    TEST_ASSERT_EQUAL_INT(FLOW_UNIT_ML_HR, tempFlowUnit);

    /* Select unit -> enter MENU_EDIT_FLOWRATE */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_EDIT_FLOWRATE, sMenuState);
    TEST_ASSERT_EQUAL_INT(FLOW_UNIT_ML_HR, gPumpJob.flowUnit);

    /* Long press cancels back to MENU_OVERVIEW */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_LONG_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);
}

/* =========================================================================
 * 4. SYRINGE WIZARD WORKFLOWS (MANUAL & AUTO CALIBRATION)
 * ========================================================================= */

void test_add_syringe_manual_workflow(void) {
    UiMenu_Init();
    /* Navigate to Add Syringe (cursor 3) */
    for (int i = 0; i < 3; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(MAIN_MENU_ADD_SYRINGE, mainMenuCursor);

    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_DIA_MODE, sMenuState);
    TEST_ASSERT_EQUAL_UINT8(0, diaModeSelection); /* MANUAL */

    /* Select MANUAL -> goes to MENU_ADD_SYRINGE_DIA */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_DIA, sMenuState);
    TEST_ASSERT_EQUAL_UINT8(2, digit_cursor);

    /* Set diameter to 14.50 mm */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    for (int i = 0; i < 4; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    for (int i = 0; i < 5; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 14.50f, tempAddSyringe.innerDiameter_mm);

    /* Select -> advances to MENU_ADD_SYRINGE_VOL */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_VOL, sMenuState);

    /* Set volume to 20.0 mL */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, tempAddSyringe.maxVolume_ml);

    /* Select -> advances to MENU_ADD_SYRINGE_NAME */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_NAME, sMenuState);
    TEST_ASSERT_EQUAL_UINT8(0, digit_cursor);

    /* Set first character to 'B' */
    UiMenu_ProcessEvent(EVENT_ENC_CW); /* 'A' */
    UiMenu_ProcessEvent(EVENT_ENC_CW); /* 'B' */
    TEST_ASSERT_EQUAL_INT8('B', tempAddSyringe.name[0]);

    /* Move to char 1 */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    TEST_ASSERT_EQUAL_UINT8(1, digit_cursor);
    UiMenu_ProcessEvent(EVENT_ENC_CW); /* 'A' */
    TEST_ASSERT_EQUAL_INT8('A', tempAddSyringe.name[1]);

    /* Commit syringe to DB */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);

    /* Check that syringe was added to DB */
    const SyringeDatabase_t *db = FlashStorage_GetDB();
    TEST_ASSERT_EQUAL_UINT32(1, db->count);
    const SyringeProfile_t *p = SyringeDB_Get(db, 0);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_STRING("BA             ", p->name);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 14.5f, p->innerDiameter_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, p->maxVolume_ml);
}

void test_add_syringe_auto_calibration_workflow(void) {
    UiMenu_Init();
    for (int i = 0; i < 3; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_DIA_MODE, sMenuState);

    /* Select AUTO */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(1, diaModeSelection);

    /* Press Select -> enter MENU_ADD_SYRINGE_AUTO_PROMPT */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_AUTO_PROMPT, sMenuState);

    /* Press Select -> launches 10mm calibration pump */
    UiMenuAction_e action = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_START_CALIBRATION_PUMP, action);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_AUTO_PUMPING, sMenuState);

    /* Simulate motor completing calibration pump */
    UiMenu_OnCalibrationMotorComplete();
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_AUTO_WEIGHT, sMenuState);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, tempWeight_g);
    TEST_ASSERT_EQUAL_UINT8(5, digit_cursor);

    /* Adjust weight to 1.64652 g and confirm -> transitions to Z-factor entry */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 4 (0.1) */
    for (int i = 0; i < 6; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 3 (0.01) */
    for (int i = 0; i < 4; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 2 (0.001) */
    for (int i = 0; i < 6; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 1 (0.0001) */
    for (int i = 0; i < 5; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 0 (0.00001) */
    for (int i = 0; i < 2; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 1.64652f, tempWeight_g);

    /* Press Select -> enter MENU_ADD_SYRINGE_AUTO_Z_FACTOR */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_AUTO_Z_FACTOR, sMenuState);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, tempZFactor);
    TEST_ASSERT_EQUAL_UINT8(5, digit_cursor);

    /* Verify cursor bounds and set Z-factor to 1.00290 uL/mg */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS); /* index 6 (10.0) */
    TEST_ASSERT_EQUAL_UINT8(6, digit_cursor);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS); /* clamped at 6 */
    TEST_ASSERT_EQUAL_UINT8(6, digit_cursor);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 5 (1.0) */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 4 (0.1) */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 3 (0.01) */
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 2 (0.001) */
    for (int i = 0; i < 2; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS); /* index 1 (0.0001) */
    for (int i = 0; i < 9; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_FLOAT_WITHIN(0.00005f, 1.00290f, tempZFactor);

    /* Press Select -> diameter calculated via pump_calc_diameter_from_weight using V = m * Z */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_ADD_SYRINGE_DIA, sMenuState);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 14.50f, tempAddSyringe.innerDiameter_mm);
}

/* =========================================================================
 * 5. PUMPING LIFECYCLE, PAUSE, RESUME, CANCEL, AND STALL ALERT
 * ========================================================================= */

void test_pump_launch_pause_resume_cancel(void) {
    SyringeProfile_t s1 = {"TEST_SYR", 20.0f, 15.0f};
    FlashStorage_AddSyringe(&s1);

    UiMenu_Init();
    for (int i = 0; i < 5; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(MAIN_MENU_PUMP, mainMenuCursor);

    /* Launch pump */
    UiMenuAction_e act = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_START_PUMP, act);
    TEST_ASSERT_EQUAL_INT(MENU_PUMPING_VIEW, sMenuState);
    TEST_ASSERT_TRUE(UiMenu_IsPumpingView());

    /* While pumping, update live view */
    UiMenu_UpdatePumpingView(0.5f);
    UiMenu_UpdatePumpingView(1.0f);

    /* User presses Pause (EXTI trigger) */
    UiMenu_OnPauseRequested();
    TEST_ASSERT_EQUAL_INT(MENU_PUMP_PAUSE_CONFIRM, sMenuState);
    TEST_ASSERT_EQUAL_UINT8(1, cancelSelection); /* Default NO */

    /* Select NO -> RESUME pump */
    act = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_RESUME_PUMP, act);
    TEST_ASSERT_EQUAL_INT(MENU_PUMPING_VIEW, sMenuState);

    /* Pause again */
    UiMenu_OnPauseRequested();
    TEST_ASSERT_EQUAL_INT(MENU_PUMP_PAUSE_CONFIRM, sMenuState);

    /* Toggle to YES (cancel) */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(0, cancelSelection);

    /* Select YES -> CANCEL pump */
    act = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_CANCEL_PUMP, act);
    TEST_ASSERT_EQUAL_INT(MENU_PUMP_CANCELLED, sMenuState);

    /* Any button dismisses result screen back to overview */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);

    /* Test natural motor completion */
    UiMenu_OnMotorComplete();
    TEST_ASSERT_EQUAL_INT(MENU_PUMP_COMPLETED, sMenuState);
    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);
}

void test_pump_warning_dialogs(void) {
    /* Attempt to pump when DB is empty */
    FlashStorage_Init();
    UiMenu_Init();

    for (int i = 0; i < 5; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(MAIN_MENU_PUMP, mainMenuCursor);

    UiMenuAction_e act = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, act);
    TEST_ASSERT_EQUAL_INT(MENU_WARNING_NO_SYRINGE, sMenuState);

    /* Dismiss warning */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);

    /* Syringe volume exceeded warning */
    SyringeProfile_t smallSyr = {"SMALL", 5.0f, 10.0f};
    FlashStorage_AddSyringe(&smallSyr);
    UiMenu_Init();

    gPumpJob.targetVolume = 10.0f;
    gPumpJob.volumeUnit = VOL_UNIT_ML;
    gPumpJob.syringeMaxVolume_ml = 5.0f;

    for (int i = 0; i < 5; i++) UiMenu_ProcessEvent(EVENT_ENC_CW);
    act = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, act);
    TEST_ASSERT_EQUAL_INT(MENU_WARNING_VOLUME, sMenuState);

    /* Dismiss warning */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);
}

void test_pumping_view_edge_cases(void) {
    sMenuState = MENU_PUMPING_VIEW;

    /* Zero flow rate -> remaining time 0, no division by zero */
    gPumpJob.targetVolume = 10.0f;
    gPumpJob.volumeUnit = VOL_UNIT_ML;
    gPumpJob.targetFlowRate = 0.0f;
    gPumpJob.flowUnit = FLOW_UNIT_ML_MIN;
    UiMenu_UpdatePumpingView(5.0f);

    /* Huge remaining time -> hours clamped to 99 */
    gPumpJob.targetVolume = 60.0f;
    gPumpJob.targetFlowRate = 0.0001f;
    gPumpJob.flowUnit = FLOW_UNIT_ML_HR;
    UiMenu_UpdatePumpingView(0.0f);
    UiMenu_UpdatePumpingView(0.0f);

    /* Over-delivered volume (livePumpedVolume > target) */
    gPumpJob.targetVolume = 5.0f;
    UiMenu_UpdatePumpingView(6.0f);
    TEST_ASSERT_TRUE(true);
}

void test_stall_alert_dialog_and_actions(void) {
    /* 1. Direct entry to MENU_STALL_ALERT */
    UiMenu_OnStallAlert();
    TEST_ASSERT_EQUAL_INT(MENU_STALL_ALERT, sMenuState);
    TEST_ASSERT_EQUAL_UINT8(0, stallAlertSelection);
    TEST_ASSERT_EQUAL_STRING_LEN("! MOTOR STALLED ", gMockLcdLines[0], 16);
    TEST_ASSERT_EQUAL_STRING(">Continue  Abort", gMockLcdLines[1]);

    /* 2. EVENT_STALL_DETECTED transitions from active pumping view */
    sMenuState = MENU_PUMPING_VIEW;
    UiMenuAction_e act = UiMenu_ProcessEvent(EVENT_STALL_DETECTED);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_NONE, act);
    TEST_ASSERT_EQUAL_INT(MENU_STALL_ALERT, sMenuState);
    TEST_ASSERT_EQUAL_UINT8(0, stallAlertSelection);
    TEST_ASSERT_EQUAL_STRING(">Continue  Abort", gMockLcdLines[1]);

    /* 3. Navigation between Continue (0) and Abort (1) */
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(1, stallAlertSelection);
    TEST_ASSERT_EQUAL_STRING(" Continue >Abort", gMockLcdLines[1]);

    UiMenu_ProcessEvent(EVENT_ENC_CCW);
    TEST_ASSERT_EQUAL_UINT8(0, stallAlertSelection);
    TEST_ASSERT_EQUAL_STRING(">Continue  Abort", gMockLcdLines[1]);

    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_RIGHT_PRESS);
    TEST_ASSERT_EQUAL_UINT8(1, stallAlertSelection);
    TEST_ASSERT_EQUAL_STRING(" Continue >Abort", gMockLcdLines[1]);

    UiMenu_ProcessEvent(EVENT_BTN_CURSOR_LEFT_PRESS);
    TEST_ASSERT_EQUAL_UINT8(0, stallAlertSelection);
    TEST_ASSERT_EQUAL_STRING(">Continue  Abort", gMockLcdLines[1]);

    /* 4. User selects CONTINUE (selection == 0) */
    UiMenuAction_e actResume = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_RESUME_STALL_PUMP, actResume);
    TEST_ASSERT_EQUAL_INT(MENU_PUMPING_VIEW, sMenuState);

    /* 5. User selects ABORT (selection == 1) */
    UiMenu_ProcessEvent(EVENT_STALL_DETECTED);
    UiMenu_ProcessEvent(EVENT_ENC_CW);
    TEST_ASSERT_EQUAL_UINT8(1, stallAlertSelection);

    UiMenuAction_e actAbort = UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(UI_ACTION_ABORT_STALL_PUMP, actAbort);
    TEST_ASSERT_EQUAL_INT(MENU_PUMP_CANCELLED, sMenuState);
    TEST_ASSERT_EQUAL_STRING("Pump Cancelled! ", gMockLcdLines[0]);
    TEST_ASSERT_EQUAL_STRING("Press any button", gMockLcdLines[1]);

    /* Any button in MENU_PUMP_CANCELLED returns to overview */
    UiMenu_ProcessEvent(EVENT_BTN_SELECT_PRESS);
    TEST_ASSERT_EQUAL_INT(MENU_OVERVIEW, sMenuState);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_clamp_volume_boundaries);
    RUN_TEST(test_clamp_flow_rate_boundaries);
    RUN_TEST(test_unit_string_helpers);
    RUN_TEST(test_main_menu_cursor_bounds);
    RUN_TEST(test_volume_edit_workflow_and_cursors);
    RUN_TEST(test_flowrate_edit_workflow_and_cursors);
    RUN_TEST(test_add_syringe_manual_workflow);
    RUN_TEST(test_add_syringe_auto_calibration_workflow);
    RUN_TEST(test_pump_launch_pause_resume_cancel);
    RUN_TEST(test_pump_warning_dialogs);
    RUN_TEST(test_pumping_view_edge_cases);
    RUN_TEST(test_stall_alert_dialog_and_actions);

    return UNITY_END();
}
