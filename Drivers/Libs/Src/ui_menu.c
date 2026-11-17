/**
 * @file    ui_menu.c
 * @brief   Single-module LCD menu pages implementation for the Syringe Pump.
 * @author  eyazici
 * @date    2026
 *
 * @details Encapsulates all 25 LCD screen pages, numeric cursor editing,
 *          syringe database wizard workflows, and LCD rendering routines.
 *          Decoupled from FreeRTOS task coordination and motor execution.
 */

#include "printf.h"
#include "ui_menu.h"
#include "rtos_tasks.h"
#include "liquidcrystal_i2c.h"
#include "pump_kinematics.h"
#include "ui_editor.h"
#include "stepper.h"
#include "flash_storage.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ========================================================================== */
/*                         Constants and Static Tables                        */
/* ========================================================================== */

static const char* MainMenuStrings[MAIN_MENU_MAX] = {
    "> Select Syringe",
    "> Volume",
    "> Flowrate",
    "> Add Syringe",
    "> Set Direction",
    "> Pump",
    "> Manual Mode"
};

/**
 * @brief  Character set for syringe name entry.
 *
 * @details The encoder scrolls through this array to select each character
 *          position in the syringe name. Includes space, A–Z, and 0–9 (37 chars).
 */
static const char *const syringeNameCharset[37] = {
    " ", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L",
    "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y",
    "Z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"
};

/* ========================================================================== */
/*                           Private State Variables                          */
/* ========================================================================== */

static MenuState_e sMenuState = MENU_OVERVIEW;
static uint8_t mainMenuCursor = 0;
static uint8_t digit_cursor = 2;
static VolumeUnit_e tempVolUnit;
static FlowUnit_e tempFlowUnit;
static float tempVolume;
static float tempFlowRate;
static SyringeProfile_t tempAddSyringe;
static uint8_t list_cursor = 0;
static bool isModifyingSyringe = false;
static float tempWeight_g = 1.0f;
static float tempZFactor = 1.0f;
static MotorDirection_e tempDirection = DIR_FORWARD;
static MotorDirection_e currentDirection = DIR_FORWARD;

/** @brief Current selection in the ACTION menu (0=MODIFY, 1=DELETE) */
static uint8_t actionSelection = 0;
/** @brief Current selection in the DELETE confirm menu (0=YES, 1=NO) */
static uint8_t deleteConfirmSelection = 1;
/** @brief Current selection in DIA MODE menu (0=MANUAL, 1=AUTO) */
static uint8_t diaModeSelection = 0;

static uint8_t anim_idx = 0;        ///< Animation frame index for pumping view
static uint8_t cancelSelection = 1;  ///< Pause confirm dialog: 0 = YES, 1 = NO
static uint8_t stallAlertSelection = 0;  ///< Stall alert dialog: 0 = Continue, 1 = Abort

/* ========================================================================== */
/*                         Data Handling Helper Functions                     */
/* ========================================================================== */

/**
 * @brief  Clamp a volume value to valid range for the given unit.
 * @param  val  Volume value to clamp.
 * @param  unit Volume unit (determines max range: mL → 60, µL → 1000).
 * @retval Clamped volume value (>= 0).
 */
static inline float ClampVolume(float val, VolumeUnit_e unit) {
    if (val < 0.0f) val = 0.0f;
    float max_val = (unit == VOL_UNIT_ML) ? 60.0f : 1000.0f;
    if (val > max_val) val = max_val;
    return val;
}

/**
 * @brief  Clamp a flow rate value to valid range for the given unit.
 * @param  val  Flow rate value to clamp.
 * @param  unit Flow rate unit (determines max range per unit type).
 * @retval Clamped flow rate value (>= 0).
 */
static inline float ClampFlowRate(float val, FlowUnit_e unit) {
    if (val < 0.0f) val = 0.0f;
    float max_val = 1000.0f;
    switch (unit) {
        case FLOW_UNIT_ML_MIN: max_val = gPumpHw.maxFlowRate_ml_min; break;
        case FLOW_UNIT_UL_MIN: max_val = gPumpHw.maxFlowRate_ul_min; break;
        case FLOW_UNIT_ML_HR:  max_val = 999.99f; break;
        case FLOW_UNIT_UL_HR:  max_val = 9999.99f; break;
    }
    if (val > max_val) val = max_val;
    return val;
}

/**
 * @brief  Get the display string for a volume unit enum.
 * @param  unit Volume unit enum value.
 * @retval Static string: "mL" or "uL".
 */
static inline const char* GetVolumeUnitStr(VolumeUnit_e unit) {
    return (unit == VOL_UNIT_ML) ? "mL" : "uL";
}

/**
 * @brief  Get the display string for a flow rate unit enum.
 * @param  unit Flow rate unit enum value.
 * @retval Static string: "mL/m", "uL/m", "mL/h", or "uL/h".
 */
static inline const char* GetFlowUnitStr(FlowUnit_e unit) {
    switch (unit) {
        case FLOW_UNIT_ML_MIN: return "mL/m";
        case FLOW_UNIT_UL_MIN: return "uL/m";
        case FLOW_UNIT_ML_HR:  return "mL/h";
        case FLOW_UNIT_UL_HR:  return "uL/h";
    }
    return "";
}

/* ========================================================================== */
/*                         UI Drawing Helpers                                 */
/* ========================================================================== */

/**
 * @brief  Render the main menu overview screen on the LCD.
 *
 * @details Draws a two-line display showing the currently selected menu item
 *          (prefixed with ">") and context-dependent information on the second
 *          line. The menu acts as a virtual scrollable list.
 */
static void DrawMainMenu(void) {
    char buf[17];
    HD44780_NoBlink();
    HD44780_NoCursor();
    HD44780_Clear();
    HD44780_SetCursor(0, 0);

    switch ((MainMenuItem_e)mainMenuCursor) {
        case MAIN_MENU_SELECT_SYRINGE:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_SELECT_SYRINGE]);
            HD44780_SetCursor(0, 1);
            snprintf(buf, sizeof(buf), "  %.14s", gPumpJob.syringeName);
            HD44780_PrintStr(buf);
            break;

        case MAIN_MENU_VOLUME:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_VOLUME]);
            HD44780_SetCursor(0, 1);
            snprintf(buf, sizeof(buf), "  %.2f %s", gPumpJob.targetVolume, GetVolumeUnitStr(gPumpJob.volumeUnit));
            HD44780_PrintStr(buf);
            break;

        case MAIN_MENU_FLOWRATE:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_FLOWRATE]);
            HD44780_SetCursor(0, 1);
            snprintf(buf, sizeof(buf), "  %.2f %s", gPumpJob.targetFlowRate, GetFlowUnitStr(gPumpJob.flowUnit));
            HD44780_PrintStr(buf);
            break;

        case MAIN_MENU_ADD_SYRINGE:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_ADD_SYRINGE]);
            HD44780_SetCursor(0, 1);
            HD44780_PrintStr("  Set Direction");
            break;

        case MAIN_MENU_DIRECTION:
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_DIRECTION]);
            HD44780_SetCursor(0, 1);
            HD44780_PrintStr(currentDirection == DIR_FORWARD ? "  Forward" : "  Backward");
            break;

        case MAIN_MENU_PUMP:
            HD44780_PrintStr(" Set Direction");
            HD44780_SetCursor(0, 1);
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_PUMP]);
            break;

        case MAIN_MENU_FREE_MOVE:
            HD44780_PrintStr("  Pump");
            HD44780_SetCursor(0, 1);
            HD44780_PrintStr(MainMenuStrings[MAIN_MENU_FREE_MOVE]);
            break;

        default:
            break;
    }
}

/**
 * @brief  Display a numeric value on line 2 with cursor at the active digit.
 * @param  val       The numeric value to display.
 * @param  prefix    String printed before the number (e.g., "").
 * @param  suffix    Unit string printed after the number (e.g., "mL").
 * @param  precision Number of fractional digits.
 *
 * @details Formats the value as "prefix XX.XX suffix" on LCD line 2, then
 *          positions the hardware blinking cursor on the digit corresponding
 *          to the current digit_cursor index.
 */
static void PrintNumberWithCursor(float val, const char* prefix, const char* suffix, int precision) {
    char buf[20];
    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%s%%.%df %%s", precision);
    snprintf(buf, sizeof(buf), fmt, prefix, val, suffix);

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("                ");  /* Blank the entire line (16 chars) */
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(buf);

    char *dot = strchr(buf, '.');
    if (dot != NULL) {
        int dot_idx = (int)(dot - buf);
        int cur_x;
        if (digit_cursor < precision) {
            /* Fractional part */
            cur_x = dot_idx + (precision - digit_cursor);
        } else {
            /* Integer part */
            cur_x = dot_idx - (digit_cursor - precision + 1);
        }

        if (cur_x < 0) cur_x = 0;
        if (cur_x > 15) cur_x = 15;

        HD44780_SetCursor((uint8_t)cur_x, 1);
        HD44780_Cursor();
        HD44780_NoBlink();
    }
}

/**
 * @brief  Display a unit selection screen with a title and current unit.
 * @param  title   Title string for LCD line 1 (e.g., "Select Unit").
 * @param  unitStr Current unit string for LCD line 2 (e.g., "mL/m").
 */
static void PrintUnitSelect(const char* title, const char* unitStr) {
    char buf[17];
    HD44780_NoBlink();
    HD44780_NoCursor();
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(title);

    snprintf(buf, sizeof(buf), "> Unit: %s", unitStr);
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(buf);
}

/* ========================================================================== */
/*                         Public Interface Functions                         */
/* ========================================================================== */

void UiMenu_Init(void) {
    sMenuState = MENU_OVERVIEW;
    mainMenuCursor = 0;
    digit_cursor = 2;
    list_cursor = 0;
    isModifyingSyringe = false;
    tempWeight_g = 1.0f;
    tempZFactor = 1.0f;
    tempDirection = DIR_FORWARD;
    currentDirection = DIR_FORWARD;
    actionSelection = 0;
    deleteConfirmSelection = 1;
    diaModeSelection = 0;
    anim_idx = 0;
    cancelSelection = 1;
    stallAlertSelection = 0;

    const SyringeDatabase_t* db = FlashStorage_GetDB();
    const SyringeProfile_t *pFirst = SyringeDB_Get(db, 0);
    if (pFirst != NULL) {
        gPumpJob.syringeDiameter_mm = pFirst->innerDiameter_mm;
        gPumpJob.syringeMaxVolume_ml = pFirst->maxVolume_ml;
        strncpy(gPumpJob.syringeName, pFirst->name, sizeof(gPumpJob.syringeName) - 1);
        gPumpJob.syringeName[sizeof(gPumpJob.syringeName) - 1] = '\0';
    } else {
        gPumpJob.syringeDiameter_mm = 0.0f;
        gPumpJob.syringeMaxVolume_ml = 0.0f;
        strncpy(gPumpJob.syringeName, "NO SYRINGE", sizeof(gPumpJob.syringeName));
        gPumpJob.syringeName[sizeof(gPumpJob.syringeName) - 1] = '\0';
    }

    if (gPumpJob.targetVolume < 0.001f || gPumpJob.targetVolume > 1000.0f) {
        gPumpJob.targetVolume = 1.0f;
        gPumpJob.volumeUnit = VOL_UNIT_ML;
    }
    if (gPumpJob.targetFlowRate < 0.001f || gPumpJob.targetFlowRate > 1000.0f) {
        gPumpJob.targetFlowRate = 2.0f;
        gPumpJob.flowUnit = FLOW_UNIT_ML_MIN;
    }

    DrawMainMenu();
}

MenuState_e UiMenu_GetState(void) {
    return sMenuState;
}

bool UiMenu_IsPumpingView(void) {
    return (sMenuState == MENU_PUMPING_VIEW);
}

MotorDirection_e UiMenu_GetDirection(void) {
    return currentDirection;
}

void UiMenu_OnMotorComplete(void) {
    sMenuState = MENU_PUMP_COMPLETED;
    mainMenuCursor = 0;
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Pump Completed!");
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("Press any button");
}

void UiMenu_OnPauseRequested(void) {
    sMenuState = MENU_PUMP_PAUSE_CONFIRM;
    cancelSelection = 1; /* Default selection: NO (don't cancel) */

    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Cancel Pump?");
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("  YES    > NO");
}

void UiMenu_OnStallAlert(void) {
    sMenuState = MENU_STALL_ALERT;
    stallAlertSelection = 0; /* Default selection: Continue */

    HD44780_NoBlink();
    HD44780_NoCursor();
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("! MOTOR STALLED !");
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(">Continue  Abort");
}

void UiMenu_OnCalibrationMotorComplete(void) {
    sMenuState = MENU_ADD_SYRINGE_AUTO_WEIGHT;
    tempWeight_g = 1.0f;
    tempZFactor = 1.0f;
    digit_cursor = 5; /* Ones digit (1.00000) */

    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Weight (g):");
    PrintNumberWithCursor(tempWeight_g, "", "g", 5);
}

void UiMenu_UpdatePumpingView(float livePumpedVolume_ml) {
    if (sMenuState != MENU_PUMPING_VIEW) {
        return;
    }
    anim_idx++;

    float target_ml = (gPumpJob.volumeUnit == VOL_UNIT_UL) ? (gPumpJob.targetVolume / 1000.0f) : gPumpJob.targetVolume;
    float remaining_ml = target_ml - livePumpedVolume_ml;
    if (remaining_ml < 0.0f) remaining_ml = 0.0f;

    float flow_ml_min = pump_flow_to_ml_min(gPumpJob.targetFlowRate, (PumpFlowUnit_e)gPumpJob.flowUnit);
    uint32_t remaining_sec = (uint32_t)pump_calc_remaining_time_sec(remaining_ml, flow_ml_min);

    uint32_t h = remaining_sec / 3600;
    if (h > 99) h = 99;
    uint32_t m = (remaining_sec % 3600) / 60;
    uint32_t s = remaining_sec % 60;

    char row1[24];
    if ((anim_idx % 4) < 2) {
        snprintf(row1, sizeof(row1), "%.1f%s         ", gPumpJob.targetFlowRate, GetFlowUnitStr(gPumpJob.flowUnit));
    } else {
        snprintf(row1, sizeof(row1), "        %02u:%02u:%02u", (unsigned int)h, (unsigned int)m, (unsigned int)s);
    }

    char row2[24];
    snprintf(row2, sizeof(row2), "%.2f/%.2fmL", livePumpedVolume_ml, target_ml);

    HD44780_NoCursor();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(row1);
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(row2);
}

UiMenuAction_e UiMenu_ProcessEvent(InputEvent_e event) {
    if (event == EVENT_STALL_DETECTED) {
        UiMenu_OnStallAlert();
        return UI_ACTION_NONE;
    }

    switch (sMenuState) {

        /* ----- MAIN MENU OVERVIEW ----- */
        case MENU_OVERVIEW:
            if (event == EVENT_ENC_CW) {
                if (mainMenuCursor < MAIN_MENU_MAX - 1) {
                    mainMenuCursor++;
                    DrawMainMenu();
                }
            } 
            else if (event == EVENT_ENC_CCW) {
                if (mainMenuCursor > 0) {
                    mainMenuCursor--;
                    DrawMainMenu();
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                switch ((MainMenuItem_e)mainMenuCursor) {
                    case MAIN_MENU_SELECT_SYRINGE: {
                        const SyringeDatabase_t *db = FlashStorage_GetDB();
                        const SyringeProfile_t *syr = SyringeDB_Get(db, 0);
                        if (syr == NULL) {
                            break;  /* Silently ignore if no syringes saved */
                        }
                        sMenuState = MENU_SELECT_SYRINGE_LIST;
                        list_cursor = 0;
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(syr->name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.1fmm, %.1fml",
                                syr->innerDiameter_mm,
                                syr->maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                        break;
                    }
                    case MAIN_MENU_VOLUME:
                        sMenuState = MENU_EDIT_VOLUME_UNIT;
                        tempVolUnit = gPumpJob.volumeUnit;
                        PrintUnitSelect("Select Unit", GetVolumeUnitStr(tempVolUnit));
                        break;
                    case MAIN_MENU_FLOWRATE:
                        sMenuState = MENU_EDIT_FLOWRATE_UNIT;
                        tempFlowUnit = gPumpJob.flowUnit;
                        PrintUnitSelect("Select Unit", GetFlowUnitStr(tempFlowUnit));
                        break;
                    case MAIN_MENU_ADD_SYRINGE:
                        sMenuState = MENU_ADD_SYRINGE_DIA_MODE;
                        isModifyingSyringe = false;
                        tempAddSyringe.maxVolume_ml = 0.0f;
                        tempAddSyringe.innerDiameter_mm = 0.0f;
                        memset(tempAddSyringe.name, ' ', SYRINGE_NAME_LEN - 1);
                        tempAddSyringe.name[SYRINGE_NAME_LEN - 1] = '\0';
                        diaModeSelection = 0; /* Default to MANUAL */
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Dia Method:");
                        HD44780_SetCursor(0, 1);
                        HD44780_PrintStr(">MANUAL   AUTO");
                        break;
                    case MAIN_MENU_DIRECTION:
                        sMenuState = MENU_EDIT_DIRECTION;
                        tempDirection = currentDirection;
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Direction:");
                        HD44780_SetCursor(0, 1);
                        if (tempDirection == DIR_FORWARD) {
                            HD44780_PrintStr(">Forward   BackW");
                        } else {
                            HD44780_PrintStr(" Forward  >BackW");
                        }
                        break;
                    case MAIN_MENU_PUMP:
                    {
                        const SyringeDatabase_t *db = FlashStorage_GetDB();
                        if (!SyringeDB_IsValid(db) || db->count == 0) {
                            sMenuState = MENU_WARNING_NO_SYRINGE;
                            HD44780_Clear();
                            HD44780_SetCursor(0, 0);
                            HD44780_PrintStr("! No Syringe !");
                            HD44780_SetCursor(0, 1);
                            HD44780_PrintStr("  DB is Empty ");
                            break;
                        }
                        
                        float vol_ml = (gPumpJob.volumeUnit == VOL_UNIT_UL) ? (gPumpJob.targetVolume / 1000.0f) : gPumpJob.targetVolume;
                        if (gPumpJob.syringeMaxVolume_ml > 0.001f && vol_ml > gPumpJob.syringeMaxVolume_ml) {
                            sMenuState = MENU_WARNING_VOLUME;
                            HD44780_Clear();
                            HD44780_SetCursor(0, 0);
                            HD44780_PrintStr("! HIGH VOLUME !");
                            HD44780_SetCursor(0, 1);
                            HD44780_PrintStr("Exceeds Syringe");
                            break;
                        }
                        
                        sMenuState = MENU_PUMPING_VIEW;
                        HD44780_Clear();
                        HD44780_SetCursor(0, 0);
                        HD44780_PrintStr("Starting...");
                        return UI_ACTION_START_PUMP;
                    }
                    case MAIN_MENU_FREE_MOVE:
                        sMenuState = MENU_FREE_MOVE_VIEW;
                        HD44780_Clear();
                        HD44780_SetCursor(1, 0);
                        HD44780_PrintStr("*MANUEL MODE*");
                        HD44780_SetCursor(0, 1);
                        HD44780_PrintStr("USE L/R BUTTONS");
                        break;
                    default:
                        break;
                }
            }
            break;

        /* ----- VOLUME UNIT SELECTION ----- */
        case MENU_EDIT_VOLUME_UNIT:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW) {
                tempVolUnit = (tempVolUnit == VOL_UNIT_ML) ? VOL_UNIT_UL : VOL_UNIT_ML;
                PrintUnitSelect("Select Unit", GetVolumeUnitStr(tempVolUnit));
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                gPumpJob.volumeUnit = tempVolUnit;
                tempVolume = ClampVolume(gPumpJob.targetVolume, gPumpJob.volumeUnit);
                sMenuState = MENU_EDIT_VOLUME;
                digit_cursor = 2; /* Default to ones digit */
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Edit Volume");
                PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- VOLUME NUMERIC EDITING ----- */
        case MENU_EDIT_VOLUME:
            if (event == EVENT_ENC_CW) {
                tempVolume += gUIEditorMults[digit_cursor];
                tempVolume = ClampVolume(tempVolume, gPumpJob.volumeUnit);
                PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
            }
            else if (event == EVENT_ENC_CCW) {
                tempVolume -= gUIEditorMults[digit_cursor];
                tempVolume = ClampVolume(tempVolume, gPumpJob.volumeUnit);
                PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                UIEditor_CursorLeft(&digit_cursor, 6);
                PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
            }
            else if (event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                UIEditor_CursorRight(&digit_cursor, 0);
                PrintNumberWithCursor(tempVolume, "", GetVolumeUnitStr(gPumpJob.volumeUnit), 2);
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                gPumpJob.targetVolume = tempVolume;
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- FLOW RATE UNIT SELECTION ----- */
        case MENU_EDIT_FLOWRATE_UNIT:
            if (event == EVENT_ENC_CW) {
                tempFlowUnit = (FlowUnit_e)((tempFlowUnit + 1) % 4);
                PrintUnitSelect("Select Unit", GetFlowUnitStr(tempFlowUnit));
            }
            else if (event == EVENT_ENC_CCW) {
                tempFlowUnit = (FlowUnit_e)((tempFlowUnit == 0) ? 3 : tempFlowUnit - 1);
                PrintUnitSelect("Select Unit", GetFlowUnitStr(tempFlowUnit));
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                gPumpJob.flowUnit = tempFlowUnit;
                tempFlowRate = ClampFlowRate(gPumpJob.targetFlowRate, gPumpJob.flowUnit);
                sMenuState = MENU_EDIT_FLOWRATE;
                digit_cursor = 2;
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Edit Flow Rate");
                PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- FLOW RATE NUMERIC EDITING ----- */
        case MENU_EDIT_FLOWRATE:
            if (event == EVENT_ENC_CW) {
                tempFlowRate += gUIEditorMults[digit_cursor];
                tempFlowRate = ClampFlowRate(tempFlowRate, gPumpJob.flowUnit);
                PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
            }
            else if (event == EVENT_ENC_CCW) {
                tempFlowRate -= gUIEditorMults[digit_cursor];
                tempFlowRate = ClampFlowRate(tempFlowRate, gPumpJob.flowUnit);
                PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                UIEditor_CursorLeft(&digit_cursor, 6);
                PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
            }
            else if (event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                UIEditor_CursorRight(&digit_cursor, 0);
                PrintNumberWithCursor(tempFlowRate, "", GetFlowUnitStr(gPumpJob.flowUnit), 2);
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                gPumpJob.targetFlowRate = tempFlowRate;
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- ADD SYRINGE: DIA MODE SELECTION ----- */
        case MENU_ADD_SYRINGE_DIA_MODE:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW) {
                diaModeSelection = !diaModeSelection;
                HD44780_SetCursor(0, 1);
                if (diaModeSelection == 0) {
                    HD44780_PrintStr(">MANUAL   AUTO");
                } else {
                    HD44780_PrintStr(" MANUAL  >AUTO");
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                if (diaModeSelection == 0) { /* MANUAL */
                    sMenuState = MENU_ADD_SYRINGE_DIA;
                    digit_cursor = 2;
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Add Syringe Dia");
                    PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
                } else { /* AUTO */
                    sMenuState = MENU_ADD_SYRINGE_AUTO_PROMPT;
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Fill w/ Water");
                    HD44780_SetCursor(0, 1);
                    HD44780_PrintStr("Press Select ->");
                }
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- ADD SYRINGE: AUTO PROMPT ----- */
        case MENU_ADD_SYRINGE_AUTO_PROMPT:
            if (event == EVENT_BTN_SELECT_PRESS) {
                sMenuState = MENU_ADD_SYRINGE_AUTO_PUMPING;
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Pumping 10mm...");
                return UI_ACTION_START_CALIBRATION_PUMP;
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- ADD SYRINGE: AUTO PUMPING ----- */
        case MENU_ADD_SYRINGE_AUTO_PUMPING:
            /* Awaiting motor complete (handled via UiMenu_OnCalibrationMotorComplete) */
            break;

        /* ----- ADD SYRINGE: AUTO WEIGHT ----- */
        case MENU_ADD_SYRINGE_AUTO_WEIGHT:
            if (event == EVENT_ENC_CW) {
                UIEditor_Increment(&tempWeight_g, digit_cursor, gUIEditorWeightMults, UI_EDITOR_WEIGHT_MULTS_COUNT, 100.0f);
                PrintNumberWithCursor(tempWeight_g, "", "g", 5);
            }
            else if (event == EVENT_ENC_CCW) {
                UIEditor_Decrement(&tempWeight_g, digit_cursor, gUIEditorWeightMults, UI_EDITOR_WEIGHT_MULTS_COUNT, 0.0f);
                PrintNumberWithCursor(tempWeight_g, "", "g", 5);
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                UIEditor_CursorLeft(&digit_cursor, 7);
                PrintNumberWithCursor(tempWeight_g, "", "g", 5);
            }
            else if (event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                UIEditor_CursorRight(&digit_cursor, 0);
                PrintNumberWithCursor(tempWeight_g, "", "g", 5);
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                sMenuState = MENU_ADD_SYRINGE_AUTO_Z_FACTOR;
                tempZFactor = 1.0f;
                digit_cursor = 5; /* Ones digit (1.00000) */
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Z-Factor (ISO):");
                PrintNumberWithCursor(tempZFactor, "", "uL/mg", 5);
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- ADD SYRINGE: AUTO Z-FACTOR (ISO 8655) ----- */
        case MENU_ADD_SYRINGE_AUTO_Z_FACTOR:
            if (event == EVENT_ENC_CW) {
                UIEditor_Increment(&tempZFactor, digit_cursor, gUIEditorWeightMults, UI_EDITOR_WEIGHT_MULTS_COUNT, 10.0f);
                PrintNumberWithCursor(tempZFactor, "", "uL/mg", 5);
            }
            else if (event == EVENT_ENC_CCW) {
                UIEditor_Decrement(&tempZFactor, digit_cursor, gUIEditorWeightMults, UI_EDITOR_WEIGHT_MULTS_COUNT, 0.0f);
                PrintNumberWithCursor(tempZFactor, "", "uL/mg", 5);
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                UIEditor_CursorLeft(&digit_cursor, 6);
                PrintNumberWithCursor(tempZFactor, "", "uL/mg", 5);
            }
            else if (event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                UIEditor_CursorRight(&digit_cursor, 0);
                PrintNumberWithCursor(tempZFactor, "", "uL/mg", 5);
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                tempAddSyringe.innerDiameter_mm = pump_calc_diameter_from_weight(tempWeight_g, tempZFactor, CALIBRATION_PUMP_DISTANCE_MM);
                sMenuState = MENU_ADD_SYRINGE_DIA;
                digit_cursor = 2;
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Add Syringe Dia");
                PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- ADD SYRINGE: DIAMETER ENTRY ----- */
        case MENU_ADD_SYRINGE_DIA:
            if (event == EVENT_ENC_CW) {
                UIEditor_Increment(&tempAddSyringe.innerDiameter_mm, digit_cursor, gUIEditorMults, UI_EDITOR_MULTS_COUNT, 50.0f);
                PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
            }
            else if (event == EVENT_ENC_CCW) {
                UIEditor_Decrement(&tempAddSyringe.innerDiameter_mm, digit_cursor, gUIEditorMults, UI_EDITOR_MULTS_COUNT, 0.0f);
                PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                UIEditor_CursorLeft(&digit_cursor, 3);
                PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
            }
            else if (event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                UIEditor_CursorRight(&digit_cursor, 0);
                PrintNumberWithCursor(tempAddSyringe.innerDiameter_mm, "", "mm", 2);
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                sMenuState = MENU_ADD_SYRINGE_VOL;
                digit_cursor = 2; /* Start at ones digit */
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Syringe Volume");
                PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- ADD SYRINGE: VOLUME ENTRY ----- */
        case MENU_ADD_SYRINGE_VOL:
            if (event == EVENT_ENC_CW) {
                UIEditor_Increment(&tempAddSyringe.maxVolume_ml, digit_cursor, gUIEditorMults, UI_EDITOR_MULTS_COUNT, 60.0f);
                PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
            }
            else if (event == EVENT_ENC_CCW) {
                UIEditor_Decrement(&tempAddSyringe.maxVolume_ml, digit_cursor, gUIEditorMults, UI_EDITOR_MULTS_COUNT, 0.0f);
                PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                UIEditor_CursorLeft(&digit_cursor, 3);
                /* Skip hundredths position for volume (only xx.x precision) */
                if (digit_cursor == 1) digit_cursor++;
                PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
            }
            else if (event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                UIEditor_CursorRight(&digit_cursor, 1);
                if (digit_cursor == 1) digit_cursor--;
                PrintNumberWithCursor(tempAddSyringe.maxVolume_ml, "", "mL", 2);
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                sMenuState = MENU_ADD_SYRINGE_NAME;
                digit_cursor = 0; /* REPURPOSED: now indexes character position (0–14) */
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Add Name:");
                HD44780_SetCursor(0, 1);
                HD44780_PrintStr(tempAddSyringe.name);
                HD44780_Cursor(); /* Enable hardware underline cursor */
                HD44780_SetCursor(digit_cursor, 1);
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- ADD SYRINGE: NAME ENTRY ----- */
        case MENU_ADD_SYRINGE_NAME:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW) {
                int charsetIdx = 0;
                for (int i = 0; i < 37; i++) {
                    if (tempAddSyringe.name[digit_cursor] == syringeNameCharset[i][0]) {
                        charsetIdx = i;
                        break;
                    }
                }
                if (event == EVENT_ENC_CW) {
                    charsetIdx = (charsetIdx + 1) % 37;
                } else {
                    charsetIdx = (charsetIdx == 0) ? 36 : charsetIdx - 1;
                }
                tempAddSyringe.name[digit_cursor] = syringeNameCharset[charsetIdx][0];

                HD44780_NoCursor();
                HD44780_SetCursor(digit_cursor, 1);
                HD44780_PrintStr(syringeNameCharset[charsetIdx]);
                HD44780_Cursor();
                HD44780_SetCursor(digit_cursor, 1);
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                if (digit_cursor > 0) {
                    digit_cursor--;
                    HD44780_SetCursor(digit_cursor, 1);
                }
            }
            else if (event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                if (digit_cursor < SYRINGE_NAME_LEN - 2) { 
                    digit_cursor++;
                    HD44780_SetCursor(digit_cursor, 1);
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                HD44780_NoCursor();
                if (isModifyingSyringe) {
                    FlashStorage_ModifySyringe(list_cursor, &tempAddSyringe);
                } else {
                    FlashStorage_AddSyringe(&tempAddSyringe);
                }
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                HD44780_NoCursor();
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- SYRINGE SELECTION LIST ----- */
        case MENU_SELECT_SYRINGE_LIST:
            if (event == EVENT_ENC_CW) {
                const SyringeDatabase_t *db = FlashStorage_GetDB();
                if (db != NULL && db->count > 0) {
                    if (list_cursor < db->count - 1) list_cursor++;
                    else list_cursor = 0;

                    const SyringeProfile_t *syr = SyringeDB_Get(db, list_cursor);
                    if (syr != NULL) {
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(syr->name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                syr->innerDiameter_mm,
                                syr->maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                    }
                }
            }
            else if (event == EVENT_ENC_CCW) {
                const SyringeDatabase_t *db = FlashStorage_GetDB();
                if (db != NULL && db->count > 0) {
                    if (list_cursor > 0) list_cursor--;
                    else list_cursor = db->count - 1;

                    const SyringeProfile_t *syr = SyringeDB_Get(db, list_cursor);
                    if (syr != NULL) {
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(syr->name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                syr->innerDiameter_mm,
                                syr->maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                    }
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                const SyringeProfile_t *syr = SyringeDB_Get(FlashStorage_GetDB(), list_cursor);
                if (syr != NULL) {
                    gPumpJob.syringeDiameter_mm = syr->innerDiameter_mm;
                    gPumpJob.syringeMaxVolume_ml = syr->maxVolume_ml;
                    strncpy(gPumpJob.syringeName, syr->name, sizeof(gPumpJob.syringeName) - 1);
                    gPumpJob.syringeName[sizeof(gPumpJob.syringeName) - 1] = '\0';
                }
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_SYRINGE_ACTION_SELECT;
                actionSelection = 0; /* Default to MODIFY */
                HD44780_Clear();
                HD44780_SetCursor(0, 0);
                HD44780_PrintStr("Select Action:");
                HD44780_SetCursor(0, 1);
                HD44780_PrintStr(">MODIFY   DELETE");
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- SYRINGE ACTION SELECT ----- */
        case MENU_SYRINGE_ACTION_SELECT:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW) {
                actionSelection = !actionSelection;
                HD44780_SetCursor(0, 1);
                if (actionSelection == 0) {
                    HD44780_PrintStr(">MODIFY   DELETE");
                } else {
                    HD44780_PrintStr(" MODIFY  >DELETE");
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                if (actionSelection == 0) { /* MODIFY */
                    isModifyingSyringe = true;
                    const SyringeProfile_t *syr = SyringeDB_Get(FlashStorage_GetDB(), list_cursor);
                    if (syr != NULL) {
                        tempAddSyringe = *syr;
                    }
                    sMenuState = MENU_ADD_SYRINGE_DIA_MODE;
                    diaModeSelection = 0;
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Diameter Method:");
                    HD44780_SetCursor(0, 1);
                    HD44780_PrintStr(">MANUAL   AUTO");
                } else { /* DELETE */
                    sMenuState = MENU_DELETE_SYRINGE_CONFIRM;
                    deleteConfirmSelection = 1; /* NO by default */
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Delete Syringe?");
                    HD44780_SetCursor(0, 1);
                    HD44780_PrintStr("  YES    > NO");
                }
            }
            else if (event == EVENT_BTN_CURSOR_LEFT_PRESS) {
                sMenuState = MENU_SELECT_SYRINGE_LIST;
                const SyringeProfile_t *syr = SyringeDB_Get(FlashStorage_GetDB(), list_cursor);
                if (syr != NULL) {
                    HD44780_Clear();
                    HD44780_PrintStr("> ");
                    HD44780_PrintStr(syr->name);
                    HD44780_SetCursor(0, 1);
                    char sbuf[17];
                    snprintf(sbuf, sizeof(sbuf), "D:%.1fmm, %.1fmL",
                            syr->innerDiameter_mm,
                            syr->maxVolume_ml);
                    HD44780_PrintStr(sbuf);
                }
            }
            break;

        /* ----- DELETE SYRINGE CONFIRM ----- */
        case MENU_DELETE_SYRINGE_CONFIRM:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW) {
                deleteConfirmSelection = !deleteConfirmSelection;
                HD44780_SetCursor(0, 1);
                if (deleteConfirmSelection == 0) {
                    HD44780_PrintStr("> YES      NO");
                } else {
                    HD44780_PrintStr("  YES    > NO");
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                if (deleteConfirmSelection == 0) { /* YES */
                    FlashStorage_DeleteSyringe(list_cursor);
                    const SyringeDatabase_t *db = FlashStorage_GetDB();
                    if (db == NULL || db->count == 0) {
                        gPumpJob.syringeDiameter_mm = 0.0f;
                        gPumpJob.syringeMaxVolume_ml = 0.0f;
                        strncpy(gPumpJob.syringeName, "NO SYRINGE", sizeof(gPumpJob.syringeName));
                        gPumpJob.syringeName[sizeof(gPumpJob.syringeName) - 1] = '\0';
                        sMenuState = MENU_OVERVIEW;
                        DrawMainMenu();
                    } else {
                        if (list_cursor >= db->count) {
                            list_cursor = db->count - 1;
                        }
                        sMenuState = MENU_SELECT_SYRINGE_LIST;
                        const SyringeProfile_t *syr = SyringeDB_Get(db, list_cursor);
                        if (syr != NULL) {
                            HD44780_Clear();
                            HD44780_PrintStr("> ");
                            HD44780_PrintStr(syr->name);
                            HD44780_SetCursor(0, 1);
                            char sbuf[17];
                            snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                    syr->innerDiameter_mm,
                                    syr->maxVolume_ml);
                            HD44780_PrintStr(sbuf);
                        }
                    }
                } else { /* NO */
                    sMenuState = MENU_SELECT_SYRINGE_LIST;
                    const SyringeProfile_t *syr = SyringeDB_Get(FlashStorage_GetDB(), list_cursor);
                    if (syr != NULL) {
                        HD44780_Clear();
                        HD44780_PrintStr("> ");
                        HD44780_PrintStr(syr->name);
                        HD44780_SetCursor(0, 1);
                        char sbuf[17];
                        snprintf(sbuf, sizeof(sbuf), "D:%.2fmm,%.1fmL",
                                syr->innerDiameter_mm,
                                syr->maxVolume_ml);
                        HD44780_PrintStr(sbuf);
                    }
                }
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_SELECT_SYRINGE_LIST;
                const SyringeProfile_t *syr = SyringeDB_Get(FlashStorage_GetDB(), list_cursor);
                if (syr != NULL) {
                    HD44780_Clear();
                    HD44780_PrintStr("> ");
                    HD44780_PrintStr(syr->name);
                    HD44780_SetCursor(0, 1);
                    char sbuf[17];
                    snprintf(sbuf, sizeof(sbuf), "D:%.1fmm, %.1fmL",
                            syr->innerDiameter_mm,
                            syr->maxVolume_ml);
                    HD44780_PrintStr(sbuf);
                }
            }
            break;

        /* ----- WARNING DIALOGS (dismiss on any button) ----- */
        case MENU_WARNING_VOLUME:
        case MENU_WARNING_NO_SYRINGE:
            if (event == EVENT_BTN_SELECT_PRESS || event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- PUMPING VIEW (async refresh handled by timer) ----- */
        case MENU_PUMPING_VIEW:
            break;

        /* ----- PAUSE CONFIRMATION DIALOG ----- */
        case MENU_PUMP_PAUSE_CONFIRM:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW) {
                cancelSelection = !cancelSelection;
                HD44780_SetCursor(0, 1);
                if (cancelSelection == 0) HD44780_PrintStr("> YES      NO");
                else HD44780_PrintStr("  YES    > NO");
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                if (cancelSelection == 0) {
                    /* User confirmed: CANCEL the pump */
                    sMenuState = MENU_PUMP_CANCELLED;
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Pump Cancelled!");
                    HD44780_SetCursor(0, 1);
                    HD44780_PrintStr("Press any button");
                    return UI_ACTION_CANCEL_PUMP;
                } else {
                    /* User selected NO: RESUME the pump */
                    sMenuState = MENU_PUMPING_VIEW;
                    HD44780_Clear();
                    return UI_ACTION_RESUME_PUMP;
                }
            }
            break;

        /* ----- MOTOR STALL ALERT DIALOG ----- */
        case MENU_STALL_ALERT:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW ||
                event == EVENT_BTN_CURSOR_LEFT_PRESS || event == EVENT_BTN_CURSOR_RIGHT_PRESS) {
                stallAlertSelection = (stallAlertSelection == 0) ? 1 : 0;
                HD44780_SetCursor(0, 1);
                if (stallAlertSelection == 0) {
                    HD44780_PrintStr(">Continue  Abort");
                } else {
                    HD44780_PrintStr(" Continue >Abort");
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                if (stallAlertSelection == 0) {
                    /* User selected CONTINUE: resume pump */
                    sMenuState = MENU_PUMPING_VIEW;
                    HD44780_Clear();
                    return UI_ACTION_RESUME_STALL_PUMP;
                } else {
                    /* User selected ABORT: cancel pump */
                    sMenuState = MENU_PUMP_CANCELLED;
                    HD44780_Clear();
                    HD44780_SetCursor(0, 0);
                    HD44780_PrintStr("Pump Cancelled!");
                    HD44780_SetCursor(0, 1);
                    HD44780_PrintStr("Press any button");
                    return UI_ACTION_ABORT_STALL_PUMP;
                }
            }
            break;

        /* ----- POST-PUMP RESULT SCREENS (any button returns to menu) ----- */
        case MENU_PUMP_COMPLETED:
        case MENU_PUMP_CANCELLED:
            if (event != EVENT_NONE) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- MOTOR DIRECTION SELECTION ----- */
        case MENU_EDIT_DIRECTION:
            if (event == EVENT_ENC_CW || event == EVENT_ENC_CCW) {
                tempDirection = (tempDirection == DIR_FORWARD) ? DIR_BACKWARD : DIR_FORWARD;
                HD44780_SetCursor(0, 1);
                if (tempDirection == DIR_FORWARD) {
                    HD44780_PrintStr(">Forward   BackW");
                } else {
                    HD44780_PrintStr(" Forward  >BackW");
                }
            }
            else if (event == EVENT_BTN_SELECT_PRESS) {
                currentDirection = tempDirection;
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            else if (event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        /* ----- MANUAL FREE MOVE SCREEN ----- */
        case MENU_FREE_MOVE_VIEW:
            if (event == EVENT_BTN_SELECT_PRESS || event == EVENT_BTN_SELECT_LONG_PRESS) {
                sMenuState = MENU_OVERVIEW;
                DrawMainMenu();
            }
            break;

        default:
            sMenuState = MENU_OVERVIEW;
            DrawMainMenu();
            break;
    }

    return UI_ACTION_NONE;
}
