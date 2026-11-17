/**
 * @file    ui_menu.h
 * @brief   Single-module LCD menu pages interface for the Syringe Pump.
 * @author  eyazici
 * @date    2026
 *
 * @details Encapsulates all 25 LCD screen pages, numeric editing, syringe
 *          database wizard flows, and LCD rendering (HD44780).
 *          Decouples the user interface from FreeRTOS task scheduling
 *          and low-level motor driver execution.
 */

#ifndef LIBS_INC_UI_MENU_H_
#define LIBS_INC_UI_MENU_H_

#include <stdint.h>
#include <stdbool.h>
#include "rtos_tasks.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Action requested by the UI menu for the RTOS/Motor system to execute.
 */
typedef enum {
    UI_ACTION_NONE = 0,               ///< Internal UI transition; no RTOS/Motor action required
    UI_ACTION_START_PUMP,             ///< User launched pumping job
    UI_ACTION_START_CALIBRATION_PUMP, ///< User launched 10mm calibration pump
    UI_ACTION_RESUME_PUMP,            ///< User confirmed resume in pause dialog
    UI_ACTION_CANCEL_PUMP,            ///< User confirmed cancellation in pause dialog
    UI_ACTION_RESUME_STALL_PUMP,      ///< User confirmed resume in stall alert dialog
    UI_ACTION_ABORT_STALL_PUMP        ///< User confirmed abort in stall alert dialog
} UiMenuAction_e;

/**
 * @brief Initialize LCD menu state, cursors, and initial screen.
 *        Loads default syringe profile into gPumpJob if database is non-empty.
 */
void UiMenu_Init(void);

/**
 * @brief Process an input event through the active menu page state machine.
 * @param event Input event from InputQueue (encoder, buttons, etc.).
 * @return UiMenuAction_e Action for TaskUI to execute (start/pause/resume/cancel pump).
 */
UiMenuAction_e UiMenu_ProcessEvent(InputEvent_e event);

/**
 * @brief Get the current menu sub-state.
 * @return Current MenuState_e.
 */
MenuState_e UiMenu_GetState(void);

/**
 * @brief Check if the menu is currently in the active pumping view.
 * @return true if active screen is MENU_PUMPING_VIEW.
 */
bool UiMenu_IsPumpingView(void);

/**
 * @brief Asynchronously refresh the pumping progress view on the LCD.
 * @param livePumpedVolume_ml Current volume delivered in mL.
 */
void UiMenu_UpdatePumpingView(float livePumpedVolume_ml);

/**
 * @brief Handle natural motor completion event while pumping.
 *        Transitions UI to MENU_PUMP_COMPLETED and renders completion screen.
 */
void UiMenu_OnMotorComplete(void);

/**
 * @brief Handle pause requested by user via Select button EXTI during pumping.
 *        Transitions UI to MENU_PUMP_PAUSE_CONFIRM and renders prompt.
 */
void UiMenu_OnPauseRequested(void);

/**
 * @brief Handle motor stall detected event during pumping.
 *        Transitions UI to MENU_STALL_ALERT and renders stall prompt.
 */
void UiMenu_OnStallAlert(void);

/**
 * @brief Handle motor completion for the 10mm calibration pump.
 *        Transitions UI to MENU_ADD_SYRINGE_AUTO_WEIGHT and renders prompt.
 */
void UiMenu_OnCalibrationMotorComplete(void);

/**
 * @brief Get the currently configured motor direction.
 * @return DIR_FORWARD or DIR_BACKWARD.
 */
MotorDirection_e UiMenu_GetDirection(void);

#ifdef __cplusplus
}
#endif

#endif /* LIBS_INC_UI_MENU_H_ */
