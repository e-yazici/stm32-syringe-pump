/**
 * @file    liquidcrystal_i2c.h
 * @brief   HD44780 LCD driver over I2C via PCF8574 I/O expander.
 * @author  eziya (Original Author)
 *          eyazici (Modified)
 * @date    2026
 *
 * @details This driver is based on the STM32 HAL I2C HD44780 driver by eziya:
 *          https://github.com/eziya/STM32_HAL_I2C_HD44780
 *
 *          Modifications:
 *          - Added comprehensive commentary on the header file.
 *          - Integrated FreeRTOS compatibility improvements in the associated .c source file.
 *
 *          This driver controls a standard HD44780-compatible character LCD
 *          (16×2 or 20×4) through a PCF8574 I2C I/O expander backpack. All
 *          communication uses 4-bit mode, minimizing the number of I/O pins
 *          required.
 *
 *          PCF8574 pin mapping (directly on the LCD backpack):
 *          - P0 = RS  (Register Select)
 *          - P1 = RW  (Read/Write, active-low — always write)
 *          - P2 = EN  (Enable strobe)
 *          - P3 = BL  (Backlight control, active-high)
 *          - P4–P7 = D4–D7 (4-bit data bus)
 *
 * @note    Uses I2C1 peripheral handle (hi2c1, defined externally).
 *          Microsecond delays are implemented via the ARM DWT cycle counter
 *          when the RTOS scheduler is not yet running, and via vTaskDelay()
 *          once FreeRTOS is active.
 */

#ifndef LIQUIDCRYSTAL_I2C_H_
#define LIQUIDCRYSTAL_I2C_H_

#include "stm32f3xx_hal.h"

/** @defgroup LCD_Commands HD44780 Command Set
 *  @brief   Instruction codes sent to the HD44780 controller.
 *  @{
 */
#define LCD_CLEARDISPLAY 0x01    ///< Clear entire display and reset cursor to home
#define LCD_RETURNHOME 0x02      ///< Return cursor to home position (0,0)
#define LCD_ENTRYMODESET 0x04    ///< Set cursor move direction and display shift
#define LCD_DISPLAYCONTROL 0x08  ///< Control display on/off, cursor, and blink
#define LCD_CURSORSHIFT 0x10     ///< Shift cursor or display without changing DDRAM
#define LCD_FUNCTIONSET 0x20     ///< Set data bus width, line count, and font
#define LCD_SETCGRAMADDR 0x40    ///< Set CGRAM address for custom character writes
#define LCD_SETDDRAMADDR 0x80    ///< Set DDRAM address (cursor position)
/** @} */

/** @defgroup LCD_EntryMode Entry Mode Flags
 *  @brief   Flags OR'd with LCD_ENTRYMODESET.
 *  @{
 */
#define LCD_ENTRYRIGHT 0x00           ///< Cursor moves right after each character (decrement mode)
#define LCD_ENTRYLEFT 0x02            ///< Cursor moves left after each character (increment mode)
#define LCD_ENTRYSHIFTINCREMENT 0x01  ///< Shift the display when writing
#define LCD_ENTRYSHIFTDECREMENT 0x00  ///< Do not shift the display
/** @} */

/** @defgroup LCD_DisplayControl Display On/Off Control Flags
 *  @brief   Flags OR'd with LCD_DISPLAYCONTROL.
 *  @{
 */
#define LCD_DISPLAYON 0x04   ///< Turn display on
#define LCD_DISPLAYOFF 0x00  ///< Turn display off (data retained in DDRAM)
#define LCD_CURSORON 0x02    ///< Show underline cursor
#define LCD_CURSOROFF 0x00   ///< Hide cursor
#define LCD_BLINKON 0x01     ///< Enable blinking block cursor
#define LCD_BLINKOFF 0x00    ///< Disable blinking
/** @} */

/** @defgroup LCD_CursorShift Cursor/Display Shift Flags
 *  @brief   Flags OR'd with LCD_CURSORSHIFT.
 *  @{
 */
#define LCD_DISPLAYMOVE 0x08  ///< Shift the entire display
#define LCD_CURSORMOVE 0x00   ///< Move the cursor only
#define LCD_MOVERIGHT 0x04    ///< Shift/move direction: right
#define LCD_MOVELEFT 0x00     ///< Shift/move direction: left
/** @} */

/** @defgroup LCD_FunctionSet Function Set Flags
 *  @brief   Flags OR'd with LCD_FUNCTIONSET.
 *  @{
 */
#define LCD_8BITMODE 0x10  ///< 8-bit data bus interface
#define LCD_4BITMODE 0x00  ///< 4-bit data bus interface (used by this driver)
#define LCD_2LINE 0x08     ///< Two-line display mode
#define LCD_1LINE 0x00     ///< Single-line display mode
#define LCD_5x10DOTS 0x04  ///< 5×10 pixel character font
#define LCD_5x8DOTS 0x00   ///< 5×8 pixel character font (standard)
/** @} */

/** @defgroup LCD_Backlight Backlight Control
 *  @brief   PCF8574 P3 pin values for backlight control.
 *  @{
 */
#define LCD_BACKLIGHT 0x08    ///< Backlight ON  (P3 = high)
#define LCD_NOBACKLIGHT 0x00  ///< Backlight OFF (P3 = low)
/** @} */

/** @defgroup LCD_ControlBits PCF8574 Control Pin Bitmasks
 *  @brief   Bit positions for the Enable, R/W, and RS signals.
 *  @{
 */
#define ENABLE 0x04  ///< Enable strobe bit (P2) — pulsed high then low to latch data
#define RW 0x0       ///< Read/Write bit (P1) — always 0 for write operations
#define RS 0x01      ///< Register Select bit (P0) — 0 = command, 1 = data
/** @} */

/** @brief PCF8574 I2C slave address (7-bit = 0x3F, left-shifted for HAL) */
#define DEVICE_ADDR     (0x27 << 1)

/* -------------------------------------------------------------------------- */
/*                          Public API Functions                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize the LCD in 4-bit mode with the specified number of rows.
 * @param  rows Number of display rows (1, 2, or 4).
 * @note   This function executes the HD44780 power-on initialization sequence
 *         including the required delays (>40ms after power-on, 4.1ms and 100µs
 *         between retries). It also loads two custom characters into CGRAM
 *         slots 0 and 1.
 */
void HD44780_Init(uint8_t rows);

/** @brief Clear all display content and return cursor to position (0,0). */
void HD44780_Clear();

/** @brief Return cursor to home position (0,0) without clearing display. */
void HD44780_Home();

/** @brief Turn display OFF (content remains in DDRAM). */
void HD44780_NoDisplay();

/** @brief Turn display ON. */
void HD44780_Display();

/** @brief Disable blinking block cursor. */
void HD44780_NoBlink();

/** @brief Enable blinking block cursor. */
void HD44780_Blink();

/** @brief Hide the underline cursor. */
void HD44780_NoCursor();

/** @brief Show the underline cursor. */
void HD44780_Cursor();

/** @brief Scroll entire display content one position to the left. */
void HD44780_ScrollDisplayLeft();

/** @brief Scroll entire display content one position to the right. */
void HD44780_ScrollDisplayRight();

/** @brief Set text direction to print left-to-right (default). */
void HD44780_PrintLeft();

/** @brief Set text direction to print right-to-left. */
void HD44780_PrintRight();

/** @brief Set entry mode to left-to-right (cursor increment). */
void HD44780_LeftToRight();

/** @brief Set entry mode to right-to-left (cursor decrement). */
void HD44780_RightToLeft();

/** @brief Enable auto-scroll: display shifts on each character write. */
void HD44780_ShiftIncrement();

/** @brief Disable auto-scroll. */
void HD44780_ShiftDecrement();

/** @brief Turn the LCD backlight OFF. */
void HD44780_NoBacklight();

/** @brief Turn the LCD backlight ON. */
void HD44780_Backlight();

/** @brief Enable auto-scroll mode (display shifts with writes). */
void HD44780_AutoScroll();

/** @brief Disable auto-scroll mode. */
void HD44780_NoAutoScroll();

/**
 * @brief  Store a custom 5×8 character pattern in CGRAM.
 * @param  location CGRAM slot (0–7).
 * @param  charmap  Array of 8 bytes defining the pixel rows.
 */
void HD44780_CreateSpecialChar(uint8_t, uint8_t[]);

/**
 * @brief  Print a previously stored custom character from CGRAM.
 * @param  index CGRAM slot index (0–7).
 */
void HD44780_PrintSpecialChar(uint8_t);

/**
 * @brief  Set the cursor position.
 * @param  col Column index (0-based).
 * @param  row Row index (0-based).
 */
void HD44780_SetCursor(uint8_t, uint8_t);

/**
 * @brief  Set backlight state programmatically.
 * @param  new_val Non-zero = backlight ON, zero = backlight OFF.
 */
void HD44780_SetBacklight(uint8_t new_val);

/**
 * @brief  Alias for HD44780_CreateSpecialChar — load custom character into CGRAM.
 * @param  char_num CGRAM slot (0–7).
 * @param  rows     Pointer to 8-byte pixel data array.
 */
void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows);

/**
 * @brief  Print a null-terminated string at the current cursor position.
 * @param  c Null-terminated string to display.
 */
void HD44780_PrintStr(const char[]);

#endif /* LIQUIDCRYSTAL_I2C_H_ */
