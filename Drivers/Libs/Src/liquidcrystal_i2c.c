/**
 * @file    liquidcrystal_i2c.c
 * @brief   HD44780 LCD driver implementation over I2C (PCF8574 expander).
 * @author  eziya (Original Author)
 *          eyazici (Modified)
 * @date    2026
 *
 * @details This driver is based on the STM32 HAL I2C HD44780 driver by eziya:
 *          https://github.com/eziya/STM32_HAL_I2C_HD44780
 *
 *          Modifications:
 *          - Integrated FreeRTOS compatibility improvements (e.g. using vTaskDelay() for delays
 *            >= 1 ms when the scheduler is active).
 *          - Added detailed Doxygen documentation comments.
 *
 *          Implements the HD44780 character LCD initialization sequence and
 *          all display control functions using 4-bit mode over an I2C-connected
 *          PCF8574 I/O expander.
 *
 *          Data flow for each byte sent to the LCD:
 *          1. Split the byte into high nibble and low nibble.
 *          2. For each nibble: write to PCF8574 (sets D4–D7, RS, BL), then
 *             pulse the Enable pin high→low to latch the data.
 *
 *          Timing is critical during initialization (before RTOS starts):
 *          microsecond delays use the ARM DWT cycle counter. Once FreeRTOS is
 *          running, delays >= 1 ms use vTaskDelay() to yield the CPU.
 *
 * @note    Uses I2C1 peripheral handle (hi2c1).
 *          Two custom characters are pre-loaded into CGRAM slots 0 and 1
 *          during initialization.
 */

#include "liquidcrystal_i2c.h"
#include "FreeRTOS.h"
#include "task.h"

/** @brief I2C1 handle (defined in i2c.c by CubeMX) */
extern I2C_HandleTypeDef hi2c1;

/* -------------------------------------------------------------------------- */
/*                           Module-Level State                               */
/* -------------------------------------------------------------------------- */

uint8_t dpFunction;   ///< HD44780 function set register (data width, lines, font)
uint8_t dpControl;    ///< HD44780 display control register (display, cursor, blink)
uint8_t dpMode;       ///< HD44780 entry mode register (direction, shift)
uint8_t dpRows;       ///< Number of LCD rows configured at init
uint8_t dpBacklight;  ///< Current backlight state (LCD_BACKLIGHT or LCD_NOBACKLIGHT)

/* -------------------------------------------------------------------------- */
/*                      Static (Internal) Function Declarations               */
/* -------------------------------------------------------------------------- */

static void SendCommand(uint8_t);   ///< Send a command byte (RS=0)
static void SendChar(uint8_t);      ///< Send a data byte (RS=1)
static void Send(uint8_t, uint8_t); ///< Low-level: split byte into nibbles and transmit
static void Write4Bits(uint8_t);    ///< Write a 4-bit nibble to the LCD via I2C
static void ExpanderWrite(uint8_t); ///< Write a raw byte to the PCF8574
static void PulseEnable(uint8_t);   ///< Pulse the Enable pin high then low
static void DelayInit(void);        ///< Initialize DWT cycle counter for µs delays
static void DelayUS(uint32_t);      ///< Microsecond delay (DWT or vTaskDelay)

/* -------------------------------------------------------------------------- */
/*                        Custom Character Bitmaps                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Custom character bitmap for CGRAM slot 0.
 * @details Each byte represents one row of a 5×8 pixel character (LSB = rightmost pixel).
 */
uint8_t special1[8] = {0b00000, 0b11001, 0b11011, 0b00110,
                       0b01100, 0b11011, 0b10011, 0b00000};

/**
 * @brief Custom character bitmap for CGRAM slot 1 (degree symbol variant).
 */
uint8_t special2[8] = {0b11000, 0b11000, 0b00110, 0b01001,
                       0b01000, 0b01001, 0b00110, 0b00000};

/* ========================================================================== */
/*                         Public API Implementation                          */
/* ========================================================================== */

/**
 * @brief  Initialize the LCD in 4-bit mode.
 * @param  rows Number of display rows (1 or 2).
 *
 * @details Follows the HD44780 datasheet initialization-by-instruction sequence:
 *          1. Wait >40 ms after power-on.
 *          2. Send 0x03 three times (forces 8-bit mode regardless of current state).
 *          3. Send 0x02 to switch to 4-bit mode.
 *          4. Configure function set (lines, font), display control, entry mode.
 *          5. Load custom characters into CGRAM slots 0 and 1.
 *          6. Return cursor to home position.
 */
void HD44780_Init(uint8_t rows) {
  dpRows = rows;

  dpBacklight = LCD_BACKLIGHT;

  dpFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

  if (dpRows > 1) {
    dpFunction |= LCD_2LINE;
  } else {
    dpFunction |= LCD_5x10DOTS;
  }

  /* Initialize DWT cycle counter for microsecond-precision delays */
  DelayInit();
  /* HD44780 requires >40 ms delay after power-on before any commands */
  HAL_Delay(50);

  /* Send backlight state to PCF8574 (wake up the I2C expander) */
  ExpanderWrite(dpBacklight);
  HAL_Delay(1000);

  /*
   * HD44780 initialization-by-instruction sequence (datasheet Figure 24):
   * Must send 0x03 three times to reliably reset the controller to 8-bit mode,
   * regardless of what state it was previously in.
   */
  Write4Bits(0x03 << 4);  /* First attempt: set to 8-bit mode */
  DelayUS(4500);           /* Wait >4.1 ms */

  Write4Bits(0x03 << 4);  /* Second attempt */
  DelayUS(4500);           /* Wait >4.1 ms */

  Write4Bits(0x03 << 4);  /* Third attempt */
  DelayUS(4500);           /* Wait >100 µs (using 4500 for margin) */

  /* Now switch to 4-bit mode: send 0x02 as a single nibble */
  Write4Bits(0x02 << 4);
  DelayUS(100);

  /* Configure function set: 4-bit, N lines, 5×8 font */
  SendCommand(LCD_FUNCTIONSET | dpFunction);

  /* Turn display ON, cursor OFF, blink OFF */
  dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  HD44780_Display();
  HD44780_Clear();

  /* Set entry mode: cursor moves left-to-right, no display shift */
  dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
  DelayUS(4500);

  /* Load custom characters into CGRAM slots 0 and 1 */
  HD44780_CreateSpecialChar(0, special1);
  HD44780_CreateSpecialChar(1, special2);

  HD44780_Home();
}

/**
 * @brief  Clear display and return cursor to home.
 * @note   The clear command takes ~1.52 ms to execute on the HD44780.
 */
void HD44780_Clear() {
  SendCommand(LCD_CLEARDISPLAY);
  DelayUS(2000);  /* HD44780 needs ~1.52 ms to process clear */
}

/**
 * @brief  Return cursor to home position (0,0) without clearing display.
 * @note   The home command takes ~1.52 ms to execute on the HD44780.
 */
void HD44780_Home() {
  SendCommand(LCD_RETURNHOME);
  DelayUS(2000);
}

/**
 * @brief  Set the cursor position on the LCD.
 * @param  col Column index (0-based).
 * @param  row Row index (0-based, clamped to max rows-1).
 *
 * @details Uses the standard DDRAM address offsets for HD44780-compatible displays:
 *          Row 0 starts at 0x00, Row 1 at 0x40, Row 2 at 0x14, Row 3 at 0x54.
 */
void HD44780_SetCursor(uint8_t col, uint8_t row) {
  int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row >= dpRows) {
    row = dpRows - 1;  /* Clamp row to valid range */
  }
  SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/** @brief Turn display OFF (DDRAM contents are preserved). */
void HD44780_NoDisplay() {
  dpControl &= ~LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

/** @brief Turn display ON. */
void HD44780_Display() {
  dpControl |= LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

/** @brief Hide the underline cursor. */
void HD44780_NoCursor() {
  dpControl &= ~LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

/** @brief Show the underline cursor at current position. */
void HD44780_Cursor() {
  dpControl |= LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

/** @brief Disable the blinking block cursor. */
void HD44780_NoBlink() {
  dpControl &= ~LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

/** @brief Enable the blinking block cursor. */
void HD44780_Blink() {
  dpControl |= LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

/** @brief Scroll the entire display one position to the left. */
void HD44780_ScrollDisplayLeft(void) {
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

/** @brief Scroll the entire display one position to the right. */
void HD44780_ScrollDisplayRight(void) {
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/** @brief Set text entry direction to left-to-right (cursor increments). */
void HD44780_LeftToRight(void) {
  dpMode |= LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

/** @brief Set text entry direction to right-to-left (cursor decrements). */
void HD44780_RightToLeft(void) {
  dpMode &= ~LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

/** @brief Enable auto-scroll: display shifts on each character write. */
void HD44780_AutoScroll(void) {
  dpMode |= LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

/** @brief Disable auto-scroll. */
void HD44780_NoAutoScroll(void) {
  dpMode &= ~LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

/**
 * @brief  Store a custom 5×8 character in CGRAM.
 * @param  location CGRAM slot index (0–7, masked to 3 bits).
 * @param  charmap  Array of 8 bytes, each defining one pixel row.
 *
 * @details Sets the CGRAM address to (location * 8) and writes all 8 row bytes.
 *          After this call, the character can be displayed by writing its
 *          slot index (0–7) as data to the DDRAM.
 */
void HD44780_CreateSpecialChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7;  /* Mask to 3 bits — only 8 CGRAM slots available */
  SendCommand(LCD_SETCGRAMADDR | (location << 3));  /* Each char uses 8 bytes */
  for (int i = 0; i < 8; i++) {
    SendChar(charmap[i]);
  }
}

/**
 * @brief  Print a custom character stored in CGRAM.
 * @param  index CGRAM slot index (0–7).
 */
void HD44780_PrintSpecialChar(uint8_t index) { SendChar(index); }

/**
 * @brief  Alias for HD44780_CreateSpecialChar (Arduino-compatible API).
 * @param  char_num CGRAM slot (0–7).
 * @param  rows     Pointer to 8-byte pixel data array.
 */
void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows) {
  HD44780_CreateSpecialChar(char_num, rows);
}

/**
 * @brief  Print a null-terminated string at the current cursor position.
 * @param  c The string to print.
 */
void HD44780_PrintStr(const char c[]) {
  while (*c)
    SendChar(*c++);
}

/**
 * @brief  Set backlight state programmatically.
 * @param  new_val Non-zero = ON, zero = OFF.
 */
void HD44780_SetBacklight(uint8_t new_val) {
  if (new_val)
    HD44780_Backlight();
  else
    HD44780_NoBacklight();
}

/** @brief Turn the LCD backlight OFF via PCF8574 P3 pin. */
void HD44780_NoBacklight(void) {
  dpBacklight = LCD_NOBACKLIGHT;
  ExpanderWrite(0);
}

/** @brief Turn the LCD backlight ON via PCF8574 P3 pin. */
void HD44780_Backlight(void) {
  dpBacklight = LCD_BACKLIGHT;
  ExpanderWrite(0);
}

/* ========================================================================== */
/*                        Static (Internal) Functions                         */
/* ========================================================================== */

/**
 * @brief  Send a command byte to the HD44780 (RS = 0).
 * @param  cmd The command byte to send.
 */
static void SendCommand(uint8_t cmd) { Send(cmd, 0); }

/**
 * @brief  Send a data byte to the HD44780 (RS = 1).
 * @param  ch The data byte (character code or CGRAM data) to send.
 */
static void SendChar(uint8_t ch) { Send(ch, RS); }

/**
 * @brief  Split a byte into two nibbles and send them sequentially.
 * @param  value The byte to send.
 * @param  mode  Control bits to OR with each nibble (0 = command, RS = data).
 *
 * @details In 4-bit mode, each byte requires two write operations:
 *          1. Send the high nibble (bits [7:4]) with mode flags.
 *          2. Send the low nibble (bits [3:0] shifted to [7:4]) with mode flags.
 */
static void Send(uint8_t value, uint8_t mode) {
  uint8_t highnib = value & 0xF0;           /* Extract bits [7:4] in position */
  uint8_t lownib = (value << 4) & 0xF0;     /* Shift bits [3:0] into [7:4] position */
  Write4Bits((highnib) | mode);
  Write4Bits((lownib) | mode);
}

/**
 * @brief  Write a 4-bit nibble to the LCD via the I2C expander.
 * @param  value Nibble in upper 4 bits [7:4], plus RS/RW flags in lower bits.
 *
 * @details First writes the nibble value to set D4–D7 and control lines,
 *          then pulses the Enable pin to latch the data into the HD44780.
 */
static void Write4Bits(uint8_t value) {
  ExpanderWrite(value);
  PulseEnable(value);
}

/**
 * @brief  Write a raw byte to the PCF8574 I2C expander.
 * @param  _data The byte to transmit (OR'd with current backlight state).
 *
 * @details The backlight bit (P3) is automatically OR'd in so that the
 *          backlight state is maintained across all writes.
 */
static void ExpanderWrite(uint8_t _data) {
  uint8_t data = _data | dpBacklight;
  HAL_I2C_Master_Transmit(&hi2c1, DEVICE_ADDR, (uint8_t *)&data, 1, 10);
}

/**
 * @brief  Pulse the Enable pin (P2) high then low to latch data.
 * @param  _data The current data byte (Enable will be OR'd in, then cleared).
 *
 * @details The HD44780 latches data on the falling edge of Enable.
 *          Sequence: write data | EN → wait 20 µs → write data & ~EN → wait 20 µs.
 */
static void PulseEnable(uint8_t _data) {
  ExpanderWrite(_data | ENABLE);  /* EN = high: present data */
  DelayUS(20);

  ExpanderWrite(_data & ~ENABLE); /* EN = low: latch data on falling edge */
  DelayUS(20);
}

/**
 * @brief  Initialize the ARM DWT cycle counter for microsecond timing.
 *
 * @details Enables the Debug Exception and Monitor Control Register (DEMCR)
 *          trace bit, then enables the DWT cycle counter. This provides a
 *          free-running 32-bit counter clocked at SystemCoreClock (80 MHz),
 *          giving sub-microsecond resolution.
 *
 * @note   This must be called before any DelayUS() calls during early init
 *         (before the RTOS scheduler starts). The three NOP instructions
 *         provide a pipeline flush to ensure DWT is fully enabled.
 */
static void DelayInit(void) {
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;  /* Clear trace enable */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* Re-enable trace (unlocks DWT) */

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;  /* Stop cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   /* Start cycle counter */

  DWT->CYCCNT = 0;  /* Reset counter to zero */

  /* Pipeline flush: 3 NOPs ensure DWT is fully initialized before use */
  __ASM volatile("NOP");
  __ASM volatile("NOP");
  __ASM volatile("NOP");
}

/**
 * @brief  Delay for the specified number of microseconds.
 * @param  us Delay duration in microseconds.
 *
 * @details Uses two strategies depending on context:
 *          - If us >= 1000 AND the RTOS scheduler is running: convert to ms
 *            and use vTaskDelay() (yields CPU to other tasks).
 *          - Otherwise: busy-wait using the DWT cycle counter for precise timing.
 *
 *          DWT calculation: cycles = (SystemCoreClock / 1_000_000) * us
 *          At 80 MHz, 1 µs = 80 cycles.
 *
 * @note   The RTOS-aware path avoids busy-waiting during long LCD delays
 *         (e.g., the 1000 ms backlight settle time during init), but falls
 *         back to DWT for short sub-millisecond timing requirements.
 */
static void DelayUS(uint32_t us) {
  if (us >= 1000 && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    /* RTOS path: yield CPU for long delays (>= 1 ms) */
    vTaskDelay(pdMS_TO_TICKS(us / 1000));
  } 
  else {
    /* DWT path: busy-wait with cycle-accurate timing */
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    volatile uint32_t cnt;

    do {
      cnt = DWT->CYCCNT - start;
    } while (cnt < cycles);
  }
}
