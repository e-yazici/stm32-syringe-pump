# Open-Source Syringe Pump

An open-source, precision syringe pump built on the **STM32F303RE** microcontroller with **FreeRTOS** real-time scheduling. Designed for laboratory and research use, it provides configurable flow rates, multiple syringe profiles, and a fully menu-driven interface on a 16×2 character LCD.

> **Mechanical Design Credit**: The 3D-printable mechanical parts used in this project are based on the open-source syringe pump design published by [mass-spec.ru](https://www.mass-spec.ru/projects/diy/syringe_pump/eng/). Many thanks to the authors for sharing their work.
> 
> **LCD driver Credit**: Based on the [STM32 HAL I2C HD44780](https://github.com/eziya/STM32_HAL_I2C_HD44780) library by [eziya](https://github.com/eziya), with FreeRTOS compatibility modifications

---

## Features

- **Precise flow control** with S-curve acceleration profiles for smooth, vibration-free stepper motion
- **Multiple syringe profiles** stored in internal flash — survives power cycles
- **Flexible units**: Volume in µL or mL, flow rate in µL/min, mL/min, µL/hr, or mL/hr
- **Auto-calibration mode**: Determine syringe diameter automatically by pumping a known distance and weighing the dispensed water
- **Manual drive mode**: Move the plunger forward/backward using dedicated buttons
- **Configurable motor direction**: Set forward/backward direction from the menu
- **Real-time pumping display**: Alternates between flow rate and remaining time (HH:MM:SS) with a live volume progress bar
- **Pause & cancel**: Interrupt a running pump job with confirmation dialogs

---

## Hardware

| Component | Specification |
|---|---|
| **MCU** | STM32F303RE (Nucleo-64 board) |
| **Stepper Motor** | NEMA 17, 1.8°/step (200 steps/rev) |
| **Stepper Driver** | Any STEP/DIR driver (Tested with A4988) |
| **Lead Screw** | T8 trapezoidal, 2 mm lead |
| **Display** | HD44780 16×2 LCD via PCF8574 I2C module |
| **Encoder** | Rotary encoder (on TIM1 hardware encoder mode) |
| **Buttons** | 3× tactile: Cursor Left, Select (with long-press), Cursor Right |
| **Communication** | I2C1 (LCD + future sensors) |

### Pin Summary

| Function | GPIO Pin | CubeMX Label |
|---|---|---|
| `Stepper` STEP | PA0 | TIM2_CH1 |
| `Stepper` DIR | PB5 |`stepperDir` |
| `Encoder` A/B | PC0/PC1 | TIM1_CH1/CH2 |
| `Button` Back/Move Cursor Left | PA1 | `buttonCursorLeft` |
| `Button` Select/OK/Pause | PA6 | `buttonSelect` |
| `Button` Move Cursor Right | PA7 |`buttonCursorRight` |
| `I2C` SDA/SCL | PB7/PB6 | I2C1_SDA/SCL

---

## Software Architecture

The firmware follows a **3-task FreeRTOS architecture** with a shared global state. The three tasks communicate through a dedicated FreeRTOS queue and cooperate via a shared system-state struct (`gSysState`).

### Tasks

| Task | Priority | Stack | Trigger | Role |
|---|---|---|---|---|
| **InputTask** | 3 | 128 words | 50 Hz periodic (`vTaskDelayUntil`) | Reads the rotary encoder counter and all button GPIO lines, applies debounce logic, and posts an `InputMessage_t` event to `InputQueue`. The only task that directly touches hardware input peripherals. |
| **UITask** | 2 | 512 words | Blocks on `InputQueue` | Consumes input events and drives the hierarchical LCD menu state machine (`MenuState_e`). Handles all display updates, user prompts, value editing, and syringe profile management. Updates `gSysState` to signal motor commands. |
| **MotorTask** | 4 | 512 words | Blocks on `MotorStartSem` semaphore | Translates a pump request (volume, flow rate, syringe diameter) into stepper kinematics — total step count and timer frequency. Arms the TIM2 PWM output and monitors progress, updating the live display until the move completes or is cancelled. |

### ISR

The **TIM2 PeriodElapsed ISR** (in `main.c`) executes the stepper motion profile in real-time. It walks through three phases — **Accel → Cruise → Decel → Idle** — by updating the timer's Auto-Reload Register (ARR) from a pre-computed S-curve lookup table each step.

---

## Module Overview

### Stepper Driver (`stepper.h` / `stepper.c`)
S-curve acceleration engine using a cosine easing function. Generates a smooth velocity ramp stored in a lookup table, then the TIM2 ISR reads it forward (accel) and backward (decel) to produce a symmetric trapezoidal or triangular motion profile.

### Flash Storage (`flash_storage.h` / `flash_storage.c`)
Non-volatile syringe profile database stored in the last 2 KB page of STM32F3 internal flash. Supports add, modify, delete operations using a read-modify-write strategy with 64-bit doubleword programming. Up to 10 syringe profiles with name, diameter, and volume.

### LCD Driver (`liquidcrystal_i2c.h` / `liquidcrystal_i2c.c`)
HD44780 character LCD driver over I2C via PCF8574 I/O expander. Operates in 4-bit mode. Includes FreeRTOS-compatible delay functions — uses DWT cycle counter before the scheduler starts, and `vTaskDelay()` afterward.

### Menu System (`rtos_menu.h` / `rtos_menu.c`)
The main application logic. Implements a hierarchical state machine (`MenuState_e`) with ~20 states covering:
- Main menu navigation (encoder scrolling)
- Numerical value editing (volume, flow rate, diameter)
- Syringe profile management (add/modify/delete with flash persistence)
- Auto-calibration wizard (pump → weigh → calculate diameter)
- Pumping view with real-time progress
- Pause/cancel confirmation dialogs
- Manual drive mode
- Motor direction selection

---

## Menu Navigation

The main menu is scrolled with the rotary encoder. Press **Select** to enter a sub-menu, and **Cursor Left** (only in Select Syringe) to go back.

```
┌──────────────────┐
│ > Select Syringe │  Choose from saved profiles
│ > Volume         │  Set target volume (µL / mL)
│ > Flowrate       │  Set target flow rate
│ > Add Syringe    │  Create or modify syringe profiles
│ > Set Direction  │  Set motor forward/backward
│ > Pump           │  Start pumping (with pre-flight checks)
│ > Manual Mode    │  Drive the motor with left/right buttons
└──────────────────┘
```

**Long-press Select** on a syringe in the list to access Modify/Delete actions. While pumping, pause/cancel the program.

---

## Testing & Status

Basic functional testing has been completed on real hardware. The full menu flow, syringe profile management (add/modify/delete with flash persistence), encoder navigation, button inputs, and stepper motion with S-curve acceleration have all been verified to work correctly during these initial tests. 

**One known hardware limitation identified during testing:** at 1/16 microstepping, the minimum achievable step frequency becomes a bottleneck at very slow flow rates, making smooth low-speed motion impractical. To address this, the driver will be switched to a **TMC2208**, which offers up to 1/256 microstepping interpolation and significantly smoother low-speed performance.

More rigorous testing is planned going forward. This includes run-time analysis — monitoring per-task stack high-water marks, heap usage, and ISR timing to catch any latent overflow or timing issues — as well as a review of overall system behaviour under extended operation.
