# Open-Source Syringe Pump

Syringe pump firmware for the **STM32F303RE** (Nucleo-64) running **FreeRTOS**. A stepper drives a lead screw, an AS5600 magnetic encoder measures the actual plunger travel, and the user interface is a rotary encoder, three buttons and a 16×2 character LCD.

> **Mechanical Design Credit**: The 3D-printable mechanical parts used in this project are based on the open-source syringe pump design published by [mass-spec.ru](https://www.mass-spec.ru/projects/diy/syringe_pump/eng/). Many thanks to the authors for sharing their work.
>
> **LCD driver Credit**: Based on the [STM32 HAL I2C HD44780](https://github.com/eziya/STM32_HAL_I2C_HD44780) library by [eziya](https://github.com/eziya), with FreeRTOS compatibility modifications.
>
> **AS5600 Credit**: AS5600 application notes from [curiousscientist.tech](https://curiousscientist.tech/).

---

## Table of Contents

- [Features](#features)
- [Status](#status)
- [Menu Navigation](#menu-navigation)
- [Build & Tests](#build--tests)
- [Hardware](#hardware)
  - [Pin Summary](#pin-summary)
  - [I2C Addresses](#i2c-addresses)
- [Software Architecture](#software-architecture)
  - [Clocks and Timers](#clocks-and-timers)
  - [Tasks](#tasks)
  - [Stepper ISR](#stepper-isr)
  - [Kinematics](#kinematics)
  - [Closed Loop](#closed-loop)
  - [TMC2208](#tmc2208)
- [Source Layout](#source-layout)
  - [Flash Storage](#flash-storage)

---

## Features

- Target volume and flow rate entry with per-digit cursor editing
- Volume units: µL, mL. Flow rate units: µL/min, mL/min, µL/hr, mL/hr
- S-curve (cosine) acceleration and deceleration of the stepper
- Up to 10 syringe profiles (name, inner diameter, volume) stored in internal flash; add, modify, delete from the menu
- Syringe diameter entry: manual, or automatic calibration (pump 10 mm, weigh the water, diameter is calculated)
- Closed-loop supervision using the AS5600 encoder: speed-adaptive stall detection, encoder filter snap on motor stop, and post-move creep correction
- TMC2208 configured over UART (128 microsteps, SpreadCycle while moving, StealthChop while idle)
- Manual drive mode with the Cursor Left / Cursor Right buttons
- Configurable motor direction (forward / backward)
- Pump screen: flow rate and remaining time (HH:MM:SS) alternate on line 1, pumped/target volume on line 2
- Pause and cancel with confirmation; stall alert with Continue / Abort

---

## Status

All features have been checked with quick functional tests on hardware, and all hardware-independent domain modules are covered by host unit tests. Stress and long-run testing has not been done yet.

---

## Menu Navigation

Scroll the main menu with the rotary encoder. **Select** enters an item and confirms. **Cursor Left / Right** move the digit cursor while editing a value.

```
> Select Syringe   saved syringe profiles
> Volume           target volume and unit
> Flowrate         target flow rate and unit
> Add Syringe      new profile (manual or auto diameter)
> Set Direction    Forward / Backward
> Pump             start (checks: syringe selected, volume fits syringe)
> Manual Mode      drive with Left / Right buttons
```

- **Long-press Select** on a syringe in the list: Modify / Delete.
- **Auto diameter**: fill the syringe with water, the pump moves 10 mm, enter the weighed mass in grams and ISO 8655-6 Z-factor (defaults to `1.00000` uL/mg for no correction, editable up to `10.0`), and the device calculates the diameter.
- **While pumping**: Select pauses and opens the cancel dialog (YES / NO). NO resumes.

---

## Build & Tests

- **Firmware** (CMake + Ninja, `arm-none-eabi-gcc`): `cmake --preset Debug` then `ninja -C build/Debug`. A `Release` preset also exists.
- **Host unit tests**: `tests\run_tests.bat` (or `tests\run_tests.ps1`). Needs a host `gcc`, CMake and Ninja.
- **Memory budget**: 512 KB flash, 64 KB SRAM, 16 KB CCM RAM.

---

## Hardware

| Component | Specification |
|---|---|
| **MCU** | STM32F303RE (Nucleo-64 board) |
| **Stepper Motor** | NEMA 17, 1.8°/step (200 steps/rev) |
| **Stepper Driver** | TMC2208, STEP/DIR with UART configuration (128 microsteps set over UART) |
| **Lead Screw** | T8 trapezoidal, 2 mm lead |
| **Display** | HD44780 16×2 LCD via PCF8574 I2C module |
| **Encoder** | Rotary encoder (on TIM1 hardware encoder mode) |
| **Position Sensor** | AS5600 magnetic angle sensor (I2C), measures actual lead screw travel for closed-loop control |
| **Buttons** | 3× tactile: Cursor Left, Select (with long-press), Cursor Right |
| **Communication** | I2C2 @100kHz (LCD + AS5600), Half-Duplex UART1 @38400 (TMC2208) |

### Pin Summary

| Function | GPIO Pin | CubeMX Label |
|---|---|---|
| `Stepper` STEP | PA0 | TIM2_CH1 |
| `Stepper` DIR | PB5 | `stepperDir` |
| `Encoder` A/B | PC0/PC1 | TIM1_CH1/CH2 |
| `Button` Back/Move Cursor to Left | PA1 | `buttonCursorLeft` |
| `Button` Select/OK/Pause | PA6 | `buttonSelect` |
| `Button` Move Cursor to Right | PA7 | `buttonCursorRight` |
| `I2C` SDA/SCL | PA10/PA9 | I2C2_SDA/SCL |
| `UART` TX | PC4 | USART1_TX |

### I2C Addresses

| Device | 7-bit address |
|---|---|
| PCF8574 LCD backpack | `0x27` |
| AS5600 | `0x36` |

---

## Software Architecture

Five FreeRTOS tasks, created in `main.c`. Shared state lives in `rtos_tasks.c` (`gSysState`, `gPumpJob`, `gPumpHw`, `gStepperControl`, `gLivePumpedVolume_ml`).

### Clocks and Timers

- System clock: 72 MHz (HSI → PLL ×9). FreeRTOS tick: 1 kHz. Heap: `heap_4`, 14 KB.
- **TIM1**: quadrature encoder interface for the rotary encoder.
- **TIM2**: stepper STEP output (CH1, PWM, 50 % duty). Prescaler 71 gives a 1 MHz timer tick; the update ISR rewrites ARR on every step.
- **TIM6**: HAL time base.

### Tasks

| Task | Priority | Stack (words) | Trigger | Role |
|---|---|---|---|---|
| **InputTask** | 3 | 128 | 50 Hz (`vTaskDelayUntil`) | Reads the TIM1 counter and the three buttons, debounces, detects Select long-press (>600 ms), posts `InputMessage_t` to `InputQueue` (10 entries). Suspended while pumping; the Select button is then handled by an EXTI interrupt (PA6) so a pause is detected immediately. |
| **UITask** | 2 | 512 | Blocks on `InputQueue` | Passes events to the menu state machine (`ui_menu.c`) and acts on the returned `UI_ACTION_*` (start, pause/resume, cancel, calibration, stall resume/abort). |
| **MotorTask** | 4 | 512 | Polls `gSysState` every 100 ms | Converts the job (volume, flow rate, syringe diameter) to steps and step frequency, starts the stepper, then runs a 50 Hz supervision loop with the closed-loop module. |
| **EncoderTask** | 4 | 512 | 100 Hz (`vTaskDelayUntil`) | Reads the AS5600 angle over I2C2 (blocking read, 5 ms timeout, under `I2C2_Mutex`), processes it (multi-turn tracking, EMA filter with immediate reset to raw angle when `IDLE`/`PAUSED`, distance/velocity), and updates the odometry snapshot. |
| **UartTask** | 4 | 256 | Blocks on a queue (8 entries) | UART gatekeeper. Serializes TMC2208 register access on USART1 (DMA, half-duplex) (also provides an optional async TX path on USART2 VCP, currently unused). |

`I2C2_Mutex` protects I2C2 between the LCD and the AS5600.

### Stepper ISR

The TIM2 update ISR (`HAL_TIM_PeriodElapsedCallback` in `main.c`) calls `StepperProfile_StepTick()` once per step. It moves through `ACCEL → CRUISE → DECEL → IDLE` (or `PAUSED`) and sets ARR from a 256-entry normalized cosine table. TIM2 has NVIC priority 1, above the FreeRTOS syscall ceiling, so the ISR calls no FreeRTOS API. All other peripheral interrupts use priority 5 (the HAL tick on TIM6 and the FreeRTOS kernel use the lowest priority).

### Kinematics

- Steps per mm = (200 / 2 mm) × 128 microsteps = 12800
- Linear travel = volume / (π·(d/2)²); linear speed = flow rate / area
- Start frequency 3000 Hz, target acceleration 1000 steps/s², acceleration ramp clamped to 100–5000 steps
- Manual mode: 50 kHz cruise, 3000-step ramp
- Maximum flow rate accepted by the UI: 100 mL/min (arbitrary software limit in `gPumpHw`, not derived from a mechanical limit)

### Closed Loop

Every 20 ms `MotorTask` compares commanded distance/velocity with the AS5600 distance/velocity (`closed_loop.c`, defaults in `closed_loop.h`):

- Mid-cruise step injection is disabled so transient EMA phase lag and intra-revolution magnet runout do not cause overshoot (`ClosedLoop_Update()` still computes slip compensation steps for Live Watch telemetry in `gClosedLoopDebug`, though mid-cruise step injection is not applied).
- Lag above 0.030 mm and effective velocity below 15 % of cruise triggers a stall: persistence is speed-adaptive (`20 ms` at >= 0.50 mm/s, `40 ms` at moderate speeds, and scaled up to `1000 ms` below `0.05 mm/s`). On stall, the motor stops, the pump goes to `SYS_PAUSED`, and the LCD shows `! MOTOR STALLED !` with Continue / Abort.
- Stall detection is blanked for 250 ms after start or resume.
- When the motor stops (`IDLE` or `PAUSED`), `EncoderTask` immediately resets the EMA filter to the raw angle if residual lag exceeds `0.18°` (~2 LSB), eliminating the EMA settling tail. After a `20 ms` settling check, `ClosedLoop_CalcCreepSteps()` runs a forward-only correction move at 3000 Hz if under-travel exceeds 0.02 mm.
- Resuming from pause or stall re-anchors remaining steps to the ground-truth AS5600 travelled distance.
- The displayed pumped volume is calculated from the encoder distance when the encoder is valid, otherwise from the step count.
- `CLOSED_LOOP_TEST` is automatically set by CMake (`1` in `Debug`, `0` in `Release`) to expose the `gClosedLoopDebug` struct for Live Watch only during debugging.

### TMC2208

Configured over single-wire UART at 38400 baud. `stepper.c` scans addresses 0–3, writes GCONF/CHOPCONF (MRES = 128 microsteps, interpolation off), verifies by reading back and by IFCNT, and retries (up to 5 times, with UART re-init) on failure. SpreadCycle is enabled during moves and StealthChop when idle.

There is no limit switch support. A clog or blockage is handled by the stall detection above (`SYS_PAUSED` + stall dialog).

---

## Source Layout

| Path | Content |
|---|---|
| `Core/` | CubeMX-generated init code, `main.c`, FreeRTOS config |
| `Drivers/Libs/Src`, `Inc` | Application and driver code (below) |
| `ThirdParty/FreeRTOS/` | FreeRTOS kernel (ARM_CM4F port, `heap_4`) |
| `tests/` | Host unit tests (Unity) |

Hardware-independent modules (no HAL/RTOS dependency, covered by host tests):

| Module | Purpose |
|---|---|
| `pump_kinematics` | Unit conversions, syringe area, steps and frequency calculation |
| `stepper_profile` | S-curve table, ARR calculation, step tick state machine |
| `closed_loop` | Stall detection, slip telemetry, and post-move creep step calculation |
| `as5600_math` | Raw angle → degrees, multi-turn tracking, EMA filter, distance |
| `syringe_db` | Syringe profile database logic |
| `ui_editor` | Digit-cursor numeric editing and clamping |
| `ui_menu` | LCD menu state machine (tests run against mocks) |
| `TMC2208` | TMC2208 register access (vendor-style API) |

Hardware-dependent modules:

| Module | Purpose |
|---|---|
| `rtos_tasks` | Task bodies, shared state, queue/mutex creation |
| `stepper` | Starts moves, TMC2208 setup/verify/recovery, UART gatekeeper |
| `as5600` | AS5600 blocking I2C read, stationary filter snap, and odometry snapshot (`AS5600_GetSnapshot`) |
| `flash_storage` | Syringe database in flash |
| `liquidcrystal_i2c` | HD44780 over PCF8574, 4-bit mode |
| `printf` | Small `snprintf` implementation |

### Flash Storage

The syringe database occupies the last 2 KB page of flash (`0x0807F800`): magic `0xABCD1234`, count, up to 10 profiles (16-character name, diameter in mm, volume in mL). Every change erases the page and rewrites it (read-modify-write, 64-bit programming).
