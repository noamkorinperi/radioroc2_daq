# RADIOROC2 DAQ — Data Acquisition System

> **Development and Implementation of Algorithms for Expanding Sensors Measurement Range**
>
> STM32F722 firmware and a Python PC application for real-time gamma-ray
> energy spectrum acquisition using the RADIOROC2 ASIC coupled to a
> CsI(Tl) scintillator and SiPM detector.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [Firmware Architecture](#3-firmware-architecture)
   - 3.1 [Module Map](#31-module-map)
   - 3.2 [Startup Sequence](#32-startup-sequence)
   - 3.3 [Main Loop](#33-main-loop)
   - 3.4 [Event Acquisition Flow](#34-event-acquisition-flow)
   - 3.5 [Thermal Compensation](#35-thermal-compensation)
4. [Firmware Modules — Reference](#4-firmware-modules--reference)
   - 4.1 [config.h — Central Configuration](#41-configh--central-configuration)
   - 4.2 [channel_map — Active Channel Management](#42-channel_map--active-channel-management)
   - 4.3 [radioroc2_i2c — ASIC I2C Driver](#43-radioroc2_i2c--asic-i2c-driver)
   - 4.4 [system_init — ASIC Initialisation Sequence](#44-system_init--asic-initialisation-sequence)
   - 4.5 [timers — Timer Management](#45-timers--timer-management)
   - 4.6 [daq — Radiation Event Acquisition](#46-daq--radiation-event-acquisition)
   - 4.7 [compensation — Thermal Correction and keV Conversion](#47-compensation--thermal-correction-and-kev-conversion)
   - 4.8 [usb_comm — USB CDC Communication](#48-usb_comm--usb-cdc-communication)
5. [USB Communication Protocol](#5-usb-communication-protocol)
   - 5.1 [STM32 → PC: Event Packet](#51-stm32--pc-event-packet)
   - 5.2 [STM32 → PC: Histogram-Ready Marker](#52-stm32--pc-histogram-ready-marker)
   - 5.3 [PC → STM32: Command Packet](#53-pc--stm32-command-packet)
6. [Python PC Application](#6-python-pc-application)
   - 6.1 [Module Overview](#61-module-overview)
   - 6.2 [comms.py — Serial Communication](#62-commspy--serial-communication)
   - 6.3 [daq.py — Session and Histogram Data](#63-daqpy--session-and-histogram-data)
   - 6.4 [plotter.py — Real-Time Display](#64-plotterpy--real-time-display)
   - 6.5 [main.py — Entry Point and CLI](#65-mainpy--entry-point-and-cli)
7. [Configuration Guide](#7-configuration-guide)
8. [Calibration Procedure](#8-calibration-procedure)
9. [Building and Running](#9-building-and-running)
   - 9.1 [Firmware (STM32CubeIDE)](#91-firmware-stm32cubeide)
   - 9.2 [Python Application](#92-python-application)
10. [Pin Reference](#10-pin-reference)
11. [Timer Reference](#11-timer-reference)
12. [I2C Register Reference](#12-i2c-register-reference)
13. [Error Codes](#13-error-codes)

---

## 1. System Overview

The RADIOROC2 DAQ system reads energy deposits from a radiation detector
(CsI(Tl) scintillator + SiPM) using the **RADIOROC2** front-end ASIC.

```
┌─────────────┐   SiPM signal    ┌────────────┐  I2C + GPIO  ┌──────────────┐
│ CsI(Tl)     │ ────────────────▶│ RADIOROC2  │◀────────────▶│ STM32F722ZE  │
│ Scintillator│                  │   ASIC     │              │ (Nucleo-F722)│
└─────────────┘                  └────────────┘              └──────┬───────┘
                                       │                            │
                               Analog MUX output             USB CDC (VCP)
                               (OUT_AMUXHG / LG)                    │
                                       │                     ┌──────▼───────┐
                                       └──ADC1/ADC2──────────│  PC Python   │
                                                             │  Application │
                                                             └──────────────┘
```

**Data flow for one radiation event:**

1. A particle hits the scintillator → SiPM produces a current pulse.
2. RADIOROC2 detects the pulse via its local T1 threshold trigger and fires `NOR_T1OC` (falling edge on PA0).
3. The STM32 EXTI0 ISR sets `g_event_pending`.
4. The main loop reads all active channels sequentially using `CK_READ` pulses and ADC sampling.
5. Temperature correction (LY(T) polynomial) and keV conversion are applied to every sample.
6. Results are streamed to the PC over USB CDC as 7-byte packets.
7. The Python application accumulates histograms and displays them in real time.

---

## 2. Hardware Architecture

| Component | Part | Notes |
|-----------|------|-------|
| Microcontroller | STM32F722ZETx | Cortex-M7, 216 MHz |
| Development board | Nucleo-F722ZE | ST-Link v2 onboard |
| Front-end ASIC | RADIOROC2 | 64-channel SiPM readout ASIC (OMEGA/IN2P3) |
| Detector | CsI(Tl) + SiPM | Scintillator crystal with Silicon Photomultiplier |
| HG ADC input | PA4 → ADC1_IN4 | High-Gain MUX output `OUT_AMUXHG` |
| LG ADC input | PA5 → ADC2_IN5 | Low-Gain MUX output `OUT_AMUXLG` |
| ASIC I2C clock | PA8 → TIM1_CH1 | 2 MHz square wave `CLK_SM_I2C` |
| MUX advance clock | PA1 → TIM2_CH2 | Single pulse per channel `CK_READ` |
| 1-second tick | TIM3 | Temperature sampling + DAC feedback |
| I2C bus | PB8 (SCL), PB9 (SDA) → I2C1 | Non-standard 3-phase protocol |
| USB | USB OTG FS | Virtual COM Port (CDC class) |

**System clock summary (216 MHz HSE PLL):**

| Bus | Divider | Frequency | Timer clock |
|-----|---------|-----------|-------------|
| HCLK (AHB) | ÷1 | 216 MHz | — |
| PCLK1 (APB1) | ÷4 | 54 MHz | TIM2–7: 108 MHz |
| PCLK2 (APB2) | ÷2 | 108 MHz | TIM1: 216 MHz |

---

## 3. Firmware Architecture

### 3.1 Module Map

```
main.c
 ├── CubeMX-generated peripherals:
 │    MX_GPIO_Init, MX_ADC1_Init, MX_ADC2_Init,
 │    MX_I2C1_Init, MX_TIM1_Init, MX_TIM2_Init,
 │    MX_USB_DEVICE_Init
 │
 └── app_init() / app_run()          [main_app.c]
      │
      ├── channel_map                 Active-channel list and O(1) lookup
      ├── radioroc2_i2c               Non-standard 3-phase I2C driver
      ├── system_init                 Power-on ASIC configuration sequence
      ├── timers                      CLK_SM_I2C / CK_READ / 1-second tick
      ├── daq                         ISR flag + channel readout + ADC
      ├── compensation                Temperature sensor, PI DAC loop, keV conversion
      └── usb_comm                    USB CDC transmit / receive / command parser

USB_DEVICE/
 ├── usbd_cdc_if.c                    CDC_Receive_FS → usb_comm_on_receive()
 └── (other CDC middleware, generated by CubeMX)
```

### 3.2 Startup Sequence

`app_init()` in `main_app.c` is called once from `main()` after all
CubeMX peripheral initialisations:

```
1.  channel_map_init()
      Reads ACTIVE_CHANNELS from config.h.
      Builds a 64-element boolean mask and finds the highest active channel.

2.  radioroc2_i2c_init(&hi2c1)
      Binds the HAL I2C handle to the driver.

3.  compensation_init()
      Zeroes PI integrators. Writes DAC_BASELINE to every active channel via I2C.

4.  usb_comm_init()
      Resets event counter, timer reference, receive buffer, and histogram flag.

5.  system_init()    ← main ASIC bringup; returns InitStatus_t
     a. timers_start_clk_sm_i2c()   Start 2 MHz PWM on PA8 — MUST precede I2C
     b. HAL_Delay(1)                 Allow clock to stabilise
     c. RSTN_I2C low → 5 ms → high  Reset ASIC I2C slave core
     d. system_assert_reset_n()      20 ns RESET_N pulse → reset ASIC digital core
     e. HAL_Delay(10)
     f. radioroc2_is_alive()         I2C ping — returns INIT_ERR_ASIC_NOACK on failure
     g. Write SC_GLOBAL_SELTRIG      selTrig[1:0]="01", selHoldExt per HOLD_MODE
     h. Write SC_GLOBAL_DELAY        Internal hold delay (ASIC_DELAY_VAL)
     i. Write SC_GLOBAL_SLOPE        slopeTrim | ibi_discri = (ASIC_SLOPE_TRIM<<4)|0x04
     j. For each active channel:
           Write SC_SUBADD_INDAC     InDAC = DAC_BASELINE
           Write SC_SUBADD_GAIN      hgGain[3:0]=8, lgGain[7:4]=4  → 0x48
           Write SC_SUBADD_TAU       tauHG[7:4]=4, tauLG[3:0]=4   → 0x44
     k. system_assert_rstn_read()    Reset MUX to channel 0
     l. timers_start_1s_tick()       Configure and start TIM3 at 1 Hz
     m. HAL_NVIC_EnableIRQ(EXTI0)   Enable NOR_T1OC interrupt — safe to do now

6.  If system_init() ≠ INIT_OK:
      Blink LED3 (PB0) at 200 ms forever (blocking error state)
```

### 3.3 Main Loop

`app_run()` in `main_app.c` never returns:

```c
while (1) {
    if (g_event_pending)          daq_process_event();        // radiation event
    if (timers_1s_elapsed())      compensation_update_dac();  // thermal feedback
    if (usb_comm_histogram_due()) usb_comm_send_histogram_ready(); // signal PC
    usb_comm_poll_commands();                                  // handle PC commands
}
```

| Condition | Source | Period | Action |
|-----------|--------|--------|--------|
| `g_event_pending` | EXTI0 ISR, NOR_T1OC ↓ | Event-driven | Full readout, keV conversion, USB send |
| `timers_1s_elapsed()` | TIM3 IRQ | 1 s | PI loop: adjust InDAC per channel |
| `usb_comm_histogram_due()` | Internal counter/timer | N events or T seconds | Send histogram-ready marker to PC |
| `usb_comm_poll_commands()` | USB RX buffer | Every iteration | Parse and execute PC commands |

### 3.4 Event Acquisition Flow

`daq_process_event()` is the core readout routine. It executes entirely
in the main loop (not in an ISR), keeping interrupt latency minimal.

```
STEP 1  — Clear g_event_pending (prevents re-entry)

STEP 2  — Assert HOLDEXT = HIGH  [external hold mode only]
            Freezes the ASIC peak-detector output so the analog voltage
            on OUT_AMUXHG/LG is stable for the entire readout.

STEP 3  — Assert RSTN_READ (pulse low → high)
            Resets the output MUX to channel 0.
            Wait 2 µs (MUX_SETTLE_US) for the MUX output to settle.

STEP 4  — Channel scan loop: mux_pos = 0 … last_active_channel
  ┌──────────────────────────────────────────────────────────────┐
  │  timers_ck_read_pulse()                                      │
  │    Fires one pulse on CK_READ (PA1, TIM2_CH2, one-pulse mode)│
  │    Blocks ~60 µs until complete (polls TIM2 UPDATE flag).    │
  │    This advances the ASIC output MUX to mux_pos.            │
  │                                                              │
  │  if channel_map_active(mux_pos):                             │
  │    HAL_ADC_Start(hadc1) → PollForConversion → GetValue       │
  │      → g_results[idx].adc_hg  (12-bit, OUT_AMUXHG)          │
  │    HAL_ADC_Start(hadc2) → PollForConversion → GetValue       │
  │      → g_results[idx].adc_lg  (12-bit, OUT_AMUXLG)          │
  │    g_results[idx].valid = true                               │
  └──────────────────────────────────────────────────────────────┘

STEP 5  — Release HOLDEXT = LOW  [external hold mode only]

STEP 6  — Assert RSTN_READ  (reset MUX to channel 0)
STEP 7  — Assert RESET_N    (~20 ns pulse, resets ASIC peak detectors)

STEP 8  — Read temperature
            compensation_get_temperature()
            Temporarily switches ADC1 to internal TEMPSENSOR channel,
            samples (480 cycles = 26.7 µs), then restores ADC1 to IN4.

STEP 9  — Apply correction for each valid channel
            compensation_apply(adc_hg, T)  →  energy_kev
            (returns -1.0f if PI loop not yet converged)

STEP 10 — Transmit to PC for each valid channel
            usb_comm_send_event(channel, energy_kev)
```

> **Hold mode selection:**
> `HOLD_MODE_EXTERNAL` (default) is required whenever the ADC conversion
> time (~26.7 µs at 480 cycles) exceeds the ASIC's internal hold window.
> In external mode the ASIC ignores its internal delay and waits for the
> HOLDEXT pin; the STM32 drives it high at the start of readout and
> releases it after all ADC conversions are complete.
> `HOLD_MODE_INTERNAL` limits the hold to ~3.25 µs and is only viable
> with a reduced shaping time.

### 3.5 Thermal Compensation

Temperature affects detector performance in two independent ways.
Both corrections run in parallel.

#### PATH 1 — SiPM Gain Stabilisation (Hardware feedback via InDAC)

**Goal:** Keep the SiPM breakdown voltage (and therefore gain) constant
despite temperature-induced changes in the SiPM characteristics.

**Mechanism:** A PI feedback loop per channel adjusts the ASIC's `InDAC`
register (which controls the SiPM bias voltage offset) to keep the raw ADC
peak value at `V_SETPOINT`.

```
error       = V_SETPOINT − g_results[i].adc_hg
integral   += error                          [anti-windup: clamped to ±5000]
correction  = KP × error + KI × integral
new_dac     = clamp(dac_val + correction, DAC_MIN, DAC_MAX)
if new_dac ≠ dac_val:
    radioroc2_write(ch, SC_SUBADD_INDAC, new_dac)
```

The loop is inactive for the first `WARMUP_TICKS` seconds.
`compensation_is_converged()` returns `true` only when every active
channel satisfies `|error| < CONVERGENCE_THR`.

#### PATH 2 — Crystal Light Yield Correction (Software)

**Goal:** Compensate for the temperature dependence of the CsI(Tl)
scintillation light yield, which shifts the apparent energy of each event.

**Polynomial** (Saint-Gobain CsI(Tl) datasheet fit, normalised at T_REF = 27 °C):

```
LY(T) = 0.923 + 3.95×10⁻³·T − 5.56×10⁻⁵·T² − 4.43×10⁻⁷·T³ − 7.35×10⁻¹⁰·T⁴

LY_norm(T) = LY(T) / LY_REF          [LY_REF = LY(27°C) ≈ 0.9999]

adc_corrected = adc_hg / LY_norm(T)
energy_kev    = adc_corrected / S_T_REF
```

PATH 2 is gated on PATH 1 convergence. If the PI loop has not yet settled,
`compensation_apply()` returns `-1.0f` and the event is marked invalid.

---

## 4. Firmware Modules — Reference

### 4.1 `config.h` — Central Configuration

**Location:** `Core/Inc/config.h`

The **only** file that needs manual editing for hardware configuration.
Never generated or overwritten by CubeMX.

#### Channel Configuration

| Macro | Default | Description |
|-------|---------|-------------|
| `ACTIVE_CHANNELS` | `{0, 1, 2}` | Brace-initialised list of ASIC channel numbers to read (0–63) |
| `NUM_ACTIVE_CHANNELS` | `3` | **Must match** the number of entries in `ACTIVE_CHANNELS` |

#### ADC Timing

| Macro | Default | Computed time | Description |
|-------|---------|---------------|-------------|
| `ADC_SAMPLING_CYCLES_SH` | `ADC_SAMPLETIME_480CYCLES` | 26.7 µs | S&H output sampling time |
| `ADC_SAMPLING_CYCLES_TEMP` | `ADC_SAMPLETIME_480CYCLES` | 26.7 µs | Temperature sensor (minimum 10 µs per datasheet) |

#### Hold Mode

| Macro | Description |
|-------|-------------|
| `HOLD_MODE_EXTERNAL` (= 1) | STM32 drives HOLDEXT pin. Recommended — guarantees ADC finishes before hold releases. |
| `HOLD_MODE_INTERNAL` (= 0) | ASIC internal delay register. Max hold ≈ 3.25 µs. Only viable with reduced shaping time. |
| `HOLD_MODE` | Set to one of the two options above. |

#### Timing Constants

| Macro | Default | Description |
|-------|---------|-------------|
| `MUX_SETTLE_US` | `2` | Delay (µs) after RSTN_READ before first CK_READ |
| `CK_READ_PERIOD_US` | `30` | Intended CK_READ high-time in µs (see Timer Reference for actual hardware timing) |
| `RESET_N_PULSE_NOPS` | `5` | NOP count for RESET_N pulse (~23 ns at 216 MHz) |
| `ASIC_DELAY_VAL` | `0xFF` | Internal hold delay register (255 × 0.85 ns × slope) |
| `ASIC_SLOPE_TRIM` | `0x0F` | Internal hold slope trim |

#### PI Feedback (PATH 1)

| Macro | Default | Description |
|-------|---------|-------------|
| `KP` | `0.15` | Proportional gain |
| `KI` | `0.02` | Integral gain |
| `V_SETPOINT` | `2048` | Target ADC count — set to photopeak ADC value after calibration |
| `DAC_BASELINE` | `128` | InDAC starting value at T_REF |
| `DAC_MIN` / `DAC_MAX` | `0` / `255` | InDAC hardware range |
| `CONVERGENCE_THR` | `60` | Loop converged when `|error| < 60` ADC counts (~1.5% of full scale) |
| `WARMUP_TICKS` | `5` | Seconds to wait before enabling correction |

#### LY(T) Correction (PATH 2)

| Macro | Default | Description |
|-------|---------|-------------|
| `T_REF` | `27.0` | Calibration temperature (°C) |
| `LY_REF` | `0.9999` | LY polynomial value at T_REF |
| `S_T_REF` | `3.1` | ADC counts per keV at T_REF — **calibrate experimentally** |

---

### 4.2 `channel_map` — Active Channel Management

**Files:** `Core/Inc/channel_map.h`, `Core/Src/channel_map.c`

Converts the `ACTIVE_CHANNELS` list into a fast lookup table so the
readout loop can decide in O(1) whether each MUX position should be
sampled. The MUX is only scanned up to the highest active channel,
reducing dead time.

| Function / Variable | Description |
|---------------------|-------------|
| `channel_map_init()` | Builds the `s_active_mask[64]` boolean array from `ACTIVE_CHANNELS`. Finds and caches the highest channel. **Must be called once on startup.** |
| `channel_map_active(ch)` | Returns `true` if `ch` is in the active list. Called once per MUX position in the readout loop. |
| `channel_map_last()` | Returns the highest active channel number. CK_READ scanning stops at this value. |
| `channel_map_count()` | Returns `NUM_ACTIVE_CHANNELS`. |
| `g_active_channels[NUM_ACTIVE_CHANNELS]` | Global array of active channel numbers (index 0..N-1). Used by `compensation.c` and `radioroc2_i2c.c` for per-channel operations. |

---

### 4.3 `radioroc2_i2c` — ASIC I2C Driver

**Files:** `Core/Inc/radioroc2_i2c.h`, `Core/Src/radioroc2_i2c.c`

Implements the **non-standard 3-phase I2C protocol** of the RADIOROC2.
Each logical read or write consists of three separate I2C transactions,
each with its own START and STOP:

```
Phase R0  START | (ChipID + 000) | W  | channel number   | STOP
Phase R1  START | (ChipID + 001) | W  | sub-address       | STOP
Phase R2  START | (ChipID + 010) | W  | data byte (write) | STOP
    — or —
Phase R2  START | (ChipID + 010) | R  |  ← data byte      | STOP
```

The 7-bit base address is `0x08` (CHIP_ID = `0001`, hardwired on board).

| Phase | Address sent to HAL |
|-------|---------------------|
| R0 Write | `0x10` |
| R1 Write | `0x12` |
| R2 Write | `0x14` |
| R2 Read  | `0x15` |

> **Prerequisite:** The 2 MHz `CLK_SM_I2C` clock on PA8 **must be running**
> before any call to this driver. RADIOROC2 uses it internally to clock its
> I2C slave logic.

| Function | Description |
|----------|-------------|
| `radioroc2_i2c_init(hi2c)` | Stores the HAL handle. Must be called once before anything else. |
| `radioroc2_write(ch, subadd, data)` | Writes one byte to a per-channel slow-control register using the 3-phase protocol. Returns `HAL_OK` / `HAL_ERROR` / `HAL_TIMEOUT`. |
| `radioroc2_read(ch, subadd, &data)` | Reads one byte from a per-channel register. R0 and R1 are write phases; R2 uses `HAL_I2C_Master_Receive`. |
| `radioroc2_write_global(subadd, data)` | Equivalent to `radioroc2_write(65, subadd, data)`. Channel 65 addresses global registers. |
| `radioroc2_read_global(subadd, &data)` | Reads from a global register. |
| `radioroc2_write_all_channels(subadd, data)` | Iterates over `g_active_channels[]` and writes `data` to each. Continues on error; returns the last non-OK status (or `HAL_OK` if all succeeded). |
| `radioroc2_is_alive()` | Sends only the R0 phase and checks for an ACK. Used as an I2C connectivity test during `system_init()`. Returns `true` on ACK. |

---

### 4.4 `system_init` — ASIC Initialisation Sequence

**Files:** `Core/Inc/system_init.h`, `Core/Src/system_init.c`

Runs the complete 9-step ASIC bringup. See [Section 3.2](#32-startup-sequence)
for the full sequence. Returns `InitStatus_t` (see [Section 13](#13-error-codes)).

| Function | Description |
|----------|-------------|
| `system_init()` | Full power-on sequence. If any step fails the function returns immediately with an error code. On success enables EXTI0 and returns `INIT_OK`. |
| `system_assert_reset_n()` | Pulses RESET_N (PD4) low for 5 NOP instructions (~23 ns at 216 MHz), then high. Resets the ASIC peak detectors. Also called after each event readout. |
| `system_assert_rstn_read()` | Pulses RSTN_READ (PD1) low then high. Resets the output MUX to channel 0. Includes a 1 ms `HAL_Delay` settle before the first `CK_READ`. |

---

### 4.5 `timers` — Timer Management

**Files:** `Core/Inc/timers.h`, `Core/Src/timers.c`

Manages three hardware timers with distinct roles:

#### TIM1 — CLK_SM_I2C (2 MHz continuous)

Configured by CubeMX in `MX_TIM1_Init()`.
- **Pin:** PA8 (AF1 — TIM1_CH1)
- **Clock source:** TIM1 clock = 216 MHz (APB2 × 2)
- **Settings:** PSC=0, ARR=107, CCR1=53 → 2 MHz, 50% duty cycle

| Function | Description |
|----------|-------------|
| `timers_start_clk_sm_i2c()` | Calls `HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)`. Must be the very first timer call, before any I2C access. |
| `timers_stop_clk_sm_i2c()` | Stops the 2 MHz output. For debug or shutdown only. |

#### TIM2 — CK_READ (one-pulse per channel)

Configured by CubeMX in `MX_TIM2_Init()`.
- **Pin:** PA1 (AF1 — TIM2_CH2)
- **Clock source:** TIM2 clock = 108 MHz (APB1 × 2)
- **Settings:** PSC=215, ARR=29, CCR2=14 → tick = 2 µs, total period = 60 µs
- **Mode:** One Pulse Mode (counter stops at ARR after one cycle)

| Function | Description |
|----------|-------------|
| `timers_ck_read_pulse()` | Resets the counter, calls `HAL_TIM_OnePulse_Start()`, then blocks polling `TIM_FLAG_UPDATE` until the pulse completes. Clears the flag before returning. Each call advances the ASIC output MUX by one channel position. |

#### TIM3 — 1-second tick

Configured entirely in software by `timers_start_1s_tick()` (not in CubeMX).
- **Clock source:** TIM3 clock = 108 MHz (APB1 × 2)
- **Settings:** PSC=10799, ARR=9999 → 1.000 s period
- **NVIC priority:** 5 (lowest among application interrupts)

| Function | Description |
|----------|-------------|
| `timers_start_1s_tick()` | Initialises `htim3`, enables the TIM3 IRQ, and starts the timer with interrupt. Called from `system_init()`. |
| `timers_1s_elapsed()` | Returns `true` once per second and clears the internal flag. Non-blocking; call every main loop iteration. |
| `timers_on_tim3_elapsed()` | Called from `HAL_TIM_PeriodElapsedCallback` in `main.c` when `htim->Instance == TIM3`. Sets the 1-second flag. |
| `TIM3_IRQHandler()` | Defined in `timers.c`. Calls `HAL_TIM_IRQHandler(&htim3)`, which in turn calls `HAL_TIM_PeriodElapsedCallback`. |

---

### 4.6 `daq` — Radiation Event Acquisition

**Files:** `Core/Inc/daq.h`, `Core/Src/daq.c`

#### Global Variables

| Variable | Type | Description |
|----------|------|-------------|
| `g_event_pending` | `volatile bool` | Set by `daq_exti0_irq_handler()` (ISR); cleared at the start of `daq_process_event()`. |
| `g_results[NUM_ACTIVE_CHANNELS]` | `ChannelResult_t[]` | Holds the last readout for each active channel. Updated on every event. |

#### `ChannelResult_t` — Per-Channel Result Structure

| Field | Type | Description |
|-------|------|-------------|
| `channel` | `uint8_t` | ASIC channel number (0–63) |
| `adc_hg` | `uint16_t` | Raw 12-bit ADC value from OUT_AMUXHG (High Gain path) |
| `adc_lg` | `uint16_t` | Raw 12-bit ADC value from OUT_AMUXLG (Low Gain path) |
| `energy_kev` | `float` | Temperature-corrected energy in keV. `-1.0f` if PI loop not converged. |
| `valid` | `bool` | `true` if this entry was filled during the most recent event. |

#### Functions

| Function | Description |
|----------|-------------|
| `daq_exti0_irq_handler()` | Called by `EXTI0_IRQHandler` in `stm32f7xx_it.c`. Clears the EXTI pending bit for `NOR_T1_PIN` and sets `g_event_pending`. No further processing — keeps the ISR as short as possible. |
| `daq_process_event()` | Full 10-step readout sequence (see [Section 3.4](#34-event-acquisition-flow)). Reads all active channels, applies corrections, and sends results via USB. |
| `daq_channel_index(ch)` | Searches `g_active_channels[]` for `ch`. Returns the array index (0..N-1), or `-1` if not found. |

---

### 4.7 `compensation` — Thermal Correction and keV Conversion

**Files:** `Core/Inc/compensation.h`, `Core/Src/compensation.c`

See [Section 3.5](#35-thermal-compensation) for the complete algorithm description.

#### Internal State — `ChannelCompState_t` (per active channel)

| Field | Type | Description |
|-------|------|-------------|
| `integral` | `float` | PI integrator accumulator (anti-windup: clamped to ±5000) |
| `dac_val` | `uint8_t` | Current InDAC value written to the ASIC |
| `converged` | `bool` | `true` when `|error| < CONVERGENCE_THR` |
| `warmup_ticks` | `uint32_t` | Counts 1-second ticks; the loop activates after `WARMUP_TICKS` |

#### Functions

| Function | Description |
|----------|-------------|
| `compensation_init()` | Zeroes all state fields. Sets `dac_val = DAC_BASELINE` and writes it to every active channel. |
| `compensation_get_temperature()` | Reconfigures ADC1 to `ADC_CHANNEL_TEMPSENSOR`, samples with `ADC_SAMPLING_CYCLES_TEMP` (480 cycles = 26.7 µs ≥ 10 µs minimum), computes `T = (V_SENSE − 0.76) / 0.0025 + 30`, then restores ADC1 to channel IN4. Returns temperature in °C. |
| `compensation_update_dac()` | Runs one PI iteration per active channel. Skips channels with `!g_results[i].valid`. Skips all channels if still in the warmup period. Writes new InDAC only when the value changes. |
| `compensation_apply(adc_raw, T)` | Returns energy in keV after LY(T) correction and `g_s_t_ref` division. Returns `-1.0f` if `compensation_is_converged()` is false. Guards against `ly < 0.1` to prevent division by zero. |
| `compensation_is_converged()` | Returns `true` only if every active channel has `converged == true`. Used to gate PATH 2. |

---

### 4.8 `usb_comm` — USB CDC Communication

**Files:** `Core/Inc/usb_comm.h`, `Core/Src/usb_comm.c`

Provides a framed 7-byte binary protocol over the USB Virtual COM Port.

#### Global Variables

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `g_hist_mode` | `HistMode_t` | `HIST_MODE_COUNT` | Current histogram trigger mode |
| `g_hist_param` | `uint32_t` | `1000` | N events (COUNT) or T seconds (TIME) |
| `g_s_t_ref` | `float` | `S_T_REF` (3.1) | Calibration factor in ADC counts/keV — runtime-updatable via `USB_CMD_SET_CALIB` |

#### Functions

| Function | Description |
|----------|-------------|
| `usb_comm_init()` | Resets event counter, time reference (`HAL_GetTick()`), receive buffer, and `s_hist_due` flag. |
| `usb_comm_send_event(ch, kev)` | Builds a 7-byte event packet (start=0xAA, channel, float kev LE, XOR checksum) and calls `CDC_Transmit_FS()`. If `kev > 0`, increments the event counter and checks for a COUNT-mode trigger. |
| `usb_comm_send_histogram_ready()` | Sends the 7-byte histogram-ready marker (channel byte = 0xFF). Resets the TIME-mode timer reference. |
| `usb_comm_poll_commands()` | If `s_rx_ready` is set, validates the start byte (0xBB) and XOR checksum, then dispatches the command (`SET_COUNT`, `SET_TIME`, `SET_CALIB`, or `RESET`). |
| `usb_comm_histogram_due()` | COUNT mode: returns `true` and clears `s_hist_due` if the event count was reached. TIME mode: returns `true` if elapsed seconds ≥ `g_hist_param` and resets the timer. |
| `usb_comm_on_receive(buf, len)` | **CDC hook** — called from `CDC_Receive_FS()` in `usbd_cdc_if.c`. Copies up to `RX_BUF_SIZE` (32) bytes into `s_rx_buf` and sets `s_rx_ready`. |

---

## 5. USB Communication Protocol

All packets are exactly **7 bytes**. Byte 0 is the start byte; byte 6 is
the XOR checksum of bytes [1..5].

```
checksum = buf[1] XOR buf[2] XOR buf[3] XOR buf[4] XOR buf[5]
```

### 5.1 STM32 → PC: Event Packet

Sent once per active channel per radiation event.

```
 Byte:   0       1          2    3    4    5      6
       ┌──────┬─────────┬─────────────────────┬──────────┐
       │ 0xAA │ channel │  energy_kev (f32 LE) │ checksum │
       └──────┴─────────┴─────────────────────┴──────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| Start byte | 1 B | Always `0xAA` |
| Channel | 1 B | ASIC channel number, 0–63. Value `0xFF` is reserved for the histogram marker. |
| Energy (keV) | 4 B | `float32`, little-endian. `-1.0f` (`0xBF800000`) = invalid event (PI loop not converged). |
| Checksum | 1 B | XOR of bytes [1..5] |

### 5.2 STM32 → PC: Histogram-Ready Marker

Sent when N events (COUNT mode) or T seconds (TIME mode) have elapsed.

```
 Byte:   0       1       2    3    4    5      6
       ┌──────┬──────┬──────────────────────┬──────────┐
       │ 0xAA │ 0xFF │  0x00 0x00 0x00 0x00 │ checksum │
       └──────┴──────┴──────────────────────┴──────────┘
```

The Python application detects `channel == 0xFF` and uses it as the
signal to refresh the histogram display.

### 5.3 PC → STM32: Command Packet

Sent by the PC to configure acquisition parameters or reset the firmware.

```
 Byte:   0       1         2    3    4    5      6
       ┌──────┬──────────┬──────────────────┬──────────┐
       │ 0xBB │ cmd_type │  parameter (4 B) │ checksum │
       └──────┴──────────┴──────────────────┴──────────┘
```

| `cmd_type` | Constant | Parameter type | Effect |
|-----------|----------|----------------|--------|
| `0x01` | `USB_CMD_SET_COUNT` | `uint32` (LE) | Switch to COUNT mode; send histogram-ready every N events |
| `0x02` | `USB_CMD_SET_TIME` | `uint32` (LE) | Switch to TIME mode; send histogram-ready every T seconds |
| `0x03` | `USB_CMD_SET_CALIB` | `float32` (LE) | Update `g_s_t_ref` (ADC counts/keV). Ignored if ≤ 0. |
| `0x04` | `USB_CMD_RESET` | ignored | Calls `HAL_NVIC_SystemReset()` — reboots the STM32 |

---

## 6. Python PC Application

**Location:** `python/`

### 6.1 Module Overview

| Module | Role |
|--------|------|
| `main.py` | Entry point — argument parsing, connection setup, main receive loop, Ctrl+C handler |
| `comms.py` | `USBComm` class — serial port, packet framing/parsing, command sending |
| `daq.py` | `DAQSession` class — event accumulation, per-channel and combined histogram arrays |
| `plotter.py` | `HistogramPlotter` class — matplotlib interactive bar-chart display |

**Dependencies:**

```bash
pip install pyserial numpy matplotlib
```

### 6.2 `comms.py` — Serial Communication

**Class: `USBComm(port, baudrate=115200)`**

Maintains an internal `bytearray` receive buffer. Incoming bytes are
appended on each call to `receive_packet()` and consumed as complete
7-byte packets are validated.

| Method | Returns | Description |
|--------|---------|-------------|
| `connect()` | `bool` | Opens the serial port. Waits 0.5 s for USB CDC enumeration. Returns `True` on success. |
| `disconnect()` | — | Closes the serial port cleanly. |
| `receive_packet()` | `dict \| None` | Non-blocking. Reads all waiting bytes, scans for start byte `0xAA`, validates XOR checksum. Returns a parsed dict or `None`. |
| `send_mode(mode, param)` | — | Sends `CMD_SET_COUNT` (`mode="count"`) or `CMD_SET_TIME` (`mode="time"`). |
| `send_calibration(s_t_ref)` | — | Sends `CMD_SET_CALIB` with `s_t_ref` packed as `float32` little-endian. |
| `send_reset()` | — | Sends `CMD_RESET` with a zero parameter. |

**`receive_packet()` return values:**

```python
{"type": "event", "channel": int, "energy_kev": float}   # radiation event
{"type": "histogram_ready"}                                # histogram trigger
None                                                       # no complete packet yet
```

### 6.3 `daq.py` — Session and Histogram Data

**Class: `DAQSession(channels, bins=512, energy_max=3000.0)`**

Accumulates event data between histogram triggers. All histograms use
uniform bins across `[0, energy_max]` keV. Bin width = `energy_max / bins`.

| Method | Returns | Description |
|--------|---------|-------------|
| `add_event(channel, energy_kev)` | — | Adds one event to the channel histogram and the combined histogram. Silently ignores events for unknown channels or `energy_kev` outside `[0, energy_max]`. |
| `get_histogram(channel)` | `(centres, counts)` | Bin centres in keV and integer counts for one channel. |
| `get_combined_histogram()` | `(centres, counts)` | Bin centres and counts for all channels summed. |
| `total_events()` | `int` | Total events accumulated since last `reset()`. |
| `events_per_channel(ch)` | `int` | Number of events stored for the given channel. |
| `channels()` | `List[int]` | Active channel list (as passed to constructor). |
| `bins()` | `int` | Number of histogram bins. |
| `energy_max()` | `float` | Upper energy limit in keV. |
| `reset()` | — | Clears all event lists, zeroes all histogram arrays, and resets the event counter. |

### 6.4 `plotter.py` — Real-Time Display

**Class: `HistogramPlotter(channels, bins=512, energy_max=3000.0)`**

Creates a matplotlib figure with `len(channels) + 1` vertically stacked
subplots: one per active channel and one combined. All X axes are shared.
Uses `plt.ion()` (interactive mode) so the window remains responsive.

Bin centre positions are computed as:

```python
half    = energy_max / (2 * bins)
centres = np.linspace(half, energy_max - half, bins)
```

This ensures bars exactly span `[0, energy_max]` without overflow.

| Method | Description |
|--------|-------------|
| `update(session)` | Updates bar heights from the `DAQSession`. Auto-scales Y axis to `max(count) × 1.1`. Updates subplot titles with event counts. |
| `show()` | Calls `fig.canvas.draw()` and `flush_events()`. Call immediately after `update()`. |
| `save(path)` | Saves the figure to `path` (PNG or PDF) at 150 dpi. |
| `close()` | Disables interactive mode and closes the figure. Called on Ctrl+C. |

### 6.5 `main.py` — Entry Point and CLI

```
python main.py --port <PORT> [OPTIONS]
```

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--port` | str | *(required)* | Serial port: `COM3`, `/dev/ttyACM0`, etc. |
| `--baud` | int | `115200` | Serial baud rate |
| `--mode` | str | `count` | Histogram trigger mode: `count` or `time` |
| `--param` | int | `1000` | N events (count mode) or T seconds (time mode) |
| `--calib` | float | `3.1` | Calibration factor S_T_REF [ADC counts/keV] |
| `--channels` | int+ | `0 1 2` | Active channel numbers (space-separated) |
| `--bins` | int | `512` | Number of histogram bins |
| `--energy-max` | float | `3000.0` | Maximum energy axis in keV |

**Startup actions (in order):**

1. Connect to the serial port (`USBComm.connect()`).
2. Send `send_mode()` → configures STM32 histogram trigger.
3. Send `send_calibration()` → updates `g_s_t_ref` on the STM32.
4. Instantiate `DAQSession` and `HistogramPlotter`.
5. Enter the main receive loop:
   - `event` → `session.add_event()`; skip if `energy_kev < 0` (invalid / PI not converged).
   - `histogram_ready` → `plotter.update(session)` + `plotter.show()`.
6. Ctrl+C → graceful exit: serial port closed, figure closed.

**Usage examples:**

```bash
# Count mode — update histogram every 500 events, Cs-137 calibrated
python main.py --port COM3 --mode count --param 500 --calib 3.1

# Time mode — update every 60 seconds, 5 channels, 2000 keV range
python main.py --port /dev/ttyACM0 --mode time --param 60 \
               --channels 0 1 2 3 4 --energy-max 2000 --bins 256
```

---

## 7. Configuration Guide

### Changing Active Channels

Edit `Core/Inc/config.h`:

```c
#define ACTIVE_CHANNELS      { 0, 1, 2, 5, 10 }
#define NUM_ACTIVE_CHANNELS  5
```

> `NUM_ACTIVE_CHANNELS` **must** equal the number of entries in
> `ACTIVE_CHANNELS`. A mismatch causes array out-of-bounds access at runtime.

Pass the same list to the Python application:

```bash
python main.py --port COM3 --channels 0 1 2 5 10
```

### Changing the Shaping Time

The default setting (`0x44`) uses 80 ns shaping for both HG and LG paths.
For CsI(Tl) with its ~1 µs decay time, a slower shaping may improve
energy resolution:

```c
/* system_init.c — Step 6, per-channel TAU register */
i2c_status = radioroc2_write(ch, SC_SUBADD_TAU, 0x44);
//                                               ^^^^
// Bits [7:4] = tauHG index (4 = 80 ns)
// Bits [3:0] = tauLG index (4 = 80 ns)
// Increase both nibbles for slower shaping.
// See RADIOROC2 Datasheet Table 4 for the full index-to-time mapping.
```

### Tuning the PI Feedback Loop

1. Set `KI = 0.0f` in `config.h`. Increase `KP` in small steps until
   the photopeak position responds quickly without oscillating.
2. Restore a small `KI` (start at `0.01f`) and increase slowly to
   eliminate the residual steady-state offset.
3. Monitor `g_results[i].adc_hg` and `s_state[i].dac_val` in real
   time using the CubeIDE Live Expressions view or SWV.

---

## 8. Calibration Procedure

`S_T_REF` (ADC counts per keV) must be measured experimentally.
The value in `config.h` (`3.1`) is a placeholder.

**Using a Cs-137 source (662 keV photopeak):**

1. Place the Cs-137 source next to the detector at **T_REF = 27 °C**.
2. Start acquisition and wait for the PI loop to converge
   (at least `WARMUP_TICKS` seconds; typically < 60 s total).
3. Accumulate until the 662 keV photopeak is clearly resolved
   (typically 5,000–20,000 events depending on source activity).
4. Read the ADC count `P` at the centre of the photopeak.
5. Calculate:
   ```
   S_T_REF = P / 662.0   [ADC counts / keV]
   ```
6. **Option A — Recompile** (persistent, survives power cycle):
   ```c
   /* Core/Inc/config.h */
   #define S_T_REF   <measured value>f
   ```
7. **Option B — Runtime update** (no recompile, lost on reset):
   ```bash
   python main.py --port COM3 --calib <measured value>
   ```
   Or call `comm.send_calibration(new_s_t_ref)` programmatically.

After calibration, update `V_SETPOINT` in `config.h` to match the ADC
count at the photopeak peak so the PI loop stabilises at the correct
operating point.

---

## 9. Building and Running

### 9.1 Firmware (STM32CubeIDE)

**Prerequisites:** STM32CubeIDE 1.14 or later, ST-Link driver.

```
1. File → Open Projects from File System → select the radioroc2_daq directory
2. Edit Core/Inc/config.h as needed (channels, gains, hold mode, calibration)
3. Project → Build Project  (Ctrl+B)
4. Connect the Nucleo board via USB (ST-Link port)
5. Run → Run  (F11)
```

**Startup verification:**

| LED3 (PB0) state | Meaning |
|-----------------|---------|
| Blinking at 200 ms | ASIC did not respond to I2C — check CLK_SM_I2C (PA8, 2 MHz), I2C wiring (PB8/PB9), and ASIC power supply |
| Off (no blink) | ASIC initialised successfully — system is waiting for NOR_T1OC events |

### 9.2 Python Application

**Prerequisites:** Python 3.8+, pyserial, numpy, matplotlib.

```bash
pip install pyserial numpy matplotlib
```

**Finding the COM port:**

| OS | Command | Typical result |
|----|---------|---------------|
| Windows | Device Manager → Ports (COM & LPT) | `COM3`, `COM4`, … |
| Linux | `ls /dev/ttyACM*` | `/dev/ttyACM0` |
| macOS | `ls /dev/tty.usbmodem*` | `/dev/tty.usbmodem14101` |

```bash
cd python
python main.py --port COM3 --mode count --param 1000 --calib 3.1
```

The histogram window opens immediately. It refreshes each time the STM32
sends a histogram-ready marker. Press **Ctrl+C** in the terminal to stop.

---

## 10. Pin Reference

| Signal | STM32 Pin | Direction | Init state | Description |
|--------|-----------|-----------|-----------|-------------|
| `OUT_AMUXHG` | PA4 / ADC1_IN4 | Analog input | — | High-Gain MUX output from RADIOROC2 |
| `OUT_AMUXLG` | PA5 / ADC2_IN5 | Analog input | — | Low-Gain MUX output from RADIOROC2 |
| `NOR_T1OC` | PA0 / EXTI0 | Digital input (pull-up) | — | Event trigger — falling edge → EXTI0 ISR |
| `CK_READ` | PA1 / TIM2_CH2 | Digital output | — | MUX advance clock — one 60 µs pulse per channel |
| `CLK_SM_I2C` | PA8 / TIM1_CH1 | Digital output | — | ASIC I2C slave clock — 2 MHz continuous |
| `RSTN_I2C` | PD0 | Output (active low) | HIGH | Resets ASIC I2C slave core |
| `RSTN_READ` | PD1 | Output (active low) | HIGH | Resets output MUX to channel 0 |
| `HOLDEXT` | PD2 | Output (active high) | LOW | External peak-hold control |
| `ERRORN_OC` | PD3 | Input (pull-up, open-collector) | — | ASIC error flag (active low) |
| `RESET_N` | PD4 | Output (active low) | HIGH | ASIC digital core reset — minimum 20 ns pulse |
| `RSTN_SC` | PD7 | Output (active low) | HIGH | Resets ASIC slow-control core |
| `SCL` | PB8 / I2C1_SCL | I2C | — | Clock to RADIOROC2 I2C slave |
| `SDA` | PB9 / I2C1_SDA | I2C | — | Data to/from RADIOROC2 I2C slave |

---

## 11. Timer Reference

### TIM1 — CLK_SM_I2C

| Parameter | Value | Computed |
|-----------|-------|---------|
| Timer clock | APB2 × 2 | 216 MHz |
| Prescaler (PSC) | 0 | Tick = 4.63 ns |
| Auto-reload (ARR) | 107 | Period = 108 ticks = 500 ns |
| Compare (CCR1) | 53 | High time = 54 ticks = 250 ns |
| **Output** | | **2 MHz, 50% duty cycle** |

### TIM2 — CK_READ

| Parameter | Value | Computed |
|-----------|-------|---------|
| Timer clock | APB1 × 2 | 108 MHz |
| Prescaler (PSC) | 215 | Tick = 2 µs |
| Auto-reload (ARR) | 29 | Period = 30 ticks = **60 µs** |
| Compare (CCR2) | 14 | High time = 15 ticks = 30 µs |
| Mode | One Pulse Mode | Counter stops at ARR |
| **Output** | | **60 µs period; HIGH for first 30 µs** |

> The ADC requires ~26.7 µs to complete one conversion (480 sampling cycles
> at 18 MHz + 12 conversion cycles). The 60 µs pulse period provides a safe
> margin for the analog output to settle and the ADC to finish before the
> next pulse advances the MUX.

### TIM3 — 1-second tick

| Parameter | Value | Computed |
|-----------|-------|---------|
| Timer clock | APB1 × 2 | 108 MHz |
| Prescaler (PSC) | 10799 | Tick = 100 µs (10 kHz) |
| Auto-reload (ARR) | 9999 | Period = 10000 ticks = **1.000 s** |
| NVIC priority | 5 | Lowest application priority |

---

## 12. I2C Register Reference

All registers are 8-bit. The channel address is 0–63 for per-channel
registers, or 65 for global registers.

### Per-Channel Slow Control (channel = 0–63)

| Sub-address | Constant | Bit fields | Description |
|------------|----------|-----------|-------------|
| 0 | `SC_SUBADD_INDAC` | [7:0] InDAC | SiPM bias voltage offset — controls SiPM gain (0–255) |
| 1 | `SC_SUBADD_PATGAIN` | [5:0] patGain | Time preamplifier gain |
| 2 | `SC_SUBADD_GAIN` | [7:4] lgGain, [3:0] hgGain | Amplifier gains for LG and HG paths |
| 3 | `SC_SUBADD_TAU` | [7:4] tauHG, [3:0] tauLG | Shaping time indices for HG and LG |

### Global Slow Control (channel = 65)

| Sub-address | Constant | Bit fields | Description |
|------------|----------|-----------|-------------|
| 7 | `SC_GLOBAL_VREF` | [3:0] vref, EN_th1, EN_th2, EN_thQ | Voltage reference and threshold enables |
| 8 | `SC_GLOBAL_DELAY` | [7:0] delay | Internal hold delay (used in HOLD_MODE_INTERNAL) |
| 9 | `SC_GLOBAL_SLOPE` | [7:4] slopeTrim, [3:0] ibi_discri | Hold slope trim and discriminator bias |
| 12 | `SC_GLOBAL_SELTRIG` | [4] selHoldExt, [3:2] selTrig | Hold mode and trigger source selection |

**`SC_GLOBAL_SELTRIG` default values:**

| `HOLD_MODE` | Register value | Bit pattern | Meaning |
|------------|---------------|------------|---------|
| `HOLD_MODE_EXTERNAL` | `0x14` | `1_01_00` | selHoldExt=1, selTrig=01 (local T1 threshold) |
| `HOLD_MODE_INTERNAL` | `0x04` | `0_01_00` | selHoldExt=0, selTrig=01 (local T1 threshold) |

**Default gain and shaping register values:**

| Register | Constant | Value | Meaning |
|----------|----------|-------|---------|
| Gain | `SC_SUBADD_GAIN` | `0x48` | hgGain=8 (mid-range), lgGain=4 (mid-range) |
| Shaping | `SC_SUBADD_TAU` | `0x44` | tauHG=4 (80 ns), tauLG=4 (80 ns) |

---

## 13. Error Codes

Returned by `system_init()` as `InitStatus_t`:

| Constant | Value | Meaning | Likely Cause |
|----------|-------|---------|--------------|
| `INIT_OK` | 0 | Success | — |
| `INIT_ERR_ASIC_NOACK` | 1 | ASIC did not ACK the I2C ping | `CLK_SM_I2C` not running; ASIC not powered; I2C wiring fault |
| `INIT_ERR_I2C_FAIL` | 2 | An I2C register write failed | Bus stuck; noise; wrong I2C timing register (`hi2c1.Init.Timing`) |

On any non-zero return code, `app_init()` enters a **blocking error state**:
LED3 (PB0) blinks at 200 ms indefinitely and the system does not proceed to
the main loop.

---

*All timing values are computed for a 216 MHz STM32F722 system clock with
PLLM=4, PLLN=216, PLLP=2, HSE source.*
*Protocol reference: RADIOROC2 Datasheet v2.0, Sections "I2C Configuration",
"Slow Control Registers".*
*Microcontroller reference: STM32F722 Reference Manual RM0431,
STM32F7 Series Datasheet DS11532.*
