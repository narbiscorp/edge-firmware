# Smart Glasses — Original Manufacturer Firmware (v1.0)

## Executive Summary

This repository contains the original manufacturer-supplied firmware for a pair of BLE-controlled smart glasses with electronically switchable LCD lenses. The glasses allow a connected smartphone app to wirelessly adjust lens opacity and alternating strobe timing via Bluetooth Low Energy (BLE). A built-in Hall effect sensor enables automatic sleep/wake functionality triggered by folding or unfolding the glasses' arms.

This document provides a complete technical analysis of the v1.0 firmware as delivered by the hardware manufacturer, prior to any modifications or feature additions.

---

## Table of Contents

1. [Hardware Platform](#hardware-platform)
2. [System Architecture](#system-architecture)
3. [Firmware Overview](#firmware-overview)
4. [BLE Communication Protocol](#ble-communication-protocol)
5. [PWM Lens Control](#pwm-lens-control)
6. [Strobe Engine](#strobe-engine)
7. [Power Management](#power-management)
8. [Build Configuration](#build-configuration)
9. [GPIO Pin Assignments](#gpio-pin-assignments)
10. [Known Limitations and Design Issues](#known-limitations-and-design-issues)
11. [File Manifest](#file-manifest)

---

## Hardware Platform

| Parameter | Value |
|---|---|
| **MCU** | ESP32 (Xtensa architecture) |
| **Module Variant** | ESP32-PICO-D4 (ESP32-U4WDH chip) |
| **Flash Memory** | 2 MB (sdkconfig) |
| **PSRAM** | Not used |
| **CPU Frequency** | 160 MHz |
| **BLE Stack** | Bluedroid (BLE 4.2 features) |
| **BLE Device Name** | `Smart_Glasses` |
| **MAC Address** | `28:05:A5:B2:1B:D4` |
| **Lens Interface** | 7-pin FFC connector driving two LCD channels |
| **Sleep Sensor** | Hall effect sensor on GPIO4 |
| **PWM Outputs** | GPIO27 (Channel 0 / Lens 1), GPIO26 (Channel 1 / Lens 2) |

The glasses contain two independently addressable LCD lens elements connected via a flat flex cable (FFC). Each lens is driven by a dedicated PWM channel, allowing the firmware to control opacity per-lens. A Hall effect sensor embedded near the hinge detects the open/closed state of the glasses' arms to trigger sleep and wake transitions.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32 (160 MHz)                       │
│                                                         │
│  ┌──────────────┐    ┌──────────────┐                   │
│  │  BLE Stack   │    │  GATT Server │                   │
│  │  (Bluedroid) │───▶│  Service:    │                   │
│  │  BLE 4.2     │    │  UUID 0x00FF │                   │
│  └──────────────┘    │  Char: 0xFF01│                   │
│                      │  (R/W)       │                   │
│                      └──────┬───────┘                   │
│                             │ Write Event               │
│                             ▼                           │
│                      ┌──────────────┐                   │
│                      │ BCD Decoder  │                   │
│                      │ Byte 0: Duty │                   │
│                      │ Byte 1: Time │                   │
│                      └──────┬───────┘                   │
│                             │                           │
│              ┌──────────────┼──────────────┐            │
│              ▼              ▼              ▼             │
│  ┌────────────────┐ ┌─────────────┐ ┌──────────────┐   │
│  │ PWM Ch0 (27)   │ │ PWM Ch1 (26)│ │ Hall Sensor  │   │
│  │ Lens 1         │ │ Lens 2      │ │ GPIO4 (Input)│   │
│  │ 10 kHz, 10-bit │ │ 10 kHz,10bit│ │ Sleep/Wake   │   │
│  └────────────────┘ └─────────────┘ └──────────────┘   │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │ TASK_ONE (FreeRTOS Task)                          │   │
│  │ Continuous alternating strobe loop                │   │
│  │ Lens 1 ON / Lens 2 OFF  ←→  Lens 1 OFF / Lens 2 ON│ │
│  │ Period: output_tim (ms)                           │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

---

## Firmware Overview

The firmware consists of a single source file (`main.c`, 523 lines) implementing a minimal BLE GATT server with PWM lens control. It is built using the Espressif IoT Development Framework (ESP-IDF) with Bluedroid BLE stack.

### Boot Sequence

1. **NVS Flash Initialization** — Non-volatile storage is initialized for BLE stack persistence.
2. **Bluetooth Controller Init** — Classic BT memory is released; BLE-only mode is enabled.
3. **Bluedroid Stack Init** — The Bluedroid BLE stack is initialized and enabled.
4. **GAP & GATT Registration** — Callback handlers are registered for GAP (advertising) and GATT (data) events.
5. **Hall Sensor GPIO Config** — GPIO4 is configured as input with internal pull-up for the Hall effect sensor.
6. **PWM Initialization** — Both PWM channels (GPIO27 and GPIO26) are configured at 10 kHz with 10-bit resolution, initial duty of 0%.
7. **Strobe Task Creation** — `TASK_ONE` is spawned as a FreeRTOS task (8 KB stack, priority 1) to run the continuous alternating strobe loop.
8. **Main Loop** — The main loop polls `check_sleep_condition()` once per second to monitor the Hall sensor for sleep entry.

### Runtime Behavior

Once booted, the device advertises as `Smart_Glasses` and waits for a BLE connection. A connected client writes 1–2 bytes to the GATT characteristic (UUID `0xFF01`) to control lens opacity and strobe timing. The strobe task runs continuously in the background, alternating which lens is active at the configured duty cycle and interval.

---

## BLE Communication Protocol

### GATT Service Structure

| Attribute | Value |
|---|---|
| **Service UUID** | `0x00FF` |
| **Characteristic UUID** | `0xFF01` |
| **Permissions** | Read / Write |
| **Properties** | Read / Write |
| **Max Value Length** | 100 bytes |

### Command Format

The client writes 1 or 2 raw bytes to the characteristic:

| Byte | Function | Processing |
|---|---|---|
| **Byte 0** | Lens duty cycle | `value × 0.6` → clamp to 100 → BCD decode → duty % |
| **Byte 1** (optional) | Strobe interval (ms) | BCD decode directly → `output_tim` in ms |

### Byte 0 — Duty Cycle Processing Pipeline

The manufacturer uses an unusual three-stage processing pipeline for the duty cycle byte:

```
Raw byte (0x00–0xFF) → Multiply by 0.6 → Clamp to 100 → BCD decode → PWM duty %
```

**Stage 1 — Scale by 0.6:**
The raw byte value is multiplied by 0.6 and truncated to an integer. This maps the full 0–255 input range into approximately 0–153.

**Stage 2 — Clamp to 100:**
Values above 100 (0x64) are clamped to 100. Since `255 × 0.6 = 153`, any input above `0xA9` (169) will be clamped.

**Stage 3 — BCD Decode:**
The clamped value is treated as a Binary-Coded Decimal (BCD) number. The upper nibble is multiplied by 10 and added to the lower nibble: `((val >> 4) * 10) + (val & 0x0F)`.

**Example mappings:**

| Sent (Hex) | Sent (Decimal) | ×0.6 | Clamped | BCD Decode | Final Duty % |
|---|---|---|---|---|---|
| `0x00` | 0 | 0 | 0 | 0 | 0% |
| `0x32` | 50 | 30 | 30 | 30 | 30% |
| `0x64` | 100 | 60 | 60 | 60 | 60% |
| `0xA9` | 169 | 101 | 100 | 100→BCD error* | ~10% |
| `0xFF` | 255 | 153 | 100 | 100→0x64→64 | 64% |

*Note: The value 100 decimal = `0x64` hex. BCD decoding yields `(6×10)+4 = 64`. This means the **maximum achievable duty cycle through this command interface is 64%**, not 100%. This appears to be a firmware bug (see [Known Limitations](#known-limitations-and-design-issues)).

### Byte 1 — Strobe Timing

The second byte is BCD-decoded directly (no scaling) and sets `output_tim`, the delay in milliseconds between strobe state changes. Lower values produce faster strobing. The minimum is clamped to 1 ms.

### Legacy Command Set (Commented Out)

The firmware contains a commented-out alternative command handler (lines 281–334) that used fixed single-byte commands:

| Command | Action |
|---|---|
| `0x01` | Set duty to 30% |
| `0x02` | Set duty to 40% |
| `0x03` | Set duty to 50% |
| `0x04` | Set duty to 60% |
| `0x05` | Increment duty by 1% (max 60%) |
| `0x06` | Decrement duty by 1% (min 10%) |
| `0x07` | Increment strobe interval |
| `0x08` | Decrement strobe interval |

This suggests an earlier development iteration with a simpler discrete-step approach before the BCD-based continuous control was implemented. The duty range was limited to 10–60% in this older version.

---

## PWM Lens Control

### Configuration

Both PWM channels share a single timer configuration:

| Parameter | Value |
|---|---|
| **Timer** | LEDC Timer 0 |
| **Speed Mode** | Low-speed mode |
| **Resolution** | 10-bit (0–1023 steps) |
| **Frequency** | 10 kHz |
| **Initial Duty** | 0% |

### Duty Cycle Conversion

The `pwm1_setduty()` and `pwm2_setduty()` functions convert a percentage (0–100) to a 10-bit duty value:

```c
duty = 1024 * duty / 100;  // 0% → 0, 50% → 512, 100% → 1024
```

This maps the duty percentage linearly across the full 10-bit range.

### Dual-Channel Design

The hardware provides two independent PWM outputs:

- **PWM Channel 0** → GPIO27 → Lens 1
- **PWM Channel 1** → GPIO26 → Lens 2

However, hardware analysis of the PCB and FFC connector has revealed that both lens channels may be electrically tied together on the lens assembly side, making independent per-lens control ineffective in practice.

---

## Strobe Engine

The strobe engine is implemented as a continuously running FreeRTOS task (`TASK_ONE`) created at boot with 8 KB of stack space.

### Operation

The task maintains a binary state variable and alternates between two configurations each cycle:

- **State 0:** Lens 1 = configured duty, Lens 2 = 1% (near-off)
- **State 1:** Lens 1 = 1% (near-off), Lens 2 = configured duty

The delay between state transitions is `output_tim` milliseconds (default: 10 ms at boot). This creates an alternating strobe effect between the two lenses.

### Key Observations

- There is **no explicit on/off command** for the strobe. The task runs from boot and never stops.
- When `pwm_duty_set = 0` (the boot default), both lenses remain at ≤1% duty, appearing visually off.
- Sending a non-zero duty byte "activates" the strobe by making opacity changes visible.
- Sending `0x00` as byte 0 effectively "deactivates" the strobe by returning both lenses to near-zero opacity.
- The "off" state uses 1% duty rather than true 0%, which may maintain a minimal voltage across the LCD elements.

---

## Power Management

### Sleep Entry

The firmware implements a simple Hall sensor–based sleep mechanism:

1. The `check_sleep_condition()` function is called once per second from the main loop.
2. If GPIO4 (Hall sensor) reads HIGH for 5 consecutive seconds (`SLEEP_HALL_WAIT_TIME`), sleep is triggered.
3. Before entering sleep, `pwm_duty_set` is zeroed (lenses off).
4. Deep sleep is entered with `esp_deep_sleep_start()`.

### Wake Condition

- **Wake source:** External wake on GPIO4 (`ext0`), triggered on LOW level.
- **Behavior:** The Hall sensor reading LOW (arm unfolded / magnet moved away) wakes the ESP32 from deep sleep.
- **On wake:** The ESP32 performs a full reboot, re-initializing all peripherals and the BLE stack.

### Power Characteristics (v1.0)

The original firmware does not implement any power optimization beyond the basic deep sleep mechanism. The BLE stack runs continuously while awake, advertising or maintaining a connection. No light sleep, dynamic frequency scaling, or BLE power management features are enabled.

---

## Build Configuration

| Parameter | Value |
|---|---|
| **Framework** | ESP-IDF (≥4.1.0 required) |
| **Project Name** | `ESP32_Ble` |
| **Target Chip** | ESP32 |
| **Flash Size** | 2 MB |
| **Partition Table** | Single app (default) |
| **BLE Stack** | Bluedroid |
| **BLE Version** | 4.2 features enabled, 5.0 disabled |
| **CPU Frequency** | 160 MHz |
| **PSRAM** | Not used |
| **Dependencies** | `espressif/led_strip` (for onboard LED, not used in main logic) |

### Build Command

```bash
idf.py set-target esp32
idf.py build
```

### Flash Command

```bash
idf.py -p /dev/ttyUSBx flash
```

Alternatively, the ESP32 Flash Download Tool (GUI) can be used for production flashing.

---

## GPIO Pin Assignments

| GPIO | Function | Direction | Notes |
|---|---|---|---|
| **GPIO27** | PWM Channel 0 (Lens 1) | Output | 10 kHz, 10-bit resolution |
| **GPIO26** | PWM Channel 1 (Lens 2) | Output | 10 kHz, 10-bit resolution |
| **GPIO4** | Hall Effect Sensor | Input | Internal pull-up enabled; HIGH = arm folded (magnet near) |
| **GPIO5** | Onboard LED (configured but unused) | Output | From sdkconfig; not referenced in main.c |

---

## Known Limitations and Design Issues

### 1. Maximum Duty Cycle Capped at 64%

Due to the `×0.6` scaling followed by BCD decoding, the maximum achievable lens duty cycle is 64%, not 100%. The value 100 (decimal) after clamping equals `0x64` in hex, which BCD-decodes to `(6×10)+4 = 64`. There is no input byte that produces a duty cycle above 64%.

### 2. BCD Decoding Produces Invalid Values for Certain Inputs

BCD encoding requires each nibble to be in the range 0–9. After the `×0.6` scaling, certain input values produce hex representations with nibbles A–F (e.g., `0x5A`), which BCD-decode to mathematically incorrect results. For example, `0x5A` BCD-decodes to `(5×10)+10 = 60`, which happens to be valid, but this is coincidental rather than by design.

### 3. No Strobe On/Off Toggle

The strobe task runs unconditionally from boot. There is no command to pause or stop it. The only way to visually "disable" the strobe is to set duty to 0.

### 4. No Persistent Settings

The `pwm_duty_set` and `output_tim` variables are not stored in NVS (non-volatile storage). All settings reset to defaults (duty=0, time=10ms) on every boot or wake from deep sleep. The `RTC_DATA_ATTR` macro is commented out in the variable declarations, suggesting this was considered but not implemented.

### 5. No OTA Update Capability

The firmware uses a single-app partition table with no OTA partitions. Firmware updates require physical access to the device and a USB/serial connection.

### 6. Floating-Point Arithmetic on Integer MCU

The duty scaling uses floating-point multiplication (`* 0.6`), which is software-emulated on the ESP32's Xtensa core. While functionally correct, integer arithmetic (e.g., `* 3 / 5`) would be more efficient.

### 7. Minimal Error Handling

The firmware has no recovery mechanism for BLE stack failures, PWM configuration errors beyond `ESP_ERROR_CHECK` (which triggers a reboot on failure), or watchdog handling.

### 8. Unused Code and Dependencies

The project includes `led.c`, `led.h`, `heart_rate.h`, and `heart_rate_mock.c` files from Espressif example code that are not used in the main firmware. The `espressif/led_strip` dependency is declared but not functionally integrated into the glasses' operation.

---

## File Manifest

| File | Description |
|---|---|
| `main.c` | Core firmware — BLE server, PWM control, strobe task, sleep logic |
| `CMakeLists.txt` | Build system configuration (project name: `ESP32_Ble`) |
| `Kconfig.projbuild` | Menuconfig options (LED type selection, GPIO number) |
| `sdkconfig` | Full ESP-IDF configuration snapshot |
| `sdkconfig.defaults` | Minimal default config (BLE enabled, BLE 4.2) |
| `sdkconfig_defaults.esp32` | ESP32-specific defaults (LED on GPIO5) |
| `idf_component.yml` | Component manager manifest (led_strip dependency) |
| `dependencies.lock` | Locked dependency versions |
| `led.c` / `led.h` | Onboard LED control (from Espressif example, unused in main logic) |
| `heart_rate.h` / `heart_rate_mock.c` | Heart rate mock module (from Espressif example, unused) |
| `Gerber_*.G*` / `Drill_*.DRL` | PCB Gerber and drill files for the glasses circuit board |
| `FlyingProbeTesting.json` | Flying probe test configuration for PCB quality assurance |
| `Smart_Glasses-Flash.docx` | Flash programming documentation from manufacturer |

---

## Version History

| Version | Description |
|---|---|
| **v1.0** | Original manufacturer firmware. Basic BLE control with alternating lens strobe, Hall sensor sleep, and BCD-encoded command protocol. This document describes this version. |

---

*This document was prepared as part of the technical due diligence documentation for the Smart Glasses product. It describes the firmware exactly as received from the hardware manufacturer, with no modifications.*
