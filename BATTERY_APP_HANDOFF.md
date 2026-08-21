# Narbis Edge Glasses — App Handoff: Battery Level + Standalone Pacer

**Firmware:** v4.16.1 (`Narbis_Edge`, glasses). Supersedes the v4.16.0 draft of
this document, which named the wrong ADC pin — ignore any older copy.

**Audience:** the iOS (Swift/CoreBluetooth) agent and the Kotlin/Android agent.
Each of you can work from this document alone; §10 has a self-contained prompt
per platform. Read §1 before writing any UI — it explains why battery will read
"unknown" on every unit you can physically test today, and why that is correct
behavior rather than a bug to work around.

Two features are in scope:

- **A. Battery level** — new in firmware; needs display + graceful-unknown handling.
- **B. Standalone lens pacing** — already in firmware and already persistent;
  needs a settings UI so a user can set the pace the glasses use with no app
  and no sensor attached.

---

## 1. Hardware reality — read this first

The glasses measure battery through a resistive divider on VBAT feeding an ADC
pin. **On V1.1 boards — which is every unit currently in the field, including
the ones on your desk — that divider is not fitted.** BAT+ runs straight from
the cell to the TP4057 charger and on to the TPS63802 buck-boost; nothing taps
it to an ADC input.

So on today's hardware:

- `soc` = `0xFF` (unknown), `mv` = 0, `charging` = `0xFF` (unknown)
- BLE Battery Service reads `0`
- The firmware log line says `batt: unsupported (no VBAT divider — expected on V1.1)`

**This is the correct, expected result.** Do not add a workaround, do not fake a
percentage, and do not treat it as a connection failure. Your job is to make the
unknown state look deliberate in the UI (§4/§5) so that when V1.2 hardware
arrives the same build lights up with no changes.

The V1.2 respin spec (board `STS-USA50925-ESP32-ZZ`) fits the divider as 2× 1 MΩ
1% 0402 + 100 nF X7R, feeding **GPIO36 / SENSOR_VP / physical pin 5 /
ADC1_CHANNEL_0** — a 3.0–4.2 V cell becomes 1.5–2.1 V at the pin, ~2 µA
quiescent. The firmware already targets that pin and falls back to probing
GPIO39/34/32/33 in case the respin moves it. You do not need to care which pin
wins; you only need to handle supported vs unsupported.

> The **earclip** is a completely different device (XIAO ESP32-C6, different
> battery circuit). Its battery already works and arrives over a different frame
> (`0xF8`). Do not conflate the two in the UI — see §3.4.

---

## 2. BLE connection basics

| Item | Value |
|---|---|
| Advertised name | `Narbis_Edge` |
| Primary service | `0x00FF` (16-bit) |
| Control (write) | `0xFF01` |
| OTA (write) | `0xFF02` — not your concern |
| **Status (notify)** | **`0xFF03`** — all telemetry arrives here |
| PPG (notify) | `0xFF04` |
| Battery Service | `0x180F`, Battery Level `0x2A19` (read + notify) |

Everything on `0xFF03` is a **type-tagged binary frame**: byte 0 is the frame
type, the rest is that type's payload. Subscribe once and demultiplex on byte 0.
On subscribing, the firmware immediately sends a hello log line containing its
version string — parse it to gate features (§6).

---

## 3. Battery wire protocol

### 3.1 Frame `0xFB` — glasses battery (primary source)

Sent on `0xFF03`. 5 bytes:

| byte | field | meaning |
|---|---|---|
| 0 | `0xFB` | frame type |
| 1–2 | `mv` u16 **little-endian** | smoothed pack voltage, millivolts. 0 = unknown |
| 3 | `soc` u8 | state of charge 0–100. **`0xFF` = unknown** |
| 4 | `charging` u8 | 0 = discharging, 1 = charging, **`0xFF` = unknown** |

Example: `FB 6E 0F 48 00` → 3950 mV, 72 %, not charging.
Unsupported board: `FB 00 00 FF FF`.

**Cadence:** immediately on subscribe, then every 30 s, plus on demand via
`0xC7 0x00`. After a cold boot the first frame may legitimately be `unknown`
while the divider probe runs; a real value follows within ~35 s.

### 3.2 Battery Service `0x180F` (secondary, for generic clients)

Characteristic `0x2A19`, read + notify, value 0–100. While unknown it reads `0`,
because the standard has no unknown encoding. **Prefer `0xFB`** — it is the only
source that distinguishes "0 %" from "I don't know". Use `0x180F` only if you
want an OS-level battery indicator for free.

On iOS, `0x180F` may be cached or served by the system; do not rely on it as
your only path. On Android it is a normal GATT service. Web Bluetooth clients
must add `battery_service` to `optionalServices`.

### 3.3 Control opcode `0xC7` (write to `0xFF01`)

| Bytes | Effect |
|---|---|
| `C7 00` | Emit a `0xFB` frame now + one human-readable log line |
| `C7 01` | Diagnostic: clear cached pin, re-probe all candidates, dump each channel's mV to the log, then emit |

`C7 00` is what you call on pull-to-refresh. `C7 01` is a bench/engineering
action — put it behind a debug screen if you expose it at all, since it takes a
few seconds and briefly interrupts sampling.

### 3.4 Do not confuse with frame `0xF8`

`0xF8` has an **identical 4-byte payload layout** but reports the **earclip's**
battery, relayed through the glasses. If you already parse `0xF8`, reuse the
decoder but keep the two values in separate fields and separate UI slots:

- `0xFB` → glasses battery
- `0xF8` → earclip battery

### 3.5 Semantics you must honor

- `soc == 0xFF` → render "—" or hide the element. **Never render `255 %`.**
  This is the single most likely bug; write a test for it.
- `charging` is a **best-effort voltage-slope heuristic** with ~90 s latency —
  the charger's status pins drive indicator LEDs only and have no route to the
  MCU. Render it as a charging glyph. Never use it to claim "charge complete" or
  to drive a time-remaining estimate.
- `mv` is the trustworthy raw signal; log it in session files so the SoC curve
  can be tuned later against real discharge data.
- The firmware applies **no** low-battery behavior. Any warning is yours to
  implement: suggest ≤ 15 % warn, ≤ 5 % urgent, or `mv ≤ 3550`.

---

## 4. iOS implementation notes (CoreBluetooth)

Decode inside your existing `0xFF03` notification handler:

```swift
struct GlassesBattery {
    let millivolts: Int?     // nil when unknown
    let percent: Int?        // nil when unknown
    let isCharging: Bool?    // nil when unknown
}

// data = value of characteristic 0xFF03
guard data.count >= 5, data[0] == 0xFB else { return }
let mv  = Int(data[1]) | (Int(data[2]) << 8)
let soc = data[3]
let chg = data[4]

let battery = GlassesBattery(
    millivolts: mv == 0 ? nil : mv,
    percent:    soc == 0xFF ? nil : Int(soc),
    isCharging: chg == 0xFF ? nil : (chg == 1)
)
```

- Publish it on your device-state object; the battery view binds to `percent`
  as an optional and renders "—" for `nil`.
- Refresh action → write `Data([0xC7, 0x00])` to `0xFF01` (write-with-response).
- If you also read `0x180F`, add it to the services you discover, but treat
  `0xFB` as authoritative.

## 5. Kotlin / Android implementation notes

Repo: `C:\CODE\Edge-Muse-Android`. There is already a `0xFF03` frame dispatcher
that handles `0xF8` (earclip battery) — add `0xFB` beside it rather than writing
a new path.

```kotlin
data class GlassesBattery(
    val millivolts: Int?,   // null when unknown
    val percent: Int?,      // null when unknown
    val isCharging: Boolean?,
)

fun parseGlassesBattery(d: ByteArray): GlassesBattery? {
    if (d.size < 5 || d[0] != 0xFB.toByte()) return null
    val mv  = (d[1].toInt() and 0xFF) or ((d[2].toInt() and 0xFF) shl 8)
    val soc = d[3].toInt() and 0xFF
    val chg = d[4].toInt() and 0xFF
    return GlassesBattery(
        millivolts = mv.takeIf { it != 0 },
        percent    = soc.takeIf { it != 0xFF },
        isCharging = if (chg == 0xFF) null else chg == 1,
    )
}
```

- Surface as `glassesBattery` in the same state holder that already exposes the
  earclip battery. The glasses card binds to this one.
- Refresh → `writeCharacteristic(ctrl0xFF01, byteArrayOf(0xC7.toByte(), 0x00))`.
- Watch sign extension: `d[1].toInt() and 0xFF`. Forgetting the mask on bytes
  ≥ 0x80 gives negative millivolts.

---

## 6. Feature gating by firmware version

On subscribe to `0xFF03`, the firmware sends a `0xF1` log frame (byte 0 = `0xF1`,
rest = ASCII) that begins:

```
Narbis fw v4.16.1-battery test=0 mode=0
```

Parse the version. **Show battery UI only for ≥ 4.16.0.** Older firmware never
sends `0xFB`, so a UI that waits for one would spin forever — degrade to hidden
or "—", never to a spinner.

---

## 7. Standalone lens pacing — settings UI (please implement)

> **⚠️ SUPERSEDED as of firmware v4.17.0** — see
> `STANDALONE_PROGRAMS_APP_HANDOFF.md`. The three-program cycle described below
> no longer exists: the glasses now run one standalone program (Breathe at the
> saved pace) and a quick close-and-reopen of the arm does nothing. The pacing
> opcodes in §7.2 are all still correct and still persist; only the "Program 1 /
> 2 / 3" framing and the tap-to-cycle behavior have changed. Read §7.2, then go
> to the new document for what to build.

### 7.1 Confirmed behavior

I verified this against the firmware source this session, and it works the way
you'd hope:

**The glasses remember the last pace you sent, across power cycles.** Opcode
`0xB1` writes the value to NVS (flash) immediately, and boot restores it. So:
set 5 BPM from the app → close the arms (power off) → reopen tomorrow → the
glasses start breathing at 5 BPM with no app present. Confirmed in `main.c`
(`0xB1` handler → `prefs_set_u8(KEY_BREATHE_BPM, …)`, restored in `prefs_load()`
with factory default 6).

Two related facts worth knowing:

- **The program does not persist.** Every power-on lands on Program 1 (BREATHE).
  Only the *parameters* persist, not which program was last selected.
- **Persistence is unconditional.** Any `0xB1` you send is saved, including one
  sent mid-session for a transient reason. If your app has a "session pace" that
  differs from the user's preferred standalone pace, you must re-assert the
  standalone value at session end — otherwise the glasses keep the session value
  forever. This is the main correctness trap in this feature.

### 7.2 The persistent pacing opcodes

All are writes to `0xFF01`, 2 bytes, `[opcode][value]`, all saved to NVS:

| Opcode | Meaning | Range | Default |
|---|---|---|---|
| `0xB1` | **Breaths per minute** | 1–30 | 6 |
| `0xB2` | Inhale percentage of cycle | 10–90 | 40 |
| `0xB3` | Hold at top, ×100 ms | 0–50 | 0 |
| `0xB4` | Hold at bottom, ×100 ms | 0–50 | 0 |
| `0xB5` | Waveform: 0 = sine, 1 = linear | 0–1 | 0 |
| `0xA2` | Max tint / brightness % | 0–100 | 100 |
| `0xA4` | Session length, minutes | 1–60 | 30 |

Out-of-range values are clamped by the firmware, not rejected — clamp in the UI
too so the user sees what actually took effect.

`0xBF 0x00` is a factory reset of all preferences. Offer it as a clearly
destructive "Reset glasses to defaults" action, if at all.

Example — set 5 breaths/min: write `B1 05`.

### 7.3 What to build

Add to the settings menu a **"Standalone mode"** (or "Glasses without app")
section, framed for the user as: *what the glasses do on their own, with no
phone and no sensor connected.*

Required:

1. **Pace control** — slider or stepper, 1–30 breaths/min, default 6, showing
   the current value. On commit, write `0xB1`.
2. **An explicit "Save to glasses" button.** Do not rely on a silent
   write-on-change: the point of this screen is that the user understands the
   value is stored *on the device* and will be used without the app. Confirm
   success visibly ("Saved to glasses — they'll use 5 breaths/min on their own").
3. **Read-back on open.** There is no config-read characteristic for these
   values, so mirror them in app-side storage and show your last-known value,
   labelled as such. Do not display a fabricated "current device value".
4. **Re-assert on session end.** If your session used a different pace, write
   the user's saved standalone pace back when the session finishes.

Optional but nice: expose max tint (`0xA2`) and session length (`0xA4`) in the
same section — they have the same persist-and-run-standalone character.

---

## 8. Startup behavior change in v4.16.1 (affects your copy/screenshots)

> **⚠️ PARTLY SUPERSEDED by firmware v4.17.0.** The removal of the Program-1
> startup pulse described here still holds. The rest — "2 pulses = Program 2,
> 3 pulses = Program 3" — applies only to firmware < 4.17.0. On v4.17.0+ a quick
> close-and-reopen does nothing unless an app has explicitly enabled a
> multi-program cycle. See `STANDALONE_PROGRAMS_APP_HANDOFF.md` §1.

The glasses used to pulse the lenses once, a couple of seconds after opening, to
indicate "Program 1". **That pulse is gone.** Opening the glasses now goes
directly into breathing at the saved pace, with no indicator.

Programs 2 and 3 still announce themselves with 2 and 3 pulses when you tap
(close-and-reopen within ~4 s), and cycling back around to Program 1 is silent.

If any onboarding copy, help text, or video says "watch for one pulse to confirm
Program 1", it needs updating. The user-visible rule is now:

> Open the glasses → they start breathing. Tap to change program: 2 pulses =
> Program 2, 3 pulses = Program 3, no pulses = back to Program 1.

The semantic indicators are unchanged: 5 pulses = PPG sensor detected, 3 pulses
= earclip linked, 2 pulses = earclip lost.

---

## 9. Test plan

### 9.1 Battery — what you can test today (V1.1, no divider)

| # | Test | PASS |
|---|---|---|
| A1 | Connect, subscribe `0xFF03` | A `0xFB` frame arrives within ~5 s |
| A2 | Decode it | `soc` = 0xFF → your model holds `null` |
| A3 | UI | Shows "—" or hides the element. **No "255 %", no crash, no infinite spinner** |
| A4 | `0x180F` read | Returns 0; UI still shows unknown, not "0 %" |
| A5 | Refresh action | `C7 00` → a fresh `0xFB` arrives |
| A6 | Old firmware (< 4.16.0) | Battery UI hidden/"—", no spinner |
| A7 | Earclip connected too | Earclip (`0xF8`) and glasses (`0xFB`) battery show in separate slots, neither overwrites the other |

### 9.2 Battery — when V1.2 hardware arrives

| # | Test | PASS |
|---|---|---|
| B1 | Fresh charge | `soc` ≥ 80 %, `mv` ≥ 4000 |
| B2 | Cadence | Frame every 30 s ±2 s |
| B3 | `0x180F` vs `0xFB` | Agree within 1 % |
| B4 | 30-min session | `soc` non-increasing (±1 jitter), no jump > 5 |
| B5 | Plug USB | `charging` → 1 within ~3 min; → 0 within ~3 min of unplug |
| B6 | Low battery | Your warning fires at the threshold you chose |

If B1 or B4 fails on real V1.2 hardware, that is a firmware/curve issue — report
it back with raw `mv` values rather than adjusting the app.

### 9.3 Pacer

> **⚠️ Rows C1–C6 and D1–D4 are superseded on firmware v4.17.0+** by
> `STANDALONE_PROGRAMS_APP_HANDOFF.md` §7. C1–C3, C5 and C6 still pass as
> written; C4 and every D row assume the old tap-to-cycle behavior.

| # | Test | PASS |
|---|---|---|
| C1 | Set 5 BPM in settings, tap Save | Confirmation shown |
| C2 | Disconnect app, close + reopen glasses | Glasses breathe at 5/min unaided (count for 60 s) |
| C3 | Power-cycle again, no app in range | Still 5/min |
| C4 | Run an app session at a different pace, end it | Glasses return to the saved standalone pace |
| C5 | Set 1 and 30 | Both accepted; UI matches device behavior |
| C6 | Factory reset (if exposed) | Returns to 6/min |

### 9.4 Startup pulse

| # | Test | PASS |
|---|---|---|
| D1 | Open glasses | Tinting begins; **no pulse at any point** in the first 15 s |
| D2 | Close + reopen within ~4 s | 2 pulses, then Program 2 |
| D3 | Repeat | 3 pulses, then Program 3 |
| D4 | Repeat | **No pulses**, back to Program 1 breathing |

---

## 10. Self-contained agent prompts

### 10.1 iOS agent

> The Narbis Edge glasses firmware v4.16.1 adds battery reporting and I need the
> iOS app updated. Read `BATTERY_APP_HANDOFF.md` in the `edge-firmware` repo —
> it is the complete spec; work from it rather than guessing at the protocol.
>
> Implement:
> 1. Parse frame type `0xFB` in the existing `0xFF03` notification handler:
>    `[0]=0xFB, [1..2]=mv u16 LE, [3]=soc, [4]=charging`. `soc==0xFF` and
>    `charging==0xFF` mean unknown — model them as `nil`, never as numbers.
> 2. Show glasses battery in the device/status UI, separate from the existing
>    earclip battery (which comes from `0xF8` with the same payload layout).
>    Unknown renders as "—". Gate the UI on firmware ≥ 4.16.0, parsed from the
>    `0xF1` hello string sent on subscribe.
> 3. Pull-to-refresh writes `[0xC7, 0x00]` to `0xFF01`.
> 4. Add a "Standalone mode" settings section with a 1–30 breaths/min control
>    (default 6) and an explicit "Save to glasses" button that writes
>    `[0xB1, value]` to `0xFF01` and confirms visibly. If a session used a
>    different pace, re-assert the saved standalone pace when the session ends.
>
> Critical: every unit we can test on today is board V1.1, which has no battery
> divider fitted, so battery will legitimately report unknown. That is correct
> behavior — build the unknown state to look deliberate, do not fake a value,
> and do not treat it as a connection error. Run test sets 9.1 and 9.3 from the
> handoff doc and report results.

### 10.2 Kotlin / Android agent

> The Narbis Edge glasses firmware v4.16.1 adds battery reporting and I need the
> Android app (`C:\CODE\Edge-Muse-Android`) updated. Read
> `BATTERY_APP_HANDOFF.md` in the `edge-firmware` repo — it is the complete
> spec; work from it rather than guessing at the protocol.
>
> Implement:
> 1. In the existing `0xFF03` frame dispatcher (it already handles `0xF8` for
>    earclip battery), add a `0xFB` case for glasses battery:
>    `[0]=0xFB, [1..2]=mv u16 LE, [3]=soc, [4]=charging`. Mask bytes with
>    `and 0xFF` to avoid sign extension. `soc==0xFF`/`charging==0xFF` mean
>    unknown — model as `null`, never as numbers.
> 2. Expose `glassesBattery` in the same state holder as the earclip battery and
>    bind the glasses card to it. Unknown renders "—". Gate the UI on firmware
>    ≥ 4.16.0, parsed from the `0xF1` hello string sent on subscribe.
> 3. Refresh action writes `byteArrayOf(0xC7, 0x00)` to `0xFF01`.
> 4. Add a "Standalone mode" settings section with a 1–30 breaths/min control
>    (default 6) and an explicit "Save to glasses" button that writes
>    `byteArrayOf(0xB1, value)` to `0xFF01` and confirms visibly. If a session
>    used a different pace, re-assert the saved standalone pace at session end.
>
> Critical: every unit we can test on today is board V1.1, which has no battery
> divider fitted, so battery will legitimately report unknown. That is correct
> behavior — build the unknown state to look deliberate, do not fake a value,
> and do not treat it as a connection error. Run test sets 9.1 and 9.3 from the
> handoff doc and report results.

---

## 11. Firmware-side reference (for questions that come back)

- Battery implementation: `v4/Code-Glasses/main/main.c`, block
  `GLASSES BATTERY MONITOR` — pin rationale, probe gates, SoC curve.
- Program indicator: same file, `program_indicator()`.
- Persisted preferences: `prefs_load()` and the `KEY_*` defines.
- Delivery: merge to `main` → GitHub Actions builds → Release publishes
  `ESP32_Ble.bin` → OTA webapp updates a unit in ~3 minutes.
- The FCC build (`FCC_TEST_BUILD=1`, separate repo) compiles the same feature;
  its `0xC6` FCC-state opcode does not collide with `0xC7`.
