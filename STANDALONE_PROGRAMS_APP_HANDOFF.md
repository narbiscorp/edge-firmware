# Narbis Edge Glasses — App Handoff: Standalone Programs (v4.17.0)

**Firmware:** v4.17.0 (`Narbis_Edge`, glasses).

**Audience:** the iOS (Swift/CoreBluetooth) agent. §9 is a self-contained prompt
you can work from alone. The Kotlin/Android agent can follow the same protocol —
§3 is platform-neutral.

**Supersedes** `BATTERY_APP_HANDOFF.md` §7 ("Standalone lens pacing") and §8
("Startup behavior change in v4.16.1"), and its test rows C1–C6 / D1–D4. Those
sections describe a three-program cycle that no longer exists. Everything else
in that document (battery, `0xFB`, `0x180F`, connection basics) is still current.

Read §1 before writing any UI or copy. The default standalone behavior changed,
and the change removes a feature users could previously reach by accident.

---

## 1. What changed, and why

### Before (v4.16.x and earlier)

Opening the left arm powered the glasses on into **Program 1: BREATHE**. Closing
and re-opening the arm within ~4 seconds advanced a hard-coded three-program
cycle:

| Program | Behavior | Indicator on select |
|---|---|---|
| 1 | BREATHE — lens tint follows a 6 BPM waveform | silent |
| 2 | BREATHE + 10 Hz strobe | 2 pulses |
| 3 | Plain 10 Hz strobe | 3 pulses |

Holding the arm closed for ≥5 s slept the device.

### Now (v4.17.0)

**There is exactly one standalone program out of the box: BREATHE at the saved
pace (6 breaths/min unless an app changed it).** Programs 2 and 3 are gone from
the default cycle.

> Open the left arm → the glasses start breathing.
> Close and re-open the arm → **nothing happens.**
> Hold the arm closed for 5 seconds → the glasses sleep, exactly as before.

Nothing standalone strobes any more unless an app deliberately programs a strobe
into a slot and enables the cycle.

### Why

The strobe programs were reachable by accident. Folding an arm and opening it
again — putting the glasses down, adjusting the fit, taking them off for a
moment — advanced the program. Users landed in a 10 Hz strobe they never asked
for, with no labelled way back out and nothing on screen explaining it. The
default product is a breathing guide; strobe is now an opt-in that an app
configures, not something a fold-and-unfold can trip.

### What did *not* change

- Sleep on a ≥5 s arm close. Identical.
- Wake on opening the arm. Identical.
- Everything the app can command over BLE while connected — `0xA5` STATIC,
  `0xA6` STROBE, `0xB0` BREATHE / BREATHE+STROBE, `0xBA` breathe-sync. **An app
  can still put the lens in any mode at any time.** This change is only about
  what the *arm* can reach with no app in the picture.
- PPG-auto programs (PulseSensor plugged in, `0xB7`). Separate list, untouched.
- Battery, pairing, OTA, the earclip link.
- The semantic indicator pulses: 5 = PPG sensor detected, 3 = earclip linked,
  2 = earclip lost, 3 = forget-earclip gesture accepted.
- The five-tap recovery gesture (five arm taps within 2 s = forget the paired
  earclip and rescan). Still works with the program cycle disabled — it is a
  deliberate five-tap sequence, not something a stray fold trips.

### Copy and onboarding you need to fix

Anything that says *"tap to change program"*, *"2 pulses = Program 2"*,
*"3 pulses = Program 3"*, or describes a strobe mode reachable from the glasses
themselves is now wrong. Replace with:

> Open the glasses to start. Hold them closed for 5 seconds to turn them off.

---

## 2. The feature you are building

The firmware keeps a table of **up to 5 standalone program slots** plus a
**count** of how many the arm cycles. Both live in the glasses' flash and
survive power cycles.

- **Count = 1 (the default, and the state of every glasses in the field after
  this update).** One program. Arm tap does nothing.
- **Count = 2…5.** The arm short-tap toggles programs 1…N, with the same feel as
  the old 1→2→3 cycle — including the N-pulse "which program am I on" indicator
  (program 1 stays silent).

**Your job:** a settings screen where a user can define slots 1–5 and turn the
cycle on. It must be **off by default and require a deliberate action to
enable.** A user who never opens this screen must keep the single-program
behavior described in §1 forever.

### Slot fields

| Field | Range | `0` means | Notes |
|---|---|---|---|
| `mode` | 0–3 | *(not optional)* | 0 = Breathe, 1 = Breathe + Strobe, 2 = Strobe, 3 = Static tint |
| `bpm` | 1–30 | inherit the saved `0xB1` value | Breathing rate |
| `inhalePct` | 10–90 | inherit the saved `0xB2` value | Inhale share of the cycle |
| `strobeDeciHz` | 10–500 | inherit the saved `0xAB` value | Strobe rate ×10 (100 = 10.0 Hz) |
| `dutyPct` | 10–90 | inherit the saved `0xAC` value | Strobe duty (% of period dark) |
| `brightness` | 1–100 | inherit the saved `0xA2` value | Max tint. For `mode = 3` this **is** the static tint level. |

**`0` = inherit is the important idea.** An all-zero slot with `mode = 0` is
"Breathe at whatever pace the user saved" — which is precisely the factory
default, so a device that has never been programmed needs no migration and no
first-run write from you. Prefer sending `0` over echoing the current global
value: a slot that inherits automatically tracks later `0xB1`/`0xA2`/etc.
changes, whereas a slot that pins a value will silently override them.

Out-of-range non-zero values are **clamped by the firmware, not rejected**.
Clamp in the UI too, so the user sees what actually took effect.

---

## 3. Wire protocol

All commands are writes to control characteristic **`0xFF01`** on service
`0x00FF`. All replies arrive as type-tagged frames on the status characteristic
**`0xFF03`** (subscribe once, demultiplex on byte 0). See
`BATTERY_APP_HANDOFF.md` §2 for connection basics.

`0xFF01` supports write-with-response and write-without-response (v4.16.3). Use
**write-with-response** for these three opcodes — they touch flash and you want
the completion callback.

### 3.1 `0xBC` — set the program count (this is the on/off switch)

```
[0xBC][count]        2 bytes
```

| `count` | Effect |
|---|---|
| `0` or `1` | Cycle **off**. One program. Arm tap does nothing. **Default.** |
| `2`–`5` | Arm short-tap toggles programs 1…count. |

Persisted immediately. Values above 5 are clamped to 5, so you may send 5
without querying the firmware's limit. If the count drops below the index of the
program currently running, the firmware falls back to program 1 so the lens can
never be stuck on a program the arm can no longer reach.

Example — enable a 3-program cycle: write `BC 03`. Turn it back off: `BC 01`.

### 3.2 `0xBD` — write one slot

```
[0xBD][slot][mode][bpm][inhale][dhz_lo][dhz_hi][duty][brightness]      9 bytes
```

- `slot` is **0-based** (`0` = the program the user sees as "Program 1").
- `dhz_lo`/`dhz_hi` are the deci-Hz value as **u16 little-endian**.
- Persisted immediately.
- Writing the slot the glasses are currently running re-applies it, so the lens
  reflects the edit live — useful for a preview control.
- **This does not enable the cycle.** That is `0xBC`, separately and
  deliberately.

Examples:

```
Slot 1 = Breathe, all inherited (the factory default):
  BD 00 00 00 00 00 00 00 00

Slot 2 = Breathe at 5 BPM, 40% inhale, 80% tint:
  BD 01 00 05 28 00 00 00 50

Slot 3 = Plain strobe at 10.0 Hz, 50% duty, full tint:
  BD 02 02 00 00 64 00 32 64
         │  │  │  └──┴── 0x0064 = 100 deci-Hz = 10.0 Hz
         │  │  └──────── inhale: inherit (unused in this mode)
         │  └─────────── bpm: inherit (unused in this mode)
         └────────────── mode 2 = STROBE
```

### 3.3 `0xBE` — read the whole configuration back

```
Write:  [0xBE][0x00]      2 bytes
Reply:  frame 0xFC on 0xFF03
```

Reply payload, **44 bytes total**:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `0xFC` frame type |
| 1 | 1 | `count` — programs the arm cycles (1 = cycle off) |
| 2 | 1 | `max` — slots the firmware supports (5 today; read it, don't hardcode) |
| 3 | 1 | `active` — 0-based index of the program running right now |
| 4 | 8 | slot 0 |
| 12 | 8 | slot 1 |
| 20 | 8 | slot 2 |
| 28 | 8 | slot 3 |
| 36 | 8 | slot 4 |

Each 8-byte slot record:

| Offset | Field |
|---|---|
| 0 | `mode` |
| 1 | `bpm` |
| 2 | `inhalePct` |
| 3–4 | `strobeDeciHz` (u16 LE) |
| 5 | `dutyPct` |
| 6 | `brightness` |
| 7 | reserved (0) |

**This is a real read-back.** Unlike the pacing opcodes in the old handoff,
you do not have to mirror state in app-side storage and label it "last known" —
query `0xBE` on screen open and show the device's actual configuration.

### 3.4 Factory reset

`0xBF 0x00` erases all glasses preferences, including the slot table and the
count. The device returns to one Breathe program at 6 BPM. Offer it as a clearly
destructive action, if at all.

---

## 4. iOS implementation notes

### 4.1 Model

```swift
enum StandaloneMode: UInt8, CaseIterable {
    case breathe       = 0
    case breatheStrobe = 1
    case strobe        = 2
    case staticTint    = 3

    var title: String {
        switch self {
        case .breathe:       return "Breathe"
        case .breatheStrobe: return "Breathe + Strobe"
        case .strobe:        return "Strobe"
        case .staticTint:    return "Fixed tint"
        }
    }
}

/// One standalone program slot. `nil` on an optional field means
/// "inherit the value saved on the glasses" and encodes as 0.
struct StandaloneProgram: Equatable {
    var mode: StandaloneMode = .breathe
    var bpm: UInt8?          = nil     // 1...30
    var inhalePct: UInt8?    = nil     // 10...90
    var strobeDeciHz: UInt16? = nil    // 10...500
    var dutyPct: UInt8?      = nil     // 10...90
    var brightness: UInt8?   = nil     // 1...100

    static let inherited = StandaloneProgram()
}

struct StandaloneConfig {
    var count: UInt8              // 1 = arm cycle off
    var maxSlots: UInt8           // from the device, do not hardcode
    var activeIndex: UInt8
    var slots: [StandaloneProgram]
}
```

### 4.2 Encoding

```swift
extension StandaloneProgram {
    /// The 7 payload bytes that follow [0xBD][slot].
    var wireBytes: [UInt8] {
        let dhz = strobeDeciHz ?? 0
        return [
            mode.rawValue,
            bpm ?? 0,
            inhalePct ?? 0,
            UInt8(dhz & 0xFF),
            UInt8(dhz >> 8),
            dutyPct ?? 0,
            brightness ?? 0,
        ]
    }
}

func writeSlot(_ index: UInt8, _ program: StandaloneProgram) {
    // 9 B total: [0xBD][slot] + 7 payload bytes. The write form has NO
    // reserved byte — see the warning below.
    let packet: [UInt8] = [0xBD, index] + program.wireBytes
    precondition(packet.count == 9)
    peripheral.writeValue(Data(packet), for: controlChar, type: .withResponse)
}

func setProgramCount(_ count: UInt8) {
    peripheral.writeValue(Data([0xBC, count]), for: controlChar, type: .withResponse)
}

func requestStandaloneConfig() {
    peripheral.writeValue(Data([0xBE, 0x00]), for: controlChar, type: .withResponse)
}
```

> ⚠️ **The read and write slot layouts differ by one byte.** A `0xFC` **record
> is 8 bytes** — it carries a trailing reserved byte. The `0xBD` **write payload
> is 7 bytes**, giving 9 on the wire once you prepend the opcode and slot index,
> and has **no** reserved byte. Do not echo a decoded record straight back into
> a write — that produces a 10-byte frame. (The firmware happens to tolerate it:
> it checks `len >= 9` and reads only the first 7 payload bytes. Don't rely
> on it.)

Keep the `precondition`; a malformed write is silently dropped by the firmware,
which is painless to ship and painful to debug.

### 4.3 Decoding the `0xFC` frame

In your existing `0xFF03` demultiplexer, add:

```swift
case 0xFC:
    guard data.count >= 4 else { return }
    let maxSlots = Int(data[2])
    var slots: [StandaloneProgram] = []
    for i in 0..<maxSlots {
        let o = 4 + i * 8
        guard data.count >= o + 8 else { break }
        slots.append(StandaloneProgram(
            mode:         StandaloneMode(rawValue: data[o]) ?? .breathe,
            bpm:          data[o+1] == 0 ? nil : data[o+1],
            inhalePct:    data[o+2] == 0 ? nil : data[o+2],
            strobeDeciHz: (UInt16(data[o+3]) | UInt16(data[o+4]) << 8) == 0
                          ? nil : UInt16(data[o+3]) | UInt16(data[o+4]) << 8,
            dutyPct:      data[o+5] == 0 ? nil : data[o+5],
            brightness:   data[o+6] == 0 ? nil : data[o+6]))
    }
    self.standaloneConfig = StandaloneConfig(count: data[1],
                                             maxSlots: data[2],
                                             activeIndex: data[3],
                                             slots: slots)
```

Bounds-check on `data.count` at every offset, as above — a firmware that grows
`max` past 5 will send a longer frame, and one that predates v4.17.0 sends no
frame at all (§4.5).

### 4.4 Write ordering

When the user saves a multi-program setup, **write every slot first, then the
count.** Writing the count first briefly enables the arm cycle over slots that
have not been written yet.

```swift
for (i, program) in edited.enumerated() where i < Int(config.maxSlots) {
    writeSlot(UInt8(i), program)
}
setProgramCount(enabled ? UInt8(edited.count) : 1)
requestStandaloneConfig()          // confirm from the device, then update UI
```

Each write is `.withResponse`; chain them through
`peripheral(_:didWriteValueFor:error:)` rather than firing all six at once.

### 4.5 Version gating

Parse the version string from the hello log line the firmware sends on subscribe
(same mechanism as `BATTERY_APP_HANDOFF.md` §6). `0xBC`/`0xBD`/`0xBE` exist only
in **v4.17.0 and later**.

- **< 4.17.0** — hide the whole standalone-programs screen. Do not send these
  opcodes; older firmware ignores unknown opcodes silently, so a failed write is
  indistinguishable from a successful one and the UI would lie.
- On older firmware the three-program strobe cycle from §1 is still live. If you
  keep any copy describing it, gate that copy on the same version check.

---

## 5. What to build

Add a **"Standalone mode"** section to settings, framed as: *what the glasses do
on their own, with no phone and no sensor connected.*

### 5.1 Default state (what almost every user will see)

```
Standalone mode
─────────────────────────────────────────
When the glasses are on their own, they
breathe at 6 breaths per minute.

  Breathing pace          6 /min   ›

  Multiple programs        [ Off ]

Hold the glasses closed for 5 seconds to
turn them off.
```

- **Breathing pace** — the existing `0xB1` control. Keep it, keep it prominent;
  for the overwhelming majority of users this remains the only standalone
  setting they will ever touch. (Sending `0xB1` also updates every slot that
  inherits `bpm`, which is the behavior you want.)
- **Multiple programs** — a toggle, **off**, that reveals the editor in §5.2.

Do not auto-enable the toggle. Do not enable it as a side effect of the user
opening the editor, or of them editing a slot. Only an explicit switch flip
followed by an explicit save should ever send `0xBC` with a value ≥ 2.

### 5.2 The editor (revealed by the toggle)

```
Programs on the glasses            [ On ]
─────────────────────────────────────────
Tapping the arm — closing and reopening it
quickly — switches between these. The
glasses flash to show which one you're on.

  1.  Breathe · 6/min                    ›
  2.  Breathe + Strobe · 5/min, 10 Hz     ›
  3.  Strobe · 10 Hz                      ›

  + Add program                (up to 5)

              [ Save to glasses ]
```

Requirements:

1. **1 to `maxSlots` programs**, reorderable or not (your call), each opening a
   detail editor with the fields in §2 for the chosen mode. Show only the fields
   the mode uses: Breathe → pace + inhale + tint; Strobe → rate + duty + tint;
   Breathe + Strobe → all of them; Fixed tint → tint only.
2. **Program 1 must always exist and should default to plain Breathe.** It is
   what every power-on lands on. Do not let the user delete it.
3. **An explicit "Save to glasses" button.** Do not write on every field change:
   this screen's whole point is that the user understands the configuration is
   stored *on the device* and used without the app. Confirm visibly — "Saved to
   glasses — tap the arm to switch between your 3 programs."
4. **Read back on open.** Send `0xBE` when the screen appears and render the
   device's actual configuration. If no `0xFC` arrives within ~2 s, show a
   "couldn't read the glasses" state rather than an empty editor the user might
   save over the top of a real configuration.
5. **Show which program is running.** The `0xFC` frame carries `active`; mark
   that row. Re-query after the user reports tapping the arm.
6. **Turning the toggle off writes `BC 01`.** State plainly that this restores
   "tapping the arm does nothing" — and that the programs they configured are
   kept, not deleted, so flipping it back on restores them.

### 5.3 Copy for the strobe modes

A user enabling strobe on a wearable that sits in front of their eyes deserves a
plain-language warning next to the mode picker, not buried in a legal screen:

> **Strobe** flashes the lenses rapidly. Skip it if you have photosensitive
> epilepsy or are prone to migraines. It is off unless you add it here.

Use your existing house wording for photosensitivity if you have one.

---

## 6. Interaction with the rest of the app

- **Session pace vs standalone pace.** Unchanged trap from the old handoff: any
  `0xB1` you send persists. If a session runs at a different pace, re-assert the
  user's standalone pace at session end — otherwise the glasses keep the session
  value. Slots that inherit `bpm` follow whatever the last write was.
- **Slots that pin a value do not follow `0xB1`.** If your standalone editor
  writes an explicit `bpm` into slot 1, later changes to the "Breathing pace"
  control no longer affect that slot. Either keep slot 1 inheriting (send `0`),
  or make the pace control write slot 1 too. Inheriting is simpler and is what
  §2 recommends.
- **PPG / earclip sessions are unaffected.** With a sensor attached the glasses
  run the PPG program list and the arm cycles *those* — the standalone table is
  not consulted. Nothing to do here, but don't be surprised in testing.
- **Program selection still does not persist across power cycles.** Every
  power-on lands on program 1. Only the table and the count persist. Say so in
  the UI if you list the programs.

---

## 7. Test plan

Replaces `BATTERY_APP_HANDOFF.md` rows C1–C6 and D1–D4.

### 7.1 The new default (no app configuration at all)

| # | Test | PASS |
|---|---|---|
| S1 | Factory-reset glasses (`BF 00`), power-cycle, open the arm | Tinting begins; **no pulse at any point** in the first 15 s |
| S2 | Close and re-open the arm quickly (~1 s) | **Nothing happens.** No pulse, no strobe, no change in the breathing rhythm |
| S3 | Repeat S2 five more times | Still nothing visible each time (the 5-tap earclip-forget gesture may fire on the fifth — 3 fast pulses — which is expected and separate) |
| S4 | Close the arm and hold it for 5 s | Glasses sleep |
| S5 | Open the arm again | Glasses wake and breathe |
| S6 | Set 5 BPM from the app, disconnect, power-cycle, count for 60 s | Breathes at 5/min unaided |

**S2 is the headline test for this release.** If anything visible happens on a
quick close-and-reopen of a default-configured device, the release is wrong.

### 7.2 The programmable cycle

| # | Test | PASS |
|---|---|---|
| S7 | Configure 3 programs (Breathe / Breathe+Strobe / Strobe), save | Confirmation shown; `0xBE` read-back matches what was saved |
| S8 | Disconnect the app. Close + reopen the arm | 2 pulses, then Breathe + Strobe |
| S9 | Repeat | 3 pulses, then Strobe |
| S10 | Repeat | No pulses, back to Breathe |
| S11 | Power-cycle, no app in range | Wakes on program 1 (Breathe); tapping still cycles all 3 |
| S12 | Turn the "Multiple programs" toggle off, save, disconnect | Arm tap does nothing again; sleep-on-5 s-hold unaffected |
| S13 | Turn it back on | The 3 programs are still configured, not lost |
| S14 | Configure 5 programs | All 5 reachable by tapping; the 5th wraps back to the 1st |
| S15 | Configure 3, save, then reduce to 2 while the glasses sit on program 3 | Glasses fall back to program 1; tapping toggles 1↔2 only |
| S16 | `0xBF 00` factory reset | Back to the S1–S3 default: one program, arm tap does nothing |

### 7.3 App-side

| # | Test | PASS |
|---|---|---|
| S17 | Open the settings screen on a fresh device | Toggle reads Off; one Breathe program shown |
| S18 | Open it on firmware < 4.17.0 | Section hidden entirely; no `0xBC`/`0xBD`/`0xBE` written |
| S19 | Open it with the glasses out of range mid-read | "Couldn't read the glasses" state, not an empty editor |
| S20 | Edit a slot without saving, leave the screen, come back | Device configuration shown, not the abandoned edit |
| S21 | Save 3 programs with BLE dropping mid-write | Error surfaced; on reconnect `0xBE` shows the real (possibly partial) state |

---

## 8. Firmware-side reference

All in `v4/Code-Glasses/main/main.c`:

- `STANDALONE PROGRAM SLOTS` block — the model, the inherit-on-zero rule, and
  the rationale for the default.
- `apply_program()` — how a slot resolves into lens behavior.
- `HALL SENSOR — GESTURE STATE MACHINE` block — exactly what each arm gesture
  does and which parts are gated on the program count.
- `CHANGELOG v4.17.0` block — the full before/after.
- Opcode handlers `0xBC` / `0xBD` / `0xBE` in `process_command()`.
- Persistence: `KEY_SA_COUNT`, `KEY_SA_TABLE`, `sa_table_load/save()`.

Delivery: merge to `main` → GitHub Actions builds → Release publishes
`ESP32_Ble.bin` → OTA webapp updates a unit in ~3 minutes.

---

## 9. Self-contained agent prompt (iOS)

> You are adding a **Standalone Programs** settings screen to the Narbis iOS app,
> talking to Narbis Edge glasses over CoreBluetooth.
>
> **Context.** The glasses run a "standalone program" whenever no phone is
> connected. As of firmware v4.17.0 they ship with exactly one — breathing at
> the user's saved pace — and closing/re-opening the left arm does nothing.
> (Before v4.17.0, an arm tap cycled three hard-coded programs including a 10 Hz
> strobe, which users hit by accident. That is the bug this removes.) Holding
> the arm closed for 5 seconds still sleeps the device.
>
> Firmware now supports up to 5 programmable slots plus a count of how many the
> arm cycles. **The cycle must stay off unless the user explicitly turns it on.**
>
> **Protocol.** Service `0x00FF`, control write `0xFF01`, status notify `0xFF03`
> (type-tagged frames, byte 0 = type).
> - `[0xBC][count]` — count 0/1 = cycle off (default), 2–5 = arm cycles that
>   many programs. Persisted.
> - `[0xBD][slot][mode][bpm][inhale][dhz_lo][dhz_hi][duty][brightness]` — 9
>   bytes, slot 0-based, `dhz` u16 LE deci-Hz. mode: 0 Breathe, 1 Breathe+Strobe,
>   2 Strobe, 3 Fixed tint. **Every numeric field: 0 = inherit the value already
>   saved on the glasses.** Persisted.
> - `[0xBE][0x00]` — read back. Replies with frame `0xFC` on `0xFF03`:
>   `[0xFC][count][max][active]` then `max` × 8-byte slot records
>   `[mode][bpm][inhale][dhz_lo][dhz_hi][duty][brightness][reserved]`.
>
> Use write-with-response. **Write all slots before writing the count** —
> writing the count first briefly enables the cycle over unwritten slots.
>
> **Build:** a "Standalone mode" settings section containing the existing
> breathing-pace control (`0xB1`) and a **"Multiple programs" toggle that is off
> by default**. On, it reveals an editor for 1–`max` programs, each with a mode
> picker and only the fields that mode uses. Requirements: an explicit "Save to
> glasses" button (no write-on-change); `0xBE` read-back on screen open with a
> failure state if no `0xFC` arrives in ~2 s; highlight the running program from
> `active`; a photosensitivity warning next to the strobe modes; program 1 not
> deletable; turning the toggle off writes `BC 01` and keeps the configured
> programs rather than deleting them.
>
> **Gate the whole section on firmware ≥ 4.17.0** (parse the version from the
> hello log line the glasses send on subscribe). Older firmware silently ignores
> unknown opcodes, so an ungated screen would show a UI that does nothing.
>
> **Also update existing copy:** anything saying "tap to change program",
> "2 pulses = Program 2", or "3 pulses = Program 3" is now wrong for v4.17.0+.
> The user-visible rule is: *Open the glasses to start. Hold them closed for 5
> seconds to turn them off.*
>
> Test rows S17–S21 in `STANDALONE_PROGRAMS_APP_HANDOFF.md` §7.3 are your
> acceptance criteria.
