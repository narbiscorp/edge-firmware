# Glasses Battery Monitoring — App-Side Handoff (fw v4.16.0)

Audience: the agent implementing battery display/logic in the companion apps
(`C:\CODE\Edge-Muse-Android`, and the iOS app via the edge-SDK) and whoever
runs the acceptance tests. Firmware side is DONE (this repo, v4.16.0); the
tests below gate whether that firmware may be **delivered** (OTA'd to partner
units — BrainMaster's bridge integration is the driving request).

## 1. What the firmware now does

The glasses measure their **own** battery (previously only the earclip's
battery was relayed, as 0xF8). Hardware truth, discovered during this work:

- Production PCB (`STS-USA50925-ESP32-ZZ`, bare ESP32-U4WDH) rev **1.3**
  added a VBAT divider: R17/R18 = 1 MΩ/1 MΩ + C23 100 nF (BOM V1.3,
  2026-05-21). The v1.0 schematic PDF in Drive **predates** it, so the ADC
  pin is not documented anywhere we control (the change was agreed on WeChat).
- Firmware therefore **auto-locates** the divider at boot: it probes all 8
  ADC1 inputs and accepts the one that is stable and inside the band a 1M/1M
  divider implies (VBAT 2.7–4.6 V). GPIO34 is tried first.
- Old dev boards (WROOM-32 modules, incl. the PulseSensor-bodge units) are
  detected by efuse package ≠ U4WDH and report **unsupported** — never
  garbage numbers.
- The TP4057 charger's CHRG/STDBY pins do **not** route to the MCU, so
  `charging` is a best-effort slope heuristic (see §2 semantics).

## 2. Wire protocol (this is the spec the app implements against)

### 2.1 Status frame 0xFB on characteristic 0xFF03 (notify)

Same transport as every other status frame. 5 bytes total:

| byte | field    | meaning |
|------|----------|---------|
| 0    | `0xFB`   | frame type: glasses battery |
| 1–2  | mv u16 LE| smoothed VBAT in millivolts (0 = unknown) |
| 3    | soc u8   | state of charge 0–100; **0xFF = unknown/unsupported** |
| 4    | charging | 0 = discharging, 1 = charging (heuristic), **0xFF = unknown** |

Example: `FB 6E 0F 48 00` → 3950 mV, 72 %, not charging.

Cadence: immediately when the client subscribes to 0xFF03, then every 30 s,
plus on-demand via `0xC7 0x00`. The very first on-subscribe frame after a
cold boot may still be `unknown` (probe takes a few seconds); a real frame
follows within ≤35 s.

Layout is byte-identical to the 0xF8 earclip relay frame — reuse the parser.
0xF8 = earclip battery, 0xFB = glasses battery. Do not conflate them in UI.

### 2.2 Standard Battery Service (BAS)

- Service 0x180F, characteristic 0x2A19 (Battery Level), read + notify.
- Value = soc 0–100. While unknown it reads **0** (BAS has no "unknown"
  encoding) — clients that care about the distinction must use 0xFB.
- Added AFTER the 0xFF00-family service, so 0xFF01–0xFF04 handles are
  unchanged. Web Bluetooth users must add `battery_service` to
  `optionalServices` to see it.
- This is the zero-effort path for the BrainMaster bridge and nRF Connect.

### 2.3 Control opcodes (write to 0xFF01)

- `C7 00` — emit a 0xFB frame now + one human-readable `0xF1` log line
  (`batt: 3950 mV soc=72% chg=0 (gpio34)`).
- `C7 01` — re-run the divider probe and dump all 8 candidate readings to
  the firmware log (two lines: `probe g34=… g35=… g32=… g33=…` and
  `probe g36=… g39=… g37=… g38=…`, pin-side mV). Diagnostic only.

### 2.4 Semantics the app must honor

- `soc=0xFF` → show "—" / hide the battery UI element. Do NOT render 255 %.
- `charging` is best-effort (voltage slope, ~90 s latency). Render it as a
  charging glyph, never as a charge-completion promise.
- `mv` is the trustworthy raw truth; log it in session files for later
  curve tuning.
- Low battery: firmware applies **no** behavior at low battery (deliberate —
  subsystem scoping). If the app wants a warning, threshold on soc (suggest
  ≤15 % warn, ≤5 % urgent) or mv ≤ 3550.

## 3. App-side implementation notes

- Android (`Edge-Muse-Android`): the 0xFF03 listener already dispatches on
  frame type — add `0xFB` next to the existing `0xF8` case and surface as
  `glassesBattery` alongside the existing earclip battery state. The battery
  strip started in the "HRV coherence engine handoff" session should bind to
  THIS (glasses) value for the glasses card.
- iOS/SDK: `edge-SDK` protocol doc currently says glasses battery is NOT
  readable — that section is now stale; update it with §2 of this doc
  (v2.0.0 doc, `docs/bluetooth-protocol.md`).
- Firmware identifies itself on subscribe: `Narbis fw v4.16.0-battery`.
  Gate any battery UI on fw ≥ 4.16.0 (older firmware never sends 0xFB;
  UI must degrade to "—", not spin).

## 4. Acceptance tests — run BEFORE the firmware is delivered

Delivery = merging is not enough: the release binary must pass these on a
**production** (U4WDH) unit before it is OTA'd to partner/end-user devices.
Tools: dashboard or Manual Controller (`narbiscorp.github.io/Edge-Manual-Controller`)
for opcodes + log, nRF Connect for BAS. Each test states PASS and FAIL.

| # | Test | Procedure | PASS | FAIL → action |
|---|------|-----------|------|----------------|
| T1 | Divider located | Connect, subscribe 0xFF03, read hello + `batt:` log line | Line reports a GPIO and 3000–4300 mV | `unsupported` or absurd mV → send `C7 01`, save both `probe` lines, stop delivery (divider likely on ADC2 or absent → firmware rev needed) |
| T2 | Frame cadence | Watch 0xFB frames for 2 min | First frame ≤5 s after subscribe; then every 30 s ±2 s | Missing/irregular → check `batt_task` started (serial boot log) |
| T3 | BAS | nRF Connect → Battery Service | 0x2A19 read matches 0xFB soc ±1; notify ticks | Service absent → GATT table regression |
| T4 | Plausibility | Fully-charged unit | ≥4000 mV and ≥80 % | Out of band → curve or divider-ratio wrong; capture mv |
| T5 | Discharge sanity | 30-min lens session, log 0xFB | soc monotonic non-increasing (allow ±1 jitter), no jump >5, mv falls tens of mV | Jumps/rises while unplugged → smoothing bug |
| T6 | Charging heuristic | Plug USB 5 min, then unplug 5 min | `charging`→1 within ~3 min of plug; →0 within ~3 min of unplug | Never flips → accept+document OR drop the field to 0xFF before delivery; do not ship a field that lies |
| T7 | Old-board degrade (if a WROOM dev unit is on hand) | Same subscribe flow | `batt: unsupported`, soc=0xFF, BAS reads 0 | Numbers appear → package gate broken |
| T8 | Regression | Standard post-OTA checklist | Earclip relay chain reaches READY 9/9, IBI ticks, lens opcodes (A0/A1/A2/A5/B0) respond, OTA to next build still works | Any regression blocks delivery |
| T9 | Poll opcode | Send `C7 00` | Immediate 0xFB + `batt:` line | — |
| T10 | Coexistence | Compare 0xF3 health `jitter_max_us` before/after 4.16.0 during a strobe session | No sustained increase | Increase → ADC lock contention; lower batt sample rate |

**Ship gate:** T1–T6, T8, T9 all PASS on a production unit (T7 optional,
T10 spot-check). Any FAIL: firmware stays unreleased to partners; the OTA
release stays available for internal units only. Record results (date,
unit, fw string, mv at start/end) in this file or the PR thread.

## 5. Delivery pipeline reminder

Merge to `main` → GitHub Actions builds → Release with `ESP32_Ble.bin` →
OTA via the webapp (~3 min/unit). The FCC build (`FCC_TEST_BUILD=1`,
separate repo/CI) also compiles this feature; 0xC6 remains FCC-state there
and does not collide with 0xC7.

## 6. Known limitations (document, don't rediscover)

- Charging detection is inferential; while trickling near full it may hold
  the last state (plateau rule).
- SoC is a voltage-curve estimate under light load — during heavy lens
  strobe the terminal voltage sags a few tens of mV; the 4-sample EMA
  absorbs most of it. No coulomb counter exists on this hardware.
- If probe ever mis-locks on a production unit (theoretically possible on a
  floating pin), `C7 01` shows every channel; the fix is a firmware rev
  pinning the channel — flag it to the firmware side immediately.
- Yellow-lens units run the separate `edge-firmware-yellow` repo — this
  feature ports there in a follow-up; until then yellow units send no 0xFB.
