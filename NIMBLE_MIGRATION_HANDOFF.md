# Glasses BLE: Bluedroid → NimBLE migration handoff

**Date:** 2026-05-14
**Status:** Diagnosis complete, root cause confirmed, migration ready to begin. Implementation plan at `~/.claude/plans/nimble-migration.md` on the originating developer's machine; reproduced in summary below.

> Read this BEFORE looking at `PAIRING_HANDOFF.md` or `PAIRING_HANDOFF_v2.md`. Those are now historical — they document the cascade of workarounds (PRs #5–#41) that all failed for the same underlying reason. This doc supersedes them.

---

## TL;DR

After ~25 PRs trying to make ESP-IDF's **Bluedroid** stack work for the glasses' dual-role BLE (central to earclip + peripheral to dashboard), the earclip-side log on 2026-05-14 unambiguously confirmed that Bluedroid is silently dropping GATT events between the radio and our app callbacks under central+peripheral contention. The pathology is in the deprecated stack itself, not our code. The path forward is **migrate the glasses to NimBLE** — the supported, modern stack that the earclip already uses successfully.

**Earclip and dashboard firmware do not need changes.** Protocol, UUIDs, characteristics, opcodes, peer_role byte all stay identical. The migration swaps the host implementation; everything above the host layer is preserved.

---

## What's confirmed

### Bluedroid is dropping events under dual-role contention

Most-recent reproduction (2026-05-14, glasses on PR #20 + PR #41 firmware):

**Earclip log (NimBLE, dual-role peripheral):**
```
I (2389) transport_ble: connected slot=0 handle=0 itvl=12 latency=0 timeout=600
I (2389) transport_ble: profile BATCHED applied to handle=0
I (2419) transport_ble: MTU slot=0 handle=0 mtu=247
I (2809) transport_ble: conn_update handle=0 status=0 itvl=80 latency=4 timeout=2000
... (no further GATT activity — no peer_role write, no CCCD writes seen) ...
I (17909) transport_ble: disconnected slot=0 handle=0 reason=0x213
```

`reason=0x213` = HCI 0x13 "Remote User Terminated Connection" — the glasses' 15 s watchdog tore the link down.

**Glasses log (Bluedroid, dual-role central+peripheral):**
```
chain c=1 d=0 mtu=0 srch=0 wdc=1 wda=1 wdf=0 wd_ok=1
auto-push hr_source failed: GATT operation already in progress.
... 15 s later ...
central: connect watchdog 15000ms in state=DISCOVER — force recovery
```

`mtu=0` means our app never received `ESP_GATTC_CFG_MTU_EVT`. But the earclip's NimBLE log proves the MTU response was transmitted on the wire and ACK'd. **Bluedroid received the MTU completion on the radio and did not deliver the event to our `gattc_cb`.**

The `"GATT operation already in progress"` line is the smoking gun — Bluedroid serializes all GATT ops across central + peripheral roles on a single host task, and when the dashboard is pushing writes (`hr_source` heartbeat) while the glasses is mid-`search_service`, events get dropped.

### What we tried and why each failed

| PR | Glasses-side change | Result |
|---|---|---|
| #5 | addr-type capture from scan | Real bug fix (random-address peers). Still relevant in NimBLE. |
| #6 | 15 s connect watchdog | Recovers from wedge, doesn't prevent it. Useful safety net regardless of stack. |
| #8 | chain counters in diag | Diagnostic, no behavior change. |
| #9 | emit_diag pre-discovery self-heal | Workaround; reverted in #19. |
| #10 | conn_id=0 self-heal gate | Real bug fix. |
| #11 | arm watchdog inside CONNECT_EVT | Defense in depth for the auto-reconnect path. |
| #12 | disable SMP bond NVS + boot bond clear | Killed one class of auto-reconnect, didn't fix the event-drop. |
| #13 | cache_clean + cache_refresh on CONNECT_EVT | Reverted in #19 — was racing itself. |
| #14 | track Bluedroid link state separately from app state | UI honesty; "linked" badge now distinct from "data flowing". Worth preserving. |
| #15 | pre-emptive gap_disconnect + clear_whitelist at boot | Reverted in #19; partial logic moved to watchdog fire in #20. |
| #16 | fire search_service in CONNECT_EVT | Reverted in #19, restored in #20 — events still drop. |
| #17 | cache-driven discovery bypass | Reverted in #19; Bluedroid's internal cache also stayed empty. |
| #18 | 1 s cache retry timer | Reverted in #19. |
| #19 | simplify cascade + actually apply SMP off in sdkconfig | Real win, eliminated noise, exposed the underlying issue. |
| #20 | search_service in CONNECT_EVT + clear_whitelist on watchdog | Partial — MTU events started arriving sometimes (34/41), SEARCH_RES_EVT never. |
| earclip #41 | raise BATCHED supervision_timeout 4 s → 20 s | Real fix for one issue (fast c/d cycling at ~3 s), exposed the host-event-drop as the *separate* primary problem. |

Net effect: dozens of careful patches, all addressing real downstream symptoms, none addressing the host-stack event-drop because we can't from inside our app.

### The earclip and dashboard are fine

- Earclip's NimBLE peripheral handles every connection correctly: MTU exchange, conn-update profile application, multi-slot management. The earclip side of the protocol is unblocked and reliable.
- Dashboard's Web Bluetooth client successfully reads/writes the glasses' GATT service when the glasses' peripheral side is responsive. The dashboard-side protocol is unblocked.

The wedge is entirely between the radio and the glasses' app callback, inside Bluedroid.

---

## The plan: migrate glasses to NimBLE

### Why NimBLE is the right answer

- Espressif explicitly recommends NimBLE for new development. Bluedroid is deprecated/feature-frozen.
- ESP-IDF v5+ ships both stacks; flipping is a sdkconfig change at the build level.
- The earclip already uses NimBLE in the exact dual-role pattern we need (peripheral with multi-conn support) without issue.
- NimBLE's event model is per-call callbacks rather than a single global event filter. Eliminates the host-layer drop pathology.
- No bonding required (we don't encrypt) — the entire SMP layer that ate PRs #12–#19 can be `=n` and not compiled in.

### Scope

**In scope:**
- [v4/Code-Glasses/components/narbis_ble_central/narbis_ble_central.c](v4/Code-Glasses/components/narbis_ble_central/narbis_ble_central.c) — full rewrite using NimBLE central APIs.
- [v4/Code-Glasses/main/main.c](v4/Code-Glasses/main/main.c) — BLE-related sections only. Rewrite the GATTS service definition, access callbacks, advertising loop, GAP event handler using NimBLE APIs.
- [v4/Code-Glasses/sdkconfig.defaults](v4/Code-Glasses/sdkconfig.defaults) — flip from Bluedroid to NimBLE.
- Delete [v4/Code-Glasses/sdkconfig](v4/Code-Glasses/sdkconfig). Add to `.gitignore`. Force regeneration from defaults every CI build. (Resolves the PR #12 → #19 override saga permanently.)

**Strictly out of scope. Do NOT touch:**
- Sleep button, hall sensor, battery monitor, LED status, OTA partition layout. PR #7 incident memo: never bundle these into a BLE PR.
- Earclip firmware (`C:\CODE\EDGE EAR CLIP\REPO\edge-earclip` — independent repo). Earclip already runs NimBLE; protocol unchanged.
- Dashboard (`C:\CODE\EDGE EAR CLIP\REPO\edge-earclip\dashboard` — Web Bluetooth client). No firmware-side change is visible to the dashboard side.
- Coherence math, beat validator, ppg pipeline, breath programs, lens LED feedback. All pure CPU + GPIO work, stack-independent.
- Polar H10 path. H10 connects to dashboard, not glasses. R-R injection arrives at glasses as 0xCA writes via the same CTRL char that today's Bluedroid handles. NimBLE serves the same char; opcode handling stays bit-identical.

### What stays bit-for-bit

| Layer | Preserved |
|---|---|
| BLE protocol | All UUIDs in `narbis_protocol.h`: NARBIS_SVC, IBI, BATTERY, CONFIG, CONFIG_WRITE, PEER_ROLE, RAW_PPG, DIAGNOSTICS. Plus the dashboard service 0xFFFF and chars 0xFF01–0xFF04. |
| Dashboard opcodes | 0xC1 forget, 0xC3 config forward, 0xC4 raw toggle, 0xC5 refresh, 0xCA H10 R-R inject, 0xB7 ppg prog, etc. |
| Frame types | 0xF1 log, 0xF2 IBI, 0xF4 config, 0xF5 raw, 0xF6 link, 0xF7 diag, 0xF8 batt. |
| Public API of `narbis_ble_central` | Every function signature in `narbis_ble_central.h`. The component's interface to main.c stays identical. |
| NVS | `narbis_pair`/`earclip_mac` namespace+key. Existing field NVS works after OTA. |
| OTA pipeline | Partition table, CI workflow, release naming, dashboard OTA webapp endpoint. |

### Data flows confirmed working under NimBLE

This was an explicit ask from the user. Verified all flows are NimBLE-compatible:

1. **Earclip → glasses (central role):** NimBLE central with `ble_gattc_disc_all_chrs` + `ble_gattc_write_flat` to CCCDs + `BLE_GAP_EVENT_NOTIFY_RX` for inbound notifies. Works exactly the way the earclip already handles its peripheral side, just inverted.
2. **Glasses → dashboard (peripheral role):** NimBLE peripheral with GATT service table, access callbacks, `ble_gatts_notify_custom` for status frames. Pattern matches earclip's `ble_service_narbis.c`.
3. **Dashboard → glasses CTRL writes:** NimBLE access callback for the CTRL characteristic; opcode dispatch unchanged from today.
4. **Polar H10 → dashboard → glasses R-R injection:** H10 ↔ dashboard happens entirely in browser via Web Bluetooth. Dashboard writes R-R to glasses via 0xCA. Glasses-side BLE stack identity is irrelevant.
5. **Onboard coherence + lens feedback:** Pure CPU work in a FreeRTOS task driven by either earclip notifies or H10-injected R-R. Stack-independent.
6. **Simultaneous central + peripheral:** NimBLE explicitly supports dual-role on the same chip — set `CONFIG_BT_NIMBLE_ROLE_CENTRAL=y` and `CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y`. The earclip already exercises a similar pattern (1 peripheral + N centrals on the same firmware).
7. **OTA:** GATT writes with response on the OTA char. NimBLE handles this identically to Bluedroid.

---

## Implementation phases

Detailed phase-by-phase plan with concrete API mappings and verification steps lives in the developer's plan file. High-level summary:

### Phase 1 — sdkconfig (small, mechanical)
- Edit `sdkconfig.defaults`: set `CONFIG_BT_NIMBLE_ENABLED=y`, `CONFIG_BT_BLUEDROID_ENABLED=n`, NimBLE role flags, `MAX_CONNECTIONS=3`, SMP disabled, MTU=247.
- Delete `sdkconfig`. Add to `.gitignore`.
- Replace `esp_bluedroid_init/enable` with `nimble_port_init` + `nimble_port_freertos_init(bt_host_task)` in main.c. Pattern from earclip's `transport_ble.c:transport_ble_init`.
- Remove the SMP bond clear loop at main.c:6240–6256 — irrelevant when SMP is not compiled in.

### Phase 2 — Peripheral (GATTS) rewrite in main.c
Replace all `esp_ble_gatts_*` and `esp_ble_gap_*` (peripheral side) with NimBLE equivalents:
- Build a `ble_gatt_svc_def` table for the dashboard service.
- Implement access callbacks for CTRL (R/W), OTA (W), STATUS (R/Notify), PPG (R/Notify).
- Replace `esp_ble_gatts_send_indicate` everywhere with `ble_gatts_notify_custom`.
- Advertising loop mirrors earclip's `start_advertising_if_room()`.

### Phase 3 — Central (GATTC) rewrite in narbis_ble_central.c
Replace all `esp_ble_gattc_*` with NimBLE:
- Preserve state machine, public API, chain counters, watchdog, per-step log lines.
- Drop `gattc_if` field, `gattc_cb`, `narbis_central_gap_event` (NimBLE merges GAP+GATTC into one event handler signature; main.c no longer routes GAP events into the central).
- Drop `cache_clean` / `clear_whitelist` / cache-bypass logic — NimBLE has no such pitfalls.
- Scan via `ble_gap_disc` with active scan, filter NARBIS_SVC_UUID, pick best RSSI.
- Connect via `ble_gap_connect`.
- Post-CONNECT chain: `ble_gattc_exchange_mtu` → `ble_gattc_disc_svc_by_uuid` → `ble_gattc_disc_all_chrs` → `ble_gattc_disc_all_dscs` for CCCDs → `ble_gattc_write_flat` for peer_role → walk CCCD subscribe chain.
- Notify reception: `BLE_GAP_EVENT_NOTIFY_RX` in the same event handler. NimBLE does NOT require a separate `register_for_notify` call — writing 0x0001 to a CCCD is sufficient.
- Watchdog: keep the 15 s `esp_timer`. On fire, `ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM)`.

### Phase 4 — Verification
1. CI build passes.
2. OTA + power-cycle test: dashboard log shows `chain 1/9 CONNECT → 2/9 MTU → 3/9 SVC_DISC → 4/9 CHR_DISC → 5/9 WRITE_ROLE_ACK → 6/9 SUB_IBI_ACK → 7/9 SUB_CFG_ACK → 8/9 SUB_BATT_ACK → 9/9 READY` within a few seconds. Charts populate. `rx ibi=≥1` ticks.
3. Functional tests: dashboard link, IBI relay, config relay, battery relay, raw PPG relay, coherence onboard, Polar H10 path, OTA, 0xC1 re-pair, 5× power-cycle resilience.

### Phase 5 — Cleanup
- Delete `PAIRING_HANDOFF.md`, `PAIRING_HANDOFF_v2.md` from repo root.
- Remove dead Bluedroid includes.
- Remove `narbis_central_gap_event` from the header.

---

## User context (preserved for next agent)

- **Solo dev**, building a consumer wearable (Narbis Edge). OTA-only workflow. Each iteration is ~3 minutes of CI + release + OTA.
- **Strict subsystem scoping required.** PR #7 incident: a sleep-button change applied to the wrong device caused a revert. Never bundle BLE work with sleep, battery, button, LED, or hall-sensor changes.
- **Never auto-merge PRs.** User squash-merges manually after review.
- **Two separate repos:** glasses at `C:\NARBIS APP\Oct_4_25\Clone\edge-firmware` (this repo), earclip at `C:\CODE\EDGE EAR CLIP\REPO\edge-earclip`. Past attempts to conflate them ended badly.
- **Terse responses preferred.** Don't bury answers in context.
- **Skeptical of handoff narratives — explicitly asks to challenge them.** The user was right to push back on the Bluedroid-cache-bypass cascade. Reading actual code beats trusting prior summaries.

---

## Reference files (read-only inputs)

### In this repo
- [v4/Code-Glasses/components/narbis_protocol/narbis_protocol.h](v4/Code-Glasses/components/narbis_protocol/narbis_protocol.h) — UUIDs, frame layouts, payload structs. **Do not modify.**
- [v4/Code-Glasses/components/narbis_ble_central/include/narbis_ble_central.h](v4/Code-Glasses/components/narbis_ble_central/include/narbis_ble_central.h) — public API contract. Preserve every function signature.
- [v4/Code-Glasses/main/main.c](v4/Code-Glasses/main/main.c) — current Bluedroid implementation of the GATTS side. The non-BLE parts (sleep, battery, ppg pipeline, coherence, lens feedback, OTA) stay exactly as-is.

### In the earclip repo (read-only reference)
- [`firmware/main/transport_ble.c`](file:///C:/CODE/EDGE%20EAR%20CLIP/REPO/edge-earclip/firmware/main/transport_ble.c) — NimBLE dual-role init, advertising, slot management, GAP event handler pattern. The glasses' peripheral side should mirror this almost line-for-line.
- [`firmware/main/ble_service_narbis.c`](file:///C:/CODE/EDGE%20EAR%20CLIP/REPO/edge-earclip/firmware/main/ble_service_narbis.c) — NimBLE GATT server pattern with `ble_gatt_svc_def` table, access callbacks, `val_handle` tracking. The glasses' dashboard service should follow the same pattern.

### Old handoffs (now obsolete, retain for archaeology only)
- [PAIRING_HANDOFF.md](PAIRING_HANDOFF.md) — original problem framing, addr-type fix, watchdog rationale. Useful for context but the recommendations are superseded.
- [PAIRING_HANDOFF_v2.md](PAIRING_HANDOFF_v2.md) — Bluedroid event-drop narrative + Path A/B/C decision tree. The "Path A: hardcoded handles" idea is no longer the right move — switching to NimBLE removes the need entirely.

---

## What NOT to do (already proven not to work or out of scope)

- Don't add more Bluedroid workarounds. We've established the host stack is the problem.
- Don't try hardcoded handles. Even if it bypassed discovery, `register_for_notify`'s host-layer filter would still drop notifies under the same contention. Doesn't fix root cause.
- Don't migrate the earclip — it's already on NimBLE and works fine. Touching it would invalidate the comparison baseline.
- Don't touch the dashboard. Web Bluetooth is stack-agnostic.
- Don't change UUIDs / opcodes / frame types. Existing field devices' protocol expectations must survive the migration.
- Don't restart the BT stack mid-session in production. It'd drop the dashboard's connection.

---

## Estimate

1–2 focused days for one developer. Most of the work is the central rewrite (state machine port to NimBLE's event shape). Peripheral is shorter — earclip's `ble_service_narbis.c` is a near-direct template.

After landing the migration: expect 1–2 small follow-up PRs to tune NimBLE-specific defaults (adv interval, conn-param negotiation timing). These are normal stack-migration polish, not architectural changes.

---

## Decision summary

The next agent's job is to execute the migration described above. Do not litigate the diagnosis — it's been thoroughly validated (earclip log + chain counters across ~25 PRs of iteration). The Bluedroid stack drops GATT events under dual-role contention on this hardware, full stop. NimBLE removes that failure surface because it doesn't share Bluedroid's host-layer architecture, and the earclip already proves NimBLE works for the same dual-role pattern at the same product scale.

If the migration is in flight and the next agent finds NimBLE has its own unrelated issues, address them surgically with the same discipline that PRs #19 and #41 used: one targeted fix per identified root cause, evidence-driven, scope-locked. Do not cascade.

Good luck.
