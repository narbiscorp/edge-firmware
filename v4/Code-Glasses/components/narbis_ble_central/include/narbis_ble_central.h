/*
 * narbis_ble_central.h — BLE central role module for the Narbis glasses.
 *
 * Discovers a Narbis earclip, connects as a BLE central, writes its peer
 * role (NARBIS_PEER_ROLE_GLASSES) to the earclip, and subscribes to IBI
 * notifications. Wraps Bluedroid's GATTC API so main.c stays clean.
 *
 * Lifecycle:
 *   narbis_central_init(ibi_cb, batt_cb)    — register callbacks, alloc state
 *   narbis_central_start()                  — begin auto-discovery / reconnect
 *   narbis_central_forget()                 — wipe NVS, force rescan
 *   narbis_central_is_connected()           — diagnostic / status display
 *
 * The module owns its own NVS namespace ("narbis_pair", key "earclip_mac")
 * — the same name the v4.14.39 ESP-NOW path used. After Path B the format
 * is a 6-byte BLE MAC, not an ESP-NOW peer MAC; the namespace is reused so
 * a previously-paired glasses unit overwrites cleanly on first connect.
 */

#ifndef NARBIS_BLE_CENTRAL_H
#define NARBIS_BLE_CENTRAL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_gap_ble_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*narbis_central_ibi_cb_t)(uint16_t ibi_ms,
                                        uint8_t  confidence_x100,
                                        uint8_t  flags);

typedef void (*narbis_central_battery_cb_t)(uint8_t  soc_pct,
                                            uint16_t mv,
                                            uint8_t  charging);

esp_err_t narbis_central_init(narbis_central_ibi_cb_t     ibi_cb,
                              narbis_central_battery_cb_t batt_cb);

esp_err_t narbis_central_start(void);

esp_err_t narbis_central_forget(void);

bool      narbis_central_is_connected(void);

/* Bluedroid registers exactly one GAP callback per stack. main.c keeps
 * its existing gap_event_handler (advertising lifecycle, connection
 * params) and forwards every event to this hook so the central can see
 * scan results / scan-stop. Safe to call before init (no-op). */
void narbis_central_gap_event(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param);

/* Optional log sink. When registered, the central forwards its key
 * scan/connect/subscribe/disconnect events to this function in addition
 * to ESP_LOG. main.c registers its existing ble_log() here so dashboard
 * users can see central activity in the BLE event log (0xF1 frames)
 * without needing a USB serial monitor.
 *
 * The sink is called with a short pre-formatted message (no newline).
 * Pass NULL to unregister. */
typedef void (*narbis_central_log_sink_t)(const char *msg);
void narbis_central_set_log_sink(narbis_central_log_sink_t sink);

/* State callback. Fires when the central reaches READY (connected +
 * IBI subscribed) and again when it disconnects. main.c uses this to
 * switch the on-glasses program (e.g. force PPG program 1 + disable
 * the internal ADC pin while the earclip is providing IBI) and revert
 * on disconnect. Pass NULL to unregister.
 *
 * connected = true:  earclip ready, IBI flowing
 * connected = false: disconnected (any reason) */
typedef void (*narbis_central_state_cb_t)(bool connected);
void narbis_central_set_state_cb(narbis_central_state_cb_t cb);

/* CONFIG relay callbacks (Path B Phase 1).
 *
 * The earclip exposes two config-related characteristics: NARBIS_CHR_CONFIG
 * (notify, current runtime config) and NARBIS_CHR_CONFIG_WRITE (write,
 * accepts a serialized narbis_runtime_config_t). The central subscribes to
 * CONFIG and performs a one-shot read on connect (the earclip only notifies
 * CONFIG on changes, never on subscribe) so dashboard sees the initial state.
 *
 * Payload to the cb is the wire format: NARBIS_CONFIG_WIRE_SIZE (50 B) of
 * struct + CRC16. The cb does not need to validate CRC; main.c forwards the
 * bytes through 0xFF03 type 0xF4 to the dashboard, which deserializes there. */
typedef void (*narbis_central_config_cb_t)(const uint8_t *bytes, uint16_t len);
void narbis_central_set_config_cb(narbis_central_config_cb_t cb);

/* RAW_PPG relay callback. Off by default — opt-in via set_raw_enabled().
 * Payload: u16 sample_rate_hz, u16 n_samples, n × (u32 red, u32 ir).
 * Up to 4 + 29*8 = 236 B per notify. */
typedef void (*narbis_central_raw_cb_t)(const uint8_t *bytes, uint16_t len);
void narbis_central_set_raw_cb(narbis_central_raw_cb_t cb);

/* Diagnostics relay callback. Payload is the firmware diagnostic frame:
 * [seq u16][n u8] then n × (stream_id u8, len u8, payload). The stream
 * IDs are NARBIS_DIAG_STREAM_* — POST_FILTER drives the dashboard's
 * Filtered chart. The earclip only emits when its
 * narbis_runtime_config_t.diagnostics_enabled = 1 AND diagnostics_mask
 * has the relevant bit set. */
typedef void (*narbis_central_diag_cb_t)(const uint8_t *bytes, uint16_t len);
void narbis_central_set_diag_cb(narbis_central_diag_cb_t cb);

/* Toggle whether the central subscribes to the earclip's RAW_PPG char.
 * - If currently connected and at ST_READY:
 *     true  → write CCCD = 0x0001 to enable notify
 *     false → write CCCD = 0x0000 to disable
 * - Otherwise: latches the desired state and applies on next connect.
 * Returns ESP_OK on dispatch (or pure latch), ESP_ERR_INVALID_STATE if
 * the GATTC interface isn't registered yet. */
esp_err_t narbis_central_set_raw_enabled(bool enabled);

/* Forward a config blob to the earclip via GATTC write to CONFIG_WRITE.
 * The dashboard sends the same NARBIS_CONFIG_WIRE_SIZE blob it would
 * write directly; main.c's 0xC3 CTRL handler invokes this with the
 * payload. Returns ESP_ERR_INVALID_STATE if not connected to earclip
 * or no CONFIG_WRITE handle was discovered (older earclip firmware).
 * Async: the GATTC write completion is logged via cb_log; the earclip
 * will then notify CONFIG which the dashboard picks up as 0xF4. */
esp_err_t narbis_central_write_earclip_config(const uint8_t *bytes, size_t len);

/* Re-emit the post-discovery handles diagnostic + current state via
 * cb_log. Used when a fresh dashboard connects after the central has
 * already reached READY — without this, the dashboard would never see
 * the discovery / ready logs that fired into the void at boot. */
void narbis_central_emit_diag(void);

#ifdef __cplusplus
}
#endif

#endif /* NARBIS_BLE_CENTRAL_H */
