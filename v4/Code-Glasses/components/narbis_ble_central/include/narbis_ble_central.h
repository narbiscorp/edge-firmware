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

#ifdef __cplusplus
}
#endif

#endif /* NARBIS_BLE_CENTRAL_H */
