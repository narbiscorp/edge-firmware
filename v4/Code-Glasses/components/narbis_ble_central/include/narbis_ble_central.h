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

#ifdef __cplusplus
}
#endif

#endif /* NARBIS_BLE_CENTRAL_H */
