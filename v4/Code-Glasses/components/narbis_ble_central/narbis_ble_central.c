/*
 * narbis_ble_central.c — Bluedroid GATTC client that connects to a Narbis
 * earclip, writes the peer-role byte (GLASSES = 0x02), and subscribes to
 * IBI (and optionally BATTERY) notifications.
 *
 * Discovery model (per Path B plan §3b):
 *   - Boot: read NVS "narbis_pair"/"earclip_mac".
 *       Present → directed scan 5 s + connect.
 *       Absent  → general scan 30 s, filter by NARBIS_SVC_UUID, pick
 *                 highest-RSSI hit, persist its MAC, connect.
 *   - On disconnect: directed scan 5 s; if not found, sleep 30 s, retry.
 *     Reconnect attempts capped at one per 30 s.
 *
 * After connect:
 *   1. MTU exchange.
 *   2. Discover NARBIS_SVC_UUID, cache char handles.
 *   3. Write 1 byte 0x02 (NARBIS_PEER_ROLE_GLASSES) to PEER_ROLE.
 *   4. Subscribe to IBI (CCCD = 0x0001).
 *   5. Optionally subscribe to BATTERY.
 *   RAW_PPG is intentionally never subscribed — wastes air time + power
 *   for a stream the glasses do not use.
 *
 * No bonding / encryption — same threat model as the rest of v1.
 */

#include "narbis_ble_central.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "narbis_protocol.h"

static const char *TAG = "narbis_central";

#define NARBIS_NVS_NS              "narbis_pair"
#define NARBIS_NVS_KEY_EARCLIP_MAC "earclip_mac"

#define NARBIS_GATTC_APP_ID        0x55  /* arbitrary; distinct from gatts app */

/* Directed scan window for the persisted earclip MAC. 30 s scan + 5 s
 * backoff keeps the radio listening ~86 % of the time, so a wake/re-
 * entry of the earclip is caught within ~5 s. Earlier 5 s/30 s ratio
 * pushed worst-case reconnect to 35 s. */
#define SCAN_DIRECTED_S            30
#define SCAN_GENERAL_S             30
#define RECONNECT_BACKOFF_MS       5000

/* Watchdog for the connect+discover+subscribe chain. Armed right after
 * each esp_ble_gattc_open() call. If state hasn't progressed to
 * ST_READY by the time this fires, we assume Bluedroid wedged (silent
 * gattc_open with no follow-up CONNECT_EVT/OPEN_EVT, or service
 * discovery stuck after CONNECT_EVT) and force-recover. 15 s is well
 * above the normal worst-case observed in logs (~13 s end-to-end on a
 * slow earclip + busy controller) but tight enough that the user
 * doesn't sit through 30 s of staring at a stuck "DISCOVER" state. */
#define CONNECT_WATCHDOG_MS        15000

/* CCCD descriptor UUID (BLE-spec well-known). */
#define BLE_UUID_CCCD              0x2902

typedef enum {
    ST_IDLE = 0,
    ST_SCANNING_DIRECTED,
    ST_SCANNING_GENERAL,
    ST_CONNECTING,
    ST_DISCOVERING,
    ST_WRITING_ROLE,
    ST_SUBSCRIBING_IBI,
    ST_SUBSCRIBING_CONFIG,        /* Path B Phase 1 */
    ST_READING_CONFIG_INITIAL,    /* one-shot read after CONFIG subscribe */
    ST_SUBSCRIBING_BATT,
    ST_SUBSCRIBING_RAW,           /* Path B Phase 2 (only if raw_enabled) */
    ST_READY,
    ST_BACKOFF,
} central_state_t;

typedef struct {
    uint8_t              bda[6];
    esp_ble_addr_type_t  addr_type;  /* captured from scan; needed for gattc_open */
    int                  rssi;
    bool                 valid;
} scan_best_t;

static struct {
    central_state_t state;

    narbis_central_ibi_cb_t     ibi_cb;
    narbis_central_battery_cb_t batt_cb;
    narbis_central_log_sink_t   log_sink;
    narbis_central_state_cb_t   state_cb;
    narbis_central_config_cb_t  config_cb;       /* Path B Phase 1 */
    narbis_central_raw_cb_t     raw_cb;          /* Path B Phase 2 */
    narbis_central_diag_cb_t    diag_cb;         /* diagnostics relay */
    bool                        raw_enabled;     /* user toggle, latched */
    bool                        last_state_emitted;  /* dedup state edges */
    /* Bluedroid link state, separate from our app state machine's
     * S.state. Set on ESP_GATTC_CONNECT_EVT, cleared on
     * ESP_GATTC_DISCONNECT_EVT and on any force-recovery path
     * (connect_watchdog_cb, emit_diag pre-discovery self-heal,
     * narbis_central_stop/forget). The dashboard's 0xF6 heartbeat
     * uses this rather than S.state==ST_READY, so the relay-link
     * badge reflects the actual BLE link the way the earclip's LED
     * does — even when our app state machine is wedged mid-
     * discovery and never reached READY. The chain counters
     * (mtu, srch, hdl_ibi=X/Y) in the diag still tell the user
     * whether our discovery chain completed end-to-end. */
    bool                        peer_connected;

    /* Bluedroid handles. */
    esp_gatt_if_t gattc_if;
    uint16_t      conn_id;

    /* Cached service + char handles after discovery. */
    uint16_t svc_start_handle;
    uint16_t svc_end_handle;
    uint16_t hdl_ibi;
    uint16_t hdl_ibi_cccd;
    uint16_t hdl_battery;
    uint16_t hdl_battery_cccd;
    uint16_t hdl_peer_role;
    uint16_t hdl_config;            /* Path B Phase 1: notify */
    uint16_t hdl_config_cccd;
    uint16_t hdl_config_write;      /* write-only, no CCCD */
    uint16_t hdl_raw;               /* Path B Phase 2: notify */
    uint16_t hdl_raw_cccd;
    uint16_t hdl_diag;              /* Diagnostics: notify */
    uint16_t hdl_diag_cccd;

    /* Pairing target. */
    uint8_t              earclip_mac[6];
    esp_ble_addr_type_t  earclip_addr_type;  /* PUBLIC by default; populated from scan match
                                              * — without this, gattc_open against a peer with
                                              * a random address (NimBLE default on ESP32-C6 when
                                              * no public BD_ADDR is in OTP) silently never fires
                                              * OPEN_EVT and the central looks "stuck after persist". */
    bool                 earclip_known;

    /* General-scan winner-tracking. */
    scan_best_t best;

    /* Reconnect bookkeeping. */
    uint32_t scan_attempts;
    int64_t  last_seen_us;

    /* esp_timer for the 30 s reconnect backoff. */
    esp_timer_handle_t backoff_timer;

    /* esp_timer watchdog for the full connect+discover chain. Armed
     * after every esp_ble_gattc_open() call. If the chain hasn't
     * reached ST_READY within CONNECT_WATCHDOG_MS, fires force-recovery
     * (gap_disconnect + gattc_close + schedule_reconnect_backoff). This
     * is the safety net for Bluedroid's well-known silent wedge where
     * gattc_open() returns ESP_OK but neither CONNECT_EVT nor OPEN_EVT
     * ever fires — leaving the central stuck in ST_CONNECTING forever,
     * or stuck in ST_DISCOVERING if CONNECT_EVT fires but the MTU /
     * service-discovery / write-role / subscribe sub-chain stalls. */
    esp_timer_handle_t connect_watchdog;

    /* Scan diagnostics — counted during each scan window, logged at
     * SCAN_INQ_CMPL_EVT. Tells us whether the central is seeing adverts
     * at all, and whether any match the NARBIS service UUID. */
    uint16_t scan_advs_seen;
    uint16_t scan_advs_matched;

    /* Per-char notify counters — printed in diag so we can see which
     * subscriptions are actually receiving data from the earclip. If
     * state=READY but all counters stay 0, the earclip isn't sending
     * (no finger on sensor) OR Bluedroid isn't dispatching to us. */
    uint32_t notify_ibi_count;
    uint32_t notify_batt_count;
    uint32_t notify_config_count;
    uint32_t notify_raw_count;
    uint32_t notify_diag_count;
    uint32_t notify_other_count;

    /* Chain-event counters — printed in diag so we can see whether the
     * connect/discover/subscribe chain progressed at all during boot, even
     * if the dashboard wasn't subscribed to the log sink when those
     * one-shot events happened. Specifically: if state=DISCOVER persists
     * with hdl_ibi=0, look at wd_armed/wd_fires to tell apart "watchdog
     * never armed" (gattc_open failed) from "watchdog armed but didn't
     * fire yet" from "watchdog fired and forced recovery". Pair `connects`
     * vs `searches` to localize whether the stall is at MTU exchange or
     * search_service. */
    uint32_t connects;       /* ESP_GATTC_CONNECT_EVT count */
    uint32_t disconnects;    /* ESP_GATTC_DISCONNECT_EVT count */
    uint32_t mtus;           /* ESP_GATTC_CFG_MTU_EVT count */
    uint32_t searches;       /* ESP_GATTC_SEARCH_RES_EVT count */
    uint32_t wd_calls;       /* arm_connect_watchdog() function entries (before NULL check) */
    uint32_t wd_armed;       /* arm_connect_watchdog() reached esp_timer_start_once */
    uint32_t wd_fires;       /* connect_watchdog_cb() fires that passed state guard */
    uint32_t self_heals;     /* emit_diag self-heal fires (post + pre-discovery) */

    /* When true, the central is paused (e.g. dashboard set HR source to
     * H10 — no need to keep scanning for earclip). Stops scans/backoff
     * timers from running and prevents disconnect-event auto-restart.
     * Cleared by narbis_central_start(). */
    bool paused;
} S;

/* ---- log sink + state callback helpers ------------------------------- */

/* Mirror an ESP_LOG-style line to both ESP_LOG (UART) and the registered
 * sink (typically main.c's ble_log → 0xFF03 frame type 0xF1). The sink
 * sees the message without a newline; main.c's ble_log adds the framing. */
static void cb_log(const char *fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof(buf)) n = sizeof(buf) - 1;
    buf[n] = '\0';
    ESP_LOGI(TAG, "%s", buf);
    if (S.log_sink) S.log_sink(buf);
}

static void emit_state(bool connected) {
    if (S.last_state_emitted == connected) return;
    S.last_state_emitted = connected;
    if (S.state_cb) S.state_cb(connected);
}

void narbis_central_set_log_sink(narbis_central_log_sink_t sink) {
    S.log_sink = sink;
}

void narbis_central_set_state_cb(narbis_central_state_cb_t cb) {
    S.state_cb = cb;
}

/* ---- NVS helpers ----------------------------------------------------- */

static esp_err_t nvs_read_earclip(uint8_t out[6]) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NARBIS_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    size_t sz = 6;
    err = nvs_get_blob(h, NARBIS_NVS_KEY_EARCLIP_MAC, out, &sz);
    nvs_close(h);
    if (err != ESP_OK) return err;
    if (sz != 6) return ESP_ERR_INVALID_SIZE;
    bool zero = true;
    for (int i = 0; i < 6; i++) { if (out[i]) { zero = false; break; } }
    return zero ? ESP_ERR_NOT_FOUND : ESP_OK;
}

static esp_err_t nvs_write_earclip(const uint8_t mac[6]) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NARBIS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, NARBIS_NVS_KEY_EARCLIP_MAC, mac, 6);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t nvs_erase_earclip(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NARBIS_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_key(h, NARBIS_NVS_KEY_EARCLIP_MAC);
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        (void)nvs_commit(h);
        err = ESP_OK;
    }
    nvs_close(h);
    return err;
}

/* ---- Scan helpers ---------------------------------------------------- */

/* The earclip's primary 128-bit service UUID, in Bluedroid little-endian
 * byte layout. Same bytes as protocol/narbis_protocol.h NARBIS_SVC_UUID_BYTES. */
static const uint8_t NARBIS_SVC_UUID_LE[16] = NARBIS_SVC_UUID_BYTES;

static bool adv_contains_narbis_svc(const uint8_t *adv, uint8_t adv_len) {
    /* Walk the AD structure list looking for a "Complete List of 128-bit
     * Service UUIDs" (0x07) or "Incomplete List..." (0x06) field that
     * carries our UUID. */
    uint8_t i = 0;
    while (i + 1 < adv_len) {
        uint8_t fld_len = adv[i];
        if (fld_len == 0 || (i + 1 + fld_len) > adv_len) return false;
        uint8_t type = adv[i + 1];
        if (type == ESP_BLE_AD_TYPE_128SRV_CMPL || type == ESP_BLE_AD_TYPE_128SRV_PART) {
            const uint8_t *uuids = &adv[i + 2];
            uint8_t uuids_len = fld_len - 1;
            for (uint8_t j = 0; j + 16 <= uuids_len; j += 16) {
                if (memcmp(&uuids[j], NARBIS_SVC_UUID_LE, 16) == 0) return true;
            }
        }
        i += 1 + fld_len;
    }
    return false;
}

static esp_ble_scan_params_t scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,    /* 50 ms */
    .scan_window        = 0x30,    /* 30 ms */
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
};

static void start_scan_directed(void) {
    /* Cancel any pending backoff so we don't double-fire a scan when
     * the timer expires after we're already scanning. */
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.state = ST_SCANNING_DIRECTED;
    S.scan_attempts++;
    int64_t now = esp_timer_get_time();
    int last_seen_s = (S.last_seen_us == 0) ? -1
                     : (int)((now - S.last_seen_us) / 1000000);
    cb_log("central: scanning attempt %lu, last seen %d s ago (directed)",
           (unsigned long)S.scan_attempts, last_seen_s);
    esp_ble_gap_set_scan_params(&scan_params);
    esp_ble_gap_start_scanning(SCAN_DIRECTED_S);
}

static void start_scan_general(void) {
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.state = ST_SCANNING_GENERAL;
    S.scan_attempts++;
    memset(&S.best, 0, sizeof(S.best));
    int64_t now = esp_timer_get_time();
    int last_seen_s = (S.last_seen_us == 0) ? -1
                     : (int)((now - S.last_seen_us) / 1000000);
    cb_log("central: scanning attempt %lu, last seen %d s ago (general)",
           (unsigned long)S.scan_attempts, last_seen_s);
    esp_ble_gap_set_scan_params(&scan_params);
    esp_ble_gap_start_scanning(SCAN_GENERAL_S);
}

static void schedule_reconnect_backoff(void) {
    S.state = ST_BACKOFF;
    cb_log("central: backoff %d ms before next scan", RECONNECT_BACKOFF_MS);
    esp_timer_start_once(S.backoff_timer, (uint64_t)RECONNECT_BACKOFF_MS * 1000ULL);
}

/* Forward decl — watchdog cb references state_name() defined later. */
static const char *state_name(central_state_t s);

static void cancel_connect_watchdog(void) {
    if (S.connect_watchdog) esp_timer_stop(S.connect_watchdog);
}

static void arm_connect_watchdog(void) {
    S.wd_calls++;
    if (!S.connect_watchdog) return;
    esp_timer_stop(S.connect_watchdog);
    esp_timer_start_once(S.connect_watchdog,
                         (uint64_t)CONNECT_WATCHDOG_MS * 1000ULL);
    S.wd_armed++;
}

static void connect_watchdog_cb(void *arg) {
    (void)arg;
    /* Only fire if we're actually mid-chain (CONNECTING through any
     * subscribe step). If we already reached READY, or got cleanly
     * thrown back to BACKOFF/scanning, ignore. */
    if (S.state < ST_CONNECTING || S.state >= ST_READY) {
        return;
    }
    S.wd_fires++;
    cb_log("central: connect watchdog %dms in state=%s — force recovery",
           CONNECT_WATCHDOG_MS, state_name(S.state));
    /* Clear any phantom GATTC handle we may be holding. */
    if (S.conn_id != 0) {
        (void)esp_ble_gattc_close(S.gattc_if, S.conn_id);
        S.conn_id = 0;
    }
    /* Force the link down at the GAP layer. No-op if Bluedroid had no
     * actual connection — that's fine, gattc_close + this together
     * covers both the "stuck open" and "actually connected mid-discovery"
     * shapes. */
    (void)esp_ble_gap_disconnect(S.earclip_mac);
    /* Link is gone from our perspective. Update the dashboard-visible
     * link state so the badge stops showing "Earclip linked". If
     * DISCONNECT_EVT also fires, the redundant emit_state(false) is a
     * no-op due to the dedup check. */
    S.peer_connected = false;
    emit_state(false);
    /* Schedule the standard backoff so we don't spin. If
     * ESP_GATTC_DISCONNECT_EVT fires from the force-disconnect above,
     * its handler will already restart the scan; schedule_reconnect_backoff
     * here is the safety net in case the disconnect was a no-op (no
     * Bluedroid connection to drop). */
    schedule_reconnect_backoff();
}

static void backoff_timer_cb(void *arg) {
    (void)arg;
    if (S.paused) {
        ESP_LOGI(TAG, "central: backoff fired but paused — skipping scan");
        return;
    }
    if (S.earclip_known) start_scan_directed();
    else                 start_scan_general();
}

/* ---- Subscribe to a notify char by writing 0x0001 to its CCCD ------- */

static void cccd_subscribe(uint16_t cccd_handle) {
    uint8_t val[2] = { 0x01, 0x00 };
    esp_err_t err = esp_ble_gattc_write_char_descr(
        S.gattc_if, S.conn_id, cccd_handle,
        sizeof(val), val,
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cccd_subscribe(%u) failed: %d", cccd_handle, err);
    }
}

/* Write a CCCD value (enable=0x0001 / disable=0x0000) with explicit enable. */
static esp_err_t cccd_set(uint16_t cccd_handle, bool enable) {
    uint8_t val[2] = { (uint8_t)(enable ? 0x01 : 0x00), 0x00 };
    return esp_ble_gattc_write_char_descr(
        S.gattc_if, S.conn_id, cccd_handle,
        sizeof(val), val,
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
}

/* ---- State-machine helpers for the post-Path-B subscribe chain ------
 *
 * Sequence (per connect):
 *   WRITING_ROLE
 *     → SUBSCRIBING_IBI                    (always; required)
 *     → SUBSCRIBING_CONFIG                 (skip if hdl_config_cccd == 0)
 *     → READING_CONFIG_INITIAL             (one-shot read of hdl_config)
 *     → SUBSCRIBING_BATT                   (skip if hdl_battery_cccd == 0)
 *     → SUBSCRIBING_RAW                    (skip unless raw_enabled && hdl_raw_cccd)
 *     → READY
 *
 * Each forward helper is called with the current state already updated,
 * fires the GATTC operation, and the corresponding EVT advances on
 * completion. */
static void advance_to_raw_or_ready(void);

/* Transition to ST_READY and emit a log line listing whatever the earclip
 * actually exposed (older earclip firmware may lack config/battery/raw).
 *
 * Idempotent: re-issues `register_for_notify` + CCCD writes for every
 * cached notify handle so this works correctly whether called via the
 * normal subscribe chain (where these are no-ops since the chain
 * already did them) OR via the self-heal path (where the chain stalled
 * mid-way and these are essential to actually start receiving data).
 *
 * Also issues a one-shot CONFIG read so the dashboard's ConfigPanel
 * populates immediately on relay-up. */
static void enter_ready(void) {
    S.state = ST_READY;
    cancel_connect_watchdog();

    /* Bluedroid requires register_for_notify to dispatch incoming
     * notifies to gattc_cb. Without it, even a CCCD-enabled peer's
     * notifies are filtered out at the host layer. */
    if (S.hdl_ibi)     esp_ble_gattc_register_for_notify(S.gattc_if, S.earclip_mac, S.hdl_ibi);
    if (S.hdl_battery) esp_ble_gattc_register_for_notify(S.gattc_if, S.earclip_mac, S.hdl_battery);
    if (S.hdl_config)  esp_ble_gattc_register_for_notify(S.gattc_if, S.earclip_mac, S.hdl_config);
    if (S.hdl_diag)    esp_ble_gattc_register_for_notify(S.gattc_if, S.earclip_mac, S.hdl_diag);
    if (S.raw_enabled && S.hdl_raw) {
        esp_ble_gattc_register_for_notify(S.gattc_if, S.earclip_mac, S.hdl_raw);
    }

    /* Write 0x0001 to each CCCD to ask the earclip to send notifies.
     * Idempotent — the peer just acks if already subscribed. */
    if (S.hdl_ibi_cccd)     cccd_set(S.hdl_ibi_cccd,     true);
    if (S.hdl_config_cccd)  cccd_set(S.hdl_config_cccd,  true);
    if (S.hdl_battery_cccd) cccd_set(S.hdl_battery_cccd, true);
    if (S.hdl_diag_cccd)    cccd_set(S.hdl_diag_cccd,    true);
    if (S.raw_enabled && S.hdl_raw_cccd) cccd_set(S.hdl_raw_cccd, true);

    /* One-shot CONFIG read so dashboard ConfigPanel populates
     * immediately. The earclip only notifies CONFIG on changes, so
     * without this initial read the panel would stay empty until the
     * user makes the first edit. */
    if (S.hdl_config) {
        esp_ble_gattc_read_char(S.gattc_if, S.conn_id, S.hdl_config,
                                ESP_GATT_AUTH_REQ_NONE);
    }

    cb_log("central: ready (IBI%s%s%s subscribed)",
           S.hdl_config_cccd  ? " + config"  : "",
           S.hdl_battery_cccd ? " + battery" : "",
           (S.raw_enabled && S.hdl_raw_cccd) ? " + raw" : "");
    emit_state(true);
}

static void advance_to_raw_or_ready(void) {
    if (S.raw_enabled && S.hdl_raw_cccd) {
        S.state = ST_SUBSCRIBING_RAW;
        if (cccd_set(S.hdl_raw_cccd, true) != ESP_OK) {
            ESP_LOGW(TAG, "raw subscribe write failed; advancing to READY");
            enter_ready();
        }
        return;
    }
    enter_ready();
}

static void advance_to_batt_or_raw_or_ready(void) {
    if (S.hdl_battery_cccd) {
        S.state = ST_SUBSCRIBING_BATT;
        cccd_subscribe(S.hdl_battery_cccd);
        return;
    }
    advance_to_raw_or_ready();
}

/* The earclip notifies CONFIG only on changes (config_apply / mode_apply),
 * so a fresh subscriber sees nothing until the first edit. Issue a one-
 * shot read so the dashboard sees the current config immediately on
 * relay connect. The READ_CHAR_EVT handler routes the response through
 * the same config_cb the notify path uses, then advances the state. */
static void read_config_initial(void) {
    S.state = ST_READING_CONFIG_INITIAL;
    esp_err_t err = esp_ble_gattc_read_char(
        S.gattc_if, S.conn_id, S.hdl_config, ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) {
        cb_log("central: config initial read failed rc=%d, advancing", err);
        advance_to_batt_or_raw_or_ready();
    }
}

static void advance_to_config_or_batt_or_raw_or_ready(void) {
    if (S.hdl_config_cccd) {
        S.state = ST_SUBSCRIBING_CONFIG;
        cccd_subscribe(S.hdl_config_cccd);
        return;
    }
    advance_to_batt_or_raw_or_ready();
}

/* ---- Discovery + role write ----------------------------------------- */

static const uint8_t NARBIS_CHR_IBI_LE[16]          = NARBIS_CHR_IBI_UUID_BYTES;
static const uint8_t NARBIS_CHR_BATTERY_LE[16]      = NARBIS_CHR_BATTERY_UUID_BYTES;
static const uint8_t NARBIS_CHR_PEER_ROLE_LE[16]    = NARBIS_CHR_PEER_ROLE_UUID_BYTES;
static const uint8_t NARBIS_CHR_CONFIG_LE[16]       = NARBIS_CHR_CONFIG_UUID_BYTES;
static const uint8_t NARBIS_CHR_CONFIG_WRITE_LE[16] = NARBIS_CHR_CONFIG_WRITE_UUID_BYTES;
static const uint8_t NARBIS_CHR_RAW_PPG_LE[16]      = NARBIS_CHR_RAW_PPG_UUID_BYTES;
static const uint8_t NARBIS_CHR_DIAG_LE[16]         = NARBIS_CHR_DIAGNOSTICS_UUID_BYTES;

static bool char_uuid_matches(const esp_bt_uuid_t *u, const uint8_t le16[16]) {
    if (u->len != ESP_UUID_LEN_128) return false;
    return memcmp(u->uuid.uuid128, le16, 16) == 0;
}

/* Walk all chars in the cached service range and stash handles. */
static void cache_handles_after_discover(void) {
    uint16_t count = 0;
    esp_gatt_status_t st = esp_ble_gattc_get_attr_count(
        S.gattc_if, S.conn_id, ESP_GATT_DB_CHARACTERISTIC,
        S.svc_start_handle, S.svc_end_handle, 0, &count);
    if (st != ESP_GATT_OK || count == 0) {
        ESP_LOGW(TAG, "no chars in service range: st=%d count=%u", st, count);
        return;
    }
    esp_gattc_char_elem_t *chrs = calloc(count, sizeof(*chrs));
    if (!chrs) return;
    uint16_t got = count;
    st = esp_ble_gattc_get_all_char(S.gattc_if, S.conn_id,
                                    S.svc_start_handle, S.svc_end_handle,
                                    chrs, &got, 0);
    if (st != ESP_GATT_OK) {
        free(chrs);
        return;
    }
    for (uint16_t i = 0; i < got; i++) {
        const esp_gattc_char_elem_t *c = &chrs[i];
        if      (char_uuid_matches(&c->uuid, NARBIS_CHR_IBI_LE))          S.hdl_ibi          = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_BATTERY_LE))      S.hdl_battery      = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_PEER_ROLE_LE))    S.hdl_peer_role    = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_CONFIG_LE))       S.hdl_config       = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_CONFIG_WRITE_LE)) S.hdl_config_write = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_RAW_PPG_LE))      S.hdl_raw          = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_DIAG_LE))         S.hdl_diag         = c->char_handle;
    }
    free(chrs);

    /* Find the CCCD (0x2902) descriptor following each notify char. */
    esp_bt_uuid_t cccd = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = BLE_UUID_CCCD } };
    if (S.hdl_ibi) {
        uint16_t dcount = 1;
        esp_gattc_descr_elem_t d;
        if (esp_ble_gattc_get_descr_by_char_handle(S.gattc_if, S.conn_id,
                                                   S.hdl_ibi, cccd,
                                                   &d, &dcount) == ESP_GATT_OK
            && dcount > 0) {
            S.hdl_ibi_cccd = d.handle;
        }
    }
    if (S.hdl_battery) {
        uint16_t dcount = 1;
        esp_gattc_descr_elem_t d;
        if (esp_ble_gattc_get_descr_by_char_handle(S.gattc_if, S.conn_id,
                                                   S.hdl_battery, cccd,
                                                   &d, &dcount) == ESP_GATT_OK
            && dcount > 0) {
            S.hdl_battery_cccd = d.handle;
        }
    }
    if (S.hdl_config) {
        uint16_t dcount = 1;
        esp_gattc_descr_elem_t d;
        if (esp_ble_gattc_get_descr_by_char_handle(S.gattc_if, S.conn_id,
                                                   S.hdl_config, cccd,
                                                   &d, &dcount) == ESP_GATT_OK
            && dcount > 0) {
            S.hdl_config_cccd = d.handle;
        }
    }
    if (S.hdl_raw) {
        uint16_t dcount = 1;
        esp_gattc_descr_elem_t d;
        if (esp_ble_gattc_get_descr_by_char_handle(S.gattc_if, S.conn_id,
                                                   S.hdl_raw, cccd,
                                                   &d, &dcount) == ESP_GATT_OK
            && dcount > 0) {
            S.hdl_raw_cccd = d.handle;
        }
    }
    if (S.hdl_diag) {
        uint16_t dcount = 1;
        esp_gattc_descr_elem_t d;
        if (esp_ble_gattc_get_descr_by_char_handle(S.gattc_if, S.conn_id,
                                                   S.hdl_diag, cccd,
                                                   &d, &dcount) == ESP_GATT_OK
            && dcount > 0) {
            S.hdl_diag_cccd = d.handle;
        }
    }
    /* Bridge to cb_log so the dashboard can see which CCCDs were found.
     * If CONFIG / RAW CCCDs are 0 here, those subscribe steps will be
     * silently skipped — this is the only way to see why. */
    cb_log("handles ibi=%u/%u batt=%u/%u role=%u cfg=%u/%u cfgw=%u raw=%u/%u",
           S.hdl_ibi, S.hdl_ibi_cccd,
           S.hdl_battery, S.hdl_battery_cccd,
           S.hdl_peer_role,
           S.hdl_config, S.hdl_config_cccd, S.hdl_config_write,
           S.hdl_raw, S.hdl_raw_cccd);
}

static void write_peer_role(void) {
    if (S.hdl_peer_role == 0) {
        ESP_LOGW(TAG, "no peer-role char; earclip is older firmware");
        /* Skip ahead — don't block IBI subscription on a missing char. */
        S.state = ST_SUBSCRIBING_IBI;
        if (S.hdl_ibi_cccd) cccd_subscribe(S.hdl_ibi_cccd);
        return;
    }
    uint8_t role = (uint8_t)NARBIS_PEER_ROLE_GLASSES;
    S.state = ST_WRITING_ROLE;
    esp_err_t err = esp_ble_gattc_write_char(
        S.gattc_if, S.conn_id, S.hdl_peer_role,
        1, &role,
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) ESP_LOGW(TAG, "write peer_role failed: %d", err);
}

/* ---- GAP event hook (called from main.c's gap_event_handler) -------
 *
 * Bluedroid registers a single GAP callback. The peripheral side already
 * owns it (for advertising lifecycle); main.c invokes this hook for every
 * GAP event so the central can react to scan results / scan-stop. */

void narbis_central_gap_event(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *p) {
    if (p == NULL) return;
    switch (event) {

    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        /* Scan params accepted; start_scanning was already called separately. */
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        const struct ble_scan_result_evt_param *r = &p->scan_rst;
        if (r->search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            if (r->search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
                /* Scan window elapsed. */
                if (S.state == ST_SCANNING_GENERAL) {
                    cb_log("central: scan done, %u adv seen, %u matched narbis",
                           (unsigned)S.scan_advs_seen,
                           (unsigned)S.scan_advs_matched);
                    if (S.best.valid) {
                        memcpy(S.earclip_mac, S.best.bda, 6);
                        S.earclip_addr_type = S.best.addr_type;
                        S.earclip_known = true;
                        (void)nvs_write_earclip(S.earclip_mac);
                        cb_log("central: best rssi=%d addr_type=%d, persisted",
                               S.best.rssi, (int)S.earclip_addr_type);
                        S.state = ST_CONNECTING;
                        esp_err_t oerr = esp_ble_gattc_open(S.gattc_if, S.earclip_mac,
                                                            S.earclip_addr_type, true);
                        if (oerr != ESP_OK) {
                            cb_log("central: gattc_open rc=%s — backing off",
                                   esp_err_to_name(oerr));
                            schedule_reconnect_backoff();
                        } else {
                            arm_connect_watchdog();
                        }
                    } else {
                        schedule_reconnect_backoff();
                    }
                } else if (S.state == ST_SCANNING_DIRECTED) {
                    cb_log("central: directed scan done, %u adv seen, target not found",
                           (unsigned)S.scan_advs_seen);
                    schedule_reconnect_backoff();
                }
                S.scan_advs_seen = 0;
                S.scan_advs_matched = 0;
            }
            break;
        }

        /* Inquiry result hit — count it for diagnostics. */
        S.scan_advs_seen++;

        if (S.state == ST_SCANNING_DIRECTED) {
            if (S.earclip_known
                && memcmp(r->bda, S.earclip_mac, 6) == 0) {
                /* Refresh on every hit: NVS persists only the MAC, so on a
                 * cold boot we don't yet know the address type until we see
                 * the first adv from the saved peer. */
                S.earclip_addr_type = r->ble_addr_type;
                esp_ble_gap_stop_scanning();
                S.state = ST_CONNECTING;
                S.last_seen_us = esp_timer_get_time();
                esp_err_t oerr = esp_ble_gattc_open(S.gattc_if, S.earclip_mac,
                                                    S.earclip_addr_type, true);
                if (oerr != ESP_OK) {
                    cb_log("central: gattc_open rc=%s — backing off",
                           esp_err_to_name(oerr));
                    schedule_reconnect_backoff();
                } else {
                    arm_connect_watchdog();
                }
            }
        } else if (S.state == ST_SCANNING_GENERAL) {
            if (adv_contains_narbis_svc(r->ble_adv, r->adv_data_len + r->scan_rsp_len)) {
                S.scan_advs_matched++;
                /* Log the first match per scan window so users can see
                 * proof-of-life without spamming with every adv. */
                if (S.scan_advs_matched == 1) {
                    cb_log("central: matched narbis adv %02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
                           r->bda[0], r->bda[1], r->bda[2],
                           r->bda[3], r->bda[4], r->bda[5], r->rssi);
                }
                if (!S.best.valid || r->rssi > S.best.rssi) {
                    S.best.valid = true;
                    S.best.rssi = r->rssi;
                    S.best.addr_type = r->ble_addr_type;
                    memcpy(S.best.bda, r->bda, 6);
                }
            }
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    default:
        break;
    }
}

/* ---- GATTC callback ------------------------------------------------- */

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *p) {
    switch (event) {

    case ESP_GATTC_REG_EVT:
        if (p->reg.status == ESP_GATT_OK) {
            S.gattc_if = gattc_if;
            ESP_LOGI(TAG, "gattc registered, if=%d", gattc_if);
        } else {
            ESP_LOGE(TAG, "gattc reg failed: %d", p->reg.status);
        }
        break;

    case ESP_GATTC_CONNECT_EVT: {
        /* Capture the previous state for diagnostics. With SMP enabled in
         * sdkconfig (CONFIG_BT_BLE_SMP_ENABLE=y, CONFIG_BT_BLE_SMP_BOND_NVS_FLASH=y),
         * Bluedroid can produce CONNECT_EVT from a bonded-peer auto-reconnect
         * without us ever calling gattc_open() — our state would still be
         * ST_SCANNING_DIRECTED / ST_SCANNING_GENERAL / ST_BACKOFF in that
         * case, and the watchdog wouldn't have been armed by the scan-
         * result handler. That's the c=1 / wdc=0 wedge shape we've been
         * seeing. Arm the watchdog HERE unconditionally so the chain is
         * covered regardless of who initiated the connect. */
        central_state_t prev_state = S.state;
        if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
        S.conn_id = p->connect.conn_id;
        S.last_seen_us = esp_timer_get_time();
        /* If we were already at READY before this CONNECT_EVT, Bluedroid
         * silently re-cycled the link (no DISCONNECT_EVT to our cb). Reset
         * the emit_state dedup so the next emit_state(true) from enter_ready
         * actually fires; otherwise the dashboard badge stays stuck at the
         * stale value. */
        if (prev_state == ST_READY) {
            emit_state(false);
        }
        S.state = ST_DISCOVERING;
        S.connects++;
        S.peer_connected = true;
        arm_connect_watchdog();
        /* Defense-in-depth: invalidate any Bluedroid GATT cache for this
         * peer before doing MTU + search_service. If Bluedroid auto-
         * reconnected from a cached prior session, the cache may have
         * stale handles and search_service will short-circuit with no
         * SEARCH_RES_EVT. cache_refresh forces a fresh discovery on the
         * link we just established. Pair with the cache_clean at boot
         * in narbis_central_start. */
        (void)esp_ble_gattc_cache_refresh(p->connect.remote_bda);
        cb_log("central: connected, conn_id=%u (was %s)",
               (unsigned)S.conn_id, state_name(prev_state));
        /* Negotiate larger MTU; the peripheral may decline and we fall
         * back to default 23. Don't gate further progress on CFG_MTU_EVT
         * — observed Bluedroid quirk where the peer initiates MTU first
         * and CFG_MTU_EVT never fires for our app-layer request. The
         * earclip's NimBLE side completes MTU=247 fine, but we'd sit
         * forever waiting for the EVT and the chain would never advance
         * (chain c=N d=N mtu=0 srch=0 across multiple cycles). */
        esp_err_t mrc = esp_ble_gattc_send_mtu_req(gattc_if, S.conn_id);
        if (mrc != ESP_OK) {
            cb_log("central: send_mtu_req rc=%s", esp_err_to_name(mrc));
        }
        /* Kick off service discovery immediately rather than waiting
         * for CFG_MTU_EVT. Search runs fine at the default MTU (23 B);
         * subsequent larger characteristic reads still benefit from
         * MTU=247 once it's exchanged at the LL layer (Bluedroid
         * handles this transparently). */
        {
            esp_bt_uuid_t svc = { .len = ESP_UUID_LEN_128 };
            memcpy(svc.uuid.uuid128, NARBIS_SVC_UUID_LE, 16);
            esp_err_t src = esp_ble_gattc_search_service(gattc_if, S.conn_id, &svc);
            if (src != ESP_OK) {
                cb_log("central: search_service rc=%s", esp_err_to_name(src));
            }
        }
        break;
    }

    case ESP_GATTC_OPEN_EVT:
        if (p->open.status != ESP_GATT_OK) {
            cb_log("central: open failed status=%d", p->open.status);
            /* Status 0x91 ESP_GATT_ALREADY_OPEN = Bluedroid still
             * tracks a previous connection (or pending open) to this
             * MAC. Backing off and retrying loops forever — the stale
             * state needs a force-disconnect to clear. The DISCONNECT
             * event will then fire and auto-restart the scan; we add
             * a short backoff as a safety net in case it doesn't. */
            if (p->open.status == ESP_GATT_ALREADY_OPEN) {
                cb_log("central: stale conn — forcing gap_disconnect");
                (void)esp_ble_gap_disconnect(S.earclip_mac);
                S.state = ST_BACKOFF;
                if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
                esp_timer_start_once(S.backoff_timer, 2 * 1000 * 1000ULL);
            } else {
                schedule_reconnect_backoff();
            }
        }
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        S.mtus++;
        ESP_LOGI(TAG, "central: mtu=%u", p->cfg_mtu.mtu);
        /* Note: search_service was already kicked off from CONNECT_EVT
         * — we no longer gate discovery on this event arriving, because
         * Bluedroid sometimes never delivers it (peer-initiated MTU
         * exchange or version quirks). This event is now purely
         * informational; the mtu counter in the chain diag tells us
         * whether MTU completion was reported back to us. */
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        S.searches++;
        S.svc_start_handle = p->search_res.start_handle;
        S.svc_end_handle   = p->search_res.end_handle;
        ESP_LOGI(TAG, "central: svc handles %u..%u",
                 S.svc_start_handle, S.svc_end_handle);
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (S.svc_start_handle == 0) {
            ESP_LOGW(TAG, "central: NARBIS service not found on peer");
            esp_ble_gattc_close(gattc_if, S.conn_id);
            break;
        }
        cache_handles_after_discover();
        write_peer_role();
        break;

    case ESP_GATTC_WRITE_CHAR_EVT:
        /* Only the connect-time peer-role write should advance the state
         * machine. Runtime config-write completions (CONFIG_WRITE while
         * in ST_READY) reach this event too — guard so they don't reset. */
        if (S.state == ST_WRITING_ROLE) {
            S.state = ST_SUBSCRIBING_IBI;
            if (S.hdl_ibi_cccd) cccd_subscribe(S.hdl_ibi_cccd);
            else { ESP_LOGW(TAG, "no IBI CCCD; skipping"); enter_ready(); }
        }
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (S.state == ST_SUBSCRIBING_IBI) {
            advance_to_config_or_batt_or_raw_or_ready();
        } else if (S.state == ST_SUBSCRIBING_CONFIG) {
            read_config_initial();
        } else if (S.state == ST_SUBSCRIBING_BATT) {
            advance_to_raw_or_ready();
        } else if (S.state == ST_SUBSCRIBING_RAW) {
            enter_ready();
        }
        /* Runtime CCCD writes (e.g. raw toggle in ST_READY) intentionally
         * fall through — no state transition. */
        break;

    case ESP_GATTC_READ_CHAR_EVT: {
        const struct gattc_read_char_evt_param *r = &p->read;
        /* Always dispatch CONFIG reads to the callback — could come
         * from the boot-time chain (state=READING_CONFIG_INITIAL) OR
         * from enter_ready's idempotent read (state=READY). Without
         * this the config blob silently drops on the relay path. */
        if (r->handle == S.hdl_config) {
            if (r->status == ESP_GATT_OK && S.config_cb) {
                S.config_cb(r->value, r->value_len);
                cb_log("central: config read ok (%u B)", r->value_len);
            } else if (r->status != ESP_GATT_OK) {
                cb_log("central: config read status=%d", r->status);
            }
            /* Only advance the state machine if we're actually in the
             * boot-time chain. In ST_READY this is just a refresh. */
            if (S.state == ST_READING_CONFIG_INITIAL) {
                advance_to_batt_or_raw_or_ready();
            }
        }
        break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
        const struct gattc_notify_evt_param *n = &p->notify;
        if (n->handle == S.hdl_ibi
            && n->value_len >= sizeof(narbis_ibi_payload_t)) {
            narbis_ibi_payload_t pl;
            memcpy(&pl, n->value, sizeof(pl));
            if (S.ibi_cb) S.ibi_cb(pl.ibi_ms, pl.confidence_x100, pl.flags);
            S.notify_ibi_count++;
        } else if (n->handle == S.hdl_battery
                   && n->value_len >= sizeof(narbis_battery_payload_t)) {
            narbis_battery_payload_t pl;
            memcpy(&pl, n->value, sizeof(pl));
            if (S.batt_cb) S.batt_cb(pl.soc_pct, pl.mv, pl.charging);
            S.notify_batt_count++;
        } else if (n->handle == S.hdl_config && S.config_cb) {
            S.config_cb(n->value, n->value_len);
            S.notify_config_count++;
        } else if (n->handle == S.hdl_raw && S.raw_cb) {
            S.raw_cb(n->value, n->value_len);
            S.notify_raw_count++;
        } else if (n->handle == S.hdl_diag && S.diag_cb) {
            S.diag_cb(n->value, n->value_len);
            S.notify_diag_count++;
        } else {
            S.notify_other_count++;
        }
        S.last_seen_us = esp_timer_get_time();
        /* Self-heal: if we're already receiving notifies but the state
         * machine never advanced to READY (e.g. WRITE_DESCR_EVT for the
         * last subscribe was lost under load), the subscribe chain
         * obviously succeeded — force the transition so emit_state(true)
         * fires and the dashboard badge updates. */
        if (S.state != ST_READY &&
            (S.state == ST_SUBSCRIBING_IBI ||
             S.state == ST_SUBSCRIBING_CONFIG ||
             S.state == ST_READING_CONFIG_INITIAL ||
             S.state == ST_SUBSCRIBING_BATT ||
             S.state == ST_SUBSCRIBING_RAW)) {
            cb_log("central: notify mid-subscribe (st=%d) — force READY", (int)S.state);
            enter_ready();
        }
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
        S.disconnects++;
        S.peer_connected = false;
        cb_log("central: disconnected reason=%d", p->disconnect.reason);
        cancel_connect_watchdog();
        emit_state(false);
        /* Unregister all per-char notify registrations so they don't
         * pile up in Bluedroid's internal table (default cap of 5
         * entries via CONFIG_BT_GATTC_NOTIF_REG_MAX). Without this,
         * after a few reconnects the table fills, new register_for_notify
         * calls fail, and downstream discovery/notify dispatch breaks. */
        if (S.hdl_ibi)     esp_ble_gattc_unregister_for_notify(S.gattc_if, S.earclip_mac, S.hdl_ibi);
        if (S.hdl_battery) esp_ble_gattc_unregister_for_notify(S.gattc_if, S.earclip_mac, S.hdl_battery);
        if (S.hdl_config)  esp_ble_gattc_unregister_for_notify(S.gattc_if, S.earclip_mac, S.hdl_config);
        if (S.hdl_raw)     esp_ble_gattc_unregister_for_notify(S.gattc_if, S.earclip_mac, S.hdl_raw);
        if (S.hdl_diag)    esp_ble_gattc_unregister_for_notify(S.gattc_if, S.earclip_mac, S.hdl_diag);
        S.conn_id = 0;
        S.svc_start_handle = S.svc_end_handle = 0;
        S.hdl_ibi = S.hdl_ibi_cccd = 0;
        S.hdl_battery = S.hdl_battery_cccd = 0;
        S.hdl_peer_role = 0;
        S.hdl_config = S.hdl_config_cccd = S.hdl_config_write = 0;
        S.hdl_raw = S.hdl_raw_cccd = 0;
        S.hdl_diag = S.hdl_diag_cccd = 0;
        S.notify_ibi_count = S.notify_config_count = 0;
        S.notify_batt_count = S.notify_raw_count = 0;
        S.notify_diag_count = S.notify_other_count = 0;
        /* Avoid re-issuing scan if forget()/start() already kicked one
         * off. Without this guard, a forced disconnect during a fresh
         * scan would restart that scan and lose progress. Also skip
         * entirely when paused (HR-source switched to H10) — the
         * disconnect was intentional and we shouldn't reconnect. */
        if (S.paused) {
            S.state = ST_IDLE;
        } else if (S.state != ST_SCANNING_DIRECTED && S.state != ST_SCANNING_GENERAL) {
            if (S.earclip_known) start_scan_directed();
            else                 start_scan_general();
        }
        break;

    default:
        break;
    }
}

/* ---- Public API ----------------------------------------------------- */

esp_err_t narbis_central_init(narbis_central_ibi_cb_t     ibi_cb,
                              narbis_central_battery_cb_t batt_cb) {
    memset(&S, 0, sizeof(S));
    S.ibi_cb  = ibi_cb;
    S.batt_cb = batt_cb;
    S.gattc_if = ESP_GATT_IF_NONE;
    /* Default raw-PPG relay ON so the central subscribes during the
     * boot-time connect chain (rather than waiting for the dashboard's
     * 0xC4=1 to arrive after READY — by then the chain has finished
     * and the toggle would only apply on next reconnect). The dashboard
     * can still turn it off mid-session via 0xC4=0. */
    S.raw_enabled = true;

    esp_err_t err;
    /* GAP callback is owned by main.c's gap_event_handler — it forwards
     * every event to narbis_central_gap_event(). Registering a second
     * callback here would silently replace the peripheral's. */
    if ((err = esp_ble_gattc_register_callback(gattc_cb)) != ESP_OK) return err;
    if ((err = esp_ble_gattc_app_register(NARBIS_GATTC_APP_ID)) != ESP_OK) return err;

    const esp_timer_create_args_t targs = {
        .callback = backoff_timer_cb,
        .name = "narbis_central_backoff",
    };
    if ((err = esp_timer_create(&targs, &S.backoff_timer)) != ESP_OK) return err;

    const esp_timer_create_args_t wargs = {
        .callback = connect_watchdog_cb,
        .name = "narbis_central_watchdog",
    };
    if ((err = esp_timer_create(&wargs, &S.connect_watchdog)) != ESP_OK) return err;

    ESP_LOGI(TAG, "central init ok");
    return ESP_OK;
}

esp_err_t narbis_central_start(void) {
    /* Clear paused state in case we're resuming after a stop() — the
     * dashboard flipped HR source back to earclip. Idempotent if already
     * unpaused. */
    if (S.paused) {
        ESP_LOGI(TAG, "central: resuming (was paused)");
        S.paused = false;
    }
    uint8_t mac[6];
    if (nvs_read_earclip(mac) == ESP_OK) {
        memcpy(S.earclip_mac, mac, 6);
        S.earclip_known = true;
        ESP_LOGI(TAG, "central: NVS earclip %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        /* Wipe any Bluedroid-cached GATT service info for this peer.
         * Bluedroid persists service caches (sometimes implicitly,
         * especially when bonding was previously enabled) and on next
         * connect will short-circuit search_service — firing
         * SEARCH_CMPL_EVT with no SEARCH_RES_EVT in between. The central
         * sees svc_start_handle=0, logs "service not found", calls
         * gattc_close, then re-cycles. d=0 c=2 srch=0 in chain diag.
         * Cleaning the cache forces a fresh over-the-air discovery on
         * the next connect, which actually produces SEARCH_RES_EVT and
         * lets us cache real handles. */
        esp_err_t cc = esp_ble_gattc_cache_clean(S.earclip_mac);
        ESP_LOGI(TAG, "central: cache_clean rc=%s", esp_err_to_name(cc));
        /* Pre-emptive disconnect of any Bluedroid-side persistent link to
         * the saved peer. Symptom: c=1 with wdc=1 (only the CONNECT_EVT-
         * level arm fired) means the connection came from Bluedroid auto-
         * reconnect, bypassing our scan handler. After our watchdog/self-
         * heal "force-disconnect" the BLE link sometimes stays alive at
         * the controller level (no DISCONNECT_EVT, d=0 in chain) — the
         * earclip's LED keeps showing connected and our directed scan
         * sees no matching adv ("757 adv seen, target not found") because
         * Bluedroid filters out advs from peers it considers connected.
         * Calling gap_disconnect on the saved MAC here knocks the stale
         * link loose so our scan actually sees the earclip's adv. The
         * second call clears the accept list / RPA list — Bluedroid uses
         * these to short-circuit reconnects without us asking. */
        (void)esp_ble_gap_disconnect(S.earclip_mac);
        esp_err_t wl = esp_ble_gap_clear_whitelist();
        ESP_LOGI(TAG, "central: pre-scan disconnect + clear_whitelist rc=%s",
                 esp_err_to_name(wl));
        start_scan_directed();
    } else {
        S.earclip_known = false;
        ESP_LOGI(TAG, "central: no paired earclip — general scan");
        start_scan_general();
    }
    return ESP_OK;
}

esp_err_t narbis_central_stop(void) {
    ESP_LOGI(TAG, "central: stop (HR source switched away from earclip)");
    S.paused = true;
    cancel_connect_watchdog();
    S.peer_connected = false;
    emit_state(false);
    /* Close any active connection; the DISCONNECT_EVT handler sees
     * S.paused=true and skips the auto-restart-scan. */
    if (S.conn_id) {
        esp_ble_gattc_close(S.gattc_if, S.conn_id);
    }
    /* Cancel any in-flight scan. */
    if (S.state == ST_SCANNING_DIRECTED || S.state == ST_SCANNING_GENERAL) {
        esp_ble_gap_stop_scanning();
    }
    /* Drop the backoff timer so a pending tick doesn't relaunch a scan. */
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.scan_attempts = 0;
    /* Leave earclip_known + earclip_mac intact so a subsequent
     * narbis_central_start() can resume directed scan to the same earclip
     * without re-pairing. */
    return ESP_OK;
}

esp_err_t narbis_central_forget(void) {
    ESP_LOGW(TAG, "central: forget paired earclip");
    cancel_connect_watchdog();
    S.peer_connected = false;
    emit_state(false);
    if (S.conn_id) {
        esp_ble_gattc_close(S.gattc_if, S.conn_id);
    }
    /* Defensive: even if our tracked conn_id is 0, Bluedroid may still
     * have a stale connection to the previous MAC (e.g. earclip
     * power-cycled mid-session). Force-disconnect on the stored MAC
     * before wiping it so the next open() doesn't return ALREADY_OPEN. */
    if (S.earclip_known) {
        (void)esp_ble_gap_disconnect(S.earclip_mac);
    }
    if (S.state == ST_SCANNING_DIRECTED || S.state == ST_SCANNING_GENERAL) {
        esp_ble_gap_stop_scanning();
    }
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.earclip_known = false;
    memset(S.earclip_mac, 0, 6);
    S.scan_attempts = 0;
    return nvs_erase_earclip();
}

bool narbis_central_is_connected(void) {
    /* Bluedroid link state, not app state machine. The dashboard's
     * 0xF6 relay-link badge wants the same notion of "connected" that
     * the earclip's LED uses (BLE link established), regardless of
     * whether our state machine reached ST_READY. Returning state
     * == ST_READY here caused the dashboard to stick at "scanning
     * earclip" any time Bluedroid silently re-cycled the link, even
     * while the earclip was happily streaming. The chain counters in
     * the diag dump still report the app-state progress separately. */
    return S.peer_connected;
}

/* ---- Path B Phase 1/2: config + raw relay setters ------------------- */

void narbis_central_set_config_cb(narbis_central_config_cb_t cb) {
    S.config_cb = cb;
}

void narbis_central_set_raw_cb(narbis_central_raw_cb_t cb) {
    S.raw_cb = cb;
}

void narbis_central_set_diag_cb(narbis_central_diag_cb_t cb) {
    S.diag_cb = cb;
}

esp_err_t narbis_central_set_raw_enabled(bool enabled) {
    S.raw_enabled = enabled;
    /* Latch only if not in a state where we can write the CCCD now. */
    if (S.state != ST_READY || S.hdl_raw_cccd == 0) {
        cb_log("central: raw subscribe %s (latched, will apply on connect)",
               enabled ? "on" : "off");
        return ESP_OK;
    }
    esp_err_t err = cccd_set(S.hdl_raw_cccd, enabled);
    cb_log("central: raw subscribe %s rc=%d", enabled ? "on" : "off", err);
    return err;
}

static const char *state_name(central_state_t s) {
    switch (s) {
    case ST_IDLE:                   return "IDLE";
    case ST_SCANNING_DIRECTED:      return "SCAN_DIR";
    case ST_SCANNING_GENERAL:       return "SCAN_GEN";
    case ST_CONNECTING:             return "CONNECTING";
    case ST_DISCOVERING:            return "DISCOVER";
    case ST_WRITING_ROLE:           return "WRITE_ROLE";
    case ST_SUBSCRIBING_IBI:        return "SUB_IBI";
    case ST_SUBSCRIBING_CONFIG:     return "SUB_CFG";
    case ST_READING_CONFIG_INITIAL: return "READ_CFG";
    case ST_SUBSCRIBING_BATT:       return "SUB_BATT";
    case ST_SUBSCRIBING_RAW:        return "SUB_RAW";
    case ST_READY:                  return "READY";
    case ST_BACKOFF:                return "BACKOFF";
    }
    return "?";
}

void narbis_central_emit_diag(void) {
    cb_log("relay state=%s ready=%d", state_name(S.state),
           narbis_central_is_connected() ? 1 : 0);
    /* Split handles across two lines so neither overruns the 64-byte
     * ble_log buffer (which truncates the trailing CCCD value). */
    cb_log("hdl ibi=%u/%u batt=%u/%u role=%u",
           S.hdl_ibi, S.hdl_ibi_cccd,
           S.hdl_battery, S.hdl_battery_cccd,
           S.hdl_peer_role);
    cb_log("hdl cfg=%u/%u cfgw=%u raw=%u/%u diag=%u/%u",
           S.hdl_config, S.hdl_config_cccd, S.hdl_config_write,
           S.hdl_raw, S.hdl_raw_cccd,
           S.hdl_diag, S.hdl_diag_cccd);
    /* Notify counters — if state=READY but all of these are 0, the
     * earclip isn't sending OR Bluedroid isn't dispatching. Non-zero
     * means data is actually flowing. */
    cb_log("rx ibi=%u cfg=%u batt=%u raw=%u diag=%u other=%u",
           (unsigned)S.notify_ibi_count, (unsigned)S.notify_config_count,
           (unsigned)S.notify_batt_count, (unsigned)S.notify_raw_count,
           (unsigned)S.notify_diag_count, (unsigned)S.notify_other_count);
    /* Chain progress: pairs with the relay state= line to localize wedges
     * that happen before the dashboard subscribes (one-shot CONNECT/MTU/
     * SEARCH events otherwise leave no BLE-visible trace). wd_ok=0 means
     * S.connect_watchdog is NULL — the timer-based watchdog is disabled
     * (likely esp_timer_create failed at init) and any wedge in
     * CONNECTING/DISCOVERING is unrecoverable without the self-heal
     * fallback below. sh = self-heal fires since boot. */
    cb_log("chain c=%u d=%u mtu=%u srch=%u wdc=%u wda=%u wdf=%u wd_ok=%d sh=%u",
           (unsigned)S.connects, (unsigned)S.disconnects,
           (unsigned)S.mtus, (unsigned)S.searches,
           (unsigned)S.wd_calls, (unsigned)S.wd_armed, (unsigned)S.wd_fires,
           S.connect_watchdog ? 1 : 0,
           (unsigned)S.self_heals);
    /* Self-heal: handles cached but chain didn't reach READY — force the
     * transition (idempotent re-register + CCCD writes inside enter_ready
     * actually start data flow). Doesn't gate on S.conn_id because
     * Bluedroid can legitimately assign conn_id=0; state >= ST_WRITING_ROLE
     * already implies we're connected (handles are only cached after
     * SEARCH_CMPL_EVT). */
    if (S.state >= ST_WRITING_ROLE && S.state < ST_READY && S.hdl_ibi != 0) {
        S.self_heals++;
        cb_log("self-heal: handles cached state=%s — forcing READY",
               state_name(S.state));
        enter_ready();
    }
    /* Pre-discovery wedge fallback: post-CONNECT_EVT but no handles cached
     * yet (MTU exchange or service discovery never completed). Same shape
     * as PR #6's timer-based watchdog, but driven by the diag tick so it
     * still recovers when wd_ok=0 or the timer somehow isn't arming. Gate
     * on >15 s elapsed since CONNECT_EVT (S.last_seen_us is set at line
     * ~763) so we don't race a legitimate in-flight discovery. emit_diag
     * runs at dashboard-connect AND on the ~30 s periodic alive log, so
     * worst-case recovery time is ~45 s after the wedge starts.
     * No conn_id != 0 check — Bluedroid can legitimately assign conn_id=0,
     * and state == ST_DISCOVERING is only reachable via the CONNECT_EVT
     * handler so we know there's an active link. */
    else if (S.state >= ST_DISCOVERING && S.state < ST_READY &&
             S.hdl_ibi == 0 &&
             S.last_seen_us != 0 &&
             (esp_timer_get_time() - S.last_seen_us) > (int64_t)CONNECT_WATCHDOG_MS * 1000LL) {
        int64_t elapsed_ms = (esp_timer_get_time() - S.last_seen_us) / 1000;
        S.self_heals++;
        cb_log("self-heal: pre-discovery wedge (%lld ms, state=%s conn_id=%u) — force-disconnect",
               (long long)elapsed_ms, state_name(S.state), (unsigned)S.conn_id);
        (void)esp_ble_gattc_close(S.gattc_if, S.conn_id);
        S.conn_id = 0;
        (void)esp_ble_gap_disconnect(S.earclip_mac);
        S.peer_connected = false;
        emit_state(false);
        schedule_reconnect_backoff();
    }
}

esp_err_t narbis_central_write_earclip_config(const uint8_t *bytes, size_t len) {
    if (S.state != ST_READY || S.hdl_config_write == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (bytes == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = esp_ble_gattc_write_char(
        S.gattc_if, S.conn_id, S.hdl_config_write,
        (uint16_t)len, (uint8_t *)bytes,
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    cb_log("central: config write rc=%d (%u B)", err, (unsigned)len);
    return err;
}
