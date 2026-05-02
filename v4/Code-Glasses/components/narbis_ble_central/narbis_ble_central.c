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

#define SCAN_DIRECTED_S            5
#define SCAN_GENERAL_S             30
#define RECONNECT_BACKOFF_MS       30000

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
    ST_SUBSCRIBING_BATT,
    ST_READY,
    ST_BACKOFF,
} central_state_t;

typedef struct {
    uint8_t  bda[6];
    int      rssi;
    bool     valid;
} scan_best_t;

static struct {
    central_state_t state;

    narbis_central_ibi_cb_t     ibi_cb;
    narbis_central_battery_cb_t batt_cb;
    narbis_central_log_sink_t   log_sink;
    narbis_central_state_cb_t   state_cb;
    bool                        last_state_emitted;  /* dedup state edges */

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

    /* Pairing target. */
    uint8_t  earclip_mac[6];
    bool     earclip_known;

    /* General-scan winner-tracking. */
    scan_best_t best;

    /* Reconnect bookkeeping. */
    uint32_t scan_attempts;
    int64_t  last_seen_us;

    /* esp_timer for the 30 s reconnect backoff. */
    esp_timer_handle_t backoff_timer;

    /* Scan diagnostics — counted during each scan window, logged at
     * SCAN_INQ_CMPL_EVT. Tells us whether the central is seeing adverts
     * at all, and whether any match the NARBIS service UUID. */
    uint16_t scan_advs_seen;
    uint16_t scan_advs_matched;
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

static void backoff_timer_cb(void *arg) {
    (void)arg;
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

/* ---- Discovery + role write ----------------------------------------- */

static const uint8_t NARBIS_CHR_IBI_LE[16]       = NARBIS_CHR_IBI_UUID_BYTES;
static const uint8_t NARBIS_CHR_BATTERY_LE[16]   = NARBIS_CHR_BATTERY_UUID_BYTES;
static const uint8_t NARBIS_CHR_PEER_ROLE_LE[16] = NARBIS_CHR_PEER_ROLE_UUID_BYTES;

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
        if      (char_uuid_matches(&c->uuid, NARBIS_CHR_IBI_LE))       S.hdl_ibi       = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_BATTERY_LE))   S.hdl_battery   = c->char_handle;
        else if (char_uuid_matches(&c->uuid, NARBIS_CHR_PEER_ROLE_LE)) S.hdl_peer_role = c->char_handle;
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
    ESP_LOGI(TAG, "handles: ibi=%u/cccd=%u batt=%u/cccd=%u role=%u",
             S.hdl_ibi, S.hdl_ibi_cccd, S.hdl_battery, S.hdl_battery_cccd, S.hdl_peer_role);
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
                        S.earclip_known = true;
                        (void)nvs_write_earclip(S.earclip_mac);
                        cb_log("central: best rssi=%d, persisted", S.best.rssi);
                        S.state = ST_CONNECTING;
                        esp_ble_gattc_open(S.gattc_if, S.earclip_mac, BLE_ADDR_TYPE_PUBLIC, true);
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
                esp_ble_gap_stop_scanning();
                S.state = ST_CONNECTING;
                S.last_seen_us = esp_timer_get_time();
                esp_ble_gattc_open(S.gattc_if, S.earclip_mac, BLE_ADDR_TYPE_PUBLIC, true);
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

    case ESP_GATTC_CONNECT_EVT:
        S.conn_id = p->connect.conn_id;
        S.last_seen_us = esp_timer_get_time();
        S.state = ST_DISCOVERING;
        cb_log("central: connected, conn_id=%u", S.conn_id);
        /* Negotiate larger MTU; safe default 200, falls back to 23 on
         * peripherals that decline. */
        esp_ble_gattc_send_mtu_req(gattc_if, S.conn_id);
        break;

    case ESP_GATTC_OPEN_EVT:
        if (p->open.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "central: open failed status=%d", p->open.status);
            schedule_reconnect_backoff();
        }
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(TAG, "central: mtu=%u", p->cfg_mtu.mtu);
        /* Kick off service discovery for our 128-bit service UUID only. */
        {
            esp_bt_uuid_t svc = { .len = ESP_UUID_LEN_128 };
            memcpy(svc.uuid.uuid128, NARBIS_SVC_UUID_LE, 16);
            esp_ble_gattc_search_service(gattc_if, S.conn_id, &svc);
        }
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
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
        if (S.state == ST_WRITING_ROLE) {
            S.state = ST_SUBSCRIBING_IBI;
            if (S.hdl_ibi_cccd) cccd_subscribe(S.hdl_ibi_cccd);
            else { ESP_LOGW(TAG, "no IBI CCCD; skipping"); S.state = ST_READY; }
        }
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (S.state == ST_SUBSCRIBING_IBI) {
            if (S.hdl_battery_cccd) {
                S.state = ST_SUBSCRIBING_BATT;
                cccd_subscribe(S.hdl_battery_cccd);
            } else {
                S.state = ST_READY;
                cb_log("central: ready (IBI subscribed, no battery)");
                emit_state(true);
            }
        } else if (S.state == ST_SUBSCRIBING_BATT) {
            S.state = ST_READY;
            cb_log("central: ready (IBI + battery subscribed)");
            emit_state(true);
        }
        break;

    case ESP_GATTC_NOTIFY_EVT: {
        const struct gattc_notify_evt_param *n = &p->notify;
        if (n->handle == S.hdl_ibi
            && n->value_len >= sizeof(narbis_ibi_payload_t)) {
            narbis_ibi_payload_t pl;
            memcpy(&pl, n->value, sizeof(pl));
            if (S.ibi_cb) S.ibi_cb(pl.ibi_ms, pl.confidence_x100, pl.flags);
        } else if (n->handle == S.hdl_battery
                   && n->value_len >= sizeof(narbis_battery_payload_t)) {
            narbis_battery_payload_t pl;
            memcpy(&pl, n->value, sizeof(pl));
            if (S.batt_cb) S.batt_cb(pl.soc_pct, pl.mv, pl.charging);
        }
        S.last_seen_us = esp_timer_get_time();
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
        cb_log("central: disconnected reason=%d", p->disconnect.reason);
        emit_state(false);
        S.conn_id = 0;
        S.svc_start_handle = S.svc_end_handle = 0;
        S.hdl_ibi = S.hdl_ibi_cccd = 0;
        S.hdl_battery = S.hdl_battery_cccd = 0;
        S.hdl_peer_role = 0;
        if (S.earclip_known) start_scan_directed();
        else                 start_scan_general();
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

    ESP_LOGI(TAG, "central init ok");
    return ESP_OK;
}

esp_err_t narbis_central_start(void) {
    uint8_t mac[6];
    if (nvs_read_earclip(mac) == ESP_OK) {
        memcpy(S.earclip_mac, mac, 6);
        S.earclip_known = true;
        ESP_LOGI(TAG, "central: NVS earclip %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        start_scan_directed();
    } else {
        S.earclip_known = false;
        ESP_LOGI(TAG, "central: no paired earclip — general scan");
        start_scan_general();
    }
    return ESP_OK;
}

esp_err_t narbis_central_forget(void) {
    ESP_LOGW(TAG, "central: forget paired earclip");
    if (S.conn_id) {
        esp_ble_gattc_close(S.gattc_if, S.conn_id);
    } else if (S.state == ST_SCANNING_DIRECTED || S.state == ST_SCANNING_GENERAL) {
        esp_ble_gap_stop_scanning();
    }
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.earclip_known = false;
    memset(S.earclip_mac, 0, 6);
    S.scan_attempts = 0;
    return nvs_erase_earclip();
}

bool narbis_central_is_connected(void) {
    return S.state == ST_READY;
}
