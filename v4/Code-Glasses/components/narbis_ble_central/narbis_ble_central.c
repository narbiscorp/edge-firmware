/*
 * narbis_ble_central.c — NimBLE GATTC client that connects to a Narbis
 * earclip, writes the peer-role byte (GLASSES = 0x02), and subscribes to
 * IBI / CONFIG / BATTERY / RAW / DIAG notifications.
 *
 * Migrated from Bluedroid (PRs #5-#20 and #41 of historical interest;
 * see NIMBLE_MIGRATION_HANDOFF.md for the full diagnosis of why we left
 * Bluedroid behind). NimBLE provides per-call callbacks and per-step
 * GATT discovery primitives, which eliminate Bluedroid's host-layer
 * silent event-drop pathology while keeping the protocol identical.
 *
 * Discovery model (unchanged from Path B):
 *   - Boot: read NVS "narbis_pair"/"earclip_mac".
 *       Present -> directed scan 30 s + connect.
 *       Absent  -> general scan 30 s, filter by NARBIS_SVC_UUID, pick
 *                  highest-RSSI hit, persist its MAC, connect.
 *   - On disconnect: directed scan; if not found, backoff and retry.
 *
 * Connect chain (9 numbered steps, preserved verbatim for dashboard log
 * parsing):
 *   1. CONNECT (BLE_GAP_EVENT_CONNECT)
 *   2. MTU         (BLE_GAP_EVENT_MTU)
 *   3. SEARCH_RES  (svc disc callback per match)
 *   4. SEARCH_CMPL (svc disc callback EDONE)
 *   5. WRITE_ROLE_ACK
 *   6. SUB_IBI_ACK
 *   7. SUB_CFG_ACK
 *   8. SUB_BATT_ACK
 *   9. READY
 *
 * No bonding / encryption — same threat model as the rest of v1, and
 * CONFIG_BT_NIMBLE_SM_LEGACY=n + CONFIG_BT_NIMBLE_SM_SC=n in sdkconfig
 * make sure SMP code isn't compiled in.
 */

#include "narbis_ble_central.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"

#include "narbis_protocol.h"

static const char *TAG = "narbis_central";

#define NARBIS_NVS_NS              "narbis_pair"
#define NARBIS_NVS_KEY_EARCLIP_MAC "earclip_mac"

/* Directed scan window for the persisted earclip MAC. 30 s window + ~5 s
 * backoff keeps the radio listening most of the time so a wake/re-entry
 * of the earclip is caught within ~5 s. */
#define SCAN_DIRECTED_MS           30000
#define SCAN_GENERAL_MS            30000
#define RECONNECT_BACKOFF_MS       5000

/* Watchdog for the connect+discover+subscribe chain. Armed right after
 * each ble_gap_connect() call. If state hasn't progressed to ST_READY by
 * the time this fires, we assume the chain wedged and force-recover.
 * 25 s tolerates the slow chain-step turnaround seen at weak RSSI
 * (-90 dBm) under the earclip's BATCHED profile (100 ms itvl + lat=4 ~
 * 500 ms effective per ATT exchange — bench log on PR #22 showed 11 s
 * just for descriptor discovery at -94 dBm). At healthy RSSI the chain
 * still completes in ~3-5 s so this doesn't slow the common path. */
#define CONNECT_WATCHDOG_MS        25000

/* CCCD descriptor UUID (BLE-spec well-known). */
#define BLE_UUID_CCCD              0x2902

/* Connection sentinel (NimBLE's BLE_HS_CONN_HANDLE_NONE = 0xFFFF). */
#define CONN_HANDLE_NONE           BLE_HS_CONN_HANDLE_NONE

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
    ST_SUBSCRIBING_DIAG,
    ST_READY,
    ST_BACKOFF,
} central_state_t;

typedef struct {
    uint8_t  bda[6];
    uint8_t  addr_type;        /* NimBLE ble_addr_t.type */
    int      rssi;
    bool     valid;
} scan_best_t;

/* Per-characteristic descriptor-discovery cursor — we discover all chars
 * first, then walk each notify char's descriptor range looking for its
 * CCCD. dsc_pending_idx points into the dsc_targets array below. */
typedef struct {
    uint16_t  *val_handle;     /* (filled by chr disc) */
    uint16_t  *cccd_handle;    /* (filled by dsc disc when we find 0x2902 in range) */
    uint16_t  next_chr_handle; /* first handle beyond this char (set after chr disc) */
} dsc_target_t;

static struct {
    central_state_t state;

    narbis_central_ibi_cb_t     ibi_cb;
    narbis_central_battery_cb_t batt_cb;
    narbis_central_log_sink_t   log_sink;
    narbis_central_state_cb_t   state_cb;
    narbis_central_config_cb_t  config_cb;
    narbis_central_raw_cb_t     raw_cb;
    narbis_central_diag_cb_t    diag_cb;
    bool                        raw_enabled;
    bool                        last_state_emitted;
    /* Link state, separate from app state machine. Set on BLE_GAP_EVENT_
     * CONNECT success, cleared on DISCONNECT and on any force-recovery.
     * Dashboard's 0xF6 relay-link badge uses this so it reflects the
     * actual BLE link even when our state machine is mid-chain. */
    bool                        peer_connected;

    /* NimBLE connection handle. 0xFFFF when no link. */
    uint16_t conn_handle;
    uint8_t  own_addr_type;
    bool     own_addr_resolved;
    bool     start_pending;    /* narbis_central_start called before sync */

    /* Service handle range from svc discovery. */
    uint16_t svc_start_handle;
    uint16_t svc_end_handle;

    /* Cached char + CCCD handles. */
    uint16_t hdl_ibi;          uint16_t hdl_ibi_cccd;
    uint16_t hdl_battery;      uint16_t hdl_battery_cccd;
    uint16_t hdl_peer_role;
    uint16_t hdl_config;       uint16_t hdl_config_cccd;
    uint16_t hdl_config_write;
    uint16_t hdl_raw;          uint16_t hdl_raw_cccd;
    uint16_t hdl_diag;         uint16_t hdl_diag_cccd;

    /* Cursor + targets for descriptor discovery. We discover all dscs
     * in the svc range in one shot; per-descriptor cb sorts them into
     * the right char's CCCD slot by handle range. */
    dsc_target_t dsc_targets[5];   /* IBI, BATTERY, CONFIG, RAW, DIAG */
    int          dsc_target_count;

    /* Pairing target. */
    uint8_t  earclip_mac[6];
    uint8_t  earclip_addr_type;
    bool     earclip_known;

    /* General-scan winner. */
    scan_best_t best;

    /* Reconnect bookkeeping. */
    uint32_t scan_attempts;
    int64_t  last_seen_us;

    /* esp_timer for the reconnect backoff. */
    esp_timer_handle_t backoff_timer;

    /* esp_timer watchdog for the full connect+discover chain. Armed after
     * every ble_gap_connect() call. Fires force-recovery
     * (ble_gap_terminate + schedule_reconnect_backoff) if we don't reach
     * ST_READY within CONNECT_WATCHDOG_MS. */
    esp_timer_handle_t connect_watchdog;

    /* Scan diagnostics. */
    uint16_t scan_advs_seen;
    uint16_t scan_advs_matched;

    /* Per-char notify counters. */
    uint32_t notify_ibi_count;
    uint32_t notify_batt_count;
    uint32_t notify_config_count;
    uint32_t notify_raw_count;
    uint32_t notify_diag_count;
    uint32_t notify_other_count;

    /* Chain-event counters — diagnose where a connect chain stalled. */
    uint32_t connects;
    uint32_t disconnects;
    uint32_t mtus;
    uint32_t searches;
    uint32_t wd_calls;
    uint32_t wd_armed;
    uint32_t wd_fires;
    /* When true, the central is paused — stops scans/backoff and prevents
     * disconnect-event auto-restart. Cleared by narbis_central_start. */
    bool paused;
} S;

/* Forward decls — many of these reference each other in the chain. */
static int  gap_event_cb(struct ble_gap_event *event, void *arg);
static const char *state_name(central_state_t s);
static void start_scan_directed(void);
static void start_scan_general(void);
static void schedule_reconnect_backoff(void);
static void arm_connect_watchdog(void);
static void cancel_connect_watchdog(void);
static void initiate_connect(void);
static int  on_mtu_complete(uint16_t conn_handle, const struct ble_gatt_error *err,
                            uint16_t mtu, void *arg);
static int  on_svc_disc(uint16_t conn_handle, const struct ble_gatt_error *err,
                        const struct ble_gatt_svc *svc, void *arg);
static int  on_chr_disc(uint16_t conn_handle, const struct ble_gatt_error *err,
                        const struct ble_gatt_chr *chr, void *arg);
static int  on_dsc_disc(uint16_t conn_handle, const struct ble_gatt_error *err,
                        uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                        void *arg);
static int  on_role_written(uint16_t conn_handle, const struct ble_gatt_error *err,
                            struct ble_gatt_attr *attr, void *arg);
static int  on_cccd_written(uint16_t conn_handle, const struct ble_gatt_error *err,
                            struct ble_gatt_attr *attr, void *arg);
static int  on_config_read(uint16_t conn_handle, const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr, void *arg);
static int  on_config_write_done(uint16_t conn_handle, const struct ble_gatt_error *err,
                                 struct ble_gatt_attr *attr, void *arg);
static void write_peer_role(void);
static void cccd_subscribe_ibi(void);
static void advance_to_config_or_batt_or_raw_or_diag_or_ready(void);
static void advance_to_batt_or_raw_or_diag_or_ready(void);
static void advance_to_raw_or_diag_or_ready(void);
static void advance_to_diag_or_ready(void);
static void enter_ready(void);

/* ---- log sink + state callback helpers ------------------------------- */

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

/* ---- UUIDs ----------------------------------------------------------- */

/* NimBLE ble_uuid128_t with little-endian 16-byte payload matching
 * narbis_protocol.h NARBIS_*_UUID_BYTES. The byte layout is identical
 * to what Bluedroid's esp_bt_uuid_t.uuid128 used (per the explicit
 * note in narbis_protocol.h:50-60), so we reuse the same constants. */
#define NARBIS_UUID128(name, bytes)                                       \
    static const ble_uuid128_t name = {                                   \
        .u = { .type = BLE_UUID_TYPE_128 },                               \
        .value = bytes,                                                   \
    }

NARBIS_UUID128(UUID_SVC,           NARBIS_SVC_UUID_BYTES);
NARBIS_UUID128(UUID_CHR_IBI,       NARBIS_CHR_IBI_UUID_BYTES);
NARBIS_UUID128(UUID_CHR_BATTERY,   NARBIS_CHR_BATTERY_UUID_BYTES);
NARBIS_UUID128(UUID_CHR_PEER_ROLE, NARBIS_CHR_PEER_ROLE_UUID_BYTES);
NARBIS_UUID128(UUID_CHR_CONFIG,    NARBIS_CHR_CONFIG_UUID_BYTES);
NARBIS_UUID128(UUID_CHR_CONFIG_WRITE, NARBIS_CHR_CONFIG_WRITE_UUID_BYTES);
NARBIS_UUID128(UUID_CHR_RAW,       NARBIS_CHR_RAW_PPG_UUID_BYTES);
NARBIS_UUID128(UUID_CHR_DIAG,      NARBIS_CHR_DIAGNOSTICS_UUID_BYTES);

static const ble_uuid16_t UUID_CCCD = BLE_UUID16_INIT(BLE_UUID_CCCD);

/* ---- Adv parsing for general scan ----------------------------------- */

static bool adv_contains_narbis_svc(const struct ble_hs_adv_fields *fields) {
    for (int i = 0; i < fields->num_uuids128; i++) {
        if (ble_uuid_cmp(&fields->uuids128[i].u, &UUID_SVC.u) == 0) {
            return true;
        }
    }
    return false;
}

/* ---- Scan ----------------------------------------------------------- */

static struct ble_gap_disc_params build_disc_params(void) {
    struct ble_gap_disc_params p = {0};
    p.itvl   = 0x50;        /* 50 ms */
    p.window = 0x30;        /* 30 ms */
    p.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
    p.passive = 0;          /* active scan */
    p.limited = 0;
    p.filter_duplicates = 0;
    return p;
}

static bool host_synced(void) {
    if (S.own_addr_resolved) return true;
    int rc = ble_hs_id_infer_auto(0, &S.own_addr_type);
    if (rc != 0) return false;
    S.own_addr_resolved = true;
    return true;
}

static void start_scan_directed(void) {
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.state = ST_SCANNING_DIRECTED;
    S.scan_attempts++;
    int64_t now = esp_timer_get_time();
    int last_seen_s = (S.last_seen_us == 0) ? -1
                     : (int)((now - S.last_seen_us) / 1000000);
    cb_log("central: scanning attempt %lu, last seen %d s ago (directed)",
           (unsigned long)S.scan_attempts, last_seen_s);
    if (!host_synced()) {
        cb_log("central: host not synced — will retry via backoff");
        schedule_reconnect_backoff();
        return;
    }
    struct ble_gap_disc_params dp = build_disc_params();
    int rc = ble_gap_disc(S.own_addr_type, SCAN_DIRECTED_MS, &dp,
                          gap_event_cb, NULL);
    if (rc != 0) {
        cb_log("central: ble_gap_disc rc=%d — backing off", rc);
        schedule_reconnect_backoff();
    }
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
    if (!host_synced()) {
        cb_log("central: host not synced — will retry via backoff");
        schedule_reconnect_backoff();
        return;
    }
    struct ble_gap_disc_params dp = build_disc_params();
    int rc = ble_gap_disc(S.own_addr_type, SCAN_GENERAL_MS, &dp,
                          gap_event_cb, NULL);
    if (rc != 0) {
        cb_log("central: ble_gap_disc rc=%d — backing off", rc);
        schedule_reconnect_backoff();
    }
}

static void schedule_reconnect_backoff(void) {
    S.state = ST_BACKOFF;
    cb_log("central: backoff %d ms before next scan", RECONNECT_BACKOFF_MS);
    if (S.backoff_timer) {
        esp_timer_start_once(S.backoff_timer, (uint64_t)RECONNECT_BACKOFF_MS * 1000ULL);
    }
}

/* ---- Watchdog ------------------------------------------------------- */

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
    if (S.state < ST_CONNECTING || S.state >= ST_READY) return;
    S.wd_fires++;
    cb_log("central: connect watchdog %dms in state=%s — force recovery",
           CONNECT_WATCHDOG_MS, state_name(S.state));
    /* If we have an established link, terminate it. Otherwise (still in
     * ST_CONNECTING with no conn_handle yet), cancel the connect attempt. */
    if (S.conn_handle != CONN_HANDLE_NONE) {
        (void)ble_gap_terminate(S.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    } else {
        (void)ble_gap_conn_cancel();
    }
    S.peer_connected = false;
    emit_state(false);
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

/* ---- Connect initiation --------------------------------------------- */

static void initiate_connect(void) {
    S.state = ST_CONNECTING;
    S.last_seen_us = esp_timer_get_time();
    ble_addr_t peer = {0};
    peer.type = S.earclip_addr_type;
    memcpy(peer.val, S.earclip_mac, 6);
    int rc = ble_gap_connect(S.own_addr_type, &peer, 30000 /* ms */,
                             NULL /* default conn params */,
                             gap_event_cb, NULL);
    if (rc != 0) {
        cb_log("central: ble_gap_connect rc=%d — backing off", rc);
        schedule_reconnect_backoff();
    } else {
        arm_connect_watchdog();
    }
}

/* ---- Chain step 2: MTU exchange ------------------------------------- */

static int on_mtu_complete(uint16_t conn_handle, const struct ble_gatt_error *err,
                           uint16_t mtu, void *arg) {
    (void)arg;
    if (conn_handle != S.conn_handle) return 0;
    if (err && err->status != 0) {
        cb_log("central: mtu_exchange status=%d", err->status);
        /* Continue anyway — service discovery works at default MTU. */
    } else {
        S.mtus++;
        cb_log("central: chain 2/9 MTU=%u", mtu);
    }
    return 0;
}

/* ---- Chain step 3-4: service discovery ------------------------------ */

static int on_svc_disc(uint16_t conn_handle, const struct ble_gatt_error *err,
                       const struct ble_gatt_svc *svc, void *arg) {
    (void)arg;
    if (conn_handle != S.conn_handle) return 0;

    if (err && err->status == 0 && svc != NULL) {
        S.searches++;
        S.svc_start_handle = svc->start_handle;
        S.svc_end_handle   = svc->end_handle;
        cb_log("central: chain 3/9 SEARCH_RES %u..%u",
               S.svc_start_handle, S.svc_end_handle);
        return 0;
    }

    if (err && err->status == BLE_HS_EDONE) {
        if (S.svc_start_handle == 0) {
            cb_log("central: chain 4/9 SEARCH_CMPL svc-not-found");
            (void)ble_gap_terminate(S.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        cb_log("central: chain 4/9 SEARCH_CMPL ok");
        /* Kick off characteristic discovery for the whole service range. */
        int rc = ble_gattc_disc_all_chrs(S.conn_handle,
                                         S.svc_start_handle,
                                         S.svc_end_handle,
                                         on_chr_disc, NULL);
        if (rc != 0) cb_log("central: disc_all_chrs rc=%d", rc);
        return 0;
    }

    if (err && err->status != 0 && err->status != BLE_HS_EDONE) {
        cb_log("central: svc_disc status=%d", err->status);
    }
    return 0;
}

/* ---- Chain: characteristic discovery -------------------------------- */

static int on_chr_disc(uint16_t conn_handle, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg) {
    (void)arg;
    if (conn_handle != S.conn_handle) return 0;

    if (err && err->status == 0 && chr != NULL) {
        const ble_uuid_t *u = &chr->uuid.u;
        uint16_t h = chr->val_handle;
        if      (ble_uuid_cmp(u, &UUID_CHR_IBI.u)          == 0) S.hdl_ibi          = h;
        else if (ble_uuid_cmp(u, &UUID_CHR_BATTERY.u)      == 0) S.hdl_battery      = h;
        else if (ble_uuid_cmp(u, &UUID_CHR_PEER_ROLE.u)    == 0) S.hdl_peer_role    = h;
        else if (ble_uuid_cmp(u, &UUID_CHR_CONFIG.u)       == 0) S.hdl_config       = h;
        else if (ble_uuid_cmp(u, &UUID_CHR_CONFIG_WRITE.u) == 0) S.hdl_config_write = h;
        else if (ble_uuid_cmp(u, &UUID_CHR_RAW.u)          == 0) S.hdl_raw          = h;
        else if (ble_uuid_cmp(u, &UUID_CHR_DIAG.u)         == 0) S.hdl_diag         = h;
        return 0;
    }

    if (err && err->status == BLE_HS_EDONE) {
        /* Build the dsc_targets table — one entry per notify char that
         * was actually present. Sort by val_handle so the next_chr_handle
         * computation works (each target's range ends at the next
         * target's val_handle, or svc_end_handle for the last). */
        S.dsc_target_count = 0;
        struct {
            uint16_t *vh;
            uint16_t *ch;
        } cands[] = {
            { &S.hdl_ibi,     &S.hdl_ibi_cccd     },
            { &S.hdl_battery, &S.hdl_battery_cccd },
            { &S.hdl_config,  &S.hdl_config_cccd  },
            { &S.hdl_raw,     &S.hdl_raw_cccd     },
            { &S.hdl_diag,    &S.hdl_diag_cccd    },
        };
        const int N = sizeof(cands) / sizeof(cands[0]);
        for (int i = 0; i < N; i++) {
            if (*cands[i].vh != 0) {
                S.dsc_targets[S.dsc_target_count].val_handle  = cands[i].vh;
                S.dsc_targets[S.dsc_target_count].cccd_handle = cands[i].ch;
                S.dsc_target_count++;
            }
        }
        /* Insertion sort by val_handle ascending. */
        for (int i = 1; i < S.dsc_target_count; i++) {
            dsc_target_t t = S.dsc_targets[i];
            int j = i - 1;
            while (j >= 0 && *S.dsc_targets[j].val_handle > *t.val_handle) {
                S.dsc_targets[j + 1] = S.dsc_targets[j];
                j--;
            }
            S.dsc_targets[j + 1] = t;
        }
        /* Compute next_chr_handle for each target (range upper bound). */
        for (int i = 0; i < S.dsc_target_count; i++) {
            S.dsc_targets[i].next_chr_handle =
                (i + 1 < S.dsc_target_count)
                    ? *S.dsc_targets[i + 1].val_handle
                    : (uint16_t)(S.svc_end_handle + 1);
        }

        /* Discover all dscs in the whole service range in one shot. The
         * dsc cb sorts each 0x2902 it sees into the right char's CCCD slot
         * by handle range. */
        if (S.svc_start_handle + 1 > S.svc_end_handle) {
            /* No room for descriptors — skip ahead. */
            write_peer_role();
            return 0;
        }
        int rc = ble_gattc_disc_all_dscs(S.conn_handle,
                                         (uint16_t)(S.svc_start_handle + 1),
                                         S.svc_end_handle,
                                         on_dsc_disc, NULL);
        if (rc != 0) {
            cb_log("central: disc_all_dscs rc=%d", rc);
            /* Best-effort fallthrough: try to subscribe without CCCD
             * handles. cccd_subscribe will skip when handle is 0. */
            write_peer_role();
        }
        return 0;
    }

    if (err && err->status != 0 && err->status != BLE_HS_EDONE) {
        cb_log("central: chr_disc status=%d", err->status);
    }
    return 0;
}

/* ---- Chain: descriptor discovery (CCCDs) ---------------------------- */

static int on_dsc_disc(uint16_t conn_handle, const struct ble_gatt_error *err,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg) {
    (void)arg; (void)chr_val_handle;
    if (conn_handle != S.conn_handle) return 0;

    if (err && err->status == 0 && dsc != NULL) {
        if (ble_uuid_cmp(&dsc->uuid.u, &UUID_CCCD.u) == 0) {
            /* Find which char this CCCD belongs to. NimBLE's dsc disc
             * walks ascending; the owning char is the one whose
             * [val_handle+1, next_chr_handle-1] range contains dsc->handle.
             *
             * Guard against overwrite: if an unsubscribed notify char sits
             * in the gap between two subscribed targets (concrete example:
             * the earclip exposes IBI, SQI, RAW... and we don't track SQI),
             * SQI's CCCD falls inside IBI's range and would clobber IBI's
             * real CCCD (which was just assigned on the prior dsc cb).
             * Per BLE spec the CCCD comes immediately after the val handle,
             * so the FIRST CCCD seen in any target's range is the right one. */
            for (int i = 0; i < S.dsc_target_count; i++) {
                uint16_t lo = (uint16_t)(*S.dsc_targets[i].val_handle + 1);
                uint16_t hi = (uint16_t)(S.dsc_targets[i].next_chr_handle - 1);
                if (dsc->handle >= lo && dsc->handle <= hi) {
                    if (*S.dsc_targets[i].cccd_handle == 0) {
                        *S.dsc_targets[i].cccd_handle = dsc->handle;
                    }
                    break;
                }
            }
        }
        return 0;
    }

    if (err && err->status == BLE_HS_EDONE) {
        /* All descriptors discovered. Log what we got, then start the
         * write/subscribe chain. */
        cb_log("handles ibi=%u/%u batt=%u/%u role=%u cfg=%u/%u cfgw=%u raw=%u/%u",
               S.hdl_ibi, S.hdl_ibi_cccd,
               S.hdl_battery, S.hdl_battery_cccd,
               S.hdl_peer_role,
               S.hdl_config, S.hdl_config_cccd, S.hdl_config_write,
               S.hdl_raw, S.hdl_raw_cccd);
        write_peer_role();
        return 0;
    }

    if (err && err->status != 0 && err->status != BLE_HS_EDONE) {
        cb_log("central: dsc_disc status=%d", err->status);
    }
    return 0;
}

/* ---- Chain step 5: write peer-role ---------------------------------- */

static void write_peer_role(void) {
    if (S.hdl_peer_role == 0) {
        ESP_LOGW(TAG, "no peer-role char; earclip is older firmware");
        S.state = ST_SUBSCRIBING_IBI;
        cccd_subscribe_ibi();
        return;
    }
    uint8_t role = (uint8_t)NARBIS_PEER_ROLE_GLASSES;
    S.state = ST_WRITING_ROLE;
    int rc = ble_gattc_write_flat(S.conn_handle, S.hdl_peer_role,
                                  &role, 1, on_role_written, NULL);
    if (rc != 0) ESP_LOGW(TAG, "write peer_role rc=%d", rc);
}

static int on_role_written(uint16_t conn_handle, const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr, void *arg) {
    (void)arg; (void)attr;
    if (conn_handle != S.conn_handle) return 0;
    if (err && err->status != 0) {
        cb_log("central: write_role status=%d", err->status);
    }
    cb_log("central: chain 5/9 WRITE_ROLE_ACK");
    S.state = ST_SUBSCRIBING_IBI;
    cccd_subscribe_ibi();
    return 0;
}

/* ---- Chain steps 6-9: CCCD subscribe chain -------------------------- */

static int cccd_write(uint16_t cccd_handle, bool enable,
                      ble_gatt_attr_fn *cb) {
    if (cccd_handle == 0) return -1;
    uint8_t val[2] = { (uint8_t)(enable ? 0x01 : 0x00), 0x00 };
    return ble_gattc_write_flat(S.conn_handle, cccd_handle, val, 2, cb, NULL);
}

static void cccd_subscribe_ibi(void) {
    if (S.hdl_ibi_cccd == 0) {
        ESP_LOGW(TAG, "no IBI CCCD; skipping to READY");
        enter_ready();
        return;
    }
    cccd_write(S.hdl_ibi_cccd, true, on_cccd_written);
}

static void advance_to_config_or_batt_or_raw_or_diag_or_ready(void) {
    if (S.hdl_config_cccd) {
        S.state = ST_SUBSCRIBING_CONFIG;
        cccd_write(S.hdl_config_cccd, true, on_cccd_written);
        return;
    }
    advance_to_batt_or_raw_or_diag_or_ready();
}

/* Initial one-shot CONFIG read so dashboard's ConfigPanel populates
 * immediately (earclip only notifies CONFIG on changes). */
static void read_config_initial(void) {
    S.state = ST_READING_CONFIG_INITIAL;
    int rc = ble_gattc_read(S.conn_handle, S.hdl_config, on_config_read, NULL);
    if (rc != 0) {
        cb_log("central: config initial read rc=%d, advancing", rc);
        advance_to_batt_or_raw_or_diag_or_ready();
    }
}

static int on_config_read(uint16_t conn_handle, const struct ble_gatt_error *err,
                          struct ble_gatt_attr *attr, void *arg) {
    (void)arg;
    if (conn_handle != S.conn_handle) return 0;
    if (err && err->status == 0 && attr && attr->om && S.config_cb) {
        /* Pull the mbuf contents into a flat buffer for the cb. */
        uint16_t len = OS_MBUF_PKTLEN(attr->om);
        if (len > 0 && len <= 512) {
            uint8_t *buf = malloc(len);
            if (buf) {
                uint16_t copied = 0;
                if (ble_hs_mbuf_to_flat(attr->om, buf, len, &copied) == 0) {
                    S.config_cb(buf, copied);
                    cb_log("central: config read ok (%u B)", copied);
                }
                free(buf);
            }
        }
    } else if (err && err->status != 0) {
        cb_log("central: config read status=%d", err->status);
    }
    if (S.state == ST_READING_CONFIG_INITIAL) {
        advance_to_batt_or_raw_or_diag_or_ready();
    }
    return 0;
}

static void advance_to_batt_or_raw_or_diag_or_ready(void) {
    if (S.hdl_battery_cccd) {
        S.state = ST_SUBSCRIBING_BATT;
        cccd_write(S.hdl_battery_cccd, true, on_cccd_written);
        return;
    }
    advance_to_raw_or_diag_or_ready();
}

static void advance_to_raw_or_diag_or_ready(void) {
    if (S.raw_enabled && S.hdl_raw_cccd) {
        S.state = ST_SUBSCRIBING_RAW;
        if (cccd_write(S.hdl_raw_cccd, true, on_cccd_written) != 0) {
            ESP_LOGW(TAG, "raw subscribe failed; advancing");
            advance_to_diag_or_ready();
        }
        return;
    }
    advance_to_diag_or_ready();
}

static void advance_to_diag_or_ready(void) {
    if (S.hdl_diag_cccd) {
        S.state = ST_SUBSCRIBING_DIAG;
        if (cccd_write(S.hdl_diag_cccd, true, on_cccd_written) != 0) {
            ESP_LOGW(TAG, "diag subscribe failed; entering READY");
            enter_ready();
        }
        return;
    }
    enter_ready();
}

static int on_cccd_written(uint16_t conn_handle, const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr, void *arg) {
    (void)arg; (void)attr;
    if (conn_handle != S.conn_handle) return 0;
    if (err && err->status != 0) {
        cb_log("central: cccd write status=%d (st=%s)",
               err->status, state_name(S.state));
    }
    if (S.state == ST_SUBSCRIBING_IBI) {
        cb_log("central: chain 6/9 SUB_IBI_ACK");
        advance_to_config_or_batt_or_raw_or_diag_or_ready();
    } else if (S.state == ST_SUBSCRIBING_CONFIG) {
        cb_log("central: chain 7/9 SUB_CFG_ACK");
        read_config_initial();
    } else if (S.state == ST_SUBSCRIBING_BATT) {
        cb_log("central: chain 8/9 SUB_BATT_ACK");
        advance_to_raw_or_diag_or_ready();
    } else if (S.state == ST_SUBSCRIBING_RAW) {
        advance_to_diag_or_ready();
    } else if (S.state == ST_SUBSCRIBING_DIAG) {
        enter_ready();
    }
    /* Runtime CCCD writes (raw toggle in ST_READY) intentionally fall
     * through with no state transition. */
    return 0;
}

/* ---- Chain step 9: READY ------------------------------------------- */

static void enter_ready(void) {
    S.state = ST_READY;
    cancel_connect_watchdog();
    cb_log("central: chain 9/9 READY (IBI%s%s%s%s)",
           S.hdl_config_cccd  ? "+cfg"  : "",
           S.hdl_battery_cccd ? "+batt" : "",
           (S.raw_enabled && S.hdl_raw_cccd) ? "+raw" : "",
           S.hdl_diag_cccd ? "+diag" : "");
    emit_state(true);
}

/* ---- GAP / GATT event handler --------------------------------------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    (void)arg;
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        S.scan_advs_seen++;
        const struct ble_gap_disc_desc *r = &event->disc;
        if (S.state == ST_SCANNING_DIRECTED) {
            if (S.earclip_known && memcmp(r->addr.val, S.earclip_mac, 6) == 0) {
                S.earclip_addr_type = r->addr.type;
                (void)ble_gap_disc_cancel();
                S.last_seen_us = esp_timer_get_time();
                initiate_connect();
            }
        } else if (S.state == ST_SCANNING_GENERAL) {
            struct ble_hs_adv_fields fields;
            if (ble_hs_adv_parse_fields(&fields, r->data, r->length_data) == 0 &&
                adv_contains_narbis_svc(&fields)) {
                S.scan_advs_matched++;
                if (S.scan_advs_matched == 1) {
                    /* NimBLE addr.val is little-endian wire order; print
                     * val[5..0] to match display convention (matches
                     * Bluedroid era logs + earclip serial output). */
                    cb_log("central: matched narbis adv %02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
                           r->addr.val[5], r->addr.val[4], r->addr.val[3],
                           r->addr.val[2], r->addr.val[1], r->addr.val[0], r->rssi);
                }
                if (!S.best.valid || r->rssi > S.best.rssi) {
                    S.best.valid = true;
                    S.best.rssi = r->rssi;
                    S.best.addr_type = r->addr.type;
                    memcpy(S.best.bda, r->addr.val, 6);
                }
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (S.state == ST_SCANNING_GENERAL) {
            cb_log("central: scan done, %u adv seen, %u matched narbis",
                   (unsigned)S.scan_advs_seen, (unsigned)S.scan_advs_matched);
            if (S.best.valid) {
                memcpy(S.earclip_mac, S.best.bda, 6);
                S.earclip_addr_type = S.best.addr_type;
                S.earclip_known = true;
                (void)nvs_write_earclip(S.earclip_mac);
                cb_log("central: best rssi=%d addr_type=%d, persisted",
                       S.best.rssi, (int)S.earclip_addr_type);
                initiate_connect();
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
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            central_state_t prev_state = S.state;
            if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
            S.conn_handle = event->connect.conn_handle;
            S.last_seen_us = esp_timer_get_time();
            if (prev_state == ST_READY) {
                /* Silent re-cycle without DISCONNECT — reset dedup. */
                emit_state(false);
            }
            S.state = ST_DISCOVERING;
            S.connects++;
            S.peer_connected = true;
            arm_connect_watchdog();
            cb_log("central: chain 1/9 CONNECT conn=%u (was %s)",
                   (unsigned)S.conn_handle, state_name(prev_state));
            /* Kick off MTU exchange and service discovery. MTU is
             * informational; discovery starts immediately and runs at
             * default MTU until the exchange completes. */
            (void)ble_gattc_exchange_mtu(S.conn_handle, on_mtu_complete, NULL);
            (void)ble_gap_set_prefered_le_phy(S.conn_handle,
                                              BLE_GAP_LE_PHY_2M_MASK,
                                              BLE_GAP_LE_PHY_2M_MASK, 0);
            int rc = ble_gattc_disc_svc_by_uuid(S.conn_handle, &UUID_SVC.u,
                                                on_svc_disc, NULL);
            if (rc != 0) cb_log("central: disc_svc rc=%d", rc);
        } else {
            cb_log("central: connect failed status=%d", event->connect.status);
            schedule_reconnect_backoff();
        }
        return 0;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        cb_log("central: phy update conn=%u tx=%d rx=%d status=%d",
               (unsigned)event->phy_updated.conn_handle,
               event->phy_updated.tx_phy, event->phy_updated.rx_phy,
               event->phy_updated.status);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        S.disconnects++;
        S.peer_connected = false;
        cb_log("central: disconnected reason=0x%04x",
               event->disconnect.reason);
        cancel_connect_watchdog();
        emit_state(false);
        S.conn_handle = CONN_HANDLE_NONE;
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
        S.dsc_target_count = 0;
        if (S.paused) {
            S.state = ST_IDLE;
        } else if (S.state != ST_SCANNING_DIRECTED && S.state != ST_SCANNING_GENERAL) {
            if (S.earclip_known) start_scan_directed();
            else                 start_scan_general();
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        /* NimBLE delivers MTU as both a per-call cb (on_mtu_complete) and
         * a gap_event. Either path increments S.mtus / logs chain 2/9. We
         * only count the per-call cb path to avoid double-counting. */
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* event->notify_rx is an anonymous struct inside the union; access
         * its members inline rather than via a typed pointer. */
        if (event->notify_rx.conn_handle != S.conn_handle ||
            event->notify_rx.om == NULL) return 0;
        uint16_t attr_handle = event->notify_rx.attr_handle;
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        uint8_t buf[256];
        if (len > sizeof(buf)) return 0;
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &copied) != 0) {
            return 0;
        }

        if (attr_handle == S.hdl_ibi && copied >= sizeof(narbis_ibi_payload_t)) {
            narbis_ibi_payload_t pl;
            memcpy(&pl, buf, sizeof(pl));
            if (S.ibi_cb) S.ibi_cb(pl.ibi_ms, pl.confidence_x100, pl.flags);
            S.notify_ibi_count++;
        } else if (attr_handle == S.hdl_battery &&
                   copied >= sizeof(narbis_battery_payload_t)) {
            narbis_battery_payload_t pl;
            memcpy(&pl, buf, sizeof(pl));
            if (S.batt_cb) S.batt_cb(pl.soc_pct, pl.mv, pl.charging);
            S.notify_batt_count++;
        } else if (attr_handle == S.hdl_config && S.config_cb) {
            S.config_cb(buf, copied);
            S.notify_config_count++;
        } else if (attr_handle == S.hdl_raw && S.raw_cb) {
            S.raw_cb(buf, copied);
            S.notify_raw_count++;
        } else if (attr_handle == S.hdl_diag && S.diag_cb) {
            S.diag_cb(buf, copied);
            S.notify_diag_count++;
        } else {
            S.notify_other_count++;
        }
        S.last_seen_us = esp_timer_get_time();

        /* Subscribe-chain self-heal — surgical, per characteristic.
         *
         * The earclip ALWAYS sends an initial CONFIG notify the moment
         * its CCCD is enabled (and similarly for other notify chars
         * after their CCCDs go live). Under NimBLE the matching ATT
         * Write Response usually arrives right after, but the order
         * between Notify and Write Response is not guaranteed — at
         * weak RSSI we sometimes see the Notify first.
         *
         * The previous logic jumped straight to READY on any notify in
         * any subscribe state, which silently skipped the remaining
         * subscribes (bench log on PR #22: SUB_CFG triggered force-
         * READY, leaving BATT/RAW/DIAG never enabled → rx batt=0
         * raw=0 diag=0 even after the chain "completed").
         *
         * Correct behavior: advance only one step, and only when the
         * notify is on the exact characteristic we're currently
         * subscribing. Receiving a CONFIG notify in SUB_CFG means CFG
         * is live → advance to SUB_BATT (etc.). The Write Response cb
         * (on_cccd_written) does the same advance when it arrives
         * first, and is idempotent against the state check there. */
        if (S.state == ST_SUBSCRIBING_IBI && attr_handle == S.hdl_ibi) {
            cb_log("central: IBI notify mid-SUB_IBI — advance");
            advance_to_config_or_batt_or_raw_or_diag_or_ready();
        } else if (S.state == ST_SUBSCRIBING_CONFIG &&
                   attr_handle == S.hdl_config) {
            cb_log("central: CFG notify mid-SUB_CFG — advance");
            read_config_initial();
        } else if (S.state == ST_READING_CONFIG_INITIAL &&
                   attr_handle == S.hdl_config) {
            cb_log("central: CFG notify mid-READ_CFG — advance");
            advance_to_batt_or_raw_or_diag_or_ready();
        } else if (S.state == ST_SUBSCRIBING_BATT &&
                   attr_handle == S.hdl_battery) {
            cb_log("central: BATT notify mid-SUB_BATT — advance");
            advance_to_raw_or_diag_or_ready();
        } else if (S.state == ST_SUBSCRIBING_RAW &&
                   attr_handle == S.hdl_raw) {
            cb_log("central: RAW notify mid-SUB_RAW — advance");
            advance_to_diag_or_ready();
        } else if (S.state == ST_SUBSCRIBING_DIAG &&
                   attr_handle == S.hdl_diag) {
            cb_log("central: DIAG notify mid-SUB_DIAG — advance");
            enter_ready();
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* Earclip drives BATCHED / LOW_LATENCY profile from its side. Log
         * the resulting params for diagnostic visibility. */
        if (S.conn_handle != CONN_HANDLE_NONE) {
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(S.conn_handle, &desc) == 0) {
                cb_log("central: conn_update itvl=%u lat=%u tout=%u",
                       (unsigned)desc.conn_itvl,
                       (unsigned)desc.conn_latency,
                       (unsigned)desc.supervision_timeout);
            }
        }
        return 0;

    default:
        return 0;
    }
}

/* ---- Public API ----------------------------------------------------- */

esp_err_t narbis_central_init(narbis_central_ibi_cb_t     ibi_cb,
                              narbis_central_battery_cb_t batt_cb) {
    memset(&S, 0, sizeof(S));
    S.ibi_cb  = ibi_cb;
    S.batt_cb = batt_cb;
    S.conn_handle = CONN_HANDLE_NONE;
    S.raw_enabled = true;

    const esp_timer_create_args_t targs = {
        .callback = backoff_timer_cb,
        .name = "narbis_central_backoff",
    };
    esp_err_t err = esp_timer_create(&targs, &S.backoff_timer);
    if (err != ESP_OK) return err;

    const esp_timer_create_args_t wargs = {
        .callback = connect_watchdog_cb,
        .name = "narbis_central_watchdog",
    };
    err = esp_timer_create(&wargs, &S.connect_watchdog);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "central init ok (NimBLE)");
    return ESP_OK;
}

esp_err_t narbis_central_start(void) {
    if (S.paused) {
        ESP_LOGI(TAG, "central: resuming (was paused)");
        S.paused = false;
    }
    uint8_t mac[6];
    if (nvs_read_earclip(mac) == ESP_OK) {
        memcpy(S.earclip_mac, mac, 6);
        /* Address type unknown until we see the first adv from this peer;
         * BLE_ADDR_PUBLIC is the sane default for ESP-IDF v5 NimBLE which
         * burns a public BD_ADDR into eFuse on production silicon. The
         * actual addr_type is captured from the scan match below. */
        S.earclip_addr_type = BLE_ADDR_PUBLIC;
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

esp_err_t narbis_central_stop(void) {
    ESP_LOGI(TAG, "central: stop (HR source switched away from earclip)");
    S.paused = true;
    cancel_connect_watchdog();
    S.peer_connected = false;
    emit_state(false);
    if (S.conn_handle != CONN_HANDLE_NONE) {
        (void)ble_gap_terminate(S.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    /* Cancel any in-flight scan. ble_gap_disc_cancel is a no-op if no
     * scan is active. */
    (void)ble_gap_disc_cancel();
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.scan_attempts = 0;
    /* Preserve earclip_known + earclip_mac for resume on start(). */
    return ESP_OK;
}

esp_err_t narbis_central_forget(void) {
    ESP_LOGW(TAG, "central: forget paired earclip");
    cancel_connect_watchdog();
    S.peer_connected = false;
    emit_state(false);
    if (S.conn_handle != CONN_HANDLE_NONE) {
        (void)ble_gap_terminate(S.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    (void)ble_gap_disc_cancel();
    if (S.backoff_timer) esp_timer_stop(S.backoff_timer);
    S.earclip_known = false;
    memset(S.earclip_mac, 0, 6);
    S.scan_attempts = 0;
    return nvs_erase_earclip();
}

bool narbis_central_is_connected(void) {
    /* Link state, not app state machine — matches the 0xF6 relay-link
     * badge semantic the dashboard expects. */
    return S.peer_connected;
}

uint16_t narbis_central_get_conn_handle(void) {
    /* Read-only view of S.conn_handle so callers can pass it to
     * ble_gap_conn_rssi() without poking module internals. */
    return S.conn_handle;
}

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
    if (S.state != ST_READY || S.hdl_raw_cccd == 0) {
        cb_log("central: raw subscribe %s (latched, will apply on connect)",
               enabled ? "on" : "off");
        return ESP_OK;
    }
    int rc = cccd_write(S.hdl_raw_cccd, enabled, on_cccd_written);
    cb_log("central: raw subscribe %s rc=%d", enabled ? "on" : "off", rc);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
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
    case ST_SUBSCRIBING_DIAG:       return "SUB_DIAG";
    case ST_READY:                  return "READY";
    case ST_BACKOFF:                return "BACKOFF";
    }
    return "?";
}

void narbis_central_emit_diag(void) {
    cb_log("relay state=%s ready=%d", state_name(S.state),
           narbis_central_is_connected() ? 1 : 0);
    cb_log("hdl ibi=%u/%u batt=%u/%u role=%u",
           S.hdl_ibi, S.hdl_ibi_cccd,
           S.hdl_battery, S.hdl_battery_cccd,
           S.hdl_peer_role);
    cb_log("hdl cfg=%u/%u cfgw=%u raw=%u/%u diag=%u/%u",
           S.hdl_config, S.hdl_config_cccd, S.hdl_config_write,
           S.hdl_raw, S.hdl_raw_cccd,
           S.hdl_diag, S.hdl_diag_cccd);
    cb_log("rx ibi=%u cfg=%u batt=%u raw=%u diag=%u other=%u",
           (unsigned)S.notify_ibi_count, (unsigned)S.notify_config_count,
           (unsigned)S.notify_batt_count, (unsigned)S.notify_raw_count,
           (unsigned)S.notify_diag_count, (unsigned)S.notify_other_count);
    cb_log("chain c=%u d=%u mtu=%u srch=%u wdc=%u wda=%u wdf=%u wd_ok=%d",
           (unsigned)S.connects, (unsigned)S.disconnects,
           (unsigned)S.mtus, (unsigned)S.searches,
           (unsigned)S.wd_calls, (unsigned)S.wd_armed, (unsigned)S.wd_fires,
           S.connect_watchdog ? 1 : 0);
}

static int on_config_write_done(uint16_t conn_handle, const struct ble_gatt_error *err,
                                struct ble_gatt_attr *attr, void *arg) {
    (void)conn_handle; (void)attr; (void)arg;
    cb_log("central: config write rc=%d", err ? err->status : 0);
    return 0;
}

esp_err_t narbis_central_write_earclip_config(const uint8_t *bytes, size_t len) {
    if (S.state != ST_READY || S.hdl_config_write == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (bytes == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    int rc = ble_gattc_write_flat(S.conn_handle, S.hdl_config_write,
                                  bytes, (uint16_t)len,
                                  on_config_write_done, NULL);
    cb_log("central: config write dispatch rc=%d (%u B)", rc, (unsigned)len);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}
