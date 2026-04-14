/**
 * =============================================================================
 * Smart Glasses LED Controller v4.4
 * =============================================================================
 * 
 * CHANGELOG:
 *   v4.4 - Fixed critical vTaskDelay(0) bug causing 100% CPU usage and no
 *          visible strobe. Added comprehensive comments throughout.
 *   v4.3 - 40Hz strobe, no breathing (BROKEN - do not use)
 *   v4.2 - Added OTA support to v4.1
 *   v4.1 - Independent strobe/breathing modes
 *   v4.0 - Initial strobe + breathing implementation
 * 
 * HARDWARE:
 *   - ESP32-PICO-V3 (ESP32-U4WDH), 4MB flash, 40MHz crystal
 *   - PWM output: GPIO27 (LCD lens driver)
 *   - Hall sensor: GPIO4 (arm open/close detection)
 *   - Single PWM channel (both lenses tied together in hardware)
 * 
 * TIMING LIMITATION:
 *   FreeRTOS default tick rate is 100Hz (CONFIG_FREERTOS_HZ=100).
 *   This means 1 tick = 10ms minimum delay.
 *   True 40Hz strobe (25ms period) is NOT achievable with 10ms granularity.
 *   Closest achievable: ~33Hz (30ms period: 20ms dark + 10ms clear)
 *   
 *   To get 40Hz, you would need CONFIG_FREERTOS_HZ=1000 in sdkconfig,
 *   but this increases power consumption due to more frequent interrupts.
 * 
 * POWER OPTIMIZATION:
 *   - BLE TX power: -12dBm (lowest practical for ~1m range)
 *   - PWM frequency: 1kHz (lower = less switching losses)
 *   - Deep sleep: ~30µA when Hall sensor triggered
 *   - CPU: 80MHz default (set in sdkconfig if needed)
 * 
 * BLE PROTOCOL:
 *   Service UUID: 0x00FF
 *   
 *   Characteristic 0xFF01 (Control) - Read/Write:
 *     Single byte 0x00-0xFF       → Legacy duty (0=clear, 255=full dark)
 *     [0xA2] [brightness]         → Set strobe brightness 0-100%
 *     [0xA4] [minutes]            → Set session duration 1-60 min
 *     [0xA5] [duty]               → Static override, stops strobe
 *     [0xA6] [0x00]               → Resume strobe session
 *     [0xA7] [0x00]               → Enter deep sleep immediately
 *     [0xA8] [0x00]               → Start OTA mode
 *     [0xA9] [0x00]               → Finish OTA (validate + reboot)
 *     [0xAA] [0x00]               → Cancel OTA
 *     NOTE: All 0xAx commands require 2+ bytes to avoid legacy collision.
 *   
 *   Characteristic 0xFF02 (OTA Data) - Write/WriteNoResponse:
 *     Binary firmware chunks (max 512 bytes per write)
 *   
 *   Characteristic 0xFF03 (Status) - Read/Notify:
 *     0x01                        → OTA ready for data
 *     0x02 [MSB] [MID] [LSB]      → Progress (bytes received, big-endian)
 *     0x03                        → OTA success, rebooting
 *     0x04 [error_code]           → OTA error
 *     0x05                        → OTA cancelled
 * 
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

/* =============================================================================
 * BLE SERVICE CONFIGURATION
 * =============================================================================
 * Using 16-bit UUIDs for simplicity and compatibility with nRF Connect.
 * Full 128-bit UUID would be: 0000XXXX-0000-1000-8000-00805F9B34FB
 */
#define GATTS_SERVICE_UUID      0x00FF      /* Primary service */
#define GATTS_CHAR_UUID_CTRL    0xFF01      /* Control: commands + OTA triggers */
#define GATTS_CHAR_UUID_OTA     0xFF02      /* OTA data: firmware binary chunks */
#define GATTS_CHAR_UUID_STATUS  0xFF03      /* Status: OTA progress notifications */
#define GATTS_NUM_HANDLES       12          /* Service(1) + 3×Char(2) + 3×Value(1) + CCCD(1) */

#define DEVICE_NAME             "Smart_Glasses"
#define TEST_APP_ID             0

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG         (1 << 0)

static const char *TAG = "SG_v4.4";

/* =============================================================================
 * OTA PROTOCOL CONSTANTS
 * =============================================================================
 * Commands are sent to 0xFF01 as multi-byte writes.
 * Status notifications are sent from 0xFF03.
 */
#define OTA_CMD_START           0xA8        /* Enter OTA mode */
#define OTA_CMD_FINISH          0xA9        /* Validate and reboot */
#define OTA_CMD_CANCEL          0xAA        /* Abort OTA */

/* Status codes sent via 0xFF03 notify */
#define OTA_STATUS_READY        0x01        /* Ready to receive firmware */
#define OTA_STATUS_PROGRESS     0x02        /* Progress update (+ 3 bytes) */
#define OTA_STATUS_SUCCESS      0x03        /* Validation passed, rebooting */
#define OTA_STATUS_ERROR        0x04        /* Error occurred (+ 1 byte code) */
#define OTA_STATUS_CANCELLED    0x05        /* User cancelled */

/* Error codes (sent after OTA_STATUS_ERROR) */
#define OTA_ERR_BEGIN_FAIL      0x01        /* esp_ota_begin() failed */
#define OTA_ERR_WRITE_FAIL      0x02        /* esp_ota_write() failed */
#define OTA_ERR_END_FAIL        0x03        /* esp_ota_end() or set_boot failed */
#define OTA_ERR_NOT_IN_OTA      0x04        /* Data received but not in OTA mode */
#define OTA_ERR_PARTITION       0x05        /* No valid OTA partition found */

/* =============================================================================
 * STROBE TIMING CONFIGURATION
 * =============================================================================
 * 
 * CRITICAL: FreeRTOS ticks are 10ms by default (CONFIG_FREERTOS_HZ=100).
 * 
 * vTaskDelay() takes TICKS, not milliseconds!
 * vTaskDelay(ms / portTICK_PERIOD_MS) does INTEGER division.
 * 
 * Examples with portTICK_PERIOD_MS = 10:
 *   vTaskDelay(6 / 10)  = vTaskDelay(0) = NO DELAY = CPU spins at 100%!
 *   vTaskDelay(15 / 10) = vTaskDelay(1) = 10ms delay
 *   vTaskDelay(25 / 10) = vTaskDelay(2) = 20ms delay
 * 
 * For reliable timing, we define delays directly in TICKS, not milliseconds.
 * 
 * Target: ~33Hz strobe (closest to 40Hz achievable with 10ms ticks)
 *   Period: 30ms (3 ticks)
 *   Dark phase: 20ms (2 ticks) = 67% duty
 *   Clear phase: 10ms (1 tick) = 33% duty
 */
#define STROBE_FREQ_HZ          33          /* Actual frequency (for display only) */
#define STROBE_DARK_TICKS       2           /* 20ms dark phase */
#define STROBE_CLEAR_TICKS      1           /* 10ms clear phase */

/* =============================================================================
 * BLE ADVERTISING CONFIGURATION
 * =============================================================================
 */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,             /* 20ms minimum interval */
    .adv_int_max        = 0x40,             /* 40ms maximum interval */
    .adv_type           = ADV_TYPE_IND,     /* Connectable undirected */
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,     /* Advertise on all 3 channels */
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp       = false,
    .include_name       = true,             /* Include device name in adv packet */
    .include_txpower    = false,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* =============================================================================
 * GATT HANDLES AND STATE
 * =============================================================================
 * These handles are assigned dynamically by the BLE stack during service
 * creation. We store them to identify which characteristic is being accessed
 * during write events.
 */
static uint16_t service_handle = 0;
static uint16_t ctrl_char_handle = 0;       /* 0xFF01 handle */
static uint16_t ota_char_handle = 0;        /* 0xFF02 handle */
static uint16_t status_char_handle = 0;     /* 0xFF03 handle */
static uint16_t status_cccd_handle = 0;     /* Client Characteristic Config Descriptor */
static esp_gatt_if_t global_gatts_if = 0;   /* GATT interface for sending notifications */
static uint16_t global_conn_id = 0;         /* Current connection ID */
static bool client_connected = false;
static bool notifications_enabled = false;  /* Has client subscribed to 0xFF03? */

/* =============================================================================
 * OTA STATE
 * =============================================================================
 */
static bool ota_mode = false;               /* Are we currently receiving OTA data? */
static esp_ota_handle_t ota_handle = 0;     /* Handle for OTA write operations */
static const esp_partition_t *ota_partition = NULL;  /* Target partition */
static uint32_t ota_bytes_received = 0;     /* Total bytes received so far */
static uint16_t ota_last_progress = 0;      /* Last progress notification (in 4KB units) */

/* =============================================================================
 * HARDWARE PIN CONFIGURATION
 * =============================================================================
 */
#define HALL_PIN                GPIO_NUM_4  /* Hall sensor input (arm detection) */
#define SLEEP_HALL_WAIT_TIME    5           /* Seconds of Hall HIGH before sleep */

/* PWM Configuration for LCD lens driver */
#define PWM1_MODE               LEDC_LOW_SPEED_MODE
#define PWM1_OUTPUT_IO          27          /* GPIO27 drives the LCD */
#define PWM1_CHANNEL            LEDC_CHANNEL_0
#define PWM1_TIMER              LEDC_TIMER_0
#define PWM1_DUTY_RES           LEDC_TIMER_10_BIT   /* 0-1023 duty range */
#define PWM1_FREQUENCY          1000        /* 1kHz PWM frequency */

/* =============================================================================
 * LCD LENS DEADZONE COMPENSATION
 * =============================================================================
 * The LCD lens doesn't visibly respond below ~40% PWM duty cycle.
 * We remap the 0-100% user range to the 400-1024 raw PWM range.
 * 
 *   User 0%   → Raw 0    (fully clear, PWM off)
 *   User 1%   → Raw 406  (minimum visible tint)
 *   User 100% → Raw 1024 (maximum darkness)
 */
#define PWM_MIN_VISIBLE         400         /* Minimum PWM for visible effect */
#define PWM_MAX                 1024        /* Maximum PWM (10-bit) */

/* =============================================================================
 * SESSION STATE
 * =============================================================================
 */
static uint8_t brightness = 100;            /* Strobe brightness 0-100% */
static uint8_t session_minutes = 10;        /* Session duration */
static uint32_t session_start_tick = 0;     /* Tick count when session started */
static uint8_t session_active = 0;          /* Is strobe session running? */
static uint8_t session_ended = 0;           /* Flag to trigger deep sleep */
static uint8_t override_active = 0;         /* Is static override mode active? */
static uint8_t override_duty = 0;           /* Static duty when override active */


/* =============================================================================
 * PWM FUNCTIONS
 * =============================================================================
 */

/**
 * Initialize LEDC PWM peripheral for lens control.
 * Called once at boot.
 */
static void PWM_Init(void)
{
    /* Configure PWM timer */
    ledc_timer_config_t pwm1_timer = {
        .speed_mode       = PWM1_MODE,
        .duty_resolution  = PWM1_DUTY_RES,
        .timer_num        = PWM1_TIMER,
        .freq_hz          = PWM1_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm1_timer));

    /* Configure PWM channel */
    ledc_channel_config_t pwm1_channel = {
        .speed_mode     = PWM1_MODE,
        .channel        = PWM1_CHANNEL,
        .timer_sel      = PWM1_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM1_OUTPUT_IO,
        .duty           = 0,                /* Start with lens clear */
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm1_channel));
}

/**
 * Set lens duty cycle with deadzone compensation.
 * 
 * @param duty User-facing duty 0-100%
 *             0 = fully clear (PWM off)
 *             1-100 = remapped to visible range
 */
static void pwm1_setduty(uint32_t duty)
{
    uint32_t raw;
    
    if (duty == 0) {
        /* Special case: 0% = lens fully clear = PWM off */
        raw = 0;
    } else {
        /* Remap 1-100% to 400-1024 raw PWM range */
        raw = PWM_MIN_VISIBLE + (PWM_MAX - PWM_MIN_VISIBLE) * duty / 100;
    }
    
    ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, raw);
    ledc_update_duty(PWM1_MODE, PWM1_CHANNEL);
}


/* =============================================================================
 * OTA FUNCTIONS
 * =============================================================================
 */

/**
 * Send status notification to connected client via 0xFF03.
 * Does nothing if client not connected or notifications not enabled.
 */
static void ota_notify_status(uint8_t *data, uint16_t len)
{
    if (!client_connected || !notifications_enabled || global_gatts_if == 0) {
        return;
    }
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id,
                                status_char_handle, len, data, false);
}

/**
 * Enter OTA mode. Called when 0xA8 command received.
 * Finds the next OTA partition and prepares for firmware data.
 */
static void ota_begin(void)
{
    /* Find the "other" OTA partition (not the one we're running from) */
    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition == NULL) {
        ESP_LOGE(TAG, "OTA: No update partition found");
        uint8_t err[] = {OTA_STATUS_ERROR, OTA_ERR_PARTITION};
        ota_notify_status(err, 2);
        return;
    }

    ESP_LOGI(TAG, "OTA: Target partition '%s' at 0x%08x",
             ota_partition->label, (unsigned int)ota_partition->address);

    /* Begin OTA write operation */
    esp_err_t err = esp_ota_begin(ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_begin failed (%s)", esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_BEGIN_FAIL};
        ota_notify_status(status, 2);
        ota_mode = false;
        return;
    }

    /* Reset counters */
    ota_bytes_received = 0;
    ota_last_progress = 0;
    ota_mode = true;
    
    /* Clear lens during OTA (visual feedback that we're in OTA mode) */
    pwm1_setduty(0);

    ESP_LOGI(TAG, "OTA: Ready for data");
    uint8_t status[] = {OTA_STATUS_READY};
    ota_notify_status(status, 1);
}

/**
 * Write a chunk of firmware data. Called for each write to 0xFF02.
 * Sends progress notifications every 4KB.
 */
static void ota_write_chunk(uint8_t *data, uint16_t len)
{
    if (!ota_mode || ota_partition == NULL) {
        ESP_LOGW(TAG, "OTA: Data received but not in OTA mode");
        uint8_t err[] = {OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA};
        ota_notify_status(err, 2);
        return;
    }

    /* Write chunk to flash */
    esp_err_t err = esp_ota_write(ota_handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: Write failed at %lu (%s)",
                 (unsigned long)ota_bytes_received, esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_WRITE_FAIL};
        ota_notify_status(status, 2);
        esp_ota_abort(ota_handle);
        ota_mode = false;
        return;
    }

    ota_bytes_received += len;

    /* Send progress notification every 4KB to avoid flooding BLE */
    uint16_t progress_4kb = (uint16_t)(ota_bytes_received / 4096);
    if (progress_4kb != ota_last_progress) {
        ota_last_progress = progress_4kb;
        
        /* Send 4-byte progress: [0x02] [MSB] [MID] [LSB] */
        uint8_t status[] = {
            OTA_STATUS_PROGRESS,
            (uint8_t)((ota_bytes_received >> 16) & 0xFF),
            (uint8_t)((ota_bytes_received >> 8) & 0xFF),
            (uint8_t)(ota_bytes_received & 0xFF)
        };
        ota_notify_status(status, 4);

        /* Log to console every 32KB */
        if ((ota_bytes_received % (32 * 1024)) == 0) {
            ESP_LOGI(TAG, "OTA: %lu bytes", (unsigned long)ota_bytes_received);
        }
    }
}

/**
 * Finish OTA update. Called when 0xA9 command received.
 * Validates firmware image, sets boot partition, and reboots.
 */
static void ota_finish(void)
{
    if (!ota_mode) {
        ESP_LOGW(TAG, "OTA: Finish called but not in OTA mode");
        return;
    }

    ESP_LOGI(TAG, "OTA: Finishing, total %lu bytes", (unsigned long)ota_bytes_received);

    /* Validate firmware image (checks header, hash, etc.) */
    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: Validation failed (%s)", esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_END_FAIL};
        ota_notify_status(status, 2);
        ota_mode = false;
        return;
    }

    /* Set this partition as boot target */
    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: Set boot partition failed (%s)", esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_END_FAIL};
        ota_notify_status(status, 2);
        ota_mode = false;
        return;
    }

    ESP_LOGI(TAG, "OTA: Success! Rebooting in 2 seconds...");
    uint8_t status[] = {OTA_STATUS_SUCCESS};
    ota_notify_status(status, 1);

    ota_mode = false;
    
    /* Give BLE time to send notification before reboot */
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
}

/**
 * Cancel OTA update. Called when 0xAA command received or client disconnects.
 */
static void ota_cancel(void)
{
    if (ota_mode) {
        ESP_LOGW(TAG, "OTA: Cancelled by user");
        esp_ota_abort(ota_handle);
        ota_mode = false;
        ota_bytes_received = 0;

        uint8_t status[] = {OTA_STATUS_CANCELLED};
        ota_notify_status(status, 1);
    }
}


/* =============================================================================
 * BLE GAP EVENT HANDLER
 * =============================================================================
 * Handles advertising state changes.
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        /* Advertising data configured, start advertising */
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
        
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising started");
        }
        break;
        
    default:
        break;
    }
}


/* =============================================================================
 * BLE GATT EVENT HANDLER
 * =============================================================================
 * Handles service creation, characteristic reads/writes, connections, etc.
 */
static uint8_t char_add_step = 0;  /* Tracks characteristic creation sequence */

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    /* -------------------------------------------------------------------------
     * GATT Application Registered
     * First event after esp_ble_gatts_app_register(). Create service here.
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "GATT app registered");
        global_gatts_if = gatts_if;
        
        /* Set device name (appears in scan results) */
        esp_ble_gap_set_device_name(DEVICE_NAME);
        
        /* Configure advertising data */
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;

        /* Request larger MTU for faster OTA transfers */
        esp_ble_gatt_set_local_mtu(512);

        /* Create the primary service */
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_SERVICE_UUID}},
            },
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLES);
        break;
    }

    /* -------------------------------------------------------------------------
     * Service Created
     * Now add characteristics one at a time.
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(TAG, "Service created, handle %d", param->create.service_handle);
        service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service_handle);

        /* Add first characteristic: Control (0xFF01) */
        esp_bt_uuid_t ctrl_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = GATTS_CHAR_UUID_CTRL},
        };
        esp_gatt_char_prop_t ctrl_prop = ESP_GATT_CHAR_PROP_BIT_WRITE |
                                          ESP_GATT_CHAR_PROP_BIT_READ;
        esp_attr_value_t ctrl_val = { .attr_max_len = 100, .attr_len = 0, .attr_value = NULL };
        char_add_step = 0;
        esp_ble_gatts_add_char(service_handle, &ctrl_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ctrl_prop, &ctrl_val, NULL);
        break;
    }

    /* -------------------------------------------------------------------------
     * Characteristic Added
     * Chain creation of remaining characteristics.
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_ADD_CHAR_EVT: {
        ESP_LOGI(TAG, "Char added step %d, handle %d", char_add_step, param->add_char.attr_handle);

        if (char_add_step == 0) {
            /* Control char added, now add OTA Data char */
            ctrl_char_handle = param->add_char.attr_handle;

            esp_bt_uuid_t ota_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID_OTA},
            };
            /* WriteNR for speed, Write for compatibility */
            esp_gatt_char_prop_t ota_prop = ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
                                             ESP_GATT_CHAR_PROP_BIT_WRITE;
            esp_attr_value_t ota_val = { .attr_max_len = 512, .attr_len = 0, .attr_value = NULL };
            char_add_step = 1;
            esp_ble_gatts_add_char(service_handle, &ota_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ota_prop, &ota_val, NULL);

        } else if (char_add_step == 1) {
            /* OTA char added, now add Status char */
            ota_char_handle = param->add_char.attr_handle;

            esp_bt_uuid_t status_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID_STATUS},
            };
            esp_gatt_char_prop_t status_prop = ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                                                ESP_GATT_CHAR_PROP_BIT_READ;
            esp_attr_value_t status_val = { .attr_max_len = 20, .attr_len = 0, .attr_value = NULL };
            char_add_step = 2;
            esp_ble_gatts_add_char(service_handle, &status_uuid,
                                   ESP_GATT_PERM_READ,
                                   status_prop, &status_val, NULL);

        } else if (char_add_step == 2) {
            /* Status char added, now add CCCD descriptor for notifications */
            status_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Handles: ctrl=%d ota=%d status=%d",
                     ctrl_char_handle, ota_char_handle, status_char_handle);

            /* 
             * CCCD (Client Characteristic Configuration Descriptor) is REQUIRED
             * for notifications to work. Client writes 0x0001 to enable.
             * Bluedroid does NOT auto-create this - we must add it explicitly.
             */
            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
            };
            uint8_t cccd_val[2] = {0x00, 0x00};  /* Default: notifications off */
            esp_attr_value_t cccd_attr = {
                .attr_max_len = 2,
                .attr_len = 2,
                .attr_value = cccd_val,
            };
            esp_ble_gatts_add_char_descr(service_handle, &cccd_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                         &cccd_attr, NULL);
        }
        break;
    }

    /* -------------------------------------------------------------------------
     * Descriptor Added
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        if (param->add_char_descr.status == ESP_GATT_OK) {
            status_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD handle %d - BLE setup complete", status_cccd_handle);
        }
        break;
    }

    /* -------------------------------------------------------------------------
     * Write Event
     * Handle writes to all characteristics.
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_WRITE_EVT: {
        uint16_t handle = param->write.handle;
        uint16_t len = param->write.len;
        uint8_t *value = param->write.value;

        /* ----- CCCD Write (notification enable/disable) ----- */
        if (len == 2 && handle == status_cccd_handle) {
            uint16_t cccd_val = value[0] | (value[1] << 8);
            notifications_enabled = (cccd_val == 0x0001);
            ESP_LOGI(TAG, "Notifications %s", notifications_enabled ? "ENABLED" : "DISABLED");
            
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        /* ----- Control Characteristic (0xFF01) ----- */
        if (handle == ctrl_char_handle && len > 0) {
            /* Always send response first to avoid GATT timeout */
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }

            ESP_LOGI(TAG, "Ctrl write: %d bytes, first=0x%02X", len, value[0]);

            /* 
             * LEGACY SINGLE-BYTE COMMAND
             * Full 0x00-0xFF range reserved for direct duty control.
             * 0x00 = clear, 0xFF = full dark
             * Linear mapping: duty% = byte × 100 / 255
             */
            if (len == 1) {
                uint8_t raw = value[0];
                override_duty = (uint8_t)((uint16_t)raw * 100 / 255);
                override_active = 1;
                session_active = 0;
                pwm1_setduty(override_duty);
                ESP_LOGI(TAG, "Legacy: 0x%02X -> %d%%", raw, override_duty);
            } 
            /* 
             * MULTI-BYTE COMMANDS
             * First byte is command, remaining bytes are parameters.
             * All 0xAx commands require 2+ bytes to avoid collision with legacy.
             */
            else {
                uint8_t cmd = value[0];
                switch (cmd) {
                    
                case 0xA2:  /* Set brightness: [0xA2] [0-100] */
                    if (len >= 2) {
                        brightness = value[1];
                        if (brightness > 100) brightness = 100;
                        override_active = 0;
                        ESP_LOGI(TAG, "Brightness: %d%%", brightness);
                    }
                    break;

                case 0xA4:  /* Set session duration: [0xA4] [minutes] */
                    if (len >= 2) {
                        session_minutes = value[1];
                        if (session_minutes < 1) session_minutes = 1;
                        if (session_minutes > 60) session_minutes = 60;
                        override_active = 0;
                        session_start_tick = xTaskGetTickCount();
                        session_active = 1;
                        session_ended = 0;
                        ESP_LOGI(TAG, "Session: %d min", session_minutes);
                    }
                    break;

                case 0xA5:  /* Static override: [0xA5] [duty] */
                    if (len >= 2) {
                        override_duty = value[1];
                        if (override_duty > 100) override_duty = 100;
                        override_active = 1;
                        session_active = 0;
                        pwm1_setduty(override_duty);
                        ESP_LOGI(TAG, "Override: %d%% (strobe stopped)", override_duty);
                    }
                    break;

                case 0xA6:  /* Resume strobe: [0xA6] [0x00] */
                    override_active = 0;
                    session_start_tick = xTaskGetTickCount();
                    session_active = 1;
                    session_ended = 0;
                    ESP_LOGI(TAG, "Strobe resumed");
                    break;

                case 0xA7:  /* Sleep immediately: [0xA7] [0x00] */
                    ESP_LOGI(TAG, "Sleep command received");
                    session_ended = 1;
                    break;

                case OTA_CMD_START:  /* 0xA8: Enter OTA mode */
                    ESP_LOGI(TAG, "OTA: Start command");
                    session_active = 0;
                    override_active = 0;
                    ota_begin();
                    break;

                case OTA_CMD_FINISH:  /* 0xA9: Finish OTA */
                    ESP_LOGI(TAG, "OTA: Finish command");
                    ota_finish();
                    break;

                case OTA_CMD_CANCEL:  /* 0xAA: Cancel OTA */
                    ESP_LOGI(TAG, "OTA: Cancel command");
                    ota_cancel();
                    break;

                default:
                    ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
                    break;
                }
            }
        }

        /* ----- OTA Data Characteristic (0xFF02) ----- */
        else if (handle == ota_char_handle && len > 0) {
            ota_write_chunk(value, len);
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }

        /* ----- Unknown Handle ----- */
        else {
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        break;
    }

    /* -------------------------------------------------------------------------
     * MTU Exchange Complete
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU negotiated: %d bytes", param->mtu.mtu);
        break;

    /* -------------------------------------------------------------------------
     * Client Connected
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Client connected, conn_id=%d", param->connect.conn_id);
        global_conn_id = param->connect.conn_id;
        client_connected = true;
        notifications_enabled = false;  /* Client must re-enable after reconnect */

        /* Request faster connection parameters for better OTA throughput */
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x10;     /* 20ms max interval */
        conn_params.min_int = 0x08;     /* 10ms min interval */
        conn_params.timeout = 400;      /* 4 second timeout */
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    /* -------------------------------------------------------------------------
     * Client Disconnected
     * ------------------------------------------------------------------------- */
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Client disconnected");
        client_connected = false;
        notifications_enabled = false;

        /* If disconnect during OTA, abort to clean up state */
        if (ota_mode) {
            ESP_LOGW(TAG, "OTA: Aborted due to disconnect");
            esp_ota_abort(ota_handle);
            ota_mode = false;
        }

        /* Restart advertising to allow reconnection */
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

/**
 * Main GATT event dispatcher.
 */
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            global_gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "GATT registration failed");
            return;
        }
    }
    gatts_profile_event_handler(event, gatts_if, param);
}


/* =============================================================================
 * LED STROBE TASK
 * =============================================================================
 * Runs continuously, producing strobe effect when session is active.
 * 
 * CRITICAL FIX (v4.4): Use TICK counts, not milliseconds!
 * vTaskDelay(ms / portTICK_PERIOD_MS) with ms < 10 yields 0 = no delay = CPU 100%!
 */
static void led_task(void *param)
{
    while (1) {
        /* ----- OTA Mode: Pause strobe, lens clear ----- */
        if (ota_mode) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /* ----- Override Mode: Static duty, no strobe ----- */
        if (override_active) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /* ----- No Session: Idle ----- */
        if (!session_active) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /* ----- Check Session Timeout ----- */
        uint32_t now = xTaskGetTickCount();
        uint32_t session_duration_ticks = session_minutes * 60 * 1000 / portTICK_PERIOD_MS;
        uint32_t elapsed_ticks = now - session_start_tick;

        if (elapsed_ticks >= session_duration_ticks) {
            ESP_LOGI(TAG, "Session complete - entering sleep");
            pwm1_setduty(0);
            session_active = 0;
            session_ended = 1;
            continue;
        }

        /* ----- Log Progress Every 30 Seconds ----- */
        static uint32_t last_log = 0;
        if (now - last_log >= (30000 / portTICK_PERIOD_MS)) {
            last_log = now;
            uint32_t remaining_s = (session_duration_ticks - elapsed_ticks) * portTICK_PERIOD_MS / 1000;
            ESP_LOGI(TAG, "Strobe %dHz @ %d%% | %lu seconds remaining",
                     STROBE_FREQ_HZ, brightness, (unsigned long)remaining_s);
        }

        /* ----- Strobe Cycle ----- 
         * 
         * FIXED: Using TICK counts directly instead of milliseconds.
         * 
         * With CONFIG_FREERTOS_HZ=100 (default):
         *   1 tick = 10ms
         *   2 ticks = 20ms dark
         *   1 tick = 10ms clear
         *   Total = 30ms = ~33Hz strobe
         * 
         * This is the closest to 40Hz achievable with 10ms tick resolution.
         */
        pwm1_setduty(brightness);
        vTaskDelay(STROBE_DARK_TICKS);      /* Dark phase: 20ms */

        pwm1_setduty(0);
        vTaskDelay(STROBE_CLEAR_TICKS);     /* Clear phase: 10ms */
    }
}


/* =============================================================================
 * SLEEP FUNCTIONS
 * =============================================================================
 */

/**
 * Enter deep sleep mode. Wakes on Hall sensor LOW (arm opened).
 */
static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep...");
    
    /* Ensure lens is clear before sleep */
    ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, 0);
    ledc_update_duty(PWM1_MODE, PWM1_CHANNEL);
    
    /* Configure wake source: Hall sensor goes LOW when arms open */
    esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
    
    esp_deep_sleep_start();
    /* Never returns - chip resets on wake */
}

/**
 * Check sleep conditions. Called every second from main loop.
 * Triggers sleep on: session end, Hall sensor HIGH for 5+ seconds.
 */
static void check_sleep_condition(void)
{
    static uint32_t low_level_duration = 0;
    static uint32_t last_check_time = 0;
    uint32_t current_time = xTaskGetTickCount();

    /* Only check once per second */
    if (current_time - last_check_time < (1000 / portTICK_PERIOD_MS)) {
        return;
    }
    last_check_time = current_time;

    /* Don't sleep during OTA! */
    if (ota_mode) {
        low_level_duration = 0;
        return;
    }

    /* Session ended flag set by led_task or 0xA7 command */
    if (session_ended) {
        enter_deep_sleep();
    }

    /* Hall sensor HIGH = arms closed = magnet near sensor */
    if (gpio_get_level(HALL_PIN) == 1) {
        low_level_duration++;
        if (low_level_duration >= SLEEP_HALL_WAIT_TIME) {
            ESP_LOGI(TAG, "Hall sensor triggered - sleeping");
            enter_deep_sleep();
        }
    } else {
        low_level_duration = 0;
    }
}


/* =============================================================================
 * MAIN ENTRY POINT
 * =============================================================================
 */
void app_main(void)
{
    esp_err_t ret;

    /* -------------------------------------------------------------------------
     * Early Wake Check
     * If waking from deep sleep with Hall sensor still HIGH (arms closed),
     * go back to sleep immediately without full initialization.
     * ------------------------------------------------------------------------- */
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 || wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        gpio_config_t hall_conf = {
            .pin_bit_mask = 1ULL << HALL_PIN,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&hall_conf);
        vTaskDelay(50 / portTICK_PERIOD_MS);  /* Debounce */
        
        if (gpio_get_level(HALL_PIN) == 1) {
            /* Arms still closed, go back to sleep */
            esp_sleep_enable_timer_wakeup(1000000);  /* 1 second periodic check */
            esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
            esp_deep_sleep_start();
        }
    }

    /* -------------------------------------------------------------------------
     * Boot Banner
     * ------------------------------------------------------------------------- */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "===========================================================");
    ESP_LOGI(TAG, "Smart Glasses v4.4");
    ESP_LOGI(TAG, "  Strobe: %dHz @ 100%% brightness", STROBE_FREQ_HZ);
    ESP_LOGI(TAG, "  Session: %d minutes", session_minutes);
    ESP_LOGI(TAG, "  Partition: %s @ 0x%08x", running->label, (unsigned int)running->address);
    ESP_LOGI(TAG, "  Wake reason: %d", wakeup_reason);
    ESP_LOGI(TAG, "===========================================================");

    /* -------------------------------------------------------------------------
     * Initialize NVS (required for BLE)
     * ------------------------------------------------------------------------- */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* -------------------------------------------------------------------------
     * Initialize BLE Stack
     * ------------------------------------------------------------------------- */
    /* Release Classic BT memory (we only use BLE) */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(TEST_APP_ID));

    /* Set TX power to -12dBm (lowest practical level, ~1m range) */
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N12);

    /* -------------------------------------------------------------------------
     * Initialize Hall Sensor GPIO
     * ------------------------------------------------------------------------- */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HALL_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* -------------------------------------------------------------------------
     * Initialize PWM
     * ------------------------------------------------------------------------- */
    PWM_Init();

    /* -------------------------------------------------------------------------
     * Start Strobe Session
     * ------------------------------------------------------------------------- */
    session_start_tick = xTaskGetTickCount();
    session_active = 1;
    session_ended = 0;

    /* Create LED strobe task (runs on core 0 by default) */
    xTaskCreate(led_task, "led_task", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "Commands: A2=brightness A4=session A5=override A6=resume A7=sleep");
    ESP_LOGI(TAG, "OTA: A8=start A9=finish AA=cancel");
    ESP_LOGI(TAG, "===========================================================");

    /* -------------------------------------------------------------------------
     * Main Loop
     * Just checks sleep conditions. All real work is in led_task.
     * ------------------------------------------------------------------------- */
    while (1) {
        check_sleep_condition();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
