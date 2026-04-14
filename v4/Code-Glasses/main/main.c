/**
 * Smart Glasses LED Controller v4.3
 * 
 * Changes from v4.2:
 *   - Fixed 40Hz strobe (no ramping)
 *   - No breathing pattern - pure strobe only
 *   - Simplified LED task
 * 
 * Features:
 *   - BLE OTA firmware updates
 *   - 40Hz strobe (25ms period: 18.75ms dark, 6.25ms clear)
 *   - Timed sessions with auto-sleep
 *   - PWM1 only (GPIO27)
 *   - Hall sensor sleep/wake
 *   - Legacy single-byte duty commands (0x00-0xFF)
 * 
 * BLE Service: 0x00FF
 *   Characteristic 0xFF01 (Control) - Read/Write:
 *     Single byte (0x00-0xFF)       - Legacy: direct duty (0=clear, 255=dark)
 *     0xA2 [brightness]             - Set strobe brightness 0-100%
 *     0xA4 [duration_minutes]       - Set session duration (1-60 min)
 *     0xA5 [duty]                   - Static override, stops strobe
 *     0xA6 [0x00]                   - Resume strobe
 *     0xA7 [0x00]                   - Enter sleep immediately
 *     0xA8 [0x00]                   - Start OTA mode
 *     0xA9 [0x00]                   - Finish OTA (validate + reboot)
 *     0xAA [0x00]                   - Cancel OTA
 *   Note: All 0xAx commands require 2+ bytes. Single byte = legacy duty.
 *   Characteristic 0xFF02 (OTA Data) - Write/WriteNR:
 *     Binary firmware chunks during OTA
 *   Characteristic 0xFF03 (Status) - Read/Notify:
 *     OTA progress notifications
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

/* ======================== BLE UUIDs ======================== */
#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID_CTRL    0xFF01
#define GATTS_CHAR_UUID_OTA     0xFF02
#define GATTS_CHAR_UUID_STATUS  0xFF03
#define GATTS_NUM_HANDLES       12

#define DEVICE_NAME             "Smart_Glasses"
#define TEST_APP_ID             0

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)

static const char *TAG = "SG_v4.3";

/* ======================== OTA Commands ======================== */
#define OTA_CMD_START           0xA8
#define OTA_CMD_FINISH          0xA9
#define OTA_CMD_CANCEL          0xAA

#define OTA_STATUS_READY        0x01
#define OTA_STATUS_PROGRESS     0x02
#define OTA_STATUS_SUCCESS      0x03
#define OTA_STATUS_ERROR        0x04
#define OTA_STATUS_CANCELLED    0x05

#define OTA_ERR_BEGIN_FAIL      0x01
#define OTA_ERR_WRITE_FAIL      0x02
#define OTA_ERR_END_FAIL        0x03
#define OTA_ERR_NOT_IN_OTA      0x04
#define OTA_ERR_PARTITION       0x05

/* ======================== Strobe Config ======================== */
#define STROBE_HZ               40      /* Fixed 40Hz */
#define STROBE_PERIOD_MS        25      /* 1000/40 = 25ms */
#define STROBE_DARK_MS          19      /* 75% dark (18.75 rounded to 19) */
#define STROBE_CLEAR_MS         6       /* 25% clear (6.25 rounded to 6) */

/* ======================== BLE State ======================== */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp   = false,
    .include_name   = true,
    .include_txpower = false,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static uint16_t service_handle = 0;
static uint16_t ctrl_char_handle = 0;
static uint16_t ota_char_handle = 0;
static uint16_t status_char_handle = 0;
static uint16_t status_cccd_handle = 0;
static esp_gatt_if_t global_gatts_if = 0;
static uint16_t global_conn_id = 0;
static bool client_connected = false;
static bool notifications_enabled = false;

/* ======================== OTA State ======================== */
static bool ota_mode = false;
static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *ota_partition = NULL;
static uint32_t ota_bytes_received = 0;
static uint16_t ota_last_progress = 0;

/* ======================== Hardware Config ======================== */
#define HALL_PIN GPIO_NUM_4
#define SLEEP_HALL_WAIT_TIME 5

#define PWM1_MODE               LEDC_LOW_SPEED_MODE
#define PWM1_OUTPUT_IO          (27)
#define PWM1_CHANNEL            LEDC_CHANNEL_0
#define PWM1_TIMER              LEDC_TIMER_0
#define PWM1_DUTY_RES           LEDC_TIMER_10_BIT
#define PWM1_FREQUENCY          (1000)

/* ======================== Session State ======================== */
static uint8_t brightness = 100;         /* Strobe brightness 0-100% */
static uint8_t session_minutes = 10;
static uint32_t session_start_tick = 0;
static uint8_t session_active = 0;
static uint8_t session_ended = 0;
static uint8_t override_active = 0;
static uint8_t override_duty = 0;

/* ======================== PWM Functions ======================== */
static void PWM_Init(void)
{
    ledc_timer_config_t pwm1_timer = {
        .speed_mode       = PWM1_MODE,
        .duty_resolution  = PWM1_DUTY_RES,
        .timer_num        = PWM1_TIMER,
        .freq_hz          = PWM1_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm1_timer));

    ledc_channel_config_t pwm1_channel = {
        .speed_mode     = PWM1_MODE,
        .channel        = PWM1_CHANNEL,
        .timer_sel      = PWM1_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM1_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm1_channel));
}

#define PWM_MIN_VISIBLE 400
#define PWM_MAX 1024

static void pwm1_setduty(uint32_t duty) {
    uint32_t raw;
    if (duty == 0) {
        raw = 0;
    } else {
        raw = PWM_MIN_VISIBLE + (PWM_MAX - PWM_MIN_VISIBLE) * duty / 100;
    }
    ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, raw);
    ledc_update_duty(PWM1_MODE, PWM1_CHANNEL);
}

/* ======================== OTA Functions ======================== */
static void ota_notify_status(uint8_t *data, uint16_t len)
{
    if (!client_connected || !notifications_enabled || global_gatts_if == 0) {
        return;
    }
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id,
                                status_char_handle, len, data, false);
}

static void ota_begin(void)
{
    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition == NULL) {
        ESP_LOGE(TAG, "OTA: No update partition found");
        uint8_t err[] = {OTA_STATUS_ERROR, OTA_ERR_PARTITION};
        ota_notify_status(err, 2);
        return;
    }

    ESP_LOGI(TAG, "OTA: Writing to partition '%s' at 0x%08x",
             ota_partition->label, (unsigned int)ota_partition->address);

    esp_err_t err = esp_ota_begin(ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_begin failed (%s)", esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_BEGIN_FAIL};
        ota_notify_status(status, 2);
        ota_mode = false;
        return;
    }

    ota_bytes_received = 0;
    ota_last_progress = 0;
    ota_mode = true;
    pwm1_setduty(0);

    ESP_LOGI(TAG, "OTA: Ready for data");
    uint8_t status[] = {OTA_STATUS_READY};
    ota_notify_status(status, 1);
}

static void ota_write_chunk(uint8_t *data, uint16_t len)
{
    if (!ota_mode || ota_partition == NULL) {
        ESP_LOGW(TAG, "OTA: Write received but not in OTA mode");
        uint8_t err[] = {OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA};
        ota_notify_status(err, 2);
        return;
    }

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

    uint16_t progress_4kb = (uint16_t)(ota_bytes_received / 4096);
    if (progress_4kb != ota_last_progress) {
        ota_last_progress = progress_4kb;
        uint8_t status[] = {
            OTA_STATUS_PROGRESS,
            (uint8_t)((ota_bytes_received >> 16) & 0xFF),
            (uint8_t)((ota_bytes_received >> 8) & 0xFF),
            (uint8_t)(ota_bytes_received & 0xFF)
        };
        ota_notify_status(status, 4);

        if ((ota_bytes_received % (32 * 1024)) == 0) {
            ESP_LOGI(TAG, "OTA: %lu bytes", (unsigned long)ota_bytes_received);
        }
    }
}

static void ota_finish(void)
{
    if (!ota_mode) {
        ESP_LOGW(TAG, "OTA: Finish called but not in OTA mode");
        return;
    }

    ESP_LOGI(TAG, "OTA: Finishing, total %lu bytes", (unsigned long)ota_bytes_received);

    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_end failed (%s)", esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_END_FAIL};
        ota_notify_status(status, 2);
        ota_mode = false;
        return;
    }

    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: Set boot partition failed (%s)", esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_END_FAIL};
        ota_notify_status(status, 2);
        ota_mode = false;
        return;
    }

    ESP_LOGI(TAG, "OTA: Success! Rebooting...");
    uint8_t status[] = {OTA_STATUS_SUCCESS};
    ota_notify_status(status, 1);

    ota_mode = false;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
}

static void ota_cancel(void)
{
    if (ota_mode) {
        ESP_LOGW(TAG, "OTA: Cancelled");
        esp_ota_abort(ota_handle);
        ota_mode = false;
        ota_bytes_received = 0;

        uint8_t status[] = {OTA_STATUS_CANCELLED};
        ota_notify_status(status, 1);
    }
}

/* ======================== BLE Handlers ======================== */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
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

static uint8_t char_add_step = 0;

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "GATT register");
        global_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;

        esp_ble_gatt_set_local_mtu(512);

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

    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(TAG, "Service created, handle %d", param->create.service_handle);
        service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service_handle);

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

    case ESP_GATTS_ADD_CHAR_EVT: {
        ESP_LOGI(TAG, "Char added step %d, handle %d", char_add_step, param->add_char.attr_handle);

        if (char_add_step == 0) {
            ctrl_char_handle = param->add_char.attr_handle;

            esp_bt_uuid_t ota_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID_OTA},
            };
            esp_gatt_char_prop_t ota_prop = ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
                                             ESP_GATT_CHAR_PROP_BIT_WRITE;
            esp_attr_value_t ota_val = { .attr_max_len = 512, .attr_len = 0, .attr_value = NULL };
            char_add_step = 1;
            esp_ble_gatts_add_char(service_handle, &ota_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ota_prop, &ota_val, NULL);

        } else if (char_add_step == 1) {
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
            status_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Chars: ctrl=%d ota=%d status=%d",
                     ctrl_char_handle, ota_char_handle, status_char_handle);

            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
            };
            uint8_t cccd_val[2] = {0x00, 0x00};
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

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        if (param->add_char_descr.status == ESP_GATT_OK) {
            status_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD handle %d", status_cccd_handle);
        }
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        uint16_t handle = param->write.handle;
        uint16_t len = param->write.len;
        uint8_t *value = param->write.value;

        /* CCCD Write */
        if (len == 2 && handle == status_cccd_handle) {
            uint16_t cccd_val = value[0] | (value[1] << 8);
            notifications_enabled = (cccd_val == 0x0001);
            ESP_LOGI(TAG, "Notifications %s", notifications_enabled ? "ON" : "OFF");
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        /* Control Characteristic (0xFF01) */
        if (handle == ctrl_char_handle && len > 0) {
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }

            ESP_LOGI(TAG, "Ctrl write: %d bytes", len);

            /* LEGACY: Single byte = direct duty (0-255 mapped to 0-100%) */
            if (len == 1) {
                uint8_t raw = value[0];
                override_duty = (uint8_t)((uint16_t)raw * 100 / 255);
                override_active = 1;
                session_active = 0;
                pwm1_setduty(override_duty);
                ESP_LOGI(TAG, "Legacy: 0x%02X -> %d%%", raw, override_duty);
            } else {
                /* Multi-byte commands */
                uint8_t cmd = value[0];
                switch (cmd) {
                case 0xA2:  /* Brightness: [0xA2] [0-100] */
                    if (len >= 2) {
                        brightness = value[1];
                        if (brightness > 100) brightness = 100;
                        override_active = 0;
                        ESP_LOGI(TAG, "Brightness: %d%%", brightness);
                    }
                    break;

                case 0xA4:  /* Session: [0xA4] [minutes] */
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

                case 0xA5:  /* Override: [0xA5] [duty] */
                    if (len >= 2) {
                        override_duty = value[1];
                        if (override_duty > 100) override_duty = 100;
                        override_active = 1;
                        session_active = 0;
                        pwm1_setduty(override_duty);
                        ESP_LOGI(TAG, "Override: %d%%", override_duty);
                    }
                    break;

                case 0xA6:  /* Resume */
                    override_active = 0;
                    session_start_tick = xTaskGetTickCount();
                    session_active = 1;
                    session_ended = 0;
                    ESP_LOGI(TAG, "Session resumed");
                    break;

                case 0xA7:  /* Sleep */
                    ESP_LOGI(TAG, "Sleep command");
                    session_ended = 1;
                    break;

                case OTA_CMD_START:  /* 0xA8: Start OTA */
                    ESP_LOGI(TAG, "OTA: Start");
                    session_active = 0;
                    override_active = 0;
                    ota_begin();
                    break;

                case OTA_CMD_FINISH:  /* 0xA9: Finish OTA */
                    ESP_LOGI(TAG, "OTA: Finish");
                    ota_finish();
                    break;

                case OTA_CMD_CANCEL:  /* 0xAA: Cancel OTA */
                    ESP_LOGI(TAG, "OTA: Cancel");
                    ota_cancel();
                    break;

                default:
                    ESP_LOGW(TAG, "Unknown cmd: 0x%02X", cmd);
                    break;
                }
            }
        }

        /* OTA Data Characteristic (0xFF02) */
        else if (handle == ota_char_handle && len > 0) {
            ota_write_chunk(value, len);
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }

        /* Unknown handle */
        else {
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        break;
    }

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU: %d", param->mtu.mtu);
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Connected");
        global_conn_id = param->connect.conn_id;
        client_connected = true;
        notifications_enabled = false;

        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x10;
        conn_params.min_int = 0x08;
        conn_params.timeout = 400;
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Disconnected");
        client_connected = false;
        notifications_enabled = false;

        if (ota_mode) {
            ESP_LOGW(TAG, "OTA: Disconnect during OTA, aborting");
            esp_ota_abort(ota_handle);
            ota_mode = false;
        }

        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            global_gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "Reg failed");
            return;
        }
    }
    gatts_profile_event_handler(event, gatts_if, param);
}

/* ======================== LED Task ======================== */
static void led_task(void *param)
{
    while (1) {
        /* Pause during OTA */
        if (ota_mode) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /* Pause during override */
        if (override_active) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        /* Idle if no session */
        if (!session_active) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint32_t now = xTaskGetTickCount();

        /* Check session timeout */
        uint32_t session_duration_ticks = session_minutes * 60 * 1000 / portTICK_PERIOD_MS;
        uint32_t elapsed_ticks = now - session_start_tick;

        if (elapsed_ticks >= session_duration_ticks) {
            ESP_LOGI(TAG, "Session complete");
            pwm1_setduty(0);
            session_active = 0;
            session_ended = 1;
            continue;
        }

        /* Progress log every 30s */
        static uint32_t last_log = 0;
        if (now - last_log >= (30000 / portTICK_PERIOD_MS)) {
            last_log = now;
            uint32_t remaining_s = (session_duration_ticks - elapsed_ticks) * portTICK_PERIOD_MS / 1000;
            ESP_LOGI(TAG, "Strobe %dHz @ %d%% | %lus left",
                     STROBE_HZ, brightness, (unsigned long)remaining_s);
        }

        /* 40Hz Strobe: 19ms dark, 6ms clear */
        pwm1_setduty(brightness);
        vTaskDelay(STROBE_DARK_MS / portTICK_PERIOD_MS);

        pwm1_setduty(0);
        vTaskDelay(STROBE_CLEAR_MS / portTICK_PERIOD_MS);
    }
}

/* ======================== Sleep ======================== */
static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep...");
    ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, 0);
    ledc_update_duty(PWM1_MODE, PWM1_CHANNEL);
    esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
    esp_deep_sleep_start();
}

static void check_sleep_condition(void)
{
    static uint32_t low_level_duration = 0;
    static uint32_t last_check_time = 0;
    uint32_t current_time = xTaskGetTickCount();

    if (current_time - last_check_time < (1000 / portTICK_PERIOD_MS)) {
        return;
    }
    last_check_time = current_time;

    if (ota_mode) {
        low_level_duration = 0;
        return;
    }

    if (session_ended) {
        enter_deep_sleep();
    }

    if (gpio_get_level(HALL_PIN) == 1) {
        low_level_duration++;
        if (low_level_duration >= SLEEP_HALL_WAIT_TIME) {
            enter_deep_sleep();
        }
    } else {
        low_level_duration = 0;
    }
}

/* ======================== Main ======================== */
void app_main(void)
{
    esp_err_t ret;

    /* Check Hall after deep sleep wake */
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
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (gpio_get_level(HALL_PIN) == 1) {
            esp_sleep_enable_timer_wakeup(1000000);
            esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
            esp_deep_sleep_start();
        }
    }

    /* Log boot info */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Smart Glasses v4.3 (40Hz strobe, no breathing)");
    ESP_LOGI(TAG, "Partition: %s @ 0x%08x", running->label, (unsigned int)running->address);
    ESP_LOGI(TAG, "Wake reason: %d", wakeup_reason);

    /* NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* BLE */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(TEST_APP_ID));

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N12);

    /* Hall sensor */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HALL_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* PWM */
    PWM_Init();

    /* Start session */
    session_start_tick = xTaskGetTickCount();
    session_active = 1;
    session_ended = 0;

    /* LED task */
    xTaskCreate(led_task, "led_task", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "Strobe: %dHz @ %d%% | Session: %d min", STROBE_HZ, brightness, session_minutes);
    ESP_LOGI(TAG, "OTA: 0xA8=start 0xA9=finish 0xAA=cancel");
    ESP_LOGI(TAG, "===========================================");

    /* Main loop */
    while (1) {
        check_sleep_condition();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
