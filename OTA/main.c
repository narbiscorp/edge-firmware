/*
 * Smart Glasses Firmware v4.0 - OTA Update Support
 * 
 * BLE OTA update via nRF Connect or similar app.
 * 
 * Normal mode: Lens PWM control via BLE writes (same as v3.0)
 * OTA mode:    Triggered by writing 0xFD to the control characteristic.
 *              Then send firmware binary in chunks to the OTA data characteristic.
 *              Finish by writing 0xFE to control characteristic.
 *              Cancel with 0xFF.
 *
 * Service UUID:  0x00FF
 * Control Char:  0xFF01  (write: lens commands + OTA triggers)
 * OTA Data Char: 0xFF02  (write-no-response: firmware binary chunks)
 * Status Char:   0xFF03  (notify: OTA progress/status feedback)
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
#include "esp_sleep.h"

/* ======================== BLE UUIDs ======================== */
#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID_CTRL    0xFF01  /* Control: lens commands + OTA triggers */
#define GATTS_CHAR_UUID_OTA     0xFF02  /* OTA data: receives firmware chunks */
#define GATTS_CHAR_UUID_STATUS  0xFF03  /* Status: notify OTA progress */
#define GATTS_NUM_HANDLES       12      /* service(1) + 3 chars * (decl+value+desc) + extra */

#define DEVICE_NAME             "Smart_Glasses"
#define TEST_APP_ID             0

/* ======================== OTA Commands ======================== */
#define OTA_CMD_START           0xFD    /* Write to ctrl char to enter OTA mode */
#define OTA_CMD_FINISH          0xFE    /* Write to ctrl char to finish OTA */
#define OTA_CMD_CANCEL          0xFF    /* Write to ctrl char to cancel OTA */

/* OTA status codes sent via notify */
#define OTA_STATUS_READY        0x01    /* OTA mode entered, ready for data */
#define OTA_STATUS_PROGRESS     0x02    /* Progress update: [0x02, percent] */
#define OTA_STATUS_SUCCESS      0x03    /* OTA complete, rebooting */
#define OTA_STATUS_ERROR        0x04    /* Error: [0x04, error_code] */
#define OTA_STATUS_CANCELLED    0x05    /* OTA cancelled */

/* OTA error codes */
#define OTA_ERR_BEGIN_FAIL      0x01
#define OTA_ERR_WRITE_FAIL      0x02
#define OTA_ERR_END_FAIL        0x03
#define OTA_ERR_NOT_IN_OTA      0x04
#define OTA_ERR_PARTITION        0x05

static const char *TAG = "SG_OTA";

/* ======================== BLE State ======================== */
static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

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

/* GATT handles */
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
static uint32_t ota_total_expected = 0;  /* Set from first 4 bytes if desired */
static uint16_t ota_last_progress = 0;

/* ======================== PWM / Hardware ======================== */
#define HALL_PIN GPIO_NUM_4

RTC_DATA_ATTR int bootCount = 0;
uint8_t pwm_duty_set = 0;
uint8_t output_tim = 10;
#define SLEEP_HALL_WAIT_TIME 5

#define PWM1_TIMER      LEDC_TIMER_0
#define PWM1_MODE       LEDC_LOW_SPEED_MODE
#define PWM1_OUTPUT_IO  (27)
#define PWM1_CHANNEL    LEDC_CHANNEL_0
#define PWM1_DUTY_RES   LEDC_TIMER_10_BIT
#define PWM1_FREQUENCY  (10000)

#define PWM2_TIMER      LEDC_TIMER_0
#define PWM2_MODE       LEDC_LOW_SPEED_MODE
#define PWM2_OUTPUT_IO  (26)
#define PWM2_CHANNEL    LEDC_CHANNEL_1
#define PWM2_DUTY_RES   LEDC_TIMER_10_BIT
#define PWM2_FREQUENCY  (10000)

/* ======================== PWM Functions ======================== */
static void PWM_Init(void)
{
    ledc_timer_config_t pwm1_timer = {
        .speed_mode     = PWM1_MODE,
        .duty_resolution = PWM1_DUTY_RES,
        .timer_num      = PWM1_TIMER,
        .freq_hz        = PWM1_FREQUENCY,
        .clk_cfg        = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm1_timer));

    ledc_channel_config_t pwm1_channel = {
        .speed_mode = PWM1_MODE,
        .channel    = PWM1_CHANNEL,
        .timer_sel  = PWM1_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PWM1_OUTPUT_IO,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm1_channel));

    ledc_timer_config_t pwm2_timer = {
        .speed_mode     = PWM2_MODE,
        .duty_resolution = PWM2_DUTY_RES,
        .timer_num      = PWM2_TIMER,
        .freq_hz        = PWM2_FREQUENCY,
        .clk_cfg        = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm2_timer));

    ledc_channel_config_t pwm2_channel = {
        .speed_mode = PWM2_MODE,
        .channel    = PWM2_CHANNEL,
        .timer_sel  = PWM2_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PWM2_OUTPUT_IO,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm2_channel));
}

void pwm1_setduty(uint32_t duty) {
    duty = 1024 * duty / 100;
    ESP_ERROR_CHECK(ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM1_MODE, PWM1_CHANNEL));
}

void pwm2_setduty(uint32_t duty) {
    duty = 1024 * duty / 100;
    ESP_ERROR_CHECK(ledc_set_duty(PWM2_MODE, PWM2_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM2_MODE, PWM2_CHANNEL));
}

/* ======================== OTA Helper Functions ======================== */

/* Send a status notification to the connected client */
static void ota_notify_status(uint8_t *data, uint16_t len)
{
    if (!client_connected || !notifications_enabled || global_gatts_if == 0) {
        return;
    }
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id,
                                status_char_handle, len, data, false);
}

/* Begin OTA process */
static void ota_begin(void)
{
    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition == NULL) {
        ESP_LOGE(TAG, "OTA: No update partition found");
        uint8_t err[] = {OTA_STATUS_ERROR, OTA_ERR_PARTITION};
        ota_notify_status(err, 2);
        return;
    }

    ESP_LOGI(TAG, "OTA: Writing to partition '%s' at offset 0x%08x",
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

    /* Pause lens PWM during OTA */
    pwm1_setduty(0);
    pwm2_setduty(0);

    ESP_LOGI(TAG, "OTA: Ready for data");
    uint8_t status[] = {OTA_STATUS_READY};
    ota_notify_status(status, 1);
}

/* Write a chunk of OTA data */
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
        ESP_LOGE(TAG, "OTA: Write failed at offset %lu (%s)",
                 (unsigned long)ota_bytes_received, esp_err_to_name(err));
        uint8_t status[] = {OTA_STATUS_ERROR, OTA_ERR_WRITE_FAIL};
        ota_notify_status(status, 2);
        esp_ota_abort(ota_handle);
        ota_mode = false;
        return;
    }

    ota_bytes_received += len;

    /* Send progress every 4KB (avoids flooding BLE) */
    uint16_t progress_4kb = (uint16_t)(ota_bytes_received / 4096);
    if (progress_4kb != ota_last_progress) {
        ota_last_progress = progress_4kb;
        /* Notify: [PROGRESS, KB received high, KB received low] */
        uint8_t status[] = {
            OTA_STATUS_PROGRESS,
            (uint8_t)((ota_bytes_received >> 16) & 0xFF),
            (uint8_t)((ota_bytes_received >> 8) & 0xFF),
            (uint8_t)(ota_bytes_received & 0xFF)
        };
        ota_notify_status(status, 4);

        if ((ota_bytes_received % (32 * 1024)) == 0) {
            ESP_LOGI(TAG, "OTA: %lu bytes received", (unsigned long)ota_bytes_received);
        }
    }
}

/* Finish OTA and reboot */
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

    ESP_LOGI(TAG, "OTA: Success! Rebooting in 2 seconds...");
    uint8_t status[] = {OTA_STATUS_SUCCESS};
    ota_notify_status(status, 1);

    ota_mode = false;

    /* Delay to let the BLE notification reach the phone */
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
}

/* Cancel OTA */
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

/* ======================== BLE Callbacks ======================== */

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
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            ESP_LOGI(TAG, "Advertising started");
        }
        break;
    default:
        break;
    }
}

/* Track which characteristic handle is being added */
static uint8_t char_add_step = 0;

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "GATT register, app_id %d", param->reg.app_id);
        global_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;

        /* Request larger MTU for faster OTA transfers */
        esp_ble_gatt_set_local_mtu(512);

        /* Create service with enough handles for 3 characteristics */
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

        /* Add characteristic 1: Control (0xFF01) - read/write */
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
            /* Control characteristic added, save handle */
            ctrl_char_handle = param->add_char.attr_handle;

            /* Add characteristic 2: OTA Data (0xFF02) - write-no-response for speed */
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
            /* OTA data characteristic added, save handle */
            ota_char_handle = param->add_char.attr_handle;

            /* Add characteristic 3: Status (0xFF03) - notify */
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
            /* Status characteristic added */
            status_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "All characteristics added. Handles: ctrl=%d ota=%d status=%d",
                     ctrl_char_handle, ota_char_handle, status_char_handle);

            /* Add CCCD descriptor for notifications on Status characteristic.
             * Bluedroid does NOT auto-add CCCD - without this, nRF Connect
             * won't show the notification enable button. */
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
        /* CCCD descriptor for notifications - track handle */
        if (param->add_char_descr.status == ESP_GATT_OK) {
            status_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD descriptor added, handle %d", status_cccd_handle);
        }
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        uint16_t handle = param->write.handle;
        uint16_t len = param->write.len;
        uint8_t *value = param->write.value;

        /* ---------- CCCD Write (enable/disable notifications) ---------- */
        if (len == 2 && handle == status_cccd_handle) {
            uint16_t cccd_val = value[0] | (value[1] << 8);
            notifications_enabled = (cccd_val == 0x0001);
            ESP_LOGI(TAG, "Notifications %s", notifications_enabled ? "enabled" : "disabled");
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        /* ---------- Control Characteristic (0xFF01) ---------- */
        if (handle == ctrl_char_handle && len > 0) {
            uint8_t cmd = value[0];

            if (cmd == OTA_CMD_START) {
                /* Enter OTA mode */
                ESP_LOGI(TAG, "OTA: Start command received");
                ota_begin();

            } else if (cmd == OTA_CMD_FINISH) {
                /* Finish OTA */
                ESP_LOGI(TAG, "OTA: Finish command received");
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                param->write.trans_id, ESP_GATT_OK, NULL);
                }
                ota_finish();
                break;  /* Don't send response twice */

            } else if (cmd == OTA_CMD_CANCEL) {
                /* Cancel OTA */
                ESP_LOGI(TAG, "OTA: Cancel command received");
                ota_cancel();

            } else if (!ota_mode) {
                /* Normal lens control - same BCD logic as original */
                ESP_LOGI(TAG, "Lens command, len=%d", len);
                for (int i = 0; i < len; i++) {
                    ESP_LOGI(TAG, "  Byte[%d]: 0x%02X", i, value[i]);
                }

                if (len >= 1) {
                    uint8_t bcd_value = value[0] * 0.6;
                    if (bcd_value > 100) bcd_value = 100;
                    pwm_duty_set = ((bcd_value >> 4) * 10) + (bcd_value & 0x0F);
                    ESP_LOGI(TAG, "BCD decode: 0x%02X -> %d%%", bcd_value, pwm_duty_set);
                }
                if (len >= 2) {
                    uint8_t bcd_value = value[1];
                    output_tim = ((bcd_value >> 4) * 10) + (bcd_value & 0x0F);
                    ESP_LOGI(TAG, "BCD decode: 0x%02X -> %dms", bcd_value, output_tim);
                }
                if (pwm_duty_set > 100) pwm_duty_set = 100;
                if (output_tim < 1) output_tim = 1;
                ESP_LOGI(TAG, "Final: Duty=%d%%, Time=%dms", pwm_duty_set, output_tim);
            }

            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }

        /* ---------- OTA Data Characteristic (0xFF02) ---------- */
        else if (handle == ota_char_handle && len > 0) {
            ota_write_chunk(value, len);
            /* Write-no-response: no need for esp_ble_gatts_send_response */
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }

        /* ---------- Unknown handle ---------- */
        else {
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        break;
    }

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU updated to %d", param->mtu.mtu);
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Client connected, conn_id=%d", param->connect.conn_id);
        global_conn_id = param->connect.conn_id;
        client_connected = true;
        notifications_enabled = false;

        /* Request higher connection speed for OTA */
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x10;  /* 20ms */
        conn_params.min_int = 0x08;  /* 10ms */
        conn_params.timeout = 400;   /* 4 seconds */
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Client disconnected");
        client_connected = false;
        notifications_enabled = false;

        /* Cancel OTA if in progress */
        if (ota_mode) {
            ESP_LOGW(TAG, "OTA: Disconnected during OTA, aborting");
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

/* ======================== PWM Task ======================== */
TaskHandle_t TASK_HandleOne = NULL;

void TASK_ONE(void *param)
{
    static uint8_t state = 0;
    while (1) {
        /* Skip PWM toggling during OTA */
        if (ota_mode) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        switch (state) {
        case 0:
            pwm1_setduty(pwm_duty_set);
            pwm2_setduty(1);
            state = 1;
            break;
        case 1:
            pwm1_setduty(1);
            pwm2_setduty(pwm_duty_set);
            state = 0;
            break;
        default:
            break;
        }

        uint32_t delay = output_tim;
        if (delay < 1) delay = 1;
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

/* ======================== Sleep Check ======================== */
static void check_sleep_condition(void)
{
    static uint32_t low_level_duration = 0;
    static uint32_t last_check_time = 0;
    uint32_t current_time = xTaskGetTickCount();

    if (current_time - last_check_time < (1000 / portTICK_PERIOD_MS)) {
        return;
    }
    last_check_time = current_time;

    /* Don't sleep during OTA */
    if (ota_mode) {
        low_level_duration = 0;
        return;
    }

    if (gpio_get_level(HALL_PIN) == 1) {
        low_level_duration++;
        ESP_LOGI(TAG, "Hall sensor, duration: %lu sec", (unsigned long)low_level_duration);

        if (low_level_duration >= SLEEP_HALL_WAIT_TIME) {
            ESP_LOGI(TAG, "Entering deep sleep...");
            pwm_duty_set = 0;
            pwm1_setduty(0);
            pwm2_setduty(0);
            esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
            esp_deep_sleep_start();
        }
    } else {
        low_level_duration = 0;
    }
}

/* ======================== Main ======================== */
void app_main(void)
{
    esp_err_t ret;

    /* Log boot info and current running partition */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "=== Smart Glasses v4.0 (OTA) ===");
    ESP_LOGI(TAG, "Running partition: %s (addr=0x%08x)",
             running->label, (unsigned int)running->address);

    /* NVS init */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* BLE init */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(TEST_APP_ID));

    /* Hall sensor pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HALL_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* PWM init */
    PWM_Init();

    /* PWM task */
    xTaskCreate(TASK_ONE, "TaskOne", 8 * 1024, NULL, 1, &TASK_HandleOne);

    /* Main loop */
    while (1) {
        check_sleep_condition();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
