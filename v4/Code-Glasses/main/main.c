/*******************************************************************************
 * Smart Glasses Firmware v4.7
 * 
 * Based on v4.6 (1Hz strobe + deferred OTA). Changed strobe to 25Hz.
 * 
 * FEATURES:
 * - AC differential drive: GPIO26/GPIO27 alternate every 10ms (50Hz AC)
 * - 25Hz strobe: 40ms period (20ms dark, 20ms clear) at boot
 * - OTA firmware updates via BLE
 * - Power saving: -12dBm TX, 320ms advertising interval, modem sleep
 * - Session timer: 10 minutes default
 * - Hall sensor sleep: 5 seconds HIGH triggers deep sleep
 * 
 * BLE COMMANDS (multi-byte to 0xFF01):
 * - A2 [brightness]     Set brightness 0-100%
 * - A4 [minutes]        Set session duration 1-60 min
 * - A5 [duty]           Static duty override (stops strobe)
 * - A6 00               Resume strobe
 * - A7 00               Sleep immediately
 * - A8 00               Start OTA mode
 * - A9 00               Finish OTA (validate + reboot)
 * - AA 00               Cancel OTA
 * 
 * LEGACY COMMAND (single byte):
 * - 0x00-0xFF           Direct duty = byte * 100 / 255
 * 
 * OTA DATA: Write firmware chunks to 0xFF02
 * OTA STATUS: Notifications on 0xFF03
 * 
 * HARDWARE:
 * - ESP32-PICO-V3 (ESP32-U4WDH), 4MB flash, 40MHz XTAL
 * - GPIO27 (PWM1) + GPIO26 (PWM2): Differential AC drive to LCD lens
 * - GPIO4: Hall sensor (arm open/close detection)
 * - PWM: 10-bit resolution, 10kHz carrier frequency
 * 
 * AC DRIVE EXPLANATION:
 * The electrochromic lens requires AC drive to prevent material degradation.
 * Both PWM channels alternate: when PWM1 is at duty, PWM2 is ~0V, then swap.
 * This creates a net-zero DC component across the lens electrodes.
 * 
 * CHANGELOG v4.7:
 * - Changed strobe from 1Hz to 25Hz (20ms dark, 20ms clear)
 * - All other functionality unchanged from v4.6
 ******************************************************************************/

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

/*******************************************************************************
 * VERSION AND IDENTIFICATION
 ******************************************************************************/
#define FIRMWARE_VERSION "4.7"
static const char *TAG = "SG_v4.7";

/*******************************************************************************
 * BLE CONFIGURATION - ACTIVE MODE (Power Optimized)
 ******************************************************************************/
#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID_CTRL    0xFF01  /* Control commands */
#define GATTS_CHAR_UUID_OTA     0xFF02  /* OTA data chunks */
#define GATTS_CHAR_UUID_STATUS  0xFF03  /* Status notifications */
#define GATTS_NUM_HANDLE        10      /* Service + 3 chars + CCCDs */

#define DEVICE_NAME             "Smart_Glasses"
#define GATTS_APP_ID            0

/* Power-optimized advertising: 200-320ms interval (was 20-40ms)
 * 0x140 = 320 * 0.625ms = 200ms
 * 0x200 = 512 * 0.625ms = 320ms */
#define ADV_INT_MIN             0x140
#define ADV_INT_MAX             0x200

/*******************************************************************************
 * HARDWARE CONFIGURATION
 ******************************************************************************/
#define HALL_PIN                GPIO_NUM_4
#define SLEEP_HALL_WAIT_TIME    5       /* Seconds of HIGH before sleep */

/* PWM Channel 1 - GPIO27 */
#define PWM1_TIMER              LEDC_TIMER_0
#define PWM1_MODE               LEDC_LOW_SPEED_MODE
#define PWM1_OUTPUT_IO          27
#define PWM1_CHANNEL            LEDC_CHANNEL_0
#define PWM1_DUTY_RES           LEDC_TIMER_10_BIT
#define PWM1_FREQUENCY          10000   /* 10kHz carrier */

/* PWM Channel 2 - GPIO26 */
#define PWM2_TIMER              LEDC_TIMER_0
#define PWM2_MODE               LEDC_LOW_SPEED_MODE
#define PWM2_OUTPUT_IO          26
#define PWM2_CHANNEL            LEDC_CHANNEL_1
#define PWM2_DUTY_RES           LEDC_TIMER_10_BIT
#define PWM2_FREQUENCY          10000   /* 10kHz carrier */

/*******************************************************************************
 * TIMING CONFIGURATION
 * 
 * FreeRTOS runs at 100Hz (CONFIG_FREERTOS_HZ=100), so 1 tick = 10ms.
 * AC alternation: 10ms per half-cycle = 50Hz AC (matches original firmware)
 * Strobe: 25Hz = 40ms period = 20ms dark + 20ms clear
 ******************************************************************************/
#define AC_PERIOD_TICKS         1       /* 10ms = 50Hz AC alternation */
#define STROBE_HALF_PERIOD_AC   2       /* 2 AC cycles = 20ms per strobe phase */
#define DEFAULT_SESSION_MIN     10      /* 10 minute session */
#define DEFAULT_BRIGHTNESS      100     /* 100% brightness */

/*******************************************************************************
 * LCD DEADZONE COMPENSATION
 * 
 * The LCD lens doesn't respond below ~40% raw PWM duty.
 * Duty 0 = raw 0 (fully clear)
 * Duty 1-100 = raw 400-1023 (visible range)
 ******************************************************************************/
#define LCD_DEADZONE_RAW        400
#define PWM_MAX_RAW             1023

static inline uint32_t duty_to_raw(uint8_t duty_percent) {
    if (duty_percent == 0) return 0;
    if (duty_percent > 100) duty_percent = 100;
    return LCD_DEADZONE_RAW + (duty_percent * (PWM_MAX_RAW - LCD_DEADZONE_RAW) / 100);
}

/*******************************************************************************
 * GLOBAL STATE
 ******************************************************************************/
/* Session state */
static volatile bool session_active = false;
static volatile bool strobe_enabled = true;
static volatile uint8_t brightness = DEFAULT_BRIGHTNESS;
static volatile uint32_t session_duration_ms = DEFAULT_SESSION_MIN * 60 * 1000;
static volatile uint32_t session_start_tick = 0;

/* AC drive state - shared between tasks */
static volatile uint8_t effective_duty = 0;  /* Current duty for AC task to use */

/* OTA state */
static volatile bool in_ota_mode = false;
static const esp_partition_t *ota_partition = NULL;
static esp_ota_handle_t ota_handle = 0;
static uint32_t ota_bytes_written = 0;

/* OTA task — deferred execution to avoid blocking BLE callback */
typedef enum {
    OTA_TASK_NONE = 0,
    OTA_TASK_BEGIN,
    OTA_TASK_FINISH,
    OTA_TASK_CANCEL,
} ota_task_cmd_t;

static volatile ota_task_cmd_t ota_pending_cmd = OTA_TASK_NONE;
static TaskHandle_t ota_task_handle = NULL;

/* BLE handles */
static uint16_t gatts_if_global = ESP_GATT_IF_NONE;
static uint16_t conn_id_global = 0;
static uint16_t service_handle = 0;
static uint16_t ctrl_char_handle = 0;
static uint16_t ota_char_handle = 0;
static uint16_t status_char_handle = 0;
static uint16_t cccd_handle = 0;
static bool notifications_enabled = false;
static bool is_connected = false;

/* Task handles */
static TaskHandle_t ac_task_handle = NULL;
static TaskHandle_t led_task_handle = NULL;

/*******************************************************************************
 * PWM FUNCTIONS
 ******************************************************************************/
static void pwm_init(void) {
    /* Configure timer (shared by both channels) */
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM1_MODE,
        .duty_resolution  = PWM1_DUTY_RES,
        .timer_num        = PWM1_TIMER,
        .freq_hz          = PWM1_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    /* Configure PWM1 channel (GPIO27) */
    ledc_channel_config_t pwm1_conf = {
        .speed_mode     = PWM1_MODE,
        .channel        = PWM1_CHANNEL,
        .timer_sel      = PWM1_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM1_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm1_conf));

    /* Configure PWM2 channel (GPIO26) */
    ledc_channel_config_t pwm2_conf = {
        .speed_mode     = PWM2_MODE,
        .channel        = PWM2_CHANNEL,
        .timer_sel      = PWM2_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM2_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm2_conf));
    
    ESP_LOGI(TAG, "PWM initialized: GPIO26 + GPIO27, 10kHz, 10-bit");
}

static void pwm1_set_raw(uint32_t raw_duty) {
    ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, raw_duty);
    ledc_update_duty(PWM1_MODE, PWM1_CHANNEL);
}

static void pwm2_set_raw(uint32_t raw_duty) {
    ledc_set_duty(PWM2_MODE, PWM2_CHANNEL, raw_duty);
    ledc_update_duty(PWM2_MODE, PWM2_CHANNEL);
}

static void pwm_both_off(void) {
    pwm1_set_raw(0);
    pwm2_set_raw(0);
}

/*******************************************************************************
 * OTA STATUS NOTIFICATIONS
 ******************************************************************************/
#define OTA_STATUS_READY        0x01
#define OTA_STATUS_PROGRESS     0x02
#define OTA_STATUS_SUCCESS      0x03
#define OTA_STATUS_ERROR        0x04
#define OTA_STATUS_CANCELLED    0x05

#define OTA_ERR_BEGIN           0x01
#define OTA_ERR_WRITE           0x02
#define OTA_ERR_END             0x03
#define OTA_ERR_NOT_IN_OTA      0x04
#define OTA_ERR_PARTITION       0x05

static void send_ota_status(uint8_t status, uint8_t extra1, uint8_t extra2, uint8_t extra3) {
    if (!notifications_enabled || !is_connected) return;
    
    uint8_t notify_data[4] = {status, extra1, extra2, extra3};
    size_t len = (status == OTA_STATUS_PROGRESS) ? 4 : 
                 (status == OTA_STATUS_ERROR) ? 2 : 1;
    
    esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global, 
                                 status_char_handle, len, notify_data, false);
}

/*******************************************************************************
 * AC DRIVE TASK
 * 
 * This task runs continuously, alternating PWM1/PWM2 every 10ms (50Hz AC).
 * The effective_duty variable is set by the LED task based on strobe state.
 * This ensures the lens always gets proper AC drive, never sustained DC.
 ******************************************************************************/
static void ac_drive_task(void *param) {
    uint8_t ac_state = 0;
    uint32_t raw_duty;
    
    ESP_LOGI(TAG, "AC drive task started (50Hz alternation)");
    
    while (1) {
        /* Get current duty (set by LED task or commands) */
        raw_duty = duty_to_raw(effective_duty);
        
        /* Alternate electrodes every 10ms */
        if (ac_state == 0) {
            pwm1_set_raw(raw_duty);  /* GPIO27 = tint */
            pwm2_set_raw(1);         /* GPIO26 = ~0V */
            ac_state = 1;
        } else {
            pwm1_set_raw(1);         /* GPIO27 = ~0V */
            pwm2_set_raw(raw_duty);  /* GPIO26 = tint */
            ac_state = 0;
        }
        
        /* Wait 10ms (50Hz AC half-cycle) */
        vTaskDelay(AC_PERIOD_TICKS);
        
        /* If in OTA mode or session ended, keep lens clear */
        if (in_ota_mode || !session_active) {
            effective_duty = 0;
        }
    }
}

/*******************************************************************************
 * LED CONTROL TASK
 * 
 * Handles strobe timing and session management.
 * Sets effective_duty which the AC task uses for actual PWM output.
 ******************************************************************************/
static void led_task(void *param) {
    uint32_t strobe_counter = 0;
    uint8_t strobe_phase = 0;  /* 0 = dark (tinted), 1 = clear */
    uint32_t last_log_tick = 0;
    
    ESP_LOGI(TAG, "LED task started - 25Hz strobe");
    
    /* Start session */
    session_active = true;
    session_start_tick = xTaskGetTickCount();
    
    while (1) {
        /* Check session timeout */
        uint32_t elapsed = (xTaskGetTickCount() - session_start_tick) * portTICK_PERIOD_MS;
        if (elapsed >= session_duration_ms) {
            ESP_LOGI(TAG, "Session ended after %lu minutes", elapsed / 60000);
            session_active = false;
            effective_duty = 0;
            break;
        }
        
        /* Handle OTA mode - pause effects */
        if (in_ota_mode) {
            effective_duty = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        /* Calculate current duty based on strobe state */
        if (strobe_enabled) {
            if (strobe_phase == 0) {
                /* Dark phase - apply brightness */
                effective_duty = brightness;
            } else {
                /* Clear phase - lens transparent */
                effective_duty = 0;
            }
        } else {
            /* Strobe disabled - static brightness */
            effective_duty = brightness;
        }
        
        /* Wait for one AC cycle (10ms) */
        vTaskDelay(AC_PERIOD_TICKS);
        strobe_counter++;
        
        /* Toggle strobe phase every 2 AC cycles (20ms) = 25Hz strobe */
        if (strobe_counter >= STROBE_HALF_PERIOD_AC) {
            strobe_phase = !strobe_phase;
            strobe_counter = 0;
        }
        
        /* Log status every 30 seconds */
        if (xTaskGetTickCount() - last_log_tick >= pdMS_TO_TICKS(30000)) {
            last_log_tick = xTaskGetTickCount();
            uint32_t remaining_sec = (session_duration_ms - elapsed) / 1000;
            ESP_LOGI(TAG, "Strobe 25Hz @ %d%%, %lu sec remaining", 
                     brightness, remaining_sec);
        }
    }
    
    /* Session ended - task exits, main loop will handle sleep */
    led_task_handle = NULL;
    vTaskDelete(NULL);
}

/*******************************************************************************
 * DEEP SLEEP
 ******************************************************************************/
static void enter_deep_sleep(void) {
    ESP_LOGI(TAG, "Entering deep sleep...");
    
    /* Clear lens before sleep */
    effective_duty = 0;
    vTaskDelay(pdMS_TO_TICKS(50));  /* Let AC task apply it */
    pwm_both_off();
    
    /* Configure wake on Hall sensor LOW (arm opened) */
    esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
    
    esp_deep_sleep_start();
}

/*******************************************************************************
 * HALL SENSOR MONITORING
 ******************************************************************************/
static void check_hall_sensor(void) {
    static uint32_t high_duration = 0;
    static uint32_t last_check = 0;
    uint32_t now = xTaskGetTickCount();
    
    /* Check every second */
    if (now - last_check < pdMS_TO_TICKS(1000)) return;
    last_check = now;
    
    /* Block sleep during OTA */
    if (in_ota_mode) {
        high_duration = 0;
        return;
    }
    
    if (gpio_get_level(HALL_PIN) == 1) {
        high_duration++;
        ESP_LOGI(TAG, "Hall sensor HIGH, %lu seconds", high_duration);
        
        if (high_duration >= SLEEP_HALL_WAIT_TIME) {
            ESP_LOGI(TAG, "Hall sensor triggered - sleeping");
            enter_deep_sleep();
        }
    } else {
        high_duration = 0;
    }
}

/*******************************************************************************
 * OTA DEFERRED OPERATIONS
 * 
 * esp_ota_begin() and esp_ota_end() can block for seconds during flash ops.
 * Running them inside the BLE GATT callback would crash the BLE stack.
 * These functions run from a dedicated OTA task instead.
 ******************************************************************************/
static void ota_do_begin(void) {
    if (in_ota_mode) {
        ESP_LOGW(TAG, "Already in OTA mode");
        return;
    }

    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (!ota_partition) {
        ESP_LOGE(TAG, "No OTA partition found");
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_PARTITION, 0, 0);
        return;
    }

    esp_err_t err = esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_BEGIN, 0, 0);
        return;
    }

    in_ota_mode = true;
    ota_bytes_written = 0;
    effective_duty = 0;
    ESP_LOGI(TAG, "OTA started, partition: %s", ota_partition->label);
    send_ota_status(OTA_STATUS_READY, 0, 0, 0);
}

static void ota_do_finish(void) {
    if (!in_ota_mode) {
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA, 0, 0);
        return;
    }

    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_END, 0, 0);
        in_ota_mode = false;
        return;
    }

    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed: %s", esp_err_to_name(err));
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_END, 0, 0);
        in_ota_mode = false;
        return;
    }

    ESP_LOGI(TAG, "OTA complete! %lu bytes written. Rebooting...", ota_bytes_written);
    send_ota_status(OTA_STATUS_SUCCESS, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static void ota_do_cancel(void) {
    if (in_ota_mode) {
        esp_ota_abort(ota_handle);
        in_ota_mode = false;
        ota_bytes_written = 0;
        ESP_LOGI(TAG, "OTA cancelled");
        send_ota_status(OTA_STATUS_CANCELLED, 0, 0, 0);
    }
}

static void ota_task(void *param) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ota_task_cmd_t cmd = ota_pending_cmd;
        ota_pending_cmd = OTA_TASK_NONE;

        switch (cmd) {
            case OTA_TASK_BEGIN:  ota_do_begin();  break;
            case OTA_TASK_FINISH: ota_do_finish(); break;
            case OTA_TASK_CANCEL: ota_do_cancel(); break;
            default: break;
        }
    }
}

/*******************************************************************************
 * COMMAND PROCESSING
 ******************************************************************************/
static void process_command(uint8_t *data, uint16_t len) {
    /* Single-byte: legacy duty control */
    if (len == 1) {
        uint8_t byte = data[0];
        uint8_t duty = (byte * 100) / 255;
        ESP_LOGI(TAG, "Legacy duty: %d%% (byte 0x%02X)", duty, byte);
        brightness = duty;
        strobe_enabled = false;
        return;
    }
    
    /* Multi-byte commands */
    if (len < 2) return;
    
    uint8_t cmd = data[0];
    uint8_t arg = data[1];
    
    switch (cmd) {
        case 0xA2:  /* Set brightness */
            if (arg > 100) arg = 100;
            brightness = arg;
            ESP_LOGI(TAG, "Brightness: %d%%", brightness);
            break;
            
        case 0xA4:  /* Set session duration */
            if (arg < 1) arg = 1;
            if (arg > 60) arg = 60;
            session_duration_ms = arg * 60 * 1000;
            ESP_LOGI(TAG, "Session: %d minutes", arg);
            break;
            
        case 0xA5:  /* Static duty override */
            if (arg > 100) arg = 100;
            brightness = arg;
            strobe_enabled = false;
            ESP_LOGI(TAG, "Static duty: %d%%", arg);
            break;
            
        case 0xA6:  /* Resume strobe */
            strobe_enabled = true;
            ESP_LOGI(TAG, "Strobe resumed");
            break;
            
        case 0xA7:  /* Sleep immediately */
            ESP_LOGI(TAG, "Sleep command received");
            session_active = false;
            enter_deep_sleep();
            break;
            
        case 0xA8:  /* Start OTA (deferred to OTA task) */
            ESP_LOGI(TAG, "OTA: Start command (deferred)");
            session_active = false;
            effective_duty = 0;
            ota_pending_cmd = OTA_TASK_BEGIN;
            if (ota_task_handle) xTaskNotifyGive(ota_task_handle);
            break;
            
        case 0xA9:  /* Finish OTA (deferred to OTA task) */
            ESP_LOGI(TAG, "OTA: Finish command (deferred)");
            ota_pending_cmd = OTA_TASK_FINISH;
            if (ota_task_handle) xTaskNotifyGive(ota_task_handle);
            break;
            
        case 0xAA:  /* Cancel OTA (deferred to OTA task) */
            ESP_LOGI(TAG, "OTA: Cancel command (deferred)");
            ota_pending_cmd = OTA_TASK_CANCEL;
            if (ota_task_handle) xTaskNotifyGive(ota_task_handle);
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
            break;
    }
}

/*******************************************************************************
 * OTA DATA PROCESSING
 ******************************************************************************/
static void process_ota_data(uint8_t *data, uint16_t len) {
    if (!in_ota_mode) {
        ESP_LOGW(TAG, "OTA data received but not in OTA mode");
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA, 0, 0);
        return;
    }
    
    esp_err_t err = esp_ota_write(ota_handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_WRITE, 0, 0);
        esp_ota_abort(ota_handle);
        in_ota_mode = false;
        return;
    }
    
    ota_bytes_written += len;
    
    /* Send progress every 4KB */
    if ((ota_bytes_written % 4096) < len) {
        uint8_t progress_kb = (ota_bytes_written / 1024) & 0xFF;
        uint8_t progress_kb_hi = ((ota_bytes_written / 1024) >> 8) & 0xFF;
        uint8_t progress_kb_highest = ((ota_bytes_written / 1024) >> 16) & 0xFF;
        send_ota_status(OTA_STATUS_PROGRESS, progress_kb, progress_kb_hi, progress_kb_highest);
        ESP_LOGI(TAG, "OTA progress: %lu bytes", ota_bytes_written);
    }
}

/*******************************************************************************
 * BLE ADVERTISING PARAMETERS (Power Optimized)
 ******************************************************************************/
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = ADV_INT_MIN,   /* 200ms */
    .adv_int_max        = ADV_INT_MAX,   /* 320ms */
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/*******************************************************************************
 * GAP EVENT HANDLER
 ******************************************************************************/
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started");
            } else {
                ESP_LOGE(TAG, "Advertising failed");
            }
            break;
            
        default:
            break;
    }
}

/*******************************************************************************
 * GATT EVENT HANDLER
 ******************************************************************************/
static void gatts_event_handler(esp_gatts_cb_event_t event, 
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param) {
    
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT app registered");
            gatts_if_global = gatts_if;
            
            /* Set device name */
            esp_ble_gap_set_device_name(DEVICE_NAME);
            
            /* Set TX power to -12dBm for power saving */
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12);
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12);
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N12);
            
            /* Configure advertising */
            esp_ble_gap_config_adv_data(&adv_data);
            
            /* Create service */
            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid = {.uuid16 = GATTS_SERVICE_UUID}
                    }
                }
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            service_handle = param->create.service_handle;
            ESP_LOGI(TAG, "Service created, handle %d", service_handle);
            esp_ble_gatts_start_service(service_handle);
            
            /* Add Control characteristic (0xFF01) */
            esp_bt_uuid_t ctrl_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID_CTRL}
            };
            esp_ble_gatts_add_char(service_handle, &ctrl_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;
            
            if (char_uuid == GATTS_CHAR_UUID_CTRL) {
                ctrl_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Char added step 0, handle %d", ctrl_char_handle);
                
                /* Add OTA Data characteristic (0xFF02) */
                esp_bt_uuid_t ota_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_OTA}
                };
                esp_ble_gatts_add_char(service_handle, &ota_uuid,
                                       ESP_GATT_PERM_WRITE,
                                       ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                       NULL, NULL);
            }
            else if (char_uuid == GATTS_CHAR_UUID_OTA) {
                ota_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Char added step 1, handle %d", ota_char_handle);
                
                /* Add Status characteristic (0xFF03) with notify */
                esp_bt_uuid_t status_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_STATUS}
                };
                esp_ble_gatts_add_char(service_handle, &status_uuid,
                                       ESP_GATT_PERM_READ,
                                       ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                       NULL, NULL);
            }
            else if (char_uuid == GATTS_CHAR_UUID_STATUS) {
                status_char_handle = param->add_char.attr_handle;
                ESP_LOGI(TAG, "Char added step 2, handle %d", status_char_handle);
                
                /* Add CCCD for notifications */
                esp_bt_uuid_t cccd_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
                };
                esp_ble_gatts_add_char_descr(service_handle, &cccd_uuid,
                                              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                              NULL, NULL);
            }
            break;
        }
        
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD handle %d - BLE setup complete", cccd_handle);
            ESP_LOGI(TAG, "Handles: ctrl=%d ota=%d status=%d", 
                     ctrl_char_handle, ota_char_handle, status_char_handle);
            ESP_LOGI(TAG, "OTA: A8=start A9=finish AA=cancel");
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            is_connected = true;
            conn_id_global = param->connect.conn_id;
            ESP_LOGI(TAG, "Client connected, conn_id %d", conn_id_global);
            
            /* Update connection parameters for power saving */
            esp_ble_conn_update_params_t conn_params = {
                .latency = 4,           /* Skip up to 4 connection events */
                .max_int = 0x30,        /* 60ms max interval */
                .min_int = 0x20,        /* 40ms min interval */
                .timeout = 400,         /* 4 second timeout */
            };
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            is_connected = false;
            notifications_enabled = false;
            ESP_LOGI(TAG, "Client disconnected");
            
            /* Cancel OTA if in progress */
            if (in_ota_mode) {
                esp_ota_abort(ota_handle);
                in_ota_mode = false;
                ESP_LOGW(TAG, "OTA cancelled due to disconnect");
            }
            
            /* Restart advertising */
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == ctrl_char_handle) {
                process_command(param->write.value, param->write.len);
            }
            else if (param->write.handle == ota_char_handle) {
                process_ota_data(param->write.value, param->write.len);
            }
            else if (param->write.handle == cccd_handle) {
                if (param->write.len == 2) {
                    uint16_t cccd_value = param->write.value[0] | (param->write.value[1] << 8);
                    notifications_enabled = (cccd_value == 0x0001);
                    ESP_LOGI(TAG, "Notifications %s", notifications_enabled ? "enabled" : "disabled");
                }
            }
            
            /* Send response if needed */
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
            
        default:
            break;
    }
}

/*******************************************************************************
 * MAIN APPLICATION
 ******************************************************************************/
void app_main(void) {
    esp_err_t ret;
    
    /* Print startup banner */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Smart Glasses v%s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "  Strobe: 25Hz @ %d%% brightness", DEFAULT_BRIGHTNESS);
    ESP_LOGI(TAG, "  Session: %d minutes", DEFAULT_SESSION_MIN);
    ESP_LOGI(TAG, "  AC Drive: 50Hz differential (GPIO26/27)");
    ESP_LOGI(TAG, "  Power: -12dBm TX, 200-320ms adv interval");
    
    /* Get partition info */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "  Partition: %s @ 0x%lx", running->label, running->address);
    
    /* Log wake reason */
    esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "  Wake reason: %d", wake);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Commands: A2=brightness A4=session A5=override A6=resume A7=sleep");

    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize Bluetooth */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* Register BLE callbacks */
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));

    /* Set MTU for OTA chunks */
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(517));

    /* Initialize Hall sensor GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HALL_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Initialize PWM */
    pwm_init();

    /* Create AC drive task (runs continuously for electrode health) */
    xTaskCreate(ac_drive_task, "ac_drive", 2048, NULL, 2, &ac_task_handle);

    /* Create LED control task (handles strobe timing) */
    xTaskCreate(led_task, "led_ctrl", 4096, NULL, 1, &led_task_handle);

    /* Create OTA task (deferred OTA ops — esp_ota_begin/end block too long for BLE callback) */
    xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, &ota_task_handle);

    /* Main loop - monitor Hall sensor and session state */
    while (1) {
        check_hall_sensor();
        
        /* Check if session ended */
        if (!session_active && led_task_handle == NULL && !in_ota_mode) {
            ESP_LOGI(TAG, "Session complete, entering sleep");
            enter_deep_sleep();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
