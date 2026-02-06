/**
 * Smart Glasses LED Controller v4.0
 * 
 * Features:
 *   - 10-minute timed session with auto-sleep at end
 *   - Linear progression of strobe frequency and breathing pattern
 *   - Strobe: start_hz -> end_hz over session duration (default 12->8 Hz)
 *   - Breathing: inhale/exhale fixed, hold_in/hold_out 0->end over session
 *     Default: 4s-0s-4s-0s -> 4s-4s-4s-4s
 *   - PWM1 only (GPIO27), PWM2 commented out (hardware tied together)
 *   - BLE commands for configuring start/end parameters
 *   - BLE static override (0x05) to hold fixed duty
 *   - Hall sensor sleep/wake (close arms = sleep, open = wake)
 *   - Power optimized: 80MHz CPU, 1kHz PWM, -12dBm BLE TX, 1-2s adv
 * 
 * BLE Commands:
 *   Single byte (0x00-0xFF)                    - Legacy: direct duty (0=clear, 255=full dark)
 *   0xA1 [start_hz] [end_hz]                   - Set strobe range (1-50 Hz)
 *   0xA2 [brightness]                           - Set brightness 0-100%
 *   0xA3 [inh] [hold_in_end] [exh] [hold_out_end] - Set breathing (x0.1s)
 *   0xA4 [duration_minutes]                     - Set session duration (1-60 min)
 *   0xA5 [duty]                                 - Static override (0-100%), stops program
 *   0xA6                                        - Resume / restart session
 *   0xA7                                        - Enter sleep immediately
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
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

#define GATTS_SERVICE_UUID_TEST   0x00FF
#define GATTS_CHAR_UUID_TEST      0xFF01
#define GATTS_NUM_HANDLE_TEST     4

#define DEVICE_NAME            "Smart_Glasses"
#define TEST_APP_ID            0

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)

static const char *TAG = "SmartGlasses";

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x0640,  // 1000ms
    .adv_int_max        = 0x0C80,  // 2000ms
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Advertising data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static uint16_t gatt_service_handle = 0;
static esp_gatt_char_prop_t gatt_property = 0;
static esp_attr_value_t gatt_char_val = {
    .attr_max_len = 100,
    .attr_len = 0,
    .attr_value = NULL,
};

//*********************************************************** */
// Hardware Configuration
//*********************************************************** */
#define HALL_PIN GPIO_NUM_4
#define SLEEP_HALL_WAIT_TIME 5

#define PWM1_TIMER              LEDC_TIMER_0
#define PWM1_MODE               LEDC_LOW_SPEED_MODE
#define PWM1_OUTPUT_IO          (27)
#define PWM1_CHANNEL            LEDC_CHANNEL_0
#define PWM1_DUTY_RES           LEDC_TIMER_10_BIT
#define PWM1_FREQUENCY          (1000)

// PWM2 commented out - hardware appears tied to PWM1
// #define PWM2_TIMER              LEDC_TIMER_0
// #define PWM2_MODE               LEDC_LOW_SPEED_MODE
// #define PWM2_OUTPUT_IO          (26)
// #define PWM2_CHANNEL            LEDC_CHANNEL_1
// #define PWM2_DUTY_RES           LEDC_TIMER_10_BIT
// #define PWM2_FREQUENCY          (1000)

//*********************************************************** */
// Session Parameters
//*********************************************************** */
static uint8_t brightness = 100;         // Brightness 0-100%

// Strobe progression
static uint8_t start_hz = 12;           // Starting strobe frequency
static uint8_t end_hz = 8;              // Ending strobe frequency

// Breathing progression (x0.1 seconds)
// Start: inhale-0-exhale-0, End: inhale-hold_in_end-exhale-hold_out_end
static uint8_t inhale_time = 40;         // 4.0s (fixed throughout)
static uint8_t exhale_time = 40;         // 4.0s (fixed throughout)
static uint8_t hold_in_end = 40;         // End hold_in: 4.0s (starts at 0)
static uint8_t hold_out_end = 40;        // End hold_out: 4.0s (starts at 0)

// Session timing
static uint8_t session_minutes = 10;     // Session duration in minutes
static uint32_t session_start_tick = 0;  // When session started
static uint8_t session_active = 0;       // Is a timed session running?
static uint8_t session_ended = 0;        // Has session completed (trigger sleep)?

// Static override mode
static uint8_t override_active = 0;
static uint8_t override_duty = 0;

//*********************************************************** */
// PWM Functions
//*********************************************************** */
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

    // PWM2 commented out
    // ledc_channel_config_t pwm2_channel = { ... };
}

// LCD lens deadzone: below ~40% PWM (raw ~400) the lens doesn't visibly change.
// Remap: duty 0 = raw 0 (fully clear), duty 1-100 = raw 400-1024 (visible range)
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

// static void pwm2_setduty(uint32_t duty) {
//     duty = 1024 * duty / 100;
//     ledc_set_duty(PWM2_MODE, PWM2_CHANNEL, duty);
//     ledc_update_duty(PWM2_MODE, PWM2_CHANNEL);
// }

//*********************************************************** */
// BLE Handlers
//*********************************************************** */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                         esp_gatt_if_t gatts_if,
                                         esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
    {
        esp_ble_gap_set_device_name(DEVICE_NAME);
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&adv_data);

        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_SERVICE_UUID_TEST },
                },
            },
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE_TEST);
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        gatt_service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(gatt_service_handle);
        gatt_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = { .uuid16 = GATTS_CHAR_UUID_TEST },
        };
        esp_ble_gatts_add_char(gatt_service_handle, &char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               gatt_property, &gatt_char_val, NULL);
        break;

    case ESP_GATTS_WRITE_EVT:
        // ALWAYS send response first to prevent GATT stack from blocking
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }
        
        if (param->write.len > 0) {
            ESP_LOGI(TAG, "BLE Write: %d bytes", param->write.len);
            for (int i = 0; i < param->write.len; i++) {
                ESP_LOGI(TAG, "Byte[%d]: 0x%02X", i, param->write.value[i]);
            }
            
            // LEGACY: Single byte write = direct duty (0-255 mapped to 0-100%)
            // 0x00 = clear (0% duty), 0xFF = full dark (100% duty)
            // Full 0x00-0xFF range reserved for legacy app compatibility
            if (param->write.len == 1) {
                uint8_t raw = param->write.value[0];
                override_duty = (uint8_t)((uint16_t)raw * 100 / 255);
                override_active = 1;
                session_active = 0;
                pwm1_setduty(override_duty);
                ESP_LOGI(TAG, "Legacy cmd: 0x%02X -> %d%% duty", raw, override_duty);
            } else {
            
            // NEW COMMANDS: Multi-byte, first byte 0xA0+ to avoid legacy collision
            uint8_t cmd = param->write.value[0];
            switch (cmd) {
                case 0xA1:  // Set strobe range: [0xA1] [start_hz] [end_hz]
                    if (param->write.len >= 3) {
                        start_hz = param->write.value[1];
                        end_hz = param->write.value[2];
                        if (start_hz < 1) start_hz = 1;
                        if (start_hz > 50) start_hz = 50;
                        if (end_hz < 1) end_hz = 1;
                        if (end_hz > 50) end_hz = 50;
                        override_active = 0;
                        session_start_tick = xTaskGetTickCount();
                        session_active = 1;
                        session_ended = 0;
                        ESP_LOGI(TAG, "Strobe: %d->%d Hz", start_hz, end_hz);
                    }
                    break;
                case 0xA2:  // Set brightness: [0xA2] [brightness 0-100]
                    if (param->write.len >= 2) {
                        brightness = param->write.value[1];
                        if (brightness > 100) brightness = 100;
                        override_active = 0;
                        ESP_LOGI(TAG, "Brightness: %d%%", brightness);
                    }
                    break;
                case 0xA3:  // Set breathing: [0xA3] [inh] [hold_in_end] [exh] [hold_out_end]
                    if (param->write.len >= 5) {
                        inhale_time = param->write.value[1];
                        hold_in_end = param->write.value[2];
                        exhale_time = param->write.value[3];
                        hold_out_end = param->write.value[4];
                        override_active = 0;
                        session_start_tick = xTaskGetTickCount();
                        session_active = 1;
                        session_ended = 0;
                        ESP_LOGI(TAG, "Breathing: %.1f/0->%.1f/%.1f/0->%.1f",
                                 inhale_time/10.0f, hold_in_end/10.0f,
                                 exhale_time/10.0f, hold_out_end/10.0f);
                    }
                    break;
                case 0xA4:  // Set session duration: [0xA4] [minutes]
                    if (param->write.len >= 2) {
                        session_minutes = param->write.value[1];
                        if (session_minutes < 1) session_minutes = 1;
                        if (session_minutes > 60) session_minutes = 60;
                        override_active = 0;
                        session_start_tick = xTaskGetTickCount();
                        session_active = 1;
                        session_ended = 0;
                        ESP_LOGI(TAG, "Session: %d minutes", session_minutes);
                    }
                    break;
                case 0xA5:  // Static override: [0xA5] [duty 0-100]
                    if (param->write.len >= 2) {
                        override_duty = param->write.value[1];
                        if (override_duty > 100) override_duty = 100;
                        override_active = 1;
                        session_active = 0;
                        pwm1_setduty(override_duty);
                        ESP_LOGI(TAG, "Override: static %d%%", override_duty);
                    }
                    break;
                case 0xA6:  // Resume / restart session: [0xA6]
                    override_active = 0;
                    session_start_tick = xTaskGetTickCount();
                    session_active = 1;
                    session_ended = 0;
                    ESP_LOGI(TAG, "Session restarted");
                    break;
                case 0xA7:  // Sleep immediately: [0xA7]
                    ESP_LOGI(TAG, "BLE sleep command received");
                    session_ended = 1;
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
                    break;
            }
            } // end else (non-legacy)
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Client connected");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Client disconnected, restarting advertising");
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
            gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "Reg failed");
            return;
        }
    }
    gatts_profile_event_handler(event, gatts_if, param);
}

//*********************************************************** */
// LED Effect Task
//*********************************************************** */
static void led_task(void *param)
{
    // Breathing state
    uint8_t breath_phase = 0;  // 0=inhale, 1=hold_in, 2=exhale, 3=hold_out
    uint32_t phase_start = xTaskGetTickCount();
    float breath_brightness = 0.0f;
    uint8_t strobe_state = 0;
    
    while (1) {
        // If BLE override is active, skip all logic
        if (override_active) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        
        // If session not active, idle
        if (!session_active) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        
        uint32_t now = xTaskGetTickCount();
        
        // Calculate session progress (0.0 to 1.0)
        uint32_t session_duration_ticks = session_minutes * 60 * 1000 / portTICK_PERIOD_MS;
        uint32_t elapsed_ticks = now - session_start_tick;
        float progress = (float)elapsed_ticks / (float)session_duration_ticks;
        if (progress > 1.0f) progress = 1.0f;
        
        // Check if session is complete
        if (elapsed_ticks >= session_duration_ticks) {
            ESP_LOGI(TAG, "Session complete - entering sleep");
            pwm1_setduty(0);
            session_active = 0;
            session_ended = 1;
            continue;
        }
        
        // Log progress every 30 seconds
        static uint32_t last_log = 0;
        if (now - last_log >= (30000 / portTICK_PERIOD_MS)) {
            last_log = now;
            float current_hz = start_hz + (end_hz - start_hz) * progress;
            float current_hold_in = hold_in_end * progress / 10.0f;
            float current_hold_out = hold_out_end * progress / 10.0f;
            uint32_t remaining_s = (session_duration_ticks - elapsed_ticks) * portTICK_PERIOD_MS / 1000;
            ESP_LOGI(TAG, "Progress: %.0f%% | Hz: %.1f | Breath: %.1f/%.1f/%.1f/%.1f | Remaining: %lus",
                     progress * 100, current_hz,
                     inhale_time/10.0f, current_hold_in,
                     exhale_time/10.0f, current_hold_out,
                     (unsigned long)remaining_s);
        }
        
        // Calculate current strobe frequency (linear interpolation)
        float current_hz = start_hz + (end_hz - start_hz) * progress;
        if (current_hz < 1.0f) current_hz = 1.0f;
        uint32_t strobe_delay_ms = (uint32_t)(500.0f / current_hz);
        
        // Calculate current breathing hold times (linear from 0 to end)
        uint8_t current_hold_in = (uint8_t)(hold_in_end * progress);
        uint8_t current_hold_out = (uint8_t)(hold_out_end * progress);
        
        // Calculate breathing brightness
        uint32_t phase_durations[4] = {
            inhale_time * 100 / portTICK_PERIOD_MS,
            current_hold_in * 100 / portTICK_PERIOD_MS,
            exhale_time * 100 / portTICK_PERIOD_MS,
            current_hold_out * 100 / portTICK_PERIOD_MS
        };
        
        // Advance phase if current one is complete (or zero duration)
        uint32_t breath_elapsed = now - phase_start;
        uint8_t phase_checks = 0;
        while (phase_checks < 4 && (phase_durations[breath_phase] == 0 || breath_elapsed >= phase_durations[breath_phase])) {
            breath_phase = (breath_phase + 1) % 4;
            phase_start = now;
            breath_elapsed = 0;
            phase_checks++;
        }
        
        float bp = (float)breath_elapsed / (float)phase_durations[breath_phase];
        if (bp > 1.0f) bp = 1.0f;
        
        switch (breath_phase) {
            case 0: breath_brightness = bp * 100.0f; break;          // Inhale: 0->100
            case 1: breath_brightness = 100.0f; break;               // Hold in: 100
            case 2: breath_brightness = (1.0f - bp) * 100.0f; break; // Exhale: 100->0
            case 3: breath_brightness = 0.0f; break;                 // Hold out: 0
        }
        
        // Calculate final duty
        uint8_t duty = (uint8_t)(breath_brightness * brightness / 100);
        
        // Apply strobe (75% duty: dark 3/4, clear 1/4)
        uint32_t strobe_on_ms = strobe_delay_ms * 3 / 2;   // dark time (75% of full cycle)
        uint32_t strobe_off_ms = strobe_delay_ms / 2;       // clear time (25% of full cycle)
        
        // Dark phase
        pwm1_setduty(duty);
        vTaskDelay(strobe_on_ms / portTICK_PERIOD_MS);
        
        // Clear phase
        pwm1_setduty(0);
        vTaskDelay(strobe_off_ms / portTICK_PERIOD_MS);
    }
}

//*********************************************************** */
// Sleep Functions
//*********************************************************** */
static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep...");
    
    // Zero PWM output before sleep
    ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, 0);
    ledc_update_duty(PWM1_MODE, PWM1_CHANNEL);
    
    // Configure wake source and sleep - same as original
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
    
    // Session ended - go to sleep
    if (session_ended) {
        enter_deep_sleep();
    }
    
    // Hall sensor closed for SLEEP_HALL_WAIT_TIME seconds
    if (gpio_get_level(HALL_PIN) == 1) {
        low_level_duration++;
        if (low_level_duration >= SLEEP_HALL_WAIT_TIME) {
            enter_deep_sleep();
        }
    } else {
        low_level_duration = 0;
    }
}

//*********************************************************** */
// Main
//*********************************************************** */
void app_main(void)
{
    esp_err_t ret;

    // Check Hall sensor ONLY after deep sleep wake (not cold boot/power-on)
    // On cold boot, always proceed to full initialization
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
        
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Longer settle time for reliable read
        
        if (gpio_get_level(HALL_PIN) == 1) {
            // Arms still closed - go back to sleep
            esp_sleep_enable_timer_wakeup(1000000);
            esp_sleep_enable_ext0_wakeup(HALL_PIN, 0);
            esp_deep_sleep_start();
        }
    }

    ESP_LOGI(TAG, "Booting v4.0...");
    ESP_LOGI(TAG, "Wake reason: %d", wakeup_reason);

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS OK");

    // Initialize BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) { ESP_LOGE(TAG, "BT ctrl init fail: %s", esp_err_to_name(ret)); }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) { ESP_LOGE(TAG, "BT ctrl enable fail: %s", esp_err_to_name(ret)); }
    ret = esp_bluedroid_init();
    if (ret) { ESP_LOGE(TAG, "Bluedroid init fail: %s", esp_err_to_name(ret)); }
    ret = esp_bluedroid_enable();
    if (ret) { ESP_LOGE(TAG, "Bluedroid enable fail: %s", esp_err_to_name(ret)); }
    ESP_LOGI(TAG, "BLE stack OK");
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(TEST_APP_ID));

    // Reduce BLE TX power
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N12);

    // Initialize Hall sensor GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HALL_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Initialize PWM
    PWM_Init();

    // Start session immediately on boot
    session_start_tick = xTaskGetTickCount();
    session_active = 1;
    session_ended = 0;

    // Create LED task
    xTaskCreate(led_task, "led_task", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "Smart Glasses v4.0");
    ESP_LOGI(TAG, "BLE Name: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "Session: %d min | Strobe: %d->%d Hz", session_minutes, start_hz, end_hz);
    ESP_LOGI(TAG, "Breathing: %.1f/0->%.1f/%.1f/0->%.1f",
             inhale_time/10.0f, hold_in_end/10.0f,
             exhale_time/10.0f, hold_out_end/10.0f);
    ESP_LOGI(TAG, "CPU: 80MHz | PWM1 only | BLE: -12dBm");
    ESP_LOGI(TAG, "============================================");

    // Main loop
    while (1) {
        check_sleep_condition();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
