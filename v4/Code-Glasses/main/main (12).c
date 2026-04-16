/*******************************************************************************
 * Smart Glasses Firmware v4.9.12
 * 
 * Full LED control surface: strobe, static, and breathing modes.
 * All pattern timing runs on-device via gptimer ISR (strobe) and led_task
 * 10ms tick loop (breathing). BLE app only sends configuration commands.
 * 
 * FEATURES:
 * - AC differential drive: GPIO26/GPIO27 via gptimer 100µs ISR (100Hz AC,
 *   phase-synced to strobe edges for beat-free operation)
 * - Three LED modes: STROBE, STATIC, BREATHE
 * - DDS strobe: gptimer hardware ISR, ±100µs precision, sub-Hz to 50Hz
 * - Breathing engine: cosf sine / linear, configurable BPM, ratio, holds
 * - OTA firmware updates via BLE (deferred, with write drain)
 * - Power saving: -6dBm adv / -12dBm connected, 100-200ms adv interval
 * 
 * BLE COMMANDS (multi-byte to 0xFF01):
 * 
 *   COMMON:
 *   - A2 [brightness]     Set max tint 0-100%
 *   - A4 [minutes]        Set session duration 1-60 min
 *   - A7 00               Sleep immediately
 * 
 *   MODE SWITCHING:
 *   - A5 [duty]           Enter STATIC mode at given duty 0-100%
 *   - A6 00               Enter STROBE mode
 *   - B0 00               Enter BREATHE mode
 * 
 *   STROBE PARAMS (take effect immediately):
 *   - AB [freq]           Set strobe frequency 1-50 Hz
 *   - AC [duty_pct]       Set strobe duty cycle 10-90% (% of period dark)
 * 
 *   BREATHE PARAMS (take effect immediately):
 *   - B1 [bpm]            Set breathing rate 1-30 BPM
 *   - B2 [pct]            Set inhale ratio 10-90%
 *   - B3 [val]            Set hold-at-top 0-50 (×100ms, max 5s)
 *   - B4 [val]            Set hold-at-bottom 0-50
 *   - B5 [wave]           Waveform: 0=sine, 1=linear
 * 
 *   OTA:
 *   - A8 00               Start OTA mode
 *   - A9 00               Finish OTA (validate + reboot)
 *   - AA 00               Cancel OTA
 *   - AD 01               Confirm page (write buffer to flash)
 *   - AD 00               Reject page (discard, resend)
 * 
 * LEGACY: Single byte 0x00-0xFF → static mode at byte*100/255
 * 
 * CHANGELOG v4.9.12:
 * - BLE connection reliability pass, paired with webapp retry loops
 *   (controller v4.9.4, OTA v13). Addresses intermittent GATT errors
 *   that previously required multiple manual reconnect attempts.
 * - Advertising TX power: -12dBm → -6dBm. Roughly doubles link margin
 *   at the phone (+6dB ≈ 4× signal strength) so advertising packets
 *   and the connection handshake arrive well above the phone's noise
 *   floor. Biggest single contributor to unreliable first-try connects
 *   was marginal RSSI during handshake, not advertising rate.
 * - Connection TX power: remains at -12dBm. Once connected the link
 *   is established and weaker signal is sufficient — keeps per-session
 *   power draw close to v4.9.11 baseline.
 * - Advertising interval: 200-320ms → 100-200ms. Halves time between
 *   advertising events so phone scan windows align with ours sooner,
 *   cutting typical connect latency. Power cost is modest (see below).
 * - Est. idle current impact: ~+1-2mA average vs v4.9.11 (roughly
 *   +0.5-1mA from TX power bump during adv-only radio-on windows,
 *   +0.5-1mA from doubled advertising rate). Active session current
 *   essentially unchanged since connected TX stays at -12dBm.
 * - Boot banner updated to reflect new power/interval settings.
 *
 * CHANGELOG v4.9.11:
 * - AC frequency back to 100Hz (best lens charging / strongest tint)
 * - Added AC phase-sync: at each strobe clear→dark transition, AC phase is
 *   reset with alternating polarity. Every dark burst sees identical drive
 *   magnitude (no beat artifacts at any strobe freq), while alternating
 *   start polarity maintains DC balance across burst pairs.
 * - Previous versions traded tinting for beat rejection via high AC freq;
 *   phase-sync separates those concerns entirely.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
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
#include "esp_timer.h"
#include "esp_crc.h"
#include "driver/gptimer.h"
#include "soc/ledc_struct.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*******************************************************************************
 * VERSION AND IDENTIFICATION
 ******************************************************************************/
#define FIRMWARE_VERSION "4.9.12"
static const char *TAG = "SG_v4.9.12";

/*******************************************************************************
 * BLE CONFIGURATION - ACTIVE MODE (Power Optimized)
 ******************************************************************************/
#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID_CTRL    0xFF01  /* Control commands */
#define GATTS_CHAR_UUID_OTA     0xFF02  /* OTA data chunks */
#define GATTS_CHAR_UUID_STATUS  0xFF03  /* Status notifications */
#define GATTS_NUM_HANDLE        10      /* Service + 3 chars + CCCDs */

#define DEVICE_NAME             "Narbis_Edge"
#define GATTS_APP_ID            0

/* v4.9.12: Advertising interval 100-200ms (was 200-320ms in v4.9.11).
 * Halving the interval gets advertising packets in front of the phone's
 * scan window sooner, cutting typical first-try connect latency. Combined
 * with webapp retry loops (controller v4.9.4 / OTA v13) this should
 * deliver reliable first-attempt connects in most conditions.
 * Encoding: value × 0.625ms = interval in ms.
 *   0x0A0 = 160 × 0.625ms = 100ms
 *   0x140 = 320 × 0.625ms = 200ms */
#define ADV_INT_MIN             0x0A0
#define ADV_INT_MAX             0x140

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
 * AC + strobe: gptimer 100µs hardware ISR (DDS phase accumulator for strobe)
 * Breathing: led_task 10ms tick loop sets effective_duty
 ******************************************************************************/
#define AC_PERIOD_TICKS         1       /* 10ms tick for led_task (breathing mode) */
#define LED_TICK_MS             10      /* LED task tick for breathe mode */
#define AC_HALF_TICKS           50      /* 50 × 100µs = 5ms = 100Hz AC */
#define PHASE_FULL              100000U /* DDS wrap: 10000 ticks/sec × 10 for deci-Hz */
#define DEFAULT_SESSION_MIN     10      /* 10 minute session */
#define DEFAULT_BRIGHTNESS      100     /* 100% brightness */
#define DEFAULT_STROBE_DHZ      100       /* 10Hz default strobe (deci-Hz) */
#define MIN_STROBE_HZ           1
#define MAX_STROBE_HZ           50

/* OTA page buffer size — must match web app PAGE_SIZE */
#define OTA_PAGE_SIZE           4096

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
 * LED MODE DEFINITIONS
 ******************************************************************************/
typedef enum {
    LED_MODE_STROBE  = 0,
    LED_MODE_STATIC  = 1,
    LED_MODE_BREATHE = 2,
} led_mode_t;

/*******************************************************************************
 * GLOBAL STATE
 ******************************************************************************/
/* Session state */
static volatile bool session_active = false;
static volatile uint8_t brightness = DEFAULT_BRIGHTNESS;
static volatile uint32_t session_duration_ms = DEFAULT_SESSION_MIN * 60 * 1000;
static volatile uint32_t session_start_tick = 0;

/* LED mode */
static volatile led_mode_t led_mode = LED_MODE_STROBE;

/* Strobe parameters */
static volatile uint16_t strobe_dhz = DEFAULT_STROBE_DHZ;   /* Deci-Hz */
static volatile uint8_t strobe_duty_pct = 50;               /* 10-90% of period dark */
static volatile uint32_t strobe_dark_thresh = PHASE_FULL * 50 / 100;  /* Cached threshold */

/* Breathe parameters */
static volatile uint8_t breathe_bpm        = 6;    /* 1-30 BPM */
static volatile uint8_t breathe_inhale_pct = 40;   /* 10-90% of cycle is inhale */
static volatile uint8_t breathe_hold_top   = 0;    /* 0-50, units of 100ms */
static volatile uint8_t breathe_hold_bot   = 0;    /* 0-50, units of 100ms */
static volatile uint8_t breathe_wave       = 0;    /* 0=sine, 1=linear */

/* AC drive state - shared between tasks */
static volatile uint8_t effective_duty = 0;

/* Unified drive timer state (gptimer 100µs ISR — AC alternation + strobe) */
static gptimer_handle_t drive_timer = NULL;
static volatile uint32_t ac_tick = 0;
static volatile uint8_t ac_phase = 0;
static volatile uint32_t strobe_acc = 0;  /* DDS phase accumulator */

/* Strobe→AC phase-sync state (v4.9.11):
 * At each clear→dark strobe transition, AC phase is reset with alternating
 * polarity so every dark burst sees identical-magnitude AC drive (no beat),
 * while consecutive bursts start with opposite polarity (DC-balanced). */
static volatile uint8_t strobe_was_dark = 0;
static volatile uint8_t ac_reset_polarity = 0;

/* OTA state */
static volatile bool in_ota_mode = false;
static const esp_partition_t *ota_partition = NULL;
static esp_ota_handle_t ota_handle = 0;
static uint32_t ota_bytes_written = 0;

/* Page-based OTA transfer state */
static uint8_t ota_page_buf[OTA_PAGE_SIZE];
static uint16_t ota_page_offset = 0;
static uint16_t ota_page_num = 0;
static bool ota_page_pending = false;

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
 * ISR-SAFE PWM WRITES (direct LEDC register access, IRAM-resident)
 * 
 * The LEDC driver functions (ledc_set_duty etc.) live in flash, not IRAM,
 * so they crash if called from hardware ISR context. These write LEDC
 * registers directly.
 * 
 * ESP32: LEDC_LOW_SPEED_MODE = group index 1
 * PWM1 = channel 0 (GPIO27), PWM2 = channel 1 (GPIO26)
 ******************************************************************************/
static void IRAM_ATTR pwm1_set_isr(uint32_t raw_duty) {
    LEDC.channel_group[1].channel[0].duty.duty = raw_duty << 4;
    LEDC.channel_group[1].channel[0].conf1.duty_start = 1;
    LEDC.channel_group[1].channel[0].conf0.low_speed_update = 1;
}

static void IRAM_ATTR pwm2_set_isr(uint32_t raw_duty) {
    LEDC.channel_group[1].channel[1].duty.duty = raw_duty << 4;
    LEDC.channel_group[1].channel[1].conf1.duty_start = 1;
    LEDC.channel_group[1].channel[1].conf0.low_speed_update = 1;
}

static inline uint32_t IRAM_ATTR duty_to_raw_isr(uint8_t duty_pct) {
    if (duty_pct == 0) return 0;
    if (duty_pct > 100) duty_pct = 100;
    return 400 + (duty_pct * 623 / 100);
}

/*******************************************************************************
 * DRIVE TIMER (gptimer hardware timer, 100µs ISR)
 * 
 * True hardware interrupt — zero RTOS involvement, zero scheduling jitter.
 * Handles both AC electrode alternation and strobe phase in one ISR.
 * 
 * AC: toggle every AC_HALF_TICKS (50 × 100µs = 5ms = 100Hz). In strobe mode,
 *   AC phase is reset at each clear→dark transition with alternating polarity
 *   so every dark burst is identical in magnitude with DC balance preserved.
 * Strobe: DDS phase accumulator. Phase wraps at PHASE_FULL (100000).
 *   Increment per tick = strobe_dhz (deci-Hz). Average frequency is exact.
 *   Max per-cycle error = ±100µs.
 * 
 * Breathing/static: ISR reads effective_duty set by led_task (10ms loop).
 ******************************************************************************/
static bool IRAM_ATTR drive_timer_cb(gptimer_handle_t timer,
                                      const gptimer_alarm_event_data_t *edata,
                                      void *user_ctx) {
    /* ── OTA / session override ── */
    if (in_ota_mode || !session_active) {
        effective_duty = 0;
    }

    /* ── Strobe DDS phase accumulator with AC phase-sync ── */
    if (led_mode == LED_MODE_STROBE && session_active && !in_ota_mode) {
        strobe_acc += strobe_dhz;
        if (strobe_acc >= PHASE_FULL) strobe_acc -= PHASE_FULL;

        uint8_t is_dark = (strobe_acc < strobe_dark_thresh) ? 1 : 0;

        /* On clear→dark transition: reset AC phase with alternating polarity.
         * This makes every dark burst identical in magnitude (no beat) while
         * alternating consecutive bursts keeps DC balance across burst pairs. */
        if (is_dark && !strobe_was_dark) {
            ac_tick = 0;
            ac_phase = ac_reset_polarity;
            ac_reset_polarity ^= 1;
        }
        strobe_was_dark = is_dark;

        effective_duty = is_dark ? brightness : 0;
    }

    /* ── AC alternation ── */
    ac_tick++;
    if (ac_tick >= AC_HALF_TICKS) {
        ac_tick = 0;
        ac_phase = !ac_phase;
    }

    /* ── Apply PWM with AC phase ── */
    uint32_t raw = duty_to_raw_isr(effective_duty);
    if (ac_phase == 0) {
        pwm1_set_isr(raw);
        pwm2_set_isr(1);
    } else {
        pwm1_set_isr(1);
        pwm2_set_isr(raw);
    }

    return false;  /* no task wake needed */
}

static void strobe_start(void) {
    strobe_acc = 0;
    strobe_was_dark = 0;
    ac_reset_polarity = 0;
    strobe_dark_thresh = PHASE_FULL * strobe_duty_pct / 100;
    ESP_LOGI(TAG, "Strobe: %d.%dHz %d%% duty (phase-sync AC)",
             strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
}

static void strobe_stop(void) {
    /* Timer keeps running for AC drive.
     * Strobe stops because led_mode != LED_MODE_STROBE. */
}

static void strobe_update(void) {
    strobe_dark_thresh = PHASE_FULL * strobe_duty_pct / 100;
    if (led_mode == LED_MODE_STROBE && session_active) {
        strobe_start();
    }
}

static void drive_timer_init(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  /* 1MHz = 1µs resolution */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &drive_timer));

    gptimer_alarm_config_t alarm = {
        .alarm_count = 100,        /* 100µs period */
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(drive_timer, &alarm));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = drive_timer_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(drive_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(drive_timer));
    ESP_ERROR_CHECK(gptimer_start(drive_timer));

    ESP_LOGI(TAG, "Drive timer started (100µs gptimer ISR / 100Hz AC, phase-synced)");
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

/* Page-based CRC transfer status codes (sent via 0xFF03 notify) */
#define OTA_STATUS_PAGE_CRC     0x06    /* [0x06, page_hi, page_lo, crc3..crc0] (7 bytes) */
#define OTA_STATUS_PAGE_OK      0x07    /* [0x07, page_hi, page_lo] (3 bytes) */
#define OTA_STATUS_PAGE_RESEND  0x08    /* [0x08, page_hi, page_lo] (3 bytes) */

/* Page confirmation command (received on 0xFF01) */
#define OTA_CMD_PAGE_CONFIRM    0xAD    /* [0xAD, 0x01]=commit, [0xAD, 0x00]=resend */

static void send_ota_status(uint8_t status, uint8_t extra1, uint8_t extra2, uint8_t extra3) {
    if (!notifications_enabled || !is_connected) return;
    
    uint8_t notify_data[4] = {status, extra1, extra2, extra3};
    size_t len = (status == OTA_STATUS_PROGRESS) ? 4 : 
                 (status == OTA_STATUS_ERROR) ? 2 : 1;
    
    esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global, 
                                 status_char_handle, len, notify_data, false);
}

/* Send arbitrary-length OTA status notification (for page CRC etc.) */
static void send_ota_status_raw(uint8_t *data, size_t len) {
    if (!notifications_enabled || !is_connected) return;
    esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global,
                                 status_char_handle, len, data, false);
}

/*******************************************************************************
 * LED CONTROL TASK
 * 
 * Session management + breathing engine. Unified timer handles AC drive and
 * strobe. led_task handles: session timeout, breathing mode duty computation,
 * OTA pause, status logging.
 ******************************************************************************/
static void led_task(void *param) {
    uint32_t tick_count = 0;
    uint32_t last_log_tick = 0;
    
    ESP_LOGI(TAG, "LED task started - mode %d", led_mode);
    
    /* Start session */
    session_active = true;
    session_start_tick = xTaskGetTickCount();
    if (led_mode == LED_MODE_STROBE) strobe_start();
    
    while (1) {
        /* Check session timeout */
        uint32_t elapsed = (xTaskGetTickCount() - session_start_tick) * portTICK_PERIOD_MS;
        if (elapsed >= session_duration_ms) {
            ESP_LOGI(TAG, "Session ended after %lu minutes", elapsed / 60000);
            session_active = false;
            strobe_stop();
            effective_duty = 0;
            break;
        }
        
        /* Handle OTA mode - pause everything */
        if (in_ota_mode) {
            strobe_stop();
            effective_duty = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        /* ── BREATHE MODE — compute duty every 10ms tick ──────── */
        if (led_mode == LED_MODE_BREATHE) {
            uint32_t cycle_ms = 60000 / breathe_bpm;
            uint32_t hold_top_ms = (uint32_t)breathe_hold_top * 100;
            uint32_t hold_bot_ms = (uint32_t)breathe_hold_bot * 100;
            
            if (hold_top_ms + hold_bot_ms >= cycle_ms) {
                hold_top_ms = 0;
                hold_bot_ms = 0;
            }
            
            uint32_t breathing_ms = cycle_ms - hold_top_ms - hold_bot_ms;
            if (breathing_ms < 200) breathing_ms = 200;
            
            uint32_t inhale_ms = breathing_ms * breathe_inhale_pct / 100;
            uint32_t exhale_ms = breathing_ms - inhale_ms;
            if (inhale_ms < 100) inhale_ms = 100;
            if (exhale_ms < 100) exhale_ms = 100;
            
            uint32_t t = (tick_count * LED_TICK_MS) % cycle_ms;
            float frac = 0.0f;
            
            if (t < inhale_ms) {
                float p = (float)t / (float)inhale_ms;
                frac = (breathe_wave == 0) 
                    ? (1.0f - cosf((float)M_PI * p)) / 2.0f
                    : p;
            }
            else if (t < inhale_ms + hold_top_ms) {
                frac = 1.0f;
            }
            else if (t < inhale_ms + hold_top_ms + exhale_ms) {
                float p = (float)(t - inhale_ms - hold_top_ms) / (float)exhale_ms;
                frac = (breathe_wave == 0)
                    ? (1.0f + cosf((float)M_PI * p)) / 2.0f
                    : 1.0f - p;
            }
            /* else: hold at bottom, frac = 0 */
            
            effective_duty = (uint8_t)(frac * (float)brightness);
        }
        
        /* STROBE mode: gptimer ISR handles effective_duty, nothing to do here */
        /* STATIC mode: effective_duty set by command handler, nothing here */
        
        vTaskDelay(AC_PERIOD_TICKS);  /* 10ms tick */
        tick_count++;
        
        /* Log status every 30 seconds */
        if (xTaskGetTickCount() - last_log_tick >= pdMS_TO_TICKS(30000)) {
            last_log_tick = xTaskGetTickCount();
            uint32_t remaining_sec = (session_duration_ms - elapsed) / 1000;
            const char *mstr = (led_mode == LED_MODE_STROBE) ? "STROBE" :
                               (led_mode == LED_MODE_STATIC) ? "STATIC" : "BREATHE";
            ESP_LOGI(TAG, "%s duty=%d%% bright=%d%% %lu sec left", 
                     mstr, effective_duty, brightness, remaining_sec);
        }
    }
    
    /* Session ended */
    led_task_handle = NULL;
    vTaskDelete(NULL);
}

/*******************************************************************************
 * DEEP SLEEP
 ******************************************************************************/
static void enter_deep_sleep(void) {
    ESP_LOGI(TAG, "Entering deep sleep...");
    
    /* Clear lens and stop unified timer */
    effective_duty = 0;
    vTaskDelay(pdMS_TO_TICKS(10));  /* Let timer apply zero duty */
    gptimer_stop(drive_timer);
    gptimer_disable(drive_timer);
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
    ota_page_offset = 0;
    ota_page_num = 0;
    ota_page_pending = false;
    memset(ota_page_buf, 0xFF, OTA_PAGE_SIZE);
    effective_duty = 0;
    ESP_LOGI(TAG, "OTA started, partition: %s", ota_partition->label);
    send_ota_status(OTA_STATUS_READY, 0, 0, 0);
}

static void ota_do_finish(void) {
    if (!in_ota_mode) {
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA, 0, 0);
        return;
    }

    /* Wait for any pending page confirm (esp_ota_write in BTC task) to complete.
     * The OTA API is not thread-safe — watch ota_bytes_written until stable. */
    {
        uint32_t last_written = 0;
        int stable_count = 0;
        while (stable_count < 2) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (ota_bytes_written == last_written) {
                stable_count++;
            } else {
                last_written = ota_bytes_written;
                stable_count = 0;
            }
        }
    }

    /* Write any remaining data in the page buffer (last partial page) */
    if (ota_page_offset > 0) {
        ESP_LOGI(TAG, "OTA: Writing final partial page (%d bytes)", ota_page_offset);
        esp_err_t err = esp_ota_write(ota_handle, ota_page_buf, ota_page_offset);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: Final page write failed: %s", esp_err_to_name(err));
            send_ota_status(OTA_STATUS_ERROR, OTA_ERR_WRITE, 0, 0);
            in_ota_mode = false;
            return;
        }
    }

    ESP_LOGI(TAG, "OTA: Finishing, %lu bytes in %d pages + %d remainder",
             (unsigned long)ota_bytes_written, ota_page_num, ota_page_offset);

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

    ESP_LOGI(TAG, "OTA complete! %lu bytes written. Rebooting...", 
             (unsigned long)ota_bytes_written);
    send_ota_status(OTA_STATUS_SUCCESS, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static void ota_do_cancel(void) {
    if (in_ota_mode) {
        esp_ota_abort(ota_handle);
        in_ota_mode = false;
        ota_bytes_written = 0;
        ota_page_offset = 0;
        ota_page_num = 0;
        ota_page_pending = false;
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
    /* Single-byte: legacy duty control → enters static mode */
    if (len == 1) {
        uint8_t byte = data[0];
        uint8_t duty = (byte * 100) / 255;
        ESP_LOGI(TAG, "Legacy duty: %d%% (byte 0x%02X)", duty, byte);
        brightness = duty;
        strobe_stop();
        led_mode = LED_MODE_STATIC;
        effective_duty = brightness;
        return;
    }
    
    /* Multi-byte commands */
    if (len < 2) return;
    
    uint8_t cmd = data[0];
    uint8_t arg = data[1];
    
    switch (cmd) {
        /* ── COMMON ────────────────────────────────────────── */
        case 0xA2:  /* Set brightness / max tint */
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
            
        /* ── MODE SWITCHING ────────────────────────────────── */
        case 0xA5:  /* Enter STATIC mode */
            if (arg > 100) arg = 100;
            brightness = arg;
            strobe_stop();
            led_mode = LED_MODE_STATIC;
            effective_duty = brightness;
            ESP_LOGI(TAG, "Mode: STATIC @ %d%%", arg);
            break;
            
        case 0xA6:  /* Enter STROBE mode */
            led_mode = LED_MODE_STROBE;
            if (session_active) strobe_start();
            ESP_LOGI(TAG, "Mode: STROBE %d.%dHz %d%% duty", 
                     strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
            break;
            
        case 0xA7:  /* Sleep immediately */
            ESP_LOGI(TAG, "Sleep command received");
            session_active = false;
            enter_deep_sleep();
            break;
            
        /* ── STROBE PARAMS ─────────────────────────────────── */
        case 0xAB:  /* Set strobe frequency */
            if (arg < MIN_STROBE_HZ) arg = MIN_STROBE_HZ;
            if (arg > MAX_STROBE_HZ) arg = MAX_STROBE_HZ;
            strobe_dhz = arg * 10;
            strobe_update();
            ESP_LOGI(TAG, "Strobe freq: %dHz", arg);
            break;
            
        case 0xAC:  /* Set strobe duty cycle */
            if (arg < 10) arg = 10;
            if (arg > 90) arg = 90;
            strobe_duty_pct = arg;
            strobe_update();
            ESP_LOGI(TAG, "Strobe duty: %d%%", strobe_duty_pct);
            break;
            
        /* ── OTA ───────────────────────────────────────────── */
        case 0xA8:  /* Start OTA (deferred to OTA task) */
            ESP_LOGI(TAG, "OTA: Start command (deferred)");
            session_active = false;
            strobe_stop();
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
            
        case OTA_CMD_PAGE_CONFIRM:  /* 0xAD: Page confirm/resend */
            if (len >= 2 && in_ota_mode && ota_page_pending) {
                if (data[1] == 0x01) {
                    /* Page confirmed — write buffer to flash */
                    esp_err_t err = esp_ota_write(ota_handle, ota_page_buf, OTA_PAGE_SIZE);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "OTA: Flash write failed page %d: %s",
                                 ota_page_num, esp_err_to_name(err));
                        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_WRITE, 0, 0);
                        esp_ota_abort(ota_handle);
                        in_ota_mode = false;
                        ota_page_pending = false;
                        return;
                    }
                    ESP_LOGI(TAG, "OTA: Page %d committed to flash", ota_page_num);

                    uint8_t ok_status[] = {
                        OTA_STATUS_PAGE_OK,
                        (uint8_t)(ota_page_num >> 8),
                        (uint8_t)(ota_page_num & 0xFF)
                    };
                    send_ota_status_raw(ok_status, sizeof(ok_status));

                    ota_page_num++;
                    ota_page_offset = 0;
                    ota_page_pending = false;
                } else {
                    /* Page rejected — discard, web app will resend */
                    ESP_LOGW(TAG, "OTA: Page %d rejected, awaiting resend", ota_page_num);
                    ota_page_offset = 0;
                    ota_bytes_written -= OTA_PAGE_SIZE;
                    ota_page_pending = false;

                    uint8_t resend_status[] = {
                        OTA_STATUS_PAGE_RESEND,
                        (uint8_t)(ota_page_num >> 8),
                        (uint8_t)(ota_page_num & 0xFF)
                    };
                    send_ota_status_raw(resend_status, sizeof(resend_status));
                }
            }
            break;
            
        /* ── BREATHE MODE + PARAMS ─────────────────────────── */
        case 0xB0:  /* Enter BREATHE mode */
            strobe_stop();
            led_mode = LED_MODE_BREATHE;
            ESP_LOGI(TAG, "Mode: BREATHE %dBPM %d/%d %s", 
                     breathe_bpm, breathe_inhale_pct, 100 - breathe_inhale_pct,
                     breathe_wave == 0 ? "sine" : "linear");
            break;
            
        case 0xB1:  /* Set breathe BPM */
            if (arg < 1) arg = 1;
            if (arg > 30) arg = 30;
            breathe_bpm = arg;
            ESP_LOGI(TAG, "Breathe BPM: %d", breathe_bpm);
            break;
            
        case 0xB2:  /* Set inhale ratio */
            if (arg < 10) arg = 10;
            if (arg > 90) arg = 90;
            breathe_inhale_pct = arg;
            ESP_LOGI(TAG, "Breathe inhale: %d%% exhale: %d%%", arg, 100 - arg);
            break;
            
        case 0xB3:  /* Set hold-at-top (100ms units) */
            if (arg > 50) arg = 50;
            breathe_hold_top = arg;
            ESP_LOGI(TAG, "Breathe hold top: %dms", arg * 100);
            break;
            
        case 0xB4:  /* Set hold-at-bottom (100ms units) */
            if (arg > 50) arg = 50;
            breathe_hold_bot = arg;
            ESP_LOGI(TAG, "Breathe hold bot: %dms", arg * 100);
            break;
            
        case 0xB5:  /* Set breathe waveform */
            breathe_wave = (arg > 0) ? 1 : 0;
            ESP_LOGI(TAG, "Breathe wave: %s", breathe_wave == 0 ? "sine" : "linear");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
            break;
    }
}

/*******************************************************************************
 * OTA DATA PROCESSING
 ******************************************************************************/
/*******************************************************************************
 * OTA DATA PROCESSING — Page-Based with CRC
 *
 * Buffers chunks in ota_page_buf (4KB). When full, computes CRC32 and notifies
 * the web app. Web app compares CRCs: [0xAD, 0x01] to commit, [0xAD, 0x00]
 * to resend. Enables fast WriteWithoutResponse with per-page integrity.
 ******************************************************************************/
static void process_ota_data(uint8_t *data, uint16_t len) {
    if (!in_ota_mode) {
        ESP_LOGW(TAG, "OTA data received but not in OTA mode");
        send_ota_status(OTA_STATUS_ERROR, OTA_ERR_NOT_IN_OTA, 0, 0);
        return;
    }

    if (ota_page_pending) {
        ESP_LOGW(TAG, "OTA: Data received while page pending confirmation");
        return;
    }

    uint16_t data_offset = 0;
    while (data_offset < len) {
        uint16_t space = OTA_PAGE_SIZE - ota_page_offset;
        uint16_t copy_len = (len - data_offset < space) ? (len - data_offset) : space;

        memcpy(ota_page_buf + ota_page_offset, data + data_offset, copy_len);
        ota_page_offset += copy_len;
        data_offset += copy_len;
        ota_bytes_written += copy_len;

        /* Page full — compute CRC and notify web app */
        if (ota_page_offset >= OTA_PAGE_SIZE) {
            uint32_t crc = esp_crc32_le(0, ota_page_buf, OTA_PAGE_SIZE);

            ESP_LOGI(TAG, "OTA: Page %d full (%lu total), CRC=0x%08lX",
                     ota_page_num, (unsigned long)ota_bytes_written, (unsigned long)crc);

            uint8_t status[] = {
                OTA_STATUS_PAGE_CRC,
                (uint8_t)(ota_page_num >> 8),
                (uint8_t)(ota_page_num & 0xFF),
                (uint8_t)((crc >> 24) & 0xFF),
                (uint8_t)((crc >> 16) & 0xFF),
                (uint8_t)((crc >> 8) & 0xFF),
                (uint8_t)(crc & 0xFF)
            };
            send_ota_status_raw(status, sizeof(status));
            ota_page_pending = true;

            if (data_offset < len) {
                ESP_LOGW(TAG, "OTA: %d bytes crossed page boundary (discarded)",
                         len - data_offset);
            }
            break;
        }
    }
}

/*******************************************************************************
 * BLE ADVERTISING PARAMETERS (Power Optimized)
 ******************************************************************************/
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = ADV_INT_MIN,   /* 100ms (v4.9.12) */
    .adv_int_max        = ADV_INT_MAX,   /* 200ms (v4.9.12) */
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
            
            /* v4.9.12: Dual TX power scheme for reliable connects + low power.
             *   - ADV / DEFAULT at -6dBm: advertising packets arrive at the
             *     phone with ~4× the signal strength of v4.9.11, giving the
             *     handshake plenty of link margin. Biggest single win for
             *     first-try connect reliability.
             *   - CONN_HDL0 at -12dBm: once connected, the link is already
             *     established and a weaker signal is sufficient. Keeps
             *     per-session power draw close to v4.9.11 baseline.
             * Previously all three were -12dBm (overly conservative for adv). */
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,   ESP_PWR_LVL_N6);
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,       ESP_PWR_LVL_N6);
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
    ESP_LOGI(TAG, "  Boot: STROBE %d.%dHz %d%% duty @ %d%% brightness",
             DEFAULT_STROBE_DHZ / 10, DEFAULT_STROBE_DHZ % 10, 
             strobe_duty_pct, DEFAULT_BRIGHTNESS);
    ESP_LOGI(TAG, "  Session: %d minutes", DEFAULT_SESSION_MIN);
    ESP_LOGI(TAG, "  AC Drive: 100Hz gptimer ISR, strobe phase-synced (GPIO26/27)");
    ESP_LOGI(TAG, "  Power: -6dBm TX (adv) / -12dBm (conn), 100-200ms adv interval");
    
    /* Get partition info */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "  Partition: %s @ 0x%lx", running->label, running->address);
    
    /* Log wake reason */
    esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "  Wake reason: %d", wake);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Modes: A5=static A6=strobe B0=breathe");
    ESP_LOGI(TAG, "Common: A2=brightness A4=session A7=sleep");
    ESP_LOGI(TAG, "Strobe: AB=freq AC=duty | Breathe: B1-B5");

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

    /* Start hardware drive timer (100µs gptimer ISR — AC + strobe) */
    drive_timer_init();

    /* Create LED control task (session management + breathing) */
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
