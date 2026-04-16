/*******************************************************************************
 * Smart Glasses Firmware v4.11.1
 * 
 * Full LED control surface: strobe, static, and breathing modes.
 * All pattern timing runs on-device via gptimer ISR (strobe) and led_task
 * 10ms tick loop (breathing). BLE app only sends configuration commands.
 *
 * Three on-device programs cycled by a short magnet tap (0.3-4s):
 *   1. BREATHE        — 6 BPM sine, lens tint follows waveform (default at boot)
 *   2. BREATHE+STROBE — 10Hz strobe whose dark-phase duty is modulated by the
 *                       breathing waveform (strobe "breathes")
 *   3. STROBE         — plain 10Hz strobe, no breathing
 * Long magnet close (>=5s) still enters deep sleep.
 *
 * BLE ADVERTISING AUTO-OFF (v4.11.0, hardened in v4.11.1):
 *   To minimize idle current when the device is used standalone (hall-only,
 *   no app), the entire BLE stack is torn down after BLE_IDLE_TIMEOUT_MS
 *   of no connection. v4.11.0 only called esp_ble_gap_stop_advertising(),
 *   which left the BT controller and Bluedroid stack running and still
 *   scheduling periodic housekeeping RF bursts (visible as sporadic ~1.5A
 *   spikes in current traces). v4.11.1 escalates: full teardown via
 *   esp_bluedroid_disable/deinit + esp_bt_controller_disable/deinit.
 *   Radio is completely off; no RF activity of any kind.
 *
 *   Expected usage: user opens arms (boots/wakes), has a 60-second window
 *   to connect from the app. If no connection occurs in that window, BLE
 *   tears down fully and the device continues running its current
 *   hall-selected program with the radio cold.
 *
 *   Re-arming: any hall event (tap or hold) re-initializes the BLE stack
 *   from scratch and starts advertising. Cost is ~100ms of init time,
 *   which is imperceptible between the user reaching for the magnet and
 *   then reaching for their phone to connect. A long close (>=5s) still
 *   enters deep sleep normally; on wake, BLE starts fresh.
 *
 *   While connected: timeout is disabled. On disconnect, a new 60s window
 *   starts so the user can reconnect if they want.
 *   During OTA: timeout is blocked (same pattern as sleep blocking).
 *
 *   Idle current: ~8mA during active session with BLE down (just ESP32
 *   + lens drive). During the 60s advertising window after boot/wake/
 *   disconnect, idle is ~15mA (v4.9.12 advertising config).
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
 * CHANGELOG v4.11.1:
 * - BLE auto-off hardened: full BT stack teardown instead of just stopping
 *   advertising. v4.11.0 left the BT controller and Bluedroid running,
 *   which scheduled periodic housekeeping RF bursts (~1.5A spikes visible
 *   in current traces every few seconds). v4.11.1 tears down via
 *   esp_bluedroid_disable + esp_bluedroid_deinit +
 *   esp_bt_controller_disable + esp_bt_controller_deinit. Radio is cold.
 * - Added ble_stack_init() and ble_stack_teardown() helpers. Both are
 *   idempotent; state tracked via ble_stack_up flag.
 * - Re-arm path now re-initializes the full stack. Cost is ~100ms,
 *   acceptable in the user-facing latency budget (tap magnet, wait,
 *   open app, connect).
 * - Handle invalidation: gatts_if_global, service_handle, char_handles
 *   all become invalid across teardown. They get repopulated by the GATT
 *   event handler when the new GATT app registers. Code that uses these
 *   (notifications, indicate) already guards on is_connected, which is
 *   false while BLE is down.
 *
 * CHANGELOG v4.11.0:
 * - BLE advertising auto-off after 60 seconds of no connection. Cuts idle
 *   current from ~15mA (advertising at -6dBm, 100-200ms interval, inherited
 *   from v4.9.12) back down to ~11mA baseline when device is used standalone
 *   via hall-only. Matches the standalone-first UX that the hall program
 *   cycling (v4.10.0) is built around.
 * - Behavior: on boot or wake from deep sleep, advertising starts with a
 *   60s deadline. If no client connects, advertising is stopped via
 *   esp_ble_gap_stop_advertising(). BT controller and stack stay up so
 *   re-arming is instant. Any hall event (tap for program advance, or
 *   start of a long hold) resets the deadline and restarts advertising if
 *   it was off. While connected: deadline is disabled. On disconnect:
 *   new 60s window starts. During OTA: timeout is blocked.
 * - Trade: user must perform a hall gesture to reconnect after 60s of
 *   standalone use. Acceptable given the design intent (standalone OR
 *   connected, not both simultaneously for long periods).
 * - Note: the webapp side must be updated to handle the case where the
 *   device is running but not advertising — connect attempts will fail
 *   with "device not found," same as if it were deep-sleeping. User guidance:
 *   "tap the magnet once to wake BLE, then connect."
 * - Implementation: deadline checked in app_main 1Hz loop (already runs for
 *   session-end sleep). Single volatile tick deadline, zero new tasks.
 *
 * CHANGELOG v4.10.1:
 * - BLE supervision timeout 4s -> 20s. The 4-second .timeout=400 value
 *   was firing during esp_ota_begin(OTA_SIZE_UNKNOWN) partition erase
 *   (6-19s of radio silence), causing the phone to declare the link
 *   dead mid-erase and closing the connection before OTA_STATUS_READY
 *   could be sent. Symptom: "GATT operation failed" ~4s after 0xA8.
 * - 20s is well under the BLE spec limit (32s max) and still recovers
 *   quickly if the device genuinely walks out of range.
 * - No measurable power impact — supervision timeout only governs how
 *   long the central waits during anomalous silence, not normal traffic.
 * - Note: connection params are a *request* from peripheral to central.
 *   Phones may apply immediately, delay, or ignore entirely. Explains
 *   why pre-v4.10.1 OTAs worked intermittently — attempts using phone's
 *   default (often 20+ seconds) succeeded; attempts after phone applied
 *   the 4s request failed.
 *
 * CHANGELOG v4.10.0:
 * - Startup program changed from 10Hz strobe to 6 BPM sine breathing.
 * - Added hall-sensor program cycling. Short close (0.3s-4s) advances
 *   through three on-device programs: BREATHE, BREATHE+STROBE, STROBE.
 *   Long close (>=5s) still enters deep sleep, same as before.
 * - New LED mode LED_MODE_BREATHE_STROBE: 10Hz strobe where dark-phase
 *   duty is scaled by the breathing waveform (makes the strobe "breathe").
 *   ISR reads a volatile breathe_frac_q8 updated by led_task each 10ms tick.
 * - Hall polling moved off the 1Hz main loop into a dedicated 50ms task
 *   so 0.3s gestures are detectable. 50ms debounce on edges. Gesture is
 *   decided on release: short release = advance, 5s of continuous HIGH
 *   = sleep. 4-5s release window is dead-zone (no action).
 * - BLE commands (A5/A6/B0 etc.) still work and override the physical
 *   program. Hall advance always cycles the physical programs regardless
 *   of current BLE-set mode.
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
#define FIRMWARE_VERSION "4.11.1"
static const char *TAG = "SG_v4.11.1";

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

/* BLE idle advertising timeout (v4.11.0).
 * If no connection arrives within this window after boot/wake/disconnect,
 * advertising is stopped to save the ~4mA of idle PA burst current from
 * the v4.9.12 advertising config (-6dBm, 100-200ms interval). Any hall
 * event (tap or hold edge) restarts advertising and resets the window. */
#define BLE_IDLE_TIMEOUT_MS     60000

/*******************************************************************************
 * HARDWARE CONFIGURATION
 ******************************************************************************/
#define HALL_PIN                GPIO_NUM_4

/* Hall gesture detection (v4.10.0):
 * Short close = advance program, long close = sleep.
 * Poll at HALL_POLL_MS so short gestures are catchable.
 * On release: if duration in [SHORT_MIN, SHORT_MAX) → advance. Otherwise ignore.
 * While held: if duration reaches HALL_LONG_MS → sleep immediately. */
#define HALL_POLL_MS            50
#define HALL_DEBOUNCE_MS        50      /* Debounce on both edges */
#define HALL_SHORT_MIN_MS       300     /* Ignore < 300ms (noise) */
#define HALL_SHORT_MAX_MS       4000    /* Short gesture upper bound */
#define HALL_LONG_MS            5000    /* Long hold → sleep */

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
    LED_MODE_STROBE         = 0,
    LED_MODE_STATIC         = 1,
    LED_MODE_BREATHE        = 2,
    LED_MODE_BREATHE_STROBE = 3,  /* v4.10.0: strobe with breathing-modulated duty */
} led_mode_t;

/*******************************************************************************
 * PHYSICAL PROGRAMS (v4.10.0)
 * 
 * Hall-sensor-selectable programs cycled by short magnet tap.
 * Each program maps to a specific led_mode + parameter set.
 ******************************************************************************/
typedef enum {
    PROG_BREATHE         = 0,  /* Startup default */
    PROG_BREATHE_STROBE  = 1,
    PROG_STROBE          = 2,
    PROG_COUNT           = 3,
} program_t;

/*******************************************************************************
 * GLOBAL STATE
 ******************************************************************************/
/* Session state */
static volatile bool session_active = false;
static volatile uint8_t brightness = DEFAULT_BRIGHTNESS;
static volatile uint32_t session_duration_ms = DEFAULT_SESSION_MIN * 60 * 1000;
static volatile uint32_t session_start_tick = 0;

/* LED mode — v4.10.0: startup now BREATHE (Program 1), was STROBE */
static volatile led_mode_t led_mode = LED_MODE_BREATHE;

/* Current physical program (hall-selectable). Starts at PROG_BREATHE. */
static volatile program_t current_program = PROG_BREATHE;

/* BLE auto-off state (v4.11.0, expanded v4.11.1).
 * ble_adv_active: tracks whether advertising is currently running.
 * ble_idle_deadline_tick: FreeRTOS tick when BLE should tear down if
 *   still not connected. 0 = deadline disabled (while connected, or
 *   after it has already been consumed and BLE is down).
 * ble_stack_up: tracks whether the BT controller + Bluedroid are
 *   initialized and enabled. False means full radio-off. Guards the
 *   teardown/init helpers to make them idempotent. */
static volatile bool ble_adv_active = false;
static volatile uint32_t ble_idle_deadline_tick = 0;
static volatile bool ble_stack_up = false;

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

/* Breathing fraction for ISR consumption (v4.10.0).
 * 0..255 representing 0.0..1.0 of the breath cycle.
 * Written by led_task @10ms, read by drive_timer_cb @100µs.
 * Used by LED_MODE_BREATHE_STROBE to scale the strobe dark duty. */
static volatile uint8_t breathe_frac_q8 = 0;

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

/* Forward declarations for BLE auto-off helpers (v4.11.0/v4.11.1).
 * Defined after adv_params so they can reference it. Declared here so
 * hall_task and app_main (which live above adv_params) can call them. */
static void ble_adv_rearm(void);
static void ble_adv_reset_deadline(void);
static esp_err_t ble_stack_init(void);
static esp_err_t ble_stack_teardown(void);

/* Forward declarations for the GAP/GATT event handlers (v4.11.1).
 * ble_stack_init() calls esp_ble_gap_register_callback(gap_event_handler)
 * and esp_ble_gatts_register_callback(gatts_event_handler), both of which
 * are defined further down in the file. Without these forward decls the
 * v4.11.1 build fails with "undeclared identifier" on both names. */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

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
    if ((led_mode == LED_MODE_STROBE || led_mode == LED_MODE_BREATHE_STROBE)
        && session_active && !in_ota_mode) {
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

        if (led_mode == LED_MODE_BREATHE_STROBE) {
            /* Scale dark-phase duty by current breathing fraction (0..255).
             * At top of breath: full brightness dark bursts.
             * At bottom: dark bursts nearly invisible. Clear phase unchanged. */
            uint32_t scaled = (uint32_t)brightness * breathe_frac_q8 / 255;
            effective_duty = is_dark ? (uint8_t)scaled : 0;
        } else {
            effective_duty = is_dark ? brightness : 0;
        }
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
     * Strobe stops because led_mode != LED_MODE_STROBE/BREATHE_STROBE. */
}

static void strobe_update(void) {
    strobe_dark_thresh = PHASE_FULL * strobe_duty_pct / 100;
    if ((led_mode == LED_MODE_STROBE || led_mode == LED_MODE_BREATHE_STROBE)
        && session_active) {
        strobe_start();
    }
}

/*******************************************************************************
 * PHYSICAL PROGRAM APPLY (v4.10.0)
 *
 * Sets led_mode + strobe state for a given program. Called from hall-gesture
 * handler. Safe to call even before session_active — strobe_start() is gated.
 ******************************************************************************/
static void apply_program(program_t p) {
    current_program = p;
    switch (p) {
        case PROG_BREATHE:
            strobe_stop();
            led_mode = LED_MODE_BREATHE;
            /* Clear any residual strobe duty immediately; led_task will
             * overwrite effective_duty on next 10ms tick with the waveform. */
            effective_duty = 0;
            ESP_LOGI(TAG, "Program 1: BREATHE %d BPM %s",
                     breathe_bpm, breathe_wave == 0 ? "sine" : "linear");
            break;

        case PROG_BREATHE_STROBE:
            led_mode = LED_MODE_BREATHE_STROBE;
            if (session_active) strobe_start();
            ESP_LOGI(TAG, "Program 2: BREATHE+STROBE %d BPM + %d.%dHz %d%% duty",
                     breathe_bpm, strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
            break;

        case PROG_STROBE:
            led_mode = LED_MODE_STROBE;
            if (session_active) strobe_start();
            ESP_LOGI(TAG, "Program 3: STROBE %d.%dHz %d%% duty",
                     strobe_dhz / 10, strobe_dhz % 10, strobe_duty_pct);
            break;

        default:
            break;
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
    if (led_mode == LED_MODE_STROBE || led_mode == LED_MODE_BREATHE_STROBE) {
        strobe_start();
    }
    
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
        
        /* ── BREATHE / BREATHE_STROBE MODE — compute frac every 10ms tick ──
         * Both modes share the same waveform math. BREATHE drives effective_duty
         * directly (lens tint follows waveform). BREATHE_STROBE publishes
         * breathe_frac_q8 for the ISR to scale strobe dark-phase duty. */
        if (led_mode == LED_MODE_BREATHE || led_mode == LED_MODE_BREATHE_STROBE) {
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

            /* Publish for ISR (BREATHE_STROBE scales dark duty by this). */
            breathe_frac_q8 = (uint8_t)(frac * 255.0f);

            if (led_mode == LED_MODE_BREATHE) {
                effective_duty = (uint8_t)(frac * (float)brightness);
            }
            /* BREATHE_STROBE: ISR sets effective_duty from breathe_frac_q8 */
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
                               (led_mode == LED_MODE_STATIC) ? "STATIC" :
                               (led_mode == LED_MODE_BREATHE_STROBE) ? "BR+STRB" :
                               "BREATHE";
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
 * HALL SENSOR — GESTURE STATE MACHINE (v4.10.0)
 *
 * Polled at HALL_POLL_MS (50ms) from a dedicated task. Two gestures:
 *   - Short close (HALL_SHORT_MIN_MS .. HALL_SHORT_MAX_MS, decided on release)
 *       → advance to next physical program
 *   - Long close (>= HALL_LONG_MS continuous HIGH, decided while still held)
 *       → enter deep sleep
 * 4000-5000ms release window is intentional dead zone (no action) so the
 * user has a clear gap between "quick tap" and "hold to sleep".
 * 50ms debounce on both edges.
 *
 * The Hall pin reads HIGH when the magnet is near (arm closed), LOW when far.
 * In deep sleep we wake on LOW (arm opened) via ext0 — unchanged.
 ******************************************************************************/

/* Raw-edge debouncer. Returns the stable level (0 or 1). */
static uint8_t hall_debounced_level(void) {
    static uint8_t stable = 0;
    static uint8_t candidate = 0;
    static uint32_t candidate_since = 0;

    uint8_t raw = gpio_get_level(HALL_PIN) ? 1 : 0;
    uint32_t now = xTaskGetTickCount();

    if (raw != stable) {
        if (raw != candidate) {
            candidate = raw;
            candidate_since = now;
        } else if ((now - candidate_since) * portTICK_PERIOD_MS >= HALL_DEBOUNCE_MS) {
            stable = raw;
        }
    } else {
        candidate = stable;
    }
    return stable;
}

static void hall_task(void *param) {
    uint8_t prev_level = 0;
    uint32_t high_start_tick = 0;
    bool sleep_fired = false;       /* Prevent double-firing the 5s threshold */

    ESP_LOGI(TAG, "Hall gesture task started (poll=%dms short=%d-%dms long=%dms)",
             HALL_POLL_MS, HALL_SHORT_MIN_MS, HALL_SHORT_MAX_MS, HALL_LONG_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HALL_POLL_MS));

        /* Block gesture handling during OTA */
        if (in_ota_mode) {
            prev_level = 0;
            sleep_fired = false;
            continue;
        }

        uint8_t level = hall_debounced_level();
        uint32_t now = xTaskGetTickCount();

        /* Rising edge: magnet just closed */
        if (level == 1 && prev_level == 0) {
            high_start_tick = now;
            sleep_fired = false;
            /* v4.11.0: any hall activity re-arms BLE advertising.
             * Fires on the leading edge so advertising is up by the time
             * the user releases — whether they're tapping to advance
             * programs or starting a hold-to-sleep. No-op if already up. */
            ble_adv_rearm();
        }

        /* Still held HIGH: check for long-hold sleep threshold */
        if (level == 1 && !sleep_fired) {
            uint32_t held_ms = (now - high_start_tick) * portTICK_PERIOD_MS;
            if (held_ms >= HALL_LONG_MS) {
                ESP_LOGI(TAG, "Hall long-hold %lums → sleep", held_ms);
                sleep_fired = true;
                session_active = false;
                enter_deep_sleep();
                /* enter_deep_sleep() doesn't return */
            }
        }

        /* Falling edge: magnet just opened — decide the gesture */
        if (level == 0 && prev_level == 1) {
            uint32_t held_ms = (now - high_start_tick) * portTICK_PERIOD_MS;

            if (sleep_fired) {
                /* Already went to sleep — shouldn't reach here, belt-and-braces */
            } else if (held_ms >= HALL_SHORT_MIN_MS && held_ms < HALL_SHORT_MAX_MS) {
                program_t next = (program_t)((current_program + 1) % PROG_COUNT);
                ESP_LOGI(TAG, "Hall short-tap %lums → advance to program %d",
                         held_ms, next);
                apply_program(next);
            } else if (held_ms < HALL_SHORT_MIN_MS) {
                ESP_LOGI(TAG, "Hall tap %lums < %dms, ignored (noise)",
                         held_ms, HALL_SHORT_MIN_MS);
            } else {
                /* Dead zone: 4000-4999ms release. Do nothing. */
                ESP_LOGI(TAG, "Hall tap %lums in dead-zone, ignored", held_ms);
            }
        }

        prev_level = level;
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
 * BLE STACK LIFECYCLE HELPERS (v4.11.0 / v4.11.1)
 *
 * Rationale: v4.9.12 changed advertising to -6dBm at 100-200ms for faster
 * first-try connects, which added ~4mA to idle current. That cost is only
 * worth paying in the window when the user actually wants to connect.
 *
 * v4.11.0 stopped advertising after BLE_IDLE_TIMEOUT_MS but left the
 * controller and Bluedroid running — current traces showed residual
 * ~1.5A RF spikes from BT controller housekeeping. v4.11.1 escalates to
 * full BT stack teardown so the radio is completely cold.
 *
 * ble_stack_init():      Bring up BT controller + Bluedroid + GATT app.
 *                        Idempotent. Returns ESP_OK if already up.
 * ble_stack_teardown():  Tear down everything initialized by init().
 *                        Idempotent. Returns ESP_OK if already down.
 * ble_adv_rearm():       Ensure stack is up and advertising; reset deadline.
 *                        Called on boot, hall events, disconnect.
 * ble_adv_reset_deadline(): Push deadline out another window.
 *
 * The actual teardown trigger happens in app_main's 1Hz loop when the
 * deadline elapses, not in these helpers.
 *
 * State invariants:
 *   ble_stack_up == false  ⟹  ble_adv_active == false AND is_connected == false
 *   ble_adv_active == true ⟹  ble_stack_up == true
 *
 * Handle lifetimes: gatts_if_global, service_handle, ctrl_char_handle,
 * ota_char_handle, status_char_handle, cccd_handle all become invalid
 * across teardown and are repopulated by the GATT event handler when
 * the new GATT app registers on the next init.
 ******************************************************************************/
static void ble_adv_reset_deadline(void) {
    ble_idle_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(BLE_IDLE_TIMEOUT_MS);
}

static esp_err_t ble_stack_init(void) {
    if (ble_stack_up) return ESP_OK;

    ESP_LOGI(TAG, "BLE stack init...");

    esp_err_t err;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "bt_controller_init: %s", esp_err_to_name(err)); return err; }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) { ESP_LOGE(TAG, "bt_controller_enable: %s", esp_err_to_name(err)); return err; }

    err = esp_bluedroid_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "bluedroid_init: %s", esp_err_to_name(err)); return err; }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) { ESP_LOGE(TAG, "bluedroid_enable: %s", esp_err_to_name(err)); return err; }

    /* Re-register callbacks and GATT app. The GATT app registration will
     * trigger ADD_CHAR events etc. in the GATT handler, which repopulates
     * the service/char handles and ultimately starts advertising via the
     * ADV_DATA_SET_COMPLETE_EVT path. */
    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) { ESP_LOGE(TAG, "gap_register_callback: %s", esp_err_to_name(err)); return err; }

    err = esp_ble_gatts_register_callback(gatts_event_handler);
    if (err != ESP_OK) { ESP_LOGE(TAG, "gatts_register_callback: %s", esp_err_to_name(err)); return err; }

    err = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (err != ESP_OK) { ESP_LOGE(TAG, "gatts_app_register: %s", esp_err_to_name(err)); return err; }

    err = esp_ble_gatt_set_local_mtu(517);
    if (err != ESP_OK) { ESP_LOGW(TAG, "set_local_mtu: %s", esp_err_to_name(err)); /* non-fatal */ }

    ble_stack_up = true;
    ESP_LOGI(TAG, "BLE stack up");
    return ESP_OK;
}

static esp_err_t ble_stack_teardown(void) {
    if (!ble_stack_up) return ESP_OK;

    ESP_LOGI(TAG, "BLE stack teardown (radio off)");

    /* Order matters: disable/deinit Bluedroid first (it uses the controller),
     * then disable/deinit the controller. Ignore errors — we're going down
     * regardless, and partial teardown is worse than ugly logs. */
    if (ble_adv_active) {
        esp_ble_gap_stop_advertising();
        /* ADV_STOP_COMPLETE_EVT may not fire before we kill the stack —
         * clear the flag manually so state stays consistent. */
        ble_adv_active = false;
    }

    esp_err_t err;
    err = esp_bluedroid_disable();
    if (err != ESP_OK) ESP_LOGW(TAG, "bluedroid_disable: %s", esp_err_to_name(err));
    err = esp_bluedroid_deinit();
    if (err != ESP_OK) ESP_LOGW(TAG, "bluedroid_deinit: %s", esp_err_to_name(err));
    err = esp_bt_controller_disable();
    if (err != ESP_OK) ESP_LOGW(TAG, "bt_controller_disable: %s", esp_err_to_name(err));
    err = esp_bt_controller_deinit();
    if (err != ESP_OK) ESP_LOGW(TAG, "bt_controller_deinit: %s", esp_err_to_name(err));

    /* Invalidate BLE-dependent handles. is_connected must already be false
     * here since timeout gates on !is_connected. */
    ble_stack_up = false;
    gatts_if_global = ESP_GATT_IF_NONE;
    ctrl_char_handle = 0;
    ota_char_handle = 0;
    status_char_handle = 0;
    cccd_handle = 0;
    service_handle = 0;
    notifications_enabled = false;

    ble_idle_deadline_tick = 0;
    return ESP_OK;
}

static void ble_adv_rearm(void) {
    /* If connected, no point advertising; leave deadline disabled. */
    if (is_connected) {
        ble_idle_deadline_tick = 0;
        return;
    }

    /* Bring stack up if it was torn down by the auto-off timeout.
     * Advertising itself starts asynchronously via the GATT service-create
     * flow, so once the stack is up the first time we just wait. */
    if (!ble_stack_up) {
        esp_err_t err = ble_stack_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BLE re-arm failed to init stack: %s", esp_err_to_name(err));
            return;
        }
        /* Advertising will start asynchronously via ADV_DATA_SET_COMPLETE_EVT.
         * Arm the deadline now; if adv never comes up the flag stays false
         * and the timeout check is gated on ble_adv_active anyway. */
        ble_adv_reset_deadline();
        ESP_LOGI(TAG, "BLE re-armed via stack init (60s window)");
        return;
    }

    /* Stack was already up (e.g. boot path, or rapid re-arm). Just make
     * sure advertising is running and reset the deadline. */
    if (!ble_adv_active) {
        esp_err_t err = esp_ble_gap_start_advertising(&adv_params);
        if (err == ESP_OK) {
            ble_adv_active = true;
            ESP_LOGI(TAG, "BLE advertising re-armed (60s window)");
        } else {
            ESP_LOGW(TAG, "BLE adv start failed: %s", esp_err_to_name(err));
            return;
        }
    }
    ble_adv_reset_deadline();
}

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
                ble_adv_active = true;  /* v4.11.0: track state */
                ESP_LOGI(TAG, "Advertising started");
            } else {
                ble_adv_active = false;
                ESP_LOGE(TAG, "Advertising failed");
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:  /* v4.11.0: track state */
            ble_adv_active = false;
            ESP_LOGI(TAG, "Advertising stopped");
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
            ble_idle_deadline_tick = 0;  /* v4.11.0: disable auto-off while connected */
            ESP_LOGI(TAG, "Client connected, conn_id %d", conn_id_global);
            
            /* Update connection parameters for power saving.
             * v4.10.1: supervision timeout 4s -> 20s. OTA partition erase
             * (esp_ota_begin) holds the radio silent for 6-19s, which was
             * exceeding the 4s supervision timeout and causing the central
             * to drop the link mid-erase before OTA_STATUS_READY could be
             * sent. 20s (0x7D0) is well under the 32s BLE spec max.
             * Note: these params are a *request* to the central — phones
             * may honor, defer, or ignore them. */
            esp_ble_conn_update_params_t conn_params = {
                .latency = 4,           /* Skip up to 4 connection events */
                .max_int = 0x30,        /* 60ms max interval */
                .min_int = 0x20,        /* 40ms min interval */
                .timeout = 2000,        /* 20 second supervision timeout (was 400 = 4s) */
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
            
            /* Restart advertising with a fresh 60s auto-off window (v4.11.0).
             * If no reconnect arrives in that window, advertising stops and
             * device continues standalone until a hall gesture or sleep. */
            esp_ble_gap_start_advertising(&adv_params);
            ble_adv_reset_deadline();
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
    ESP_LOGI(TAG, "  Boot: Program 1 = BREATHE %d BPM %s @ %d%% brightness",
             breathe_bpm, breathe_wave == 0 ? "sine" : "linear", DEFAULT_BRIGHTNESS);
    ESP_LOGI(TAG, "  Hall: short tap (%d-%dms) = next program, %ds hold = sleep",
             HALL_SHORT_MIN_MS, HALL_SHORT_MAX_MS, HALL_LONG_MS / 1000);
    ESP_LOGI(TAG, "  Programs: 1=BREATHE 2=BREATHE+STROBE 3=STROBE");
    ESP_LOGI(TAG, "  Session: %d minutes", DEFAULT_SESSION_MIN);
    ESP_LOGI(TAG, "  AC Drive: 100Hz gptimer ISR, strobe phase-synced (GPIO26/27)");
    ESP_LOGI(TAG, "  Power: -6dBm TX (adv) / -12dBm (conn), 100-200ms adv interval");
    ESP_LOGI(TAG, "  BLE auto-off: full stack teardown after %ds no connect (v4.11.1)",
             BLE_IDLE_TIMEOUT_MS / 1000);
    
    /* Get partition info */
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "  Partition: %s @ 0x%lx", running->label, running->address);
    
    /* Log wake reason */
    esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "  Wake reason: %d", wake);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Hall: tap=next program, 5s hold=sleep");
    ESP_LOGI(TAG, "BLE: adv auto-off after 60s — tap hall to re-arm");
    ESP_LOGI(TAG, "BLE modes: A5=static A6=strobe B0=breathe");
    ESP_LOGI(TAG, "Common: A2=brightness A4=session A7=sleep");
    ESP_LOGI(TAG, "Strobe: AB=freq AC=duty | Breathe: B1-B5");

    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize Bluetooth — release Classic BT memory once; stack itself
     * is brought up via the ble_stack_init helper so the same code path
     * runs at boot and on every hall re-arm after an auto-off teardown. */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(ble_stack_init());

    /* v4.11.0: arm the initial BLE idle-off deadline.
     * Advertising itself starts asynchronously (triggered by the GAP
     * ADV_DATA_SET_COMPLETE_EVT after adv data is configured during GATT
     * service creation). Setting the deadline before adv is live is safe:
     * the main-loop timeout check gates on ble_adv_active first, so it
     * can't fire until advertising has actually started. */
    ble_adv_reset_deadline();

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

    /* Create Hall gesture task (50ms polling, short-tap = next program, 5s hold = sleep) */
    xTaskCreate(hall_task, "hall_task", 2048, NULL, 2, NULL);

    /* Main loop - monitor session state and BLE idle timeout */
    while (1) {
        /* Check if session ended */
        if (!session_active && led_task_handle == NULL && !in_ota_mode) {
            ESP_LOGI(TAG, "Session complete, entering sleep");
            enter_deep_sleep();
        }

        /* v4.11.0/v4.11.1: BLE idle auto-off.
         * After BLE_IDLE_TIMEOUT_MS of no connection, tear down the entire
         * BT stack (controller + Bluedroid). v4.11.0 only stopped advertising,
         * which left the controller running and emitting periodic RF
         * housekeeping bursts (~1.5A spikes every few seconds in traces).
         * v4.11.1 kills the radio completely.
         * Gated on ble_stack_up so it only fires once per window, and on
         * !is_connected / !in_ota_mode to avoid pulling the rug during use. */
        if (ble_stack_up && ble_adv_active && !is_connected && !in_ota_mode &&
            ble_idle_deadline_tick != 0 &&
            xTaskGetTickCount() >= ble_idle_deadline_tick) {
            ESP_LOGI(TAG, "BLE idle timeout (%ds no connect) — radio off",
                     BLE_IDLE_TIMEOUT_MS / 1000);
            ble_stack_teardown();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
