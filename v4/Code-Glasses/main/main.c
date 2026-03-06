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
#include "esp_sleep.h"




#define GATTS_SERVICE_UUID_TEST   0x00FF
#define GATTS_CHAR_UUID_TEST      0xFF01
#define GATTS_NUM_HANDLE_TEST     4

#define DEVICE_NAME            "Smart_Glasses"
#define TEST_APP_ID            0

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static const char *TAG = "BLE_SIMPLE";


// 广播参数
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// 广播内容
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// GATT 结构体
static uint16_t gatt_service_handle = 0;
static esp_gatt_char_prop_t gatt_property = 0;
static esp_attr_value_t gatt_char_val = {
    .attr_max_len = 100,
    .attr_len = 0,
    .attr_value = NULL,
};

//*********************************************************** */

#define HALL_PIN GPIO_NUM_4  // GPIO4连接霍尔传感器输出


RTC_DATA_ATTR int bootCount = 0; // 使用RTC_DATA_ATTR宏保存重启次数
 uint8_t pwm_duty_set = 0; // 使用RTC_DATA_ATTR宏保存PWM占空比
 uint8_t output_tim = 10; // 使用RTC_DATA_ATTR宏保存PWM输出时间
#define SLEEP_HALL_WAIT_TIME            5 // 睡眠唤醒等待时间，单位秒


//引脚27用于LED控制pwm1
#define PWM1_TIMER              LEDC_TIMER_0
#define PWM1_MODE               LEDC_LOW_SPEED_MODE
#define PWM1_OUTPUT_IO          (27) // Define the output GPIO
#define PWM1_CHANNEL            LEDC_CHANNEL_0
#define PWM1_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 10 bits
#define PWM1_DUTY               (512) // Set duty to 50%. (2 ** 10) * 50% = 512
#define PWM1_FREQUENCY          (10000) // Frequency in Hertz. Set frequency at 10 kHz

//引脚26用于LED控制pwm2
#define PWM2_TIMER              LEDC_TIMER_0
#define PWM2_MODE               LEDC_LOW_SPEED_MODE
#define PWM2_OUTPUT_IO          (26) // Define the output GPIO
#define PWM2_CHANNEL            LEDC_CHANNEL_1
#define PWM2_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 10 bits
#define PWM2_DUTY               (512) // Set duty to 50%. (2 ** 10) * 50% = 512
#define PWM2_FREQUENCY          (10000) // Frequency in Hertz. Set frequency at 10 kHz




static void PWM_Init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t pwm1_timer = {
        .speed_mode       = PWM1_MODE,
        .duty_resolution  = PWM1_DUTY_RES,
        .timer_num        = PWM1_TIMER,
        .freq_hz          = PWM1_FREQUENCY,  // Set output frequency at 10 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm1_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t pwm1_channel = {
        .speed_mode     = PWM1_MODE,
        .channel        = PWM1_CHANNEL,
        .timer_sel      = PWM1_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM1_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm1_channel));

        // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t pwm2_timer = {
        .speed_mode       = PWM2_MODE,
        .duty_resolution  = PWM2_DUTY_RES,
        .timer_num        = PWM2_TIMER,
        .freq_hz          = PWM2_FREQUENCY,  // Set output frequency at 10 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm2_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t pwm2_channel = {
        .speed_mode     = PWM2_MODE,
        .channel        = PWM2_CHANNEL,
        .timer_sel      = PWM2_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM2_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm2_channel));
}



void pwm1_setduty(uint32_t duty) {
    duty=1024*duty/100;//10BIT分辨率1024刻度
    ESP_ERROR_CHECK(ledc_set_duty(PWM1_MODE, PWM1_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM1_MODE, PWM1_CHANNEL));
}

void pwm2_setduty(uint32_t duty) {
    duty=1024*duty/100;//10BIT分辨率1024刻度
    ESP_ERROR_CHECK(ledc_set_duty(PWM2_MODE, PWM2_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM2_MODE, PWM2_CHANNEL));
}

void pwm1_setfreq(uint32_t freq) {
    ESP_ERROR_CHECK(ledc_set_freq(PWM1_MODE, PWM1_CHANNEL, freq));
    ESP_ERROR_CHECK(ledc_update_duty(PWM1_MODE, PWM1_CHANNEL));
}

void pwm2_setfreq(uint32_t freq) {
    ESP_ERROR_CHECK(ledc_set_freq(PWM2_MODE, PWM2_CHANNEL, freq));
    ESP_ERROR_CHECK(ledc_update_duty(PWM2_MODE, PWM2_CHANNEL));
}


//*********************************************************** */

// GAP事件回调
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) esp_ble_gap_start_advertising(&adv_params);
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

// GATT事件回调
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "GATT register, app_id %d", param->reg.app_id);
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;

        // 创建一个简单的service
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_SERVICE_UUID_TEST}},
            },
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE_TEST);
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Service created, handle %d", param->create.service_handle);
        gatt_service_handle = param->create.service_handle;

        // 添加一个写入特征
        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = GATTS_CHAR_UUID_TEST},
        };
        gatt_property = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;

        esp_ble_gatts_start_service(gatt_service_handle);
        esp_ble_gatts_add_char(gatt_service_handle, &char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               gatt_property, &gatt_char_val, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "Characteristic added, handle %d", param->add_char.attr_handle);
        break;
  case ESP_GATTS_WRITE_EVT:
    ESP_LOGI(TAG, "GATT write event, len=%d", param->write.len);
    
    if (param->write.len > 0) {
        // 详细调试信息
        ESP_LOGI(TAG, "Raw data received:");
        for (int i = 0; i < param->write.len; i++) {
            ESP_LOGI(TAG, "Byte[%d]: 0x%02X", i, param->write.value[i]);
        }
        
        // BCD解码：0x21 -> 21, 0x45 -> 45
        if (param->write.len >= 1) {
            uint8_t bcd_value = param->write.value[0]*0.6;
            if (bcd_value>100)
            {
                bcd_value=100;
                /* code */
            }
            
            pwm_duty_set = ((bcd_value >> 4) * 10) + (bcd_value & 0x0F);
            ESP_LOGI(TAG, "BCD decode: 0x%02X -> %d%%", bcd_value, pwm_duty_set);
        }
        
        if (param->write.len >= 2) {
            uint8_t bcd_value = param->write.value[1];
            output_tim = ((bcd_value >> 4) * 10) + (bcd_value & 0x0F);
            ESP_LOGI(TAG, "BCD decode: 0x%02X -> %dms", bcd_value, output_tim);
        }
        
        // 范围限制
        if (pwm_duty_set > 100) {
            ESP_LOGW(TAG, "Duty cycle %d%% exceeds maximum, clamping to 100%%", pwm_duty_set);
            pwm_duty_set = 100;
        }
        if (output_tim < 1) {
            ESP_LOGW(TAG, "Output time %dms is too small, setting to 1ms", output_tim);
            output_tim = 1;
        }
        
        ESP_LOGI(TAG, "Final settings: Duty=%d%%, Time=%dms", pwm_duty_set, output_tim);
    }

    if (param->write.need_rsp) {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                  param->write.trans_id, ESP_GATT_OK, NULL);
    }
    break;
    // case ESP_GATTS_WRITE_EVT:
    //     ESP_LOGI(TAG, "Data received, length=%d", param->write.len);
    //     ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);

    //     // ====== 在这里执行你的逻辑 ======
    //     if (param->write.len > 0) {
    //         uint8_t cmd = param->write.value[0];
    //         switch (cmd) {//30%,40%,50%,60%,增加1%，减少1%
    //             case 0x01:
    //                 pwm_duty_set=30;
    //                 ESP_LOGI(TAG, "set 30 duty cycle");
    //                 break;
    //             case 0x02:
    //                 pwm_duty_set=40;
    //                 ESP_LOGI(TAG, "set 40 duty cycle");
    //                 break;
    //             case 0x03:
    //                 pwm_duty_set=50;
    //                 ESP_LOGI(TAG, "set 50 duty cycle");
    //                 break;
    //             case 0x04:
    //                 pwm_duty_set=60;
    //                 ESP_LOGI(TAG, "set 60 duty cycle");
    //                 break;
    //             case 0x05:
    //                 if (++pwm_duty_set>=60)
    //                 {
    //                     pwm_duty_set=60;
    //                     /* code */
    //                 }
                    
    //                 ESP_LOGI(TAG, "set %d duty cycle", pwm_duty_set);
    //                 break;
    //             case 0x06:
    //                 if (--pwm_duty_set<10)
    //                 {
    //                     pwm_duty_set=10;
    //                     /* code */
    //                 }
    //                 ESP_LOGI(TAG, "set %d duty cycle", pwm_duty_set);
    //                 break;
    //             case 0x07:
    //                 output_tim++;
    //                 ESP_LOGI(TAG, "set %d output time", output_tim);
    //                 break;
    //             case 0x08:
    //                 output_tim--;
    //                 ESP_LOGI(TAG, "set %d output time", output_tim);
    //                 break;
    //             default:
    //                 ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
    //                 break;
    //         }
    //     }

    //     // 回复客户端写入成功
    //     if (param->write.need_rsp) {
    //         esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    //     }
    //     break;

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

// GATT主回调
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




TaskHandle_t TASK_HandleOne = NULL;

/* 任务一的函数体,由于入参即为NULL，因此函数体的入参需为void *参数，否则报错 */
void TASK_ONE(void *param )
{
    static uint8_t state = 0;
    while (1)
    {
        

        switch (state)
        {
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

        vTaskDelay(output_tim/ portTICK_PERIOD_MS);
    }
}



static void check_sleep_condition(void)
{
    static uint32_t low_level_duration = 0;
    static uint32_t last_check_time = 0;
    uint32_t current_time = xTaskGetTickCount();
    
    // 每秒检查一次
    if (current_time - last_check_time < (1000 / portTICK_PERIOD_MS)) {
        return;
    }
    last_check_time = current_time;
    
    if (gpio_get_level(HALL_PIN) == 1) {
        low_level_duration++;
        ESP_LOGI(TAG, "Hall sensor LOW, duration: %d seconds", low_level_duration);
        
        if (low_level_duration >= SLEEP_HALL_WAIT_TIME) {
            ESP_LOGI(TAG, "Entering deep sleep...");
            pwm_duty_set = 0;

            
            // 配置唤醒源
            esp_sleep_enable_ext0_wakeup(HALL_PIN, 0); // 低电平唤醒
            
            // 进入深度睡眠
            esp_deep_sleep_start();
        }

    } else {
        low_level_duration = 0;
    }
}


    

 

void app_main(void)
{
    esp_err_t ret;

    // 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(TEST_APP_ID));



    

    //初始化唤醒检测引脚
    //设置GPIO为输入模式
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HALL_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    PWM_Init();// 初始化PWM

    xTaskCreate(//创建pwm任务
                    TASK_ONE,          /* 任务函数 */
                    "TaskOne",         /* 任务名 */
                    8*1024,            /* 任务栈大小，根据需要自行设置*/
                    NULL,              /* 参数，入参为空 */
                    1,                 /* 优先级 */
                    &TASK_HandleOne);  /* 任务句柄 */




//***************************************DEBUG: 检查是否为深度睡眠唤醒
    // esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    // if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    //     ESP_LOGE("LowPower", "Woke up from timer!");
    // } else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    //     ESP_LOGE("LowPower", "Woke up from external pin!");
    // } else if (cause == ESP_SLEEP_WAKEUP_TOUCHPAD) {
    //     ESP_LOGE("LowPower", "Woke up from touchpad!");
    // } else {
    //     ESP_LOGE("LowPower", "Not a deep sleep wakeup, cause: %d", cause);
    // }
//*********** ******************/

    while (1)
    {
        check_sleep_condition();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}



  