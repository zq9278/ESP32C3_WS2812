#include <stdio.h>
#include "freertos/FreeRTOS.h"  // 引入FreeRTOS的头文件
#include "freertos/task.h"      // 引入FreeRTOS任务管理的头文件
#include "led_strip.h"          // 引入LED灯带控制的头文件
#include "esp_log.h"            // 引入ESP日志功能的头文件
#include "esp_err.h"            // 引入ESP错误处理的头文件

// GPIO分配
#define LED_STRIP_BLINK_GPIO 11  // 定义连接到LED灯带数据线的GPIO编号
// LED灯带中LED的数量
#define LED_STRIP_LED_NUMBERS 101  // 定义LED灯带中的LED数量
// 10MHz分辨率，1 tick = 0.1微秒（LED灯带需要高分辨率）
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)  // 定义LED灯带的RMT分辨率

static uint8_t base_r = 150;  // 基础红色值
static uint8_t base_g = 150;  // 基础绿色值
static uint8_t base_b = 150;  // 基础蓝色值

static int led_mode = 1;      // LED模式变量
static int direction = 1;     // 呼吸灯变化方向变量
static int brightness = 0;    // 当前亮度变量

static const char *TAG = "example";  // 日志标签

// 前置声明两个函数
static void led_breath(led_strip_handle_t led_strip);
static void led_flow(led_strip_handle_t led_strip);

// 配置LED灯带的函数
led_strip_handle_t configure_led(void)
{
    // LED灯带的通用配置
    led_strip_config_t strip_config = {
            .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // 连接到LED灯带数据线的GPIO
            .max_leds = LED_STRIP_LED_NUMBERS,        // 灯带中的LED数量
            .led_pixel_format = LED_PIXEL_FORMAT_GRB, // LED灯带的像素格式
            .led_model = LED_MODEL_WS2812,            // LED灯带型号
            .flags.invert_out = false,                // 是否反转输出信号
    };

    // LED灯带后端配置：RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
            .rmt_channel = 0,
#else
            .clk_src = RMT_CLK_SRC_DEFAULT,        // 不同的时钟源可能导致不同的功耗
            .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT计数器时钟频率
            .flags.with_dma = false,               // 在如ESP32-S3的ESP目标上可用的DMA特性
#endif
    };

    // 创建LED Strip对象句柄
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "创建了具有RMT后端的LED灯带对象");
    return led_strip;
}

// 闪烁任务函数
static void blink_task(void *arg)
{
    led_strip_handle_t led_strip = configure_led();

    while (1)
    {
        if (led_mode == 0)
        {
            led_breath(led_strip);  // 呼吸灯模式
        }
        else if (led_mode == 1)
        {
            led_flow(led_strip);    // 流水灯模式
        }
    }
    vTaskDelete(NULL);  // 删除任务
}

// 呼吸灯功能的实现
static void led_breath(led_strip_handle_t led_strip){
    uint8_t color_r = base_r * brightness / 255;  // 计算红色亮度
    uint8_t color_g = base_g * brightness / 255;  // 计算绿色亮度
    uint8_t color_b = base_b * brightness / 255;  // 计算蓝色亮度
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, color_r, color_g, color_b));
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    brightness += direction;  // 改变亮度
    if(brightness == 255 || brightness == 0){
        direction *= -1;  // 改变亮度变化方向
    }
}

// 流水灯功能的实现
static void led_flow(led_strip_handle_t led_strip){
    int tail_length = 50;  // 流水灯尾巴长度
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
        ESP_ERROR_CHECK(led_strip_clear(led_strip));
        for(int j = 0; j < tail_length; j++){
            int position = i - j;
            if(position >= 0){
                uint8_t tail_r = base_r - (j * (base_r / tail_length));
                uint8_t tail_g = base_g - (j * (base_g / tail_length));
                uint8_t tail_b = base_b - (j * (base_b / tail_length));
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, position, tail_r, tail_g, tail_b));
            }
        }
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, base_r, base_g, base_b));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(100 / portTICK_PERIOD_MS);  // 延迟以产生流水效果
    }
}

// 主函数
void app_main(void)
{
    xTaskCreate(&blink_task, "blink_task", 8192, NULL, 5, NULL);  // 创建闪烁任务
}
