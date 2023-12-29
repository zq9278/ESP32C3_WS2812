

#include <stdio.h>
#include "freertos/FreeRTOS.h"  // 引入FreeRTOS的头文件
#include "freertos/task.h"      // 引入FreeRTOS的任务管理头文件
#include "led_strip.h"          // 引入LED灯带的头文件
#include "esp_log.h"            // 引入ESP日志功能的头文件
#include "esp_err.h"            // 引入ESP错误处理的头文件

// GPIO分配
#define LED_STRIP_BLINK_GPIO  7   // LED灯带连接的GPIO编号
// LED灯带中的LED数量
#define LED_STRIP_LED_NUMBERS 1   // LED灯带中LED的数量
// 10MHz的分辨率，1 tick = 0.1微秒（LED灯带需要高分辨率）
#define LED_STRIP_RMT_RES_HZ  (5 * 1000 * 1000) // LED灯带的RMT解析度

static const char *TAG = "example"; // 日志标签

led_strip_handle_t configure_led(void)
{
    // LED灯带的通用初始化，根据你的LED板设计
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
            .flags.with_dma = false,               // ESP目标（如ESP32-S3）上可用的DMA特性
#endif
    };

    // LED灯带对象句柄
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "创建了具有RMT后端的LED灯带对象");
    return led_strip;
}

void app_main(void)
{
    led_strip_handle_t led_strip = configure_led(); // 配置并获取LED灯带的句柄
    bool led_on_off = true;  // LED开关状态

    ESP_LOGI(TAG, "开始闪烁LED灯带");
    // //单色常亮
    // while (1) {  // 无限循环
    //     if (led_on_off) {
    //         /* 使用RGB从0（0%）到255（100%）设置LED像素 */
    //         for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
    //             // 为每个LED设置颜色，这里设置为（0, 150, 255），代表蓝色调的颜色
    //             ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 150, 255));
    //         }
    //         /* 刷新灯带以发送数据 */
    //         ESP_ERROR_CHECK(led_strip_refresh(led_strip)); // 将设置的颜色数据发送到LED灯带
    //         ESP_LOGI(TAG, "LED开！"); // 记录日志，表示LED灯开启
    //     } else {
    //         /* 设置所有LED为关闭状态，以清除所有像素 */
    //         ESP_ERROR_CHECK(led_strip_clear(led_strip)); // 清除LED灯带上的所有像素（关闭LED灯）
    //         ESP_LOGI(TAG, "LED关！"); // 记录日志，表示LED灯关闭
    //     }

    //     //led_on_off = !led_on_off; // 切换LED灯的状态，如果取消这行注释，LED灯将交替开关
    //     vTaskDelay(pdMS_TO_TICKS(500)); // 延时500毫秒
    // }

//单色爆闪
   while (1) {
       if (led_on_off) {
           // 开启所有LED
           for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
               ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 150, 255)); // 设置为蓝色
           }
           ESP_ERROR_CHECK(led_strip_refresh(led_strip));
           ESP_LOGI(TAG, "LED开！");
       } else {
           // 关闭所有LED
           ESP_ERROR_CHECK(led_strip_clear(led_strip));
           ESP_LOGI(TAG, "LED关！");
       }

       led_on_off = !led_on_off; // 切换LED灯的状态
       vTaskDelay(pdMS_TO_TICKS(30)); // 爆闪模式下的延时缩短为100毫秒
   }

//多色爆闪
//多色爆闪
//多色爆闪
//    while (1) {
//        // 为每个LED设置随机颜色
//        for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
//            uint8_t red = rand() % 256;   // 随机生成红色分量
//            uint8_t green = rand() % 256; // 随机生成绿色分量
//            uint8_t blue = rand() % 256;  // 随机生成蓝色分量
//            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
//        }
//        ESP_ERROR_CHECK(led_strip_refresh(led_strip)); // 刷新灯带以发送数据
//        vTaskDelay(pdMS_TO_TICKS(30)); // 爆闪模式下的延时100毫秒

//        // 设置所有LED为关闭状态，以清除所有像素
//        ESP_ERROR_CHECK(led_strip_clear(led_strip));
//        vTaskDelay(pdMS_TO_TICKS(30)); // 爆闪模式下的延时100毫秒
//    }
}
