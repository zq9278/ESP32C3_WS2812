
 #include <stdio.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "led_strip.h"
 #include "esp_log.h"
 #include "esp_err.h"

 // GPIO assignment
 #define LED_STRIP_BLINK_GPIO 11
 // Numbers of the LED in the strip
 #define LED_STRIP_LED_NUMBERS 101
 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
 #define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

 static uint8_t base_r = 150;
 static uint8_t base_g = 150;
 static uint8_t base_b = 150;

 static int led_mode = 1;
 static int direction = 1;
 static int brightness = 0;

 static const char *TAG = "example";
 static void led_breath(led_strip_handle_t led_strip);
 static void led_flow(led_strip_handle_t led_strip);

 led_strip_handle_t configure_led(void)
 {
     // LED strip general initialization, according to your led board design
     led_strip_config_t strip_config = {
         .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
         .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
         .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
         .led_model = LED_MODEL_WS2812,            // LED strip model
         .flags.invert_out = false,                // whether to invert the output signal
     };

     // LED strip backend configuration: RMT
     led_strip_rmt_config_t rmt_config = {
 #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
         .rmt_channel = 0,
 #else
         .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
         .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
         .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
 #endif
     };

     // LED Strip object handle
     led_strip_handle_t led_strip;
     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
     ESP_LOGI(TAG, "Created LED strip object with RMT backend");
     return led_strip;
 }
 static void blink_task(void *arg)
 {
     led_strip_handle_t led_strip = configure_led();

     while (1)
     {
         if (led_mode == 0)
         {
             led_breath(led_strip);
         }
         else if (led_mode == 1)
         {
             led_flow(led_strip);
         }
       
     }
      vTaskDelete(NULL);
 }
 static void led_breath(led_strip_handle_t led_strip){
     uint8_t color_r=base_r*brightness/255;
     uint8_t color_g=base_g*brightness/255;
     uint8_t color_b=base_b*brightness/255;
       for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                 ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i,color_r,color_g,color_b));
             }
             ESP_ERROR_CHECK(led_strip_refresh(led_strip));
             brightness+=direction;
             if(brightness==255||brightness==0){
                 direction*=-1;
             }
 }
 static void led_flow(led_strip_handle_t led_strip){
    int tall_length=50;
   for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) {
                 ESP_ERROR_CHECK(led_strip_clear(led_strip));
                 for(int j=0;j<tall_length;j++){
                     int position=i-j;
                     if(position>=0){
                         uint8_t tail_r=base_r-(j*(base_r/tall_length));
                         uint8_t tail_g=base_g-(j*(base_g/tall_length));
                         uint8_t tail_b=base_b-(j*(base_b/tall_length));
                         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, position,tail_r,tail_g,tail_b));
                 }
             }
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i,base_r,base_g,base_b));
 ESP_ERROR_CHECK(led_strip_refresh(led_strip));
 vTaskDelay(100/portTICK_PERIOD_MS);
 }
 }


 void app_main(void)
 {
     xTaskCreate(&blink_task, "blink_task", 8192, NULL, 5, NULL);
    
    
 }


