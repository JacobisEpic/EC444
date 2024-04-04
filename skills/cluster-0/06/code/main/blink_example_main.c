#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";


#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_GPIO_13 13
#define BLINK_GPIO_12 12
#define BLINK_GPIO_27 27

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    if (s_led_state) {
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, 
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO


void binaryLEDs(uint8_t i) {
    gpio_set_level(BLINK_GPIO_13, i % 2);
    gpio_set_level(BLINK_GPIO_12, (i / 2) % 2);
    gpio_set_level(BLINK_GPIO_27, (i / 4) % 2);
}


static void countBinary() {
    // Count up
    for (uint8_t i = 0; i < 8; i++) {
        ESP_LOGI(TAG, "LED Binary Representation: %d", i);
        binaryLEDs(i);
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }


    for (uint8_t i = 7; i != 255; i--) { 
        ESP_LOGI(TAG, "LED Binary Representation: %d", i);
        binaryLEDs(i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO_13);
    gpio_reset_pin(BLINK_GPIO_12);
    gpio_reset_pin(BLINK_GPIO_27);
 
    gpio_set_direction(BLINK_GPIO_13, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLINK_GPIO_12, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLINK_GPIO_27, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void app_main(void) {
    configure_led();
    while (1) {
        countBinary();
    }
}
