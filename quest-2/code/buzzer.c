#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BUZZER_GPIO 25 // GPIO number where the buzzer is connected

void buzzer_task(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Turn on the buzzer (make it buzz)
        gpio_set_level(BUZZER_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Buzz for 1 second

        // Turn off the buzzer
        gpio_set_level(BUZZER_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second
    }
}

void app_main() {
    xTaskCreate(&buzzer_task, "buzzer_task", 2048, NULL, 5, NULL);
}
