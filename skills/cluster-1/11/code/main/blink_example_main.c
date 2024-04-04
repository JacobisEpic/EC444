/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

#ifdef CONFIG_BLINK_LED_STRIP


#elif CONFIG_BLINK_LED_GPIO
#define boardLED 13
#define LED1 12
#define LED2 27
#define LED3 33
#define led4 32
#define button 15

static void BinaryLEDs(uint8_t count) {
    gpio_set_level(LED1, count % 2);
    gpio_set_level(LED2, (count / 2) % 2);
    gpio_set_level(LED3, (count / 4) % 2);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(boardLED);
    gpio_reset_pin(LED1);
    gpio_reset_pin(LED2);
    gpio_reset_pin(LED3);
    gpio_reset_pin(led4);
    gpio_reset_pin(button);
 
    gpio_set_direction(boardLED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(led4, GPIO_MODE_OUTPUT);
    gpio_set_direction(button, GPIO_MODE_INPUT);
}

static uint8_t count = 0; 
static bool up = true;

static void TaskA(void) {
    while (1) {
        ESP_LOGI(TAG, "Count: %d", count);
        BinaryLEDs(count);

        if (up) {
            count = (count == 7) ? 0 : (count + 1);
        } else {
            count = (count == 0) ? 7 : (count - 1);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void TaskB(void) {
    while (1) {
        if (gpio_get_level(button) == 1) {
            up = !up; 

            vTaskDelay(1000 / portTICK_PERIOD_MS); 
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void TaskC(void){
    while(1){
        if (up){
            gpio_set_level(led4, 1);
        } else {
            gpio_set_level(led4, 0);  
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

#else
#error "unsupported LED type"
#endif

void app_main(void) {
    configure_led();

    xTaskCreate(TaskA, "TaskA", 2048, NULL, 5, NULL);
    xTaskCreate(TaskB, "TaskB", 2048, NULL, 5, NULL);
    xTaskCreate(TaskC, "TaskC", 2048, NULL, 5, NULL);
}



// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "led_strip.h"
// #include "sdkconfig.h"

// static const char *TAG = "example";


// #define BLINK_GPIO CONFIG_BLINK_GPIO
// #define BLINK_GPIO_13 13
// #define BLINK_GPIO_12 12
// #define BLINK_GPIO_27 27
// #define BLINK_GPIO_33 33
// #define BLINK_GPIO_15 15 //Button
// #define BLINK_GPIO_32 32

// static uint8_t s_led_state = 0;
// static bool up = true;

// #ifdef CONFIG_BLINK_LED_STRIP

// static led_strip_handle_t led_strip;

// static void BinaryLEDs(void)
// {
//     if (s_led_state) {
//         led_strip_set_pixel(led_strip, 0, 16, 16, 16);
//         led_strip_refresh(led_strip);
//     } else {
//         led_strip_clear(led_strip);
//     }
// }


// #elif CONFIG_BLINK_LED_GPIO


// void binaryLEDs(uint8_t i) {
//     gpio_set_level(BLINK_GPIO_12, i % 2);
//     gpio_set_level(BLINK_GPIO_27, (i / 2) % 2);
//     gpio_set_level(BLINK_GPIO_33, (i / 4) % 2);
// }

// // void binaryLEDs(uint8_t i) {
// //     gpio_set_level(BLINK_GPIO_12, (i >> 0) & 1);
// //     gpio_set_level(BLINK_GPIO_27, (i >> 1) & 1);
// //     gpio_set_level(BLINK_GPIO_33, (i >> 2) & 1);
// //     // gpio_set_level(BLINK_GPIO_32, (i >> 3) & 1);
// // }



// static void countBinaryUp() {
//     // Count up
//     for (uint8_t i = 0; i < 8; i++) {
//         ESP_LOGI(TAG, "LED Binary Representation: %d", i);
//         binaryLEDs(i);
//         vTaskDelay(1000 / portTICK_PERIOD_MS); 

//     }
//     // up = false;
// }
// static void countBinaryDown() {

//     for (uint8_t i = 7; i > -1; i--) { 
//         ESP_LOGI(TAG, "LED Binary Representation: %d", i);
//         binaryLEDs(i);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);  
//     }
//     // up = true;
// }

// static void configure_led(void)
// {
//     ESP_LOGI(TAG, "Example configured to blink GPIO LED!");


//     gpio_reset_pin(BLINK_GPIO_13);
//     gpio_reset_pin(BLINK_GPIO_12);
//     gpio_reset_pin(BLINK_GPIO_27);
//     gpio_reset_pin(BLINK_GPIO_33);
//     gpio_reset_pin(BLINK_GPIO_15);
//     gpio_reset_pin(BLINK_GPIO_32);
 
//     gpio_set_direction(BLINK_GPIO_12, GPIO_MODE_OUTPUT);
//     gpio_set_direction(BLINK_GPIO_27, GPIO_MODE_OUTPUT);
//     gpio_set_direction(BLINK_GPIO_33, GPIO_MODE_OUTPUT);
//     gpio_set_direction(BLINK_GPIO_32, GPIO_MODE_OUTPUT);
//     gpio_set_direction(BLINK_GPIO_15, GPIO_MODE_INPUT);
// }




// void task_1(void) {
//     while (1) {
//         if (gpio_get_level(BLINK_GPIO_15) == 1) {
//             up = !up;
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//         }
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }

// }

// void task_2(void) {
//     while (1) {
//         if (up == true) {
//             countBinaryUp();
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//         }
//         else {
//             countBinaryDown();
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//         }
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }

// }

// void task_3(void) {
//     while (1) {
//         if (up == true) {
//             gpio_set_level(BLINK_GPIO_32, 1);

//         }
//         else {
//             gpio_set_level(BLINK_GPIO_32, 0);

//         }
//     }
// }


// #else
// #error "unsupported LED type"
// #endif

// void app_main()
// {
//     configure_led();
//     xTaskCreate(task_1, "task_1",2048, NULL, 5, NULL);
//     xTaskCreate(task_2, "task_2",2048, NULL, 5, NULL);
//     xTaskCreate(task_3, "task_3",2048, NULL, 5, NULL);
// }			    		// Instantiate tasks with priorites and stack size

