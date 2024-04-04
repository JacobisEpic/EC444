#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"

#define LED_GPIO 13
#define mode1 "mode1"
#define mode2 "mode2"
#define mode3 "mode3"

// Functions to manage the LED
void setup_led() {
    esp_rom_gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

void toggle_led() {
    static bool led_status = false;
    gpio_set_level(LED_GPIO, led_status ? 0 : 1);
    led_status = !led_status;
}

void app_main() {
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    setup_led();

    int count = 0;
    char inputBuffer[100];
    char mode[100] = "default_mode";

    while (1) {
        fgets(inputBuffer, sizeof(inputBuffer), stdin);
        strtok(inputBuffer, "\n");  // Remove newline

        if (strcmp(inputBuffer, "s") == 0) {
            printf("Read: s\n");
            count += 1;
        }

        switch (count % 3) {
            case 0:
                printf("toggle mode\n");
                strcpy(mode, mode1);
                break;
            case 1:
                printf("echo mode\n");
                strcpy(mode, mode2);
                break;
        }

        while (strcmp(mode, mode1) == 0) {
            fgets(inputBuffer, sizeof(inputBuffer), stdin);
            strtok(inputBuffer, "\n");
            if (strcmp(inputBuffer, "t") == 0) {
                printf("Read: t\n");
                toggle_led();
                vTaskDelay(500 / portTICK_PERIOD_MS);
                // toggle_led();
            } else if (strcmp(inputBuffer, "s") == 0) {
                printf("echo mode\n");
                strcpy(mode, mode2);
                count += 1;
                break;
            }
        }

        while (strcmp(mode, mode2) == 0) {
            fgets(inputBuffer, sizeof(inputBuffer), stdin);
            strtok(inputBuffer, "\n");
            printf("echo: %s\n", inputBuffer);
            if (strcmp(inputBuffer, "s") == 0) {
                printf("echo dec to hex mode\n");
                strcpy(mode, mode3);
                count += 1;
                break;
            }
        }

        while (strcmp(mode, mode3) == 0) {
            printf("Enter an integer: ");
            fgets(inputBuffer, sizeof(inputBuffer), stdin);
            strtok(inputBuffer, "\n");
            int decimalValue = atoi(inputBuffer);
            printf("%s\n", inputBuffer);
            printf("Hex: %X\n", decimalValue);
            if (strcmp(inputBuffer, "s") == 0) {
                printf("toggle mode\n");
                strcpy(mode, mode1);
                count += 1;
                break;
            }
        }
    }
}
