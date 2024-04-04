#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"

// LIDAR Constants
#define LIDAR_I2C_NUM I2C_NUM_0
#define LIDAR_ADDRESS 0x62
#define REG_ACQ_COMMAND 0x00
#define REG_STATUS 0x01
#define REG_FULL_DIST_LOW 0x10
#define REG_FULL_DIST_HIGH 0x11
#define ACQ_COMMAND 0x04

// LED Constants
#define boardLED 13
#define LED1 12
#define LED2 27
#define LED3 33
#define button 15

// PID Constants
#define SETPOINT 50  // The desired distance
#define KP 1         // Proportional gain, set to 1 for simple on/off control


static const char *TAG = "example";
static uint8_t count = 0; 
static bool up = true;

void i2c_master_init() {
    // Configuration for the I2C master interface
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 23,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000  // Fast mode
    };
    i2c_param_config(LIDAR_I2C_NUM, &conf);
    i2c_driver_install(LIDAR_I2C_NUM, conf.mode, 0, 0, 0);
}

void update_leds(int error) {
    // Turn all LEDs off
    gpio_set_level(LED1, 0);
    gpio_set_level(LED2, 0);
    gpio_set_level(LED3, 0);

    // Turn the appropriate LED on based on error
    if (error < 0) {
        gpio_set_level(LED1, 1);  // Error < 0 RED LED
    } else if (error == 0) {
        gpio_set_level(LED2, 1); // Error == 0 GREEN LED
    } else {
        gpio_set_level(LED3, 1); // Error > 0 BLUE LED
    }
}


static void configure_led(void) {
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(LED1);
    gpio_reset_pin(LED2);
    gpio_reset_pin(LED3);

    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
}



// Function to write byte to a register
void lidar_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(LIDAR_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Function to read byte(s) from a register
void lidar_read_bytes(uint8_t reg, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(LIDAR_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

int read_lidar_distance() {
    uint8_t distanceArray[2];
    int distance;
    lidar_write_byte(REG_ACQ_COMMAND, ACQ_COMMAND);
    vTaskDelay(22 / portTICK_PERIOD_MS);
    lidar_read_bytes(REG_FULL_DIST_LOW, distanceArray, 2);
    distance = (distanceArray[1] << 8) | distanceArray[0];
    return distance;
}
void lidar_task(void *pvParameters) {
    uint8_t status;
    uint8_t distanceArray[2];
    int distance;

    while (1) {
        lidar_write_byte(REG_ACQ_COMMAND, ACQ_COMMAND);
        vTaskDelay(22 / portTICK_PERIOD_MS); 
        do {
            lidar_read_bytes(REG_STATUS, &status, 1);
        } while (status & 0x01); 
        lidar_read_bytes(REG_FULL_DIST_LOW, distanceArray, 2);
        distance = (distanceArray[1] << 8) | distanceArray[0]; 
        printf("Distance: %d cm\n", distance);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void pid_control_task(void *pvParameters) {
    int measured_value;
    int previous_error = 0;
    int derivative = 0;
    int error;
    int integral = 0;
    int dt = 100;
    int kd = 1;
    int ki = 0;
    int kp = 50;
    int output;


    while (1) {
        measured_value = read_lidar_distance();
        error = SETPOINT - measured_value;
        integral = integral + error * dt;
        derivative = (error - previous_error) / dt;
        output = kp * error + ki * integral + kd * derivative;
        previous_error = error;
        update_leds(output);
        ESP_LOGI(TAG, "Error: %d, Output: %d", error, output);
        vTaskDelay(pdMS_TO_TICKS(dt));
    }
}



void app_main() {
    i2c_master_init(); 
    configure_led();

    xTaskCreate(&pid_control_task, "pid_control_task", 2048, NULL, 5, NULL);
    xTaskCreate(&lidar_task, "lidar_task", 2048, NULL, 5, NULL);

}
