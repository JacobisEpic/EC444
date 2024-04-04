#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define LIDAR_I2C_NUM I2C_NUM_0
#define LIDAR_ADDRESS 0x62
#define REG_ACQ_COMMAND 0x00
#define REG_STATUS 0x01
#define REG_FULL_DIST_LOW 0x10
#define REG_FULL_DIST_HIGH 0x11
#define ACQ_COMMAND 0x04

// Function to initiate I2C communication
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

void lidar_task(void *pvParameters) {
    uint8_t status;
    uint8_t distanceArray[2];
    int distance;

    while (1) {
        // Start acquisition
        lidar_write_byte(REG_ACQ_COMMAND, ACQ_COMMAND);
        
        // Wait for the acquisition to complete
        vTaskDelay(22 / portTICK_PERIOD_MS); // Delay for boot-up
        
        // Check status
        do {
            lidar_read_bytes(REG_STATUS, &status, 1);

        } while (status & 0x01); // Loop until LSB is low which indicates the device is ready

        // Read distance from the device
        lidar_read_bytes(REG_FULL_DIST_LOW, distanceArray, 2);
        distance = (distanceArray[1] << 8) | distanceArray[0]; // Combine bytes for distance

        // Print the distance
        printf("Distance: %d cm\n", distance);

        // In a real application, you would now do something with the distance data.
        // For this assignment, we simply delay before taking the next measurement.
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms delay before the next read
    }
}

void app_main() {
    i2c_master_init();
    xTaskCreate(&lidar_task, "lidar_task", 2048, NULL, 5, NULL);
}
