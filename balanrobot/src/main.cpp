#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_SLAVE_ADDR 0X00
#define TIMEOUT_MS 1000
#define DELAY_MS 1000

#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C

static const char *TAG = "I2C";

extern "C" void app_main();

static esp_err_t set_i2c(void) {
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = 21;
    i2c_config.scl_io_num = 22;
    i2c_config.sda_pullup_en = true;
    i2c_config.scl_pullup_en = true;
    i2c_config.master.clk_speed = 400000;
    i2c_config.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_LEVEL1));
    return ESP_OK;
}

static esp_err_t mpu6050_init() {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Wake up the MPU6050
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_read_accel(float *accel_x, float *accel_y, float *accel_z) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *accel_x = (int16_t)(data[0] << 8 | data[1]) / 16384.0;
        *accel_y = (int16_t)(data[2] << 8 | data[3]) / 16384.0;
        *accel_z = (int16_t)(data[4] << 8 | data[5]) / 16384.0;
    }
    return ret;
}

extern "C" void app_main() {
    ESP_ERROR_CHECK(set_i2c());
    ESP_ERROR_CHECK(mpu6050_init());

    float accel_x, accel_y, accel_z;

    while (1) {
        esp_err_t ret = mpu6050_read_accel(&accel_x, &accel_y, &accel_z);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Accel X: %.2f, Y: %.2f, Z: %.2f m/s^2", accel_x, accel_y, accel_z);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data");
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}
