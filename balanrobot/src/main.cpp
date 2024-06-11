
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

static const char *TAG = "Main";

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
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
