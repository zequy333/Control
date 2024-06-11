
#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"

esp_err_t set_i2c(void);
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_accel(float *accel_x, float *accel_y, float *accel_z);

#endif 
