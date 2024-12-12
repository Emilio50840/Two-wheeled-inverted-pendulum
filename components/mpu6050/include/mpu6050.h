#ifndef MPU6050_H
#define MPU6050_H
#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c_master.h"

static const char mpu_tag[] = "mpu";

#define PIN_I2C_SDA GPIO_NUM_4
#define PIN_I2C_SCL GPIO_NUM_5

static i2c_master_dev_handle_t dev_handle;
static uint8_t data_wr[2] , data_rd[14] = {0};

void init_mpu6050();
void read_mpu6050(uint8_t data[14]);
#endif