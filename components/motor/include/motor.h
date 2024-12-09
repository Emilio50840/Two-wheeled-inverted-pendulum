#ifndef MOTOR_H
#define MOTOR_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "hal/ledc_types.h"

static const char motor_tag[] = "motor";

//#define INVERT_MOTOR_A
//#define INVERT_MOTOR_B

#define PWMA_pin 8
#define PWMB_pin 9
#define AIN1     10
#define AIN2     11
#define BIN1     12
#define BIN2     13
#define STBY_pin 18
#define PWM_freq 20000

void init_motor();
void set_motor(int pwmA, int pwmB);
#endif