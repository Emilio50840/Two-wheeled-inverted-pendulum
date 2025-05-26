#ifndef MOTOR_H
#define MOTOR_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "hal/ledc_types.h"

static const char motor_tag[] = "motor";

#define INVERT_MOTOR_A
#define INVERT_MOTOR_B

#define PWMA_pin 32
#define AIN1     33
#define AIN2     25
#define STBY_pin 26
#define BIN1     27
#define BIN2     14
#define PWMB_pin 12

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_7_BIT
#define LEDC_CHANNEL_A          LEDC_CHANNEL_0
#define LEDC_CHANNEL_B          LEDC_CHANNEL_1
#define PWM_freq 20000

void init_motor();
void set_motor(int pwmA, int pwmB);
#endif