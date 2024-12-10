#include <stdio.h>
#include "motor.h"

void init_motor(){
    gpio_config_t gpio = {
       .pin_bit_mask   = (1<<AIN1) | (1<<AIN2) | (1<<BIN1) | (1<<BIN2) | (1<<STBY_pin),
       .mode           = GPIO_MODE_OUTPUT,
       .pull_up_en     = GPIO_PULLUP_DISABLE,
       .pull_down_en   = GPIO_PULLDOWN_DISABLE,
       .intr_type      = GPIO_INTR_DISABLE
   };
   //initialize inputs
   ESP_ERROR_CHECK(gpio_config(&gpio));
   ledc_timer_config_t timer_conf = {0};
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.freq_hz = PWM_freq;
    timer_conf.duty_resolution = LEDC_TIMER_7_BIT;
    timer_conf.clk_cfg = LEDC_LOW_SPEED_MODE;
    timer_conf.deconfigure = false;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
    ledc_channel_config_t led_conf = {0};
    led_conf.gpio_num = PWMA_pin;
    led_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    led_conf.channel = 0;
    led_conf.intr_type = LEDC_INTR_DISABLE;
    led_conf.timer_sel = LEDC_TIMER_0;
    led_conf.flags.output_invert = 0;
    led_conf.duty = 0;
    led_conf.hpoint =  0;
    ESP_ERROR_CHECK(ledc_channel_config(& led_conf));
    led_conf.gpio_num = PWMB_pin;
    led_conf.channel = 1;
    ESP_ERROR_CHECK(ledc_channel_config(& led_conf));
}
void set_motor(int pwmA, int pwmB){
    uint8_t stby_flag = 0;
    if (pwmA > 0){
            gpio_set_level(AIN1, 1);
            gpio_set_level(AIN2, 0);
        }else if (pwmA < 0){
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 1);
        }else{
            gpio_set_level(AIN1, 0);
            gpio_set_level(AIN2, 0);
            stby_flag |= 0x1;
        }
        if (pwmB > 0){
            gpio_set_level(BIN1, 1);
            gpio_set_level(BIN2, 0);
        }else if (pwmB < 0){
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 1);
        }else{
            gpio_set_level(BIN1, 0);
            gpio_set_level(BIN2, 0);
            stby_flag |= 0x2;
        }
        /*
        if(stby_flag == 0x3){
            gpio_set_level(STBY_pin, 0);
        }else{
            gpio_set_level(STBY_pin, 1);
        }
        */
        gpio_set_level(STBY_pin, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, abs(pwmA));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, abs(pwmB));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, 1);
}
