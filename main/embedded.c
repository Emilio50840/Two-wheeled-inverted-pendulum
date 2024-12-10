#include <math.h>
#include <stdio.h>
#include "adc.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "motor.h"
#include "mpu6050.h"
#include "control.h"

static void control_timer( TimerHandle_t xTimer );

control_t ctrl;
TimerHandle_t control_timer_h = NULL;
int encoderA, encoderB;
int16_t Ax, Gy;
uint16_t adc_read[8];

void app_main(void)
{
    init_adc();
    init_encoder();
    init_motor();
    init_mpu6050();
    init_control(&ctrl);
    

    control_timer_h = xTimerCreate(
        /* Text name for the software timer - not used by FreeRTOS. */
                                     "control_proc",
        /* The software timer's period in ticks. */
                                     pdMS_TO_TICKS(10),
        /* Setting uxAutoRealod to pdTRUE creates an auto-reload timer. */
                                     pdTRUE,
        /* This example does not use the timer id. */
                                     0,
        /* Callback function to be used by the software timer being created. */
                                     control_timer );
    xTimerStart(control_timer_h, 0);
    /*
    while (1) {
        read_encoder(&encoderA, &encoderB);
        set_motor(1023, -1023);
        read_adc(adc_read);
        read_mpu6050(&Ax, &Gy);
        printf("Positive\r\n");
        printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
        vTaskDelay(pdMS_TO_TICKS(1000));


        read_encoder(&encoderA, &encoderB);
        set_motor(-1023, 1023);
        read_adc(adc_read);
        read_mpu6050(&Ax, &Gy);
        printf("Negative\r\n");
        printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //*/
}

static void control_timer( TimerHandle_t xTimer ){
    
    //set_motor(1023, 1023);
    int Ax_avg, Gy_avg;
    Ax_avg = 0;
    Gy_avg = 0;

    read_adc(adc_read);
    read_mpu6050(&Ax, &Gy);
    /*
    for (int i = 0; i < 10; i++) {
        read_mpu6050(&Ax, &Gy);
        Ax_avg += Ax;
        Gy_avg += Gy;
    }

    Ax = Ax_avg / 10;
    Gy = Gy_avg / 10;
    */
    read_encoder(&encoderA, &encoderB);
    ctrl.sl = (float)(adc_read[3]);
    ctrl.sr = (float)(adc_read[4]);
    ctrl.Ax = (float)Ax;
    ctrl.Gy = (float)Gy;
    ctrl.incl = (float) encoderA;
    ctrl.incr = (float) encoderB;
    calculate_control(&ctrl);
    //vTaskDelay(pdMS_TO_TICKS(5));
    set_motor((int)ctrl.uWl, (int)ctrl.uWr);
    printf("Alpha: %f\tTheta: %f\tuWl: %f\tuWr: %f\til: %f\tir: %f\r\n",ctrl.alpha, ctrl.theta, ctrl.uWl, ctrl.uWr, ctrl.incl, ctrl.incr);
    //printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
}