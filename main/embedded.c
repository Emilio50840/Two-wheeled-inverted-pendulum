#include <stdio.h>
#include "adc.h"
#include "encoder.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "motor.h"
#include "mpu6050.h"

void app_main(void)
{
    init_adc();
    init_encoder();
    init_motor();
    init_mpu6050();
    int encoderA, encoderB;
    uint16_t Ax, Gy;
    uint16_t adc_read[8];
    while (1) {
        read_encoder(&encoderA, &encoderB);
        set_motor(1023, 1023);
        read_adc(adc_read);
        read_mpu6050(&Ax, &Gy);
        printf("Positive\r\n");
        printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
        vTaskDelay(pdMS_TO_TICKS(1000));


        read_encoder(&encoderA, &encoderB);
        set_motor(-1023, -1023);
        read_adc(adc_read);
        read_mpu6050(&Ax, &Gy);
        printf("Negative\r\n");
        printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
