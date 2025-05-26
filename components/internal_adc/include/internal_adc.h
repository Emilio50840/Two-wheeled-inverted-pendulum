#pragma once

#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"

#define ADC1 35
#define ADC2 36

extern adc_oneshot_unit_handle_t ADC_unit;
extern adc_unit_t adc_unit;
extern adc_channel_t adc1, adc2;


extern void init_internal_adc(void);
