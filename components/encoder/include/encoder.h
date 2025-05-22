#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

static const char encoder_tag[] = "encoder";

//#define INVERT_ENCODER_A
#define INVERT_ENCODER_B

#define ENC_A1   39
#define ENC_A2   34
#define ENC_B1   15
#define ENC_B2   13

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  -1000

//static int encoderA, encoderB;

static pcnt_unit_handle_t pcntA_unit, pcntB_unit;
static pcnt_channel_handle_t pcntA1_chan = NULL, pcntA2_chan = NULL, pcntB1_chan = NULL, pcntB2_chan = NULL;

void init_encoder();
void read_encoder(int *encoderA, int *encoderB);
#endif