#include <stdio.h>
#include "encoder.h"

void init_encoder(){
    pcnt_unit_config_t unit_config = { 
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcntA_unit));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcntB_unit));
    #ifdef INVERT_ENCODER_A
    pcnt_chan_config_t chanA_config = {
        .edge_gpio_num  = ENC_A2,
        .level_gpio_num = ENC_A1,
    };  
    #else
    pcnt_chan_config_t chanA_config = {
        .edge_gpio_num  = ENC_A1,
        .level_gpio_num = ENC_A2,
    };  
    #endif
    ESP_ERROR_CHECK(pcnt_new_channel(pcntA_unit, &chanA_config, &pcntA_chan));
    #ifdef INVERT_ENCODER_B
    pcnt_chan_config_t chanB_config = {
        .edge_gpio_num  = ENC_B1,
        .level_gpio_num = ENC_B2,
    };  
    #else
    pcnt_chan_config_t chanB_config = {
        .edge_gpio_num  = ENC_B2,
        .level_gpio_num = ENC_B1,
    };  
    #endif
    ESP_ERROR_CHECK(pcnt_new_channel(pcntB_unit, &chanB_config, &pcntB_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntA_chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntA_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntB_chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntB_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcntA_unit));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcntB_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcntA_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcntB_unit));
    ESP_LOGI(encoder_tag, "pcnt units created");
}
void read_encoder(int *encoderA, int *encoderB){
    pcnt_unit_get_count(pcntA_unit, encoderA);
    pcnt_unit_clear_count(pcntA_unit);
    pcnt_unit_get_count(pcntB_unit, encoderB);
    pcnt_unit_clear_count(pcntB_unit);
}
