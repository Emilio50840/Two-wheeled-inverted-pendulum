#include <math.h>
#include <stdio.h>
#include "adc.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <string.h>

#include "motor.h"
#include "mpu6050.h"
#include "control.h"

#include "driver/uart.h"

#define EMBEBIDO

const char TAG[] = "balancin";

#define BUF_SIZE 512
#define PACKET_SIZE 9

#define TXD_PIN (GPIO_NUM_47)
#define RXD_PIN (GPIO_NUM_21)


#define PORT                        4545
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

#define RATIO 0.25

const uint8_t wifi_ssid[] = "balancin";
const uint8_t wifi_pass[] = "1q2w3e4r";
const int  wifi_chan   = 1;
const int  wifi_conn   = 3;

volatile int connection_sock, socket_active = 0;

control_t ctrl;
TimerHandle_t control_timer_h = NULL;
EventGroupHandle_t xTcpEventGroup;
int encoderA, encoderB;
int16_t Ax, Gy;
uint16_t adc_read[8];

void init_wifi();
static void control_timer( TimerHandle_t xTimer );
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void tcp_server_task(void *pvParameters);
static void TCP_receive(const int sock);
void init_uart(void);


void app_main(void)
{
    init_wifi();
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, configMAX_PRIORITIES - 2, NULL);
    init_adc();
    init_encoder();
    init_motor();
    init_mpu6050();
    init_uart();
    init_control(&ctrl);
    printf("Ready to start\r\n");
    /*/
    while(1){
        char c = getchar();
        printf("char: %x\r\n", c);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //*/

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
    //calculate_control(&ctrl);
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
    

    read_adc(adc_read);
    ESP_LOGI(TAG, "[0]: %u\t[1]: %u\t[2]: %u\t[3]: %u\t[4]: %u\t[5]: %u\t[6]: %u\t[7]: %u\t",adc_read[0] ,adc_read[1] ,adc_read[2] ,adc_read[3] ,adc_read[4] ,adc_read[5] ,adc_read[6] ,adc_read[7] );
    int sr,sl;
    sr = (int)(((float)adc_read[3]*RATIO)+((float)adc_read[2]*(1-RATIO)));
    sl = (int)(((float)adc_read[4]*RATIO)+((float)adc_read[5]*(1-RATIO)));
    uint8_t mpu_data[14];
    read_mpu6050(mpu_data);
    Ax = mpu_data[1] + (((int16_t)mpu_data[0])<<8);
    Gy = mpu_data[11] + (((int16_t)mpu_data[10])<<8);


    read_encoder(&encoderA, &encoderB);

    //printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
    ctrl.sl = (float)(adc_read[3]>>2);
    ctrl.sr = (float)(adc_read[4]>>2);
    ctrl.Ax = (float)Ax;
    ctrl.Gy = (float)Gy;
    ctrl.incl = (float) -encoderA;
    ctrl.incr = (float) -encoderB;
    //ESP_LOGI(TAG, "incr: %f\tencoderB: %i", ctrl.incr, encoderB);

    uint8_t packet[9];
    packet[0] = 0xAA;

    #ifdef EMBEBIDO

    
    
    #else
    /*/
    if(encoderB > 127){
        packet[1] = 127;
    }else if(encoderB < -128){
        packet[1] = -128;
    }else{
        packet[1] = (int8_t) (encoderB);
    }
    //*/
    if(encoderB > 127){
        packet[1] = 255;
    }else if(encoderB < -128){
        packet[1] = 0;
    }else{
        packet[1] = (int8_t) (encoderB + 127);
    }
    /*/
    if(encoderA > 127){
        packet[2] = 127;
    }else if(encoderA < -128){
        packet[2] = -128;
    }else{
        packet[2] = (int8_t) (encoderA);
    }
    //*/
    if(encoderA > 127){
        packet[2] = 255;
    }else if(encoderA < -128){
        packet[2] = 0;
    }else{
        packet[2] = (int8_t) (encoderA + 127);
    }
    //*/
    //packet[2] = (int8_t) (encoderA);
    packet[3] = mpu_data[0];
    packet[4] = mpu_data[1];
    packet[5] = mpu_data[10];
    packet[6] = mpu_data[11];
    packet[7] = (uint8_t)(adc_read[3]>>2);
    packet[8] = (uint8_t)(adc_read[4]>>2);
    //packet[7] = (uint8_t)(sr>>2);
    //packet[8] = (uint8_t)(sl>>2);
    #endif
    
    if(xEventGroupGetBits(xTcpEventGroup) & 0x1){
        calculate_control(&ctrl);
        packet[1] = ctrl.vdg;
        packet[2] = ctrl.vg;
        packet[3] = ctrl.thetag;
        packet[4] = ctrl.alphag;
        packet[5] = ctrl.omegalg;
        packet[6] = ctrl.omegarg;
        packet[7] = ctrl.ulg;
        packet[8] = ctrl.urg;
        set_motor(ctrl.uWl, ctrl.uWr);
        
        int to_write = 9;
        while (to_write > 0) {
            int written = send(connection_sock, packet + (PACKET_SIZE - to_write), to_write, 0);
            if (written < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
                // Failed to retransmit, giving up
            }
            to_write -= written;
        }
    }else {
        set_motor(0, 0);
        init_control(&ctrl);
    }
    uart_write_bytes(UART_NUM_1, packet, sizeof(packet));
    
    
    //vTaskDelay(pdMS_TO_TICKS(5));
    //printf("Alpha: %f\tTheta: %f\tuWl: %f\tuWr: %f\til: %f\tir: %f\r\n",ctrl.alpha, ctrl.theta, ctrl.uWl, ctrl.uWr, ctrl.incl, ctrl.incr);
    
    uint8_t rx_buffer[32];
    int rxBytes = uart_read_bytes(UART_NUM_1, rx_buffer, sizeof(rx_buffer), 5);
#ifndef EMBEBIDO

    int pwmA, pwmB;
    if (rxBytes >= 2) {
        if(rx_buffer[0] > 128){
            pwmA = -(rx_buffer[rxBytes-2] - 128);
        }else{
            pwmA = rx_buffer[0];
        }
        if(rx_buffer[1] > 128){
            pwmB = -(rx_buffer[rxBytes-1] - 128);
        }else{
            pwmB = rx_buffer[1];
        }
        set_motor(pwmA, pwmB);
    }else {
        set_motor(0, 0);
    }
#endif 
    
}

void init_wifi(){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.ap.ssid, (const char *)wifi_ssid);
    wifi_config.ap.ssid_len = strlen((const char *)wifi_ssid);
    wifi_config.ap.channel = wifi_chan;
    strcpy((char *)wifi_config.ap.password, (const char *)wifi_pass);
    wifi_config.ap.max_connection = wifi_conn;
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
    wifi_config.ap.authmode = WIFI_AUTH_WPA3_PSK;
    wifi_config.ap.ssid_hidden = false;
    wifi_config.ap.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
#endif
    wifi_config.ap.pmf_cfg.required = true;
    
    if (strlen((const char *)wifi_pass)== 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             wifi_ssid, wifi_pass, wifi_chan);
    xTcpEventGroup = xEventGroupCreate();

    /* Was the event group created successfully? */
    if( xTcpEventGroup == NULL )
    {
        /* The event group was not created because there was insufficient
           FreeRTOS heap available. */
        ESP_LOGE(TAG, "Error creating event group");
    }
    else
    {
        ESP_LOGI(TAG, "Event group was created successfully");
        /* The event group was created. */
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        set_motor(0, 0);
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        TCP_receive(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static void TCP_receive(int sock)
{
    xEventGroupSetBits( xTcpEventGroup,
                                0x1 );
    
    //socket_active = 1;
    connection_sock = sock;
    int len;
    uint8_t rx_buffer[BUF_SIZE];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            set_motor(0, 0);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
            set_motor(0, 0);
        } else {
            /*/
            printf("len:%i\t0x", len);
            for(int i = 0; i<len; i++){
                printf("%i",rx_buffer[i]);
            }
            printf("\r\n");
            //*/
#ifndef EMBEBIDO
            int pwmA, pwmB;
            if(rx_buffer[0] > 128){
                pwmA = -(rx_buffer[0] - 128);
            }else{
                pwmA = rx_buffer[0];
            }
            if(rx_buffer[1] > 128){
                pwmB = -(rx_buffer[1] - 128);
            }else{
                pwmB = rx_buffer[1];
            }
            set_motor(pwmA, pwmB);
#endif
        }

    } while (len > 0);

    xEventGroupClearBits( xTcpEventGroup,
                                0x1 );
    //socket_active = 0;
}

void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}