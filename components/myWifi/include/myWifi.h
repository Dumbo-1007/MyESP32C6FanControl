#include "esp_err.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <esp_wifi.h>
#include "esp_log.h"
#include "esp_event.h"
#include <nvs_flash.h>
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_http_client.h"

#define ESP_WIFI_STA_SSID "Jing"
#define ESP_WIFI_STA_PASSWD "zhangjing"

void WIFI_CallBack(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void send_data_to_server(float temperature, float humidity, float pwm_duty,float luxx);
void wificonfig();