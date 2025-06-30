#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <stdint.h>
#include <stddef.h>
#include "esp_system.h"
#include "esp_netif.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "cJSON.h"

#ifndef MYMQTT_H
#define MYMQTT_H
typedef enum
{
  FAN_MANUAL_OFF,
  FAN_MANUAL_ON,
  FAN_AUTO,
  FAN_UNKNOWN
} fan_state_t;
extern fan_state_t fan_state1;
/**
 * @brief 初始化并启动 MQTT 客户端
 *
 * 该函数配置并启动 MQTT 客户端，连接到指定的 Broker 地址。
 */
void mqtt_app_start(void);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void log_error_if_nonzero(const char *message, int error_code);

extern void control_fan(fan_state_t state);
extern fan_state_t parse_fan_state(const char *state_str);

#endif // MYMQTT_H