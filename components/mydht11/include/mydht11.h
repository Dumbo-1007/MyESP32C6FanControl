#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h" // 确保包含日志头文件
#include <driver/gpio.h>
#include <driver/ledc.h>

#define IN1_PIN 7   // 连接到 L9110 的 IN1
#define IN2_PIN 6   // 连接到 L9110 的 IN2
#define PWM_PIN 7   // 使用 IN1 引脚作为 PWM 输出
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0

void l9110_pwm_init(void);
void set_motor_speed(uint8_t speed);

