#include <stdio.h>
#include "mydht11.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h" // 确保包含日志头文件
#include <driver/gpio.h>
#include <driver/ledc.h>


#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0

static const char *TAG = "L9110_PWM_Control";
void l9110_pwm_init(void)
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << IN2_PIN);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  gpio_set_level(IN2_PIN,0);
  // 配置 PWM 输出（用于 IN1）
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = LEDC_TIMER_8_BIT,  // 分辨率：0~255
        .freq_hz          = 1000,              // PWM 频率 1kHz
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = PWM_CHANNEL,
        .duty       = 0,
        .gpio_num   = PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER,
    };
    ledc_channel_config(&ledc_channel);

    ESP_LOGI(TAG, "L9110 PWM 初始化完成");
}

// 设置电机转速（speed: 0~255）
void set_motor_speed(uint8_t speed)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
    ESP_LOGI(TAG, "设置电机速度: %d", speed);
}
