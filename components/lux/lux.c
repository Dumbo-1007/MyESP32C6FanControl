#include "lux.h"
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG9 = "LUX";
// 设置ADC的采样宽度为预定义值ADC_WIDTH；
// 配置LUX传感器使用的ADC通道及其衰减等级为ADC_ATTEN；
void lux_init(void)
{
    esp_err_t ret;

    // Configure ADC width and attenuation
    ret = adc1_config_width(ADC_WIDTH);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG9, "Failed to configure ADC width: %s", esp_err_to_name(ret));
        return;
    }

    ret = adc1_config_channel_atten(LUX_ADC_CHANNEL, ADC_ATTEN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG9, "Failed to configure ADC attenuation: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG9, "ADC initialized for LUX sensor");
}

float lux_read_raw(void)
{
    return adc1_get_raw(LUX_ADC_CHANNEL);
}

// 用于读取原始 ADC 值并将其转换为电压值返回一个 float 类型的电压值，转换公式为 (raw * 3.3 / 4095.0)。
float lux_read(void)
{
    int raw = lux_read_raw();
    return (float)raw * 3.3 / 4095.0;
}
