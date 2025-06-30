#ifndef LUX_H
#define LUX_H

#define LUX_ADC_CHANNEL ADC1_CHANNEL_5 // GPIO34
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12

#include <esp_adc/adc_continuous.h>

void lux_init(void);
float lux_read_raw(void);
float lux_read(void);
#endif // LUX_H