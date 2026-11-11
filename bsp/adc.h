// ADC:电池电压和使能开关。
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);

//电池电压,单位 V
float adc_battery_volt(void);

//使能开关,1 表示允许运动(引脚为高)
uint8_t enable_switch_ok(void);

#endif
