// 精确延时。用 DWT 周期计数器,不受编译优化影响。
#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

//用 DWT 之前必须先调一次
void delay_init(void);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif
