// 舵机。TIM9 CH1 / PA2,50Hz。
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void servo_init(void);

//脉宽单位 us,超出 500~2500 会夹住
void servo_set(uint16_t us);

#endif
