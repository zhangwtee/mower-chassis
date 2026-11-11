// 电机 PWM 与方向。TIM8 CH3/CH4,10kHz。
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_FULL_DUTY     16800     //= ARR+1,保证 CCR>ARR 是真正的 100%

//上机发现轮子反了就改这里,+1 或 -1
#define MOTOR_A_SIGN        (+1)
#define MOTOR_B_SIGN        (+1)

void motor_init(void);

//duty 带符号,正为前进;超过满占空比会夹住
void motor_set(int16_t a, int16_t b);

#endif
