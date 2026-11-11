// 编码器。TIM2 左轮,TIM3 右轮,编码器模式 3。
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

//上机发现读数反了就改这里,+1 或 -1。右轮默认取反
#define ENCODER_L_SIGN      (+1)
#define ENCODER_R_SIGN      (-1)

void encoder_init(void);

//读上一次清零后的计数,正负代表转向
int16_t encoder_left(void);
int16_t encoder_right(void);

#endif
