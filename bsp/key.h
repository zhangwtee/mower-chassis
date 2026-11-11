// 用户按键。PC5,上拉,按下为低。
#ifndef KEY_H
#define KEY_H

#include <stdint.h>

void key_init(void);

//1 = 按住
uint8_t key_pressed(void);

#endif
