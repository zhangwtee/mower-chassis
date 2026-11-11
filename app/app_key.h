// 蓝牙 APP 的按键通道(UART4)。只管按键,不做摇杆控制。
#ifndef APP_KEY_H
#define APP_KEY_H

#include <stdint.h>

void app_key_init(void);

//UART4 收到一个字节就喂进来
void app_key_feed(uint8_t c);

#endif
