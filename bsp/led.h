// RGB 灯。三路普通 GPIO 开关,不做亮度调节。
//
// 不走 PWM 是刻意的:绿灯那一路在 TIM9_CH2,而舵机占着 TIM9_CH1,
// 两个通道共用 ARR/PSC,做 PWM 会把舵机的 50Hz 一起改掉。
// 状态指示用纯色开关就够,七种颜色全是现成的。
#ifndef LED_H
#define LED_H

#include <stdint.h>

//极性:高电平点亮。
//依据原来的 PWM 写法 —— 255 映射成 CCR=10000、ARR=9999,
//也就是 100% 占空比算"全亮",所以是高有效。
//实机发现是反的,把这个改成 1
#define LED_ACTIVE_LOW      0

typedef enum {
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_YELLOW,
    LED_CYAN,
    LED_PURPLE,
    LED_WHITE
} led_color_t;

void led_init(void);

void led_set(led_color_t c);

#endif
