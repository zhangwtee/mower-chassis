// 声光指示。灯的颜色表示状态,蜂鸣器只在出事和状态切换时响。
//
// 由 hmi_task 每 100ms 调一次,所以蜂鸣最短 100ms,足够了。
#ifndef INDICATOR_H
#define INDICATOR_H

#include <stdint.h>
#include "led.h"
#include "chassis.h"

void indicator_init(void);

//每 100ms 调一次,传当前底盘状态
void indicator_step(uint8_t state);

//状态 + 闪烁相位 → 颜色。
//白=上电,绿=待命,青=跑,黄闪=自检,紫闪=急停,红=故障
static inline led_color_t ind_color(uint8_t state, uint8_t blink)
{
    switch (state) {
    case CHASSIS_IDLE:
        return LED_GREEN;
    case CHASSIS_RUN:
        return LED_CYAN;
    case CHASSIS_SELFTEST:
        return blink ? LED_YELLOW : LED_OFF;
    case CHASSIS_ESTOP:
        return blink ? LED_PURPLE : LED_OFF;
    case CHASSIS_FAULT:
        return LED_RED;
    case CHASSIS_INIT:
    default:
        return LED_WHITE;
    }
}

#endif
