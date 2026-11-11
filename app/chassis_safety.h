// 电压判定。
//
// 原来只给蜂鸣提示做了 20/21 的滞回,停机阈值是裸的。
// 电量刚好卡在 20V 附近时,状态机会在 RUN/IDLE 之间反复跳,而每次离开 RUN
// 都会作废旧指令 —— 表现就是车一窜一停,得反复重发指令。
//
// 这里给停机阈值也加滞回:低于 MIN 判欠压并锁住,高于 RESUME 才恢复。
// 锁住之后即使电压回到 20.5V 也不放行,避免在临界点上抖。
#ifndef CHASSIS_SAFETY_H
#define CHASSIS_SAFETY_H

#include <stdint.h>

#define SAFETY_VOLTAGE_MIN      20.0f   //低于这个判欠压
#define SAFETY_VOLTAGE_RESUME   21.0f   //高于这个才解除
#define SAFETY_CMD_AGE_MAX      500     //ms
#define SAFETY_BLADE_AGE_MAX    300     //ms
#define SAFETY_BOOT_MS          10000   //上电静默

//滞回判定。latched 是调用方保存的状态,初值 0。
//返回 1 表示电压可用
uint8_t safety_volt_ok(float voltage, uint8_t *latched);

uint8_t safety_move_ok(uint8_t volt_ok, uint8_t enable_switch, uint16_t cmd_age);

uint8_t safety_blade_ok(uint8_t state_ok, uint8_t volt_ok);

#endif
