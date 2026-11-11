// 底盘状态机。chassis_step 只做状态转移和允许位,不碰硬件。
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "blade.h"

//开机自动自检,不要就改成 0
#define SELFTEST_ON_BOOT    1

typedef enum {
    CHASSIS_INIT = 0,
    CHASSIS_IDLE,
    CHASSIS_RUN,
    CHASSIS_SELFTEST,
    CHASSIS_ESTOP,
    CHASSIS_FAULT
} chassis_state_t;

typedef struct {
    float voltage;
    uint8_t enable_switch;
    uint8_t estop_req;
    uint8_t estop_release_req;
    uint8_t selftest_req;

    uint8_t cmd_new;
    float cmd_vx;
    float cmd_vz;

    uint8_t servo_valid;
    uint16_t servo_us;

    uint8_t blade_valid;
    uint8_t blade_cmd;
    uint8_t blade_sticky;

    float wheel_left;
    float wheel_right;
} chassis_in_t;

typedef struct {
    chassis_state_t state;
    uint8_t move_allowed;
    float vx;
    float vz;
    uint16_t servo_us;
    uint8_t blade_on;
} chassis_out_t;

typedef struct {
    chassis_state_t state;
    uint32_t uptime;
    uint8_t volt_low;      //欠压锁存,滞回用
    uint16_t cmd_age;
    float cmd_vx;
    float cmd_vz;
    uint16_t servo_us;

    uint8_t st_step;
    uint16_t st_tick;
    float st_lo_l, st_hi_l, st_lo_r, st_hi_r;

    //从 RUN 触发自检时先停车,等轮子停稳再开始
    uint8_t st_pending;
    uint16_t st_pending_age;

    blade_t blade;
} chassis_t;

void chassis_init(chassis_t *c);

void chassis_step(chassis_t *c, const chassis_in_t *in, uint16_t dt_ms, chassis_out_t *out);

#endif
