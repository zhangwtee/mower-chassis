// 任务之间共享的状态。只有 chassis 任务写,别的任务读。
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

typedef struct {
    float voltage;
    float wheel_left;
    float wheel_right;
    float vx;
    float vz;
    uint8_t flag_stop;
    uint8_t imu_ok;
    uint8_t chassis_state;
    uint8_t enable_switch;
    uint16_t cmd_age;
    int16_t accel[3];
    int16_t gyro[3];
    int16_t dist_mm[8];
} app_state_t;

extern app_state_t g_state;

#endif
