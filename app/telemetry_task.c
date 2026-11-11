// 上行遥测。24 字节底盘帧 + 19 字节超声波帧,20Hz
#include "app.h"
#include "watchdog.h"
#include "diag.h"
#include "app_state.h"
#include "telemetry_task.h"
#include "telemetry.h"
#include "uart.h"
#include "can.h"

#define TASK_MS     50

void telemetry_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    telemetry_chassis_t t;
    uint8_t frame[TELEMETRY_CHASSIS_LEN];
    uint8_t dist[TELEMETRY_DISTANCE_LEN];
    uint8_t i;

    (void)arg;

    t.flag_stop = 0;
    t.voltage_mv = 0;
    for (i = 0; i < 3; i++) {
        t.vel_mmps[i] = 0;
        t.accel_raw[i] = 0;
        t.gyro_raw[i] = 0;
    }

    for (;;) {

        wd_kick(WD_BIT_TELEMETRY);

        diag_kick(DIAG_TELEMETRY);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TASK_MS));

        t.flag_stop = g_state.flag_stop;
        t.vel_mmps[0] = (int16_t)(g_state.vx * 1000.0f);
        t.vel_mmps[1] = 0;
        t.vel_mmps[2] = (int16_t)(g_state.vz * 1000.0f);
        t.voltage_mv = (uint16_t)(g_state.voltage * 1000.0f);

        for (i = 0; i < 3; i++) {
            t.accel_raw[i] = g_state.accel[i];
            t.gyro_raw[i] = g_state.gyro[i];

        //失能时 Z 轴角速度发 0,免得停着的车上位机看到在转
        if (g_state.flag_stop) {
            t.gyro_raw[2] = 0;
        }
        }

        telemetry_build_chassis(frame, &t);
        telemetry_build_distance(dist, g_state.dist_mm);

        uart1_write(frame, TELEMETRY_CHASSIS_LEN);
        uart1_write(dist, TELEMETRY_DISTANCE_LEN);

        uart3_write(frame, TELEMETRY_CHASSIS_LEN);
        uart3_write(dist, TELEMETRY_DISTANCE_LEN);

        //CAN 分三帧发。ID 见 bsp/can.h,不能用 0x101/0x102,那是超声波板的
        can_send(CAN_ID_CHASSIS_1, frame);
        can_send(CAN_ID_CHASSIS_2, frame + 8);
        can_send(CAN_ID_CHASSIS_3, frame + 16);
    }
}
