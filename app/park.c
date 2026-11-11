// 驻车。停住以后把保持电流放掉,斜坡上别放
#include "park.h"

void park_init(park_t *p)
{
    p->released = 0;
    p->still_ms = 0;
}

uint8_t park_released(const park_t *p)
{
    return p->released;
}

uint8_t park_is_flat(int16_t ax, int16_t ay, int16_t az)
{
    float x = (float)ax / PARK_G_LSB;
    float y = (float)ay / PARK_G_LSB;
    float z = (float)az / PARK_G_LSB;

    if (x < 0.0f) x = -x;
    if (y < 0.0f) y = -y;

    if (x > PARK_LEVEL_G) return 0;
    if (y > PARK_LEVEL_G) return 0;
    if (z < PARK_UP_G) return 0;

    return 1;
}

void park_step(park_t *p, uint8_t imu_ok, uint8_t flat, float target_mps,
               float measured_mps, uint16_t dt_ms)
{
    float speed = measured_mps;

    if (speed < 0.0f) speed = -speed;

    //只要不是"没认到 IMU / 不在平地 / 有目标速度 / 轮子还在动",就继续等
    if (!imu_ok || !flat || target_mps != 0.0f || speed > PARK_STOP_MPS) {
        p->released = 0;
        p->still_ms = 0;
        return;
    }

    if (p->still_ms < PARK_STILL_MS) {
        p->still_ms = (uint16_t)(p->still_ms + dt_ms);
        if (p->still_ms < PARK_STILL_MS) {
            return;
        }
    }

    p->released = 1;
}
