// 遥测帧组包和 BCC,纯函数
#include "telemetry.h"

//大端,负数按补码落两字节
static void put16(uint8_t *out, int16_t v)
{
    out[0] = (uint8_t)((uint16_t)v >> 8);
    out[1] = (uint8_t)v;
}

uint8_t telemetry_bcc(const uint8_t *data, uint32_t n)
{
    uint8_t v = 0;

    while (n--) {
        v ^= *data++;
    }
    return v;
}

void telemetry_build_chassis(uint8_t *out, const telemetry_chassis_t *in)
{
    out[0] = TELEMETRY_CHASSIS_HEADER;
    out[1] = in->flag_stop;

    put16(out + 2,  in->vel_mmps[0]);
    put16(out + 4,  in->vel_mmps[1]);
    put16(out + 6,  in->vel_mmps[2]);

    put16(out + 8,  in->accel_raw[0]);
    put16(out + 10, in->accel_raw[1]);
    put16(out + 12, in->accel_raw[2]);

    put16(out + 14, in->gyro_raw[0]);
    put16(out + 16, in->gyro_raw[1]);
    put16(out + 18, in->gyro_raw[2]);

    put16(out + 20, (int16_t)in->voltage_mv);

    //BCC 算 22 字节,不是 24
    out[22] = telemetry_bcc(out, TELEMETRY_CHASSIS_BCC_LEN);
    out[23] = TELEMETRY_CHASSIS_TAIL;
}

void telemetry_build_distance(uint8_t *out, const int16_t *dist_mm)
{
    uint8_t i;

    out[0] = TELEMETRY_DISTANCE_HEADER;

    for (i = 0; i < TELEMETRY_DISTANCE_SLOTS; i++) {
        put16(out + 1 + i * 2, dist_mm[i]);
    }

    out[17] = telemetry_bcc(out, TELEMETRY_DISTANCE_BCC_LEN);
    out[18] = TELEMETRY_DISTANCE_TAIL;
}
