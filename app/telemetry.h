// 遥测帧组包。24 字节底盘帧、19 字节超声波帧。
// 只按位摆放,不做换算:速度进来是 mm/s,电压进来是 mV。
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#define TELEMETRY_CHASSIS_LEN       24
#define TELEMETRY_CHASSIS_BCC_LEN   22
#define TELEMETRY_CHASSIS_HEADER    0x7B
#define TELEMETRY_CHASSIS_TAIL      0x7D

#define TELEMETRY_DISTANCE_LEN      19
#define TELEMETRY_DISTANCE_BCC_LEN  17
#define TELEMETRY_DISTANCE_HEADER   0xFA
#define TELEMETRY_DISTANCE_TAIL     0xFC
#define TELEMETRY_DISTANCE_SLOTS    8

typedef struct {
    uint8_t  flag_stop;
    int16_t  vel_mmps[3];
    int16_t  accel_raw[3];
    int16_t  gyro_raw[3];
    uint16_t voltage_mv;
} telemetry_chassis_t;

uint8_t telemetry_bcc(const uint8_t *data, uint32_t n);

void telemetry_build_chassis(uint8_t *out, const telemetry_chassis_t *in);

void telemetry_build_distance(uint8_t *out, const int16_t *dist_mm);

#endif
