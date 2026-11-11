// 驻车释放。
//
// 驻车想做的是"平地静止一段时间后把 PI 输出清掉,省电、消电流声",
// 但单位写错(diff>8.8 而 diff 范围只有 ±1.4),整段没生效过。
// 这里按它的意图重做,另外把两件它没考虑的事补上:
//   1. 必须在平地才释放。斜坡上放开等于溜车。
//   2. IMU 没认到、或者数据不新鲜时一律不释放。宁可有电流声,也不要溜车。
#ifndef PARK_H
#define PARK_H

#include <stdint.h>

#define PARK_STILL_MS       5000    //静止这么久才考虑
#define PARK_LEVEL_G        0.10f   //|ax|、|ay| 都小于 0.1g 算平,约 5.7 度
#define PARK_UP_G           0.90f   //az 要大于 0.9g,否则可能是倒扣着
#define PARK_STOP_MPS       0.02f   //轮速小于这个算停住
#define PARK_G_LSB          1671.84f//±2g 量程下 1g 对应的原始值

typedef struct {
    uint8_t released;
    uint16_t still_ms;
} park_t;

void park_init(park_t *p);

//imu_ok=0 时不释放
void park_step(park_t *p, uint8_t imu_ok, uint8_t flat, float target_mps,
               float measured_mps, uint16_t dt_ms);

uint8_t park_released(const park_t *p);

//姿态是否水平。纯函数,可测
uint8_t park_is_flat(int16_t ax, int16_t ay, int16_t az);

#endif
