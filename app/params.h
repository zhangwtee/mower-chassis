// 掉电保存的参数。
//
// 存的是"用户调过的值":速度基准、PI、平滑步进、纠偏、避障。
// 校验用 CRC32:Flash 里读出来的东西不能信,静电、掉电、写一半都可能让它变样。
#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

#define PARAMS_MAGIC        0x4D575031u     //'MWP1'
#define PARAMS_VERSION      1u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t rc_speed_mm;
    uint16_t kp;
    uint16_t ki;
    uint16_t step_mm;
    uint16_t line_diff;
    uint16_t avoid_enable;
    uint16_t avoid_mm;
    uint16_t pad;
    uint32_t crc;
} params_t;

//纯函数,可 PC 测试
uint32_t params_crc(const params_t *p);

void params_defaults(params_t *p);

//magic、版本、CRC 都对上才算有效
uint8_t params_check(const params_t *p);

//从 Flash 读。返回 0 表示读到有效参数
uint8_t params_load(params_t *out);

//写 Flash(自动先填 magic/version/crc)。返回 0 成功
uint8_t params_save(params_t *p);

#endif
