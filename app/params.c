// 参数表和校验,不碰 Flash
//
// 读写 Flash 的放在 params_io.c,这样这个文件能直接上 PC 测。
#include "params.h"

//CRC32 的位运算实现。数据只有二十几个字节,不建表。
static uint32_t crc32_byte(uint32_t crc, uint8_t b)
{
    uint8_t i;

    crc ^= (uint32_t)b;
    for (i = 0; i < 8; i++) {
        if (crc & 1u) {
            crc = (crc >> 1) ^ 0xEDB88320u;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

uint32_t params_crc(const params_t *p)
{
    const uint8_t *raw = (const uint8_t *)p;
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;

    //只算 crc 字段之前的部分
    for (i = 0; i < (uint32_t)(sizeof(params_t) - sizeof(uint32_t)); i++) {
        crc = crc32_byte(crc, raw[i]);
    }

    return crc ^ 0xFFFFFFFFu;
}

void params_defaults(params_t *p)
{
    p->magic = PARAMS_MAGIC;
    p->version = PARAMS_VERSION;
    p->rc_speed_mm = 500;
    p->kp = 300;
    p->ki = 300;
    p->step_mm = 20;
    p->line_diff = 50;
    p->avoid_enable = 0;
    p->avoid_mm = 500;
    p->pad = 0;
    p->crc = params_crc(p);
}

uint8_t params_check(const params_t *p)
{
    if (p->magic != PARAMS_MAGIC) return 1;
    if (p->version != PARAMS_VERSION) return 2;
    if (p->crc != params_crc(p)) return 3;

    //CRC 对上了但值荒唐,一样当坏数据
    if (p->rc_speed_mm > 3500u) return 4;
    if (p->kp == 0u || p->ki == 0u) return 4;
    if (p->step_mm == 0u || p->step_mm > 200u) return 4;
    if (p->line_diff > 100u) return 4;
    if (p->avoid_mm < 100u || p->avoid_mm > 2000u) return 4;

    return 0;
}
