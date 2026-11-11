// 参数的 Flash 读写。擦一页 250ms,调用方自己挑停车的时候
#include "params.h"
#include "flash.h"

//参数区在 sector 3,地址是 0x0800C000。这里直接按结构体拷,
//所以 params_t 必须是 4 字节对齐的(编译器会保证)。
uint8_t params_load(params_t *out)
{
    const uint8_t *src = (const uint8_t *)FLASH_PARAM_ADDR;
    uint8_t *dst = (uint8_t *)out;
    uint32_t i;

    for (i = 0; i < sizeof(params_t); i++) {
        dst[i] = src[i];
    }

    return params_check(out);
}

uint8_t params_save(params_t *p)
{
    p->magic = PARAMS_MAGIC;
    p->version = PARAMS_VERSION;
    p->pad = 0;
    p->crc = params_crc(p);

    return flash_param_write(p, (uint32_t)sizeof(params_t));
}
