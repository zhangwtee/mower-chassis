// 参数区。sector 3,擦之前想清楚
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "flash.h"


uint8_t flash_param_write(const void *buf, uint32_t len)
{
    const uint32_t *src = (const uint32_t *)buf;
    uint32_t words;
    uint32_t i;
    uint8_t err = 0;

    if (len == 0u || len > FLASH_PARAM_SIZE) {
        return 1;
    }

    words = (len + 3u) / 4u;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    __disable_irq();

    if (FLASH_EraseSector(FLASH_Sector_3, VoltageRange_3) != FLASH_COMPLETE) {
        err = 2;
    }

    for (i = 0; !err && i < words; i++) {
        if (FLASH_ProgramWord(FLASH_PARAM_ADDR + i * 4u, src[i]) != FLASH_COMPLETE) {
            err = 3;
        }
    }

    __enable_irq();

    FLASH_Lock();

    return err;
}
