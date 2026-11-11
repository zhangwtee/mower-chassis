// 独立看门狗。默认没开,USE_IWDG 打开才有用
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "iwdg.h"

#if USE_IWDG

//LSI 约 32kHz。256 分频后每秒约 125 个计数,12 位计数器最多约 32.7 秒。
void iwdg_init(uint32_t timeout_ms)
{
    uint32_t reload;

    reload = timeout_ms * 125u / 1000u;
    if (reload < 1u) reload = 1u;
    if (reload > 4095u) reload = 4095u;

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload((uint16_t)reload);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void iwdg_feed(void)
{
    IWDG_ReloadCounter();
}

#else

void iwdg_init(uint32_t timeout_ms)
{
    (void)timeout_ms;
}

void iwdg_feed(void)
{
}

#endif
