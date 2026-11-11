// 微秒延时,拿 DWT 计数做的
//
// 原来用 NOP 空循环,数量按 168MHz 估,实测偏快好几倍,导致软件 I2C 超过
// 400kHz 上限。改用 DWT 周期计数器,和 SysTick 方案一样准,
// 且不随优化等级变化。
#include "stm32f4xx.h"
#include "delay.h"

void delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);

    while ((DWT->CYCCNT - start) < ticks) {
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}
