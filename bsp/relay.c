// µ¶Æ¬¼ÌµçÆ÷Òý½Å¡£Âß¼­ÔÚ blade.c
#include "stm32f4xx.h"
#include "relay.h"

void relay_init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd = GPIO_PuPd_DOWN;
    g.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOD, &g);

    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

void relay_set(uint8_t on)
{
    if (on) {
        GPIO_SetBits(GPIOD, GPIO_Pin_14);
    } else {
        GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    }
}
