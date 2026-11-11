// 用户按键。上拉输入,按下是低
#include "stm32f4xx.h"
#include "key.h"

#define PORT_KEY    GPIOC
#define PIN_KEY     GPIO_Pin_5

void key_init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    g.GPIO_Pin = PIN_KEY;
    g.GPIO_Mode = GPIO_Mode_IN;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PORT_KEY, &g);
}

uint8_t key_pressed(void)
{
    //上拉输入,按下接地
    return (uint8_t)(GPIO_ReadInputDataBit(PORT_KEY, PIN_KEY) == Bit_RESET);
}
