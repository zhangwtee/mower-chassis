// 蜂鸣器。有源的,给高电平就响
#include "stm32f4xx.h"
#include "buzzer.h"

#define PORT_BUZZER     GPIOD
#define PIN_BUZZER      GPIO_Pin_11

void buzzer_init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    g.GPIO_Pin = PIN_BUZZER;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PORT_BUZZER, &g);

    buzzer_set(0);
}

void buzzer_set(uint8_t on)
{
#if BUZZER_ON_LEVEL
    if (on) {
        GPIO_SetBits(PORT_BUZZER, PIN_BUZZER);
    } else {
        GPIO_ResetBits(PORT_BUZZER, PIN_BUZZER);
    }
#else
    if (on) {
        GPIO_ResetBits(PORT_BUZZER, PIN_BUZZER);
    } else {
        GPIO_SetBits(PORT_BUZZER, PIN_BUZZER);
    }
#endif
}
