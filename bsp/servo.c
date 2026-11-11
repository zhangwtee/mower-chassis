// 舵机。50Hz PWM,脉宽 500~2500us
#include "stm32f4xx.h"
#include "servo.h"

void servo_init(void)
{
    GPIO_InitTypeDef g;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);

    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &g);

    //1MHz 计数,20000 一周期 = 50Hz
    tb.TIM_Period = 19999;
    tb.TIM_Prescaler = 167;
    tb.TIM_ClockDivision = 1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM9, &tb);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 1500;
    TIM_OC1Init(TIM9, &oc);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM9, ENABLE);
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
}

void servo_set(uint16_t us)
{
    if (us < 500) us = 500;
    if (us > 2500) us = 2500;

    TIM_SetCompare1(TIM9, us);
}
