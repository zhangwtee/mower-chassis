// 电机。两路 PWM + 四个方向脚
#include "stm32f4xx.h"
#include "motor.h"

//右轮方向引脚和左轮是反的,车体两侧电机镜像安装
static void dir_pin_init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;

    //AIN1 PB13,AIN2 PB12
    g.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12;
    GPIO_Init(GPIOB, &g);

    //BIN1 PC0
    g.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOC, &g);

    //BIN2 PE15
    g.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOE, &g);

    GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_12);
    GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    GPIO_ResetBits(GPIOE, GPIO_Pin_15);
}

static void pwm_init(void)
{
    GPIO_InitTypeDef g;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    //PC8 = TIM8_CH3,PC9 = TIM8_CH4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOC, &g);

    //168M / (16799+1) = 10kHz
    tb.TIM_Period = 16799;
    tb.TIM_Prescaler = 0;
    tb.TIM_ClockDivision = 1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &tb);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 0;

    TIM_OC3Init(TIM8, &oc);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM8, &oc);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

    //高级定时器必须开主输出
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

void motor_init(void)
{
    dir_pin_init();
    pwm_init();
}

void motor_set(int16_t a, int16_t b)
{
    int16_t da = (int16_t)(a * MOTOR_A_SIGN);
    int16_t db = (int16_t)(b * MOTOR_B_SIGN);

    //正数 = 前进。
    //PI 输出在这里取负,折进来之后等价于:
    //  前进时 A 路 AIN1=1/AIN2=0,B 路 BIN1=0/BIN2=1(两个电机镜像装)
    if (da > 0) {
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        da = (int16_t)(-da);
    }

    if (db > 0) {
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);
        GPIO_SetBits(GPIOE, GPIO_Pin_15);
    } else {
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
        GPIO_ResetBits(GPIOE, GPIO_Pin_15);
        db = (int16_t)(-db);
    }

    if (da > MOTOR_FULL_DUTY) da = MOTOR_FULL_DUTY;
    if (db > MOTOR_FULL_DUTY) db = MOTOR_FULL_DUTY;

    TIM_SetCompare4(TIM8, (uint16_t)da);
    TIM_SetCompare3(TIM8, (uint16_t)db);
}
