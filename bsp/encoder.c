// ±àÂëÆ÷¡£TIM2 ¹Ü×óÂÖ¡¢TIM3 ¹ÜÓÒÂÖ,¶¼ÓÃ±àÂëÆ÷Ä£Ê½
#include "stm32f4xx.h"
#include "encoder.h"

static void enc_init_one(GPIO_TypeDef *g1, uint16_t p1, uint16_t src1,
                         GPIO_TypeDef *g2, uint16_t p2, uint16_t src2,
                         TIM_TypeDef *tim, uint32_t clk)
{
    GPIO_InitTypeDef g;
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef ic;

    RCC_APB1PeriphClockCmd(clk, ENABLE);

    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_OD;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;

    g.GPIO_Pin = p1;
    GPIO_Init(g1, &g);
    g.GPIO_Pin = p2;
    GPIO_Init(g2, &g);

    if (tim == TIM2) {
        GPIO_PinAFConfig(g1, src1, GPIO_AF_TIM2);
        GPIO_PinAFConfig(g2, src2, GPIO_AF_TIM2);
    } else {
        GPIO_PinAFConfig(g1, src1, GPIO_AF_TIM3);
        GPIO_PinAFConfig(g2, src2, GPIO_AF_TIM3);
    }

    tb.TIM_Prescaler = 0;
    tb.TIM_Period = 65535;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &tb);

    TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&ic);
    ic.TIM_ICFilter = 0;
    TIM_ICInit(tim, &ic);

    TIM_ClearFlag(tim, TIM_FLAG_Update);
    TIM_SetCounter(tim, 0);
    TIM_Cmd(tim, ENABLE);
}

void encoder_init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    //×óÂÖ:PA15 / PB3,TIM2
    enc_init_one(GPIOA, GPIO_Pin_15, GPIO_PinSource15,
                 GPIOB, GPIO_Pin_3, GPIO_PinSource3,
                 TIM2, RCC_APB1Periph_TIM2);

    //ÓÒÂÖ:PA6 / PA7,TIM3
    enc_init_one(GPIOA, GPIO_Pin_6, GPIO_PinSource6,
                 GPIOA, GPIO_Pin_7, GPIO_PinSource7,
                 TIM3, RCC_APB1Periph_TIM3);
}

static int16_t enc_take(TIM_TypeDef *tim)
{
    int16_t v = (int16_t)(tim->CNT & 0xFFFF);

    tim->CNT = 0;
    return v;
}

int16_t encoder_left(void)
{
    return (int16_t)(ENCODER_L_SIGN * enc_take(TIM2));
}

int16_t encoder_right(void)
{
    return (int16_t)(ENCODER_R_SIGN * enc_take(TIM3));
}
