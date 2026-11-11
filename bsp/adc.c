// ADC。电池电压和使能开关都在这条路上
#include "stm32f4xx.h"
#include "adc.h"

static void adc_pin_init(void)
{
    GPIO_InitTypeDef g;

    //PB0 = ADC1_IN8,电池分压
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    g.GPIO_Mode = GPIO_Mode_AN;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    g.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &g);

    //PE4 = 使能开关,上拉输入
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    g.GPIO_Mode = GPIO_Mode_IN;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOE, &g);
}

void adc_init(void)
{
    ADC_CommonInitTypeDef common;
    ADC_InitTypeDef a;

    adc_pin_init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    common.ADC_Mode = ADC_Mode_Independent;
    common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    common.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&common);

    a.ADC_Resolution = ADC_Resolution_12b;
    a.ADC_ScanConvMode = DISABLE;
    a.ADC_ContinuousConvMode = DISABLE;
    a.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    a.ADC_DataAlign = ADC_DataAlign_Right;
    a.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &a);

    ADC_Cmd(ADC1, ENABLE);
}

static uint16_t adc_read(uint8_t ch)
{
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles);
    ADC_SoftwareStartConv(ADC1);

    uint32_t guard = 100000;

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        if (--guard == 0) return 0;     //ADC 挂了也不能把 100Hz 主环卡死
    }

    return ADC_GetConversionValue(ADC1);
}

float adc_battery_volt(void)
{
    //分压比 11:1,参考 3.3V,12 位
    return (float)adc_read(ADC_Channel_8) * 3.3f * 11.0f / 4096.0f;
}

uint8_t enable_switch_ok(void)
{
    //上拉输入,开关按下拉低。低 = 失能
    return (uint8_t)(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) != Bit_RESET);
}
