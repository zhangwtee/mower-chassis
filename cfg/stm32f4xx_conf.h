/*
 * 标准外设库裁剪配置
 *
 * stm32f4xx.h 在 USE_STDPERIPH_DRIVER 下会包含本文件。这里只开本工程实际用到的
 * 外设模块，多开一个就多一份编译时间和误用机会。
 *
 * 目前用到：GPIO / RCC / USART / TIM / ADC / CAN / PWR / DMA / SYSCFG / MISC
 * 尚未用到：FLASH（擦写后续直接操作寄存器）、I2C（软件模拟）、SPI、DAC、RTC、看门狗
 */
#ifndef STM32F4XX_CONF_H
#define STM32F4XX_CONF_H

#include "stm32f4xx_adc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

/* 断言：开发期打开，Release 前如需可改为空宏 */
//#define USE_FULL_ASSERT    1

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif

#endif /* STM32F4XX_CONF_H */
