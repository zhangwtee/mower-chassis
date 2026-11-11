// RGB 三路 PWM。板子上高电平亮
#include "stm32f4xx.h"
#include "led.h"

//R = PB15,G = PE6,B = PB14
#define PORT_R      GPIOB
#define PORT_G      GPIOE
#define PORT_B      GPIOB
#define PIN_R       GPIO_Pin_15
#define PIN_G       GPIO_Pin_6
#define PIN_B       GPIO_Pin_14

void led_init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;

    g.GPIO_Pin = PIN_R;
    GPIO_Init(PORT_R, &g);

    g.GPIO_Pin = PIN_G;
    GPIO_Init(PORT_G, &g);

    g.GPIO_Pin = PIN_B;
    GPIO_Init(PORT_B, &g);

    led_set(LED_OFF);
}

static void set_one(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
#if LED_ACTIVE_LOW
    if (on) {
        GPIO_ResetBits(port, pin);
    } else {
        GPIO_SetBits(port, pin);
    }
#else
    if (on) {
        GPIO_SetBits(port, pin);
    } else {
        GPIO_ResetBits(port, pin);
    }
#endif
}

void led_set(led_color_t c)
{
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    switch (c) {
    case LED_RED:    r = 1; break;
    case LED_GREEN:  g = 1; break;
    case LED_BLUE:   b = 1; break;
    case LED_YELLOW: r = 1; g = 1; break;
    case LED_CYAN:   g = 1; b = 1; break;
    case LED_PURPLE: r = 1; b = 1; break;
    case LED_WHITE:  r = 1; g = 1; b = 1; break;
    case LED_OFF:
    default: break;
    }

    set_one(PORT_R, PIN_R, r);
    set_one(PORT_G, PIN_G, g);
    set_one(PORT_B, PIN_B, b);
}
