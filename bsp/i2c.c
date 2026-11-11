// 软件 I2C。速率靠延时凑,别在中断里调
#include "stm32f4xx.h"
#include "i2c.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t s_lock;

#define SCL_PIN      GPIO_Pin_9
#define SDA_PIN      GPIO_Pin_8
#define SDA_MODER    (3u << (8 * 2))

#define SCL_H()      GPIO_SetBits(GPIOB, SCL_PIN)
#define SCL_L()      GPIO_ResetBits(GPIOB, SCL_PIN)
#define SDA_H()      GPIO_SetBits(GPIOB, SDA_PIN)
#define SDA_L()      GPIO_ResetBits(GPIOB, SDA_PIN)
#define SDA_READ()   ((GPIOB->IDR & SDA_PIN) ? 1 : 0)

//SDA 输出(推挽) / 输入(高阻)
static void sda_out(void)
{
    GPIOB->MODER = (GPIOB->MODER & ~SDA_MODER) | (1u << (8 * 2));
}

static void sda_in(void)
{
    GPIOB->MODER &= ~SDA_MODER;
}

static void i2c_stop(void);

//SDA 被从机拉住不放时,补 9 个时钟让它吐出来
static void bus_recover(void)
{
    uint8_t i;

    sda_in();
    for (i = 0; i < 9; i++) {
        SCL_L();
        delay_us(2);
        SCL_H();
        delay_us(2);
    }
    i2c_stop();
}

void i2c_recover(void)
{
    bus_recover();
}

void i2c_init(void)
{
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_Init(GPIOB, &g);

    SCL_H();
    SDA_H();

    s_lock = xSemaphoreCreateMutex();
    bus_recover();
}

static void i2c_start(void)
{
    sda_out();
    SDA_H();
    SCL_H();
    delay_us(2);
    SDA_L();
    delay_us(2);
    SCL_L();
}

static void i2c_stop(void)
{
    sda_out();
    SDA_L();
    delay_us(2);
    SCL_H();
    delay_us(2);
    SDA_H();
    delay_us(2);
}

//写一个字节,返回从机 ack(0 表示有应答)
static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i;
    uint8_t ack;

    sda_out();
    for (i = 0; i < 8; i++) {
        if (b & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        b = (uint8_t)(b << 1);
        delay_us(1);
        SCL_H();
        delay_us(2);
        SCL_L();
    }

    sda_in();
    delay_us(1);
    SCL_H();
    delay_us(1);
    ack = (uint8_t)(SDA_READ() ? 1 : 0);
    SCL_L();
    sda_out();

    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t b = 0;

    sda_in();
    for (i = 0; i < 8; i++) {
        delay_us(1);
        SCL_H();
        delay_us(1);
        b = (uint8_t)((b << 1) | (SDA_READ() ? 1 : 0));
        SCL_L();
    }

    sda_out();
    if (ack) {
        SDA_L();
    } else {
        SDA_H();
    }
    delay_us(1);
    SCL_H();
    delay_us(2);
    SCL_L();

    return b;
}

void i2c_lock(void)
{
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

void i2c_unlock(void)
{
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}


uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t err = 0;

    i2c_start();
    if (i2c_write_byte((uint8_t)(addr & 0xFE))) err = 1;
    if (!err && i2c_write_byte(reg)) err = 2;
    if (!err && i2c_write_byte(val)) err = 3;
    i2c_stop();

    return err;
}

uint8_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n)
{
    uint16_t i;
    uint8_t err = 0;

    i2c_start();
    if (i2c_write_byte((uint8_t)(addr & 0xFE))) err = 1;
    if (!err && i2c_write_byte(reg)) err = 2;

    if (!err) {
        i2c_start();
        if (i2c_write_byte((uint8_t)(addr | 0x01))) err = 3;
    }

    if (!err) {
        for (i = 0; i < n; i++) {
            buf[i] = i2c_read_byte((uint8_t)(i + 1 < n));
        }
    }

    i2c_stop();
    return err;
}
