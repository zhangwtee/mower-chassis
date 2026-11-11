// SSD1306,128x64,四线 SPI。不是 I2C
#include <string.h>

#include "stm32f4xx.h"
#include "oled.h"
#include "oledfont.h"
#include "delay.h"

#define PORT_OLED       GPIOD
#define PIN_SCL         GPIO_Pin_7
#define PIN_SDA         GPIO_Pin_6
#define PIN_RES         GPIO_Pin_5
#define PIN_DC          GPIO_Pin_4

#define CTRL_CMD        0
#define CTRL_DATA       1

#define ROWS            4
#define COLS            128

static uint8_t s_gram[8][COLS];
static uint8_t s_row;

//一字节。cmd=0 是命令,1 是数据,高位在前
static void wr(uint8_t dat, uint8_t cmd)
{
    uint8_t i;

    if (cmd) {
        GPIO_SetBits(PORT_OLED, PIN_DC);
    } else {
        GPIO_ResetBits(PORT_OLED, PIN_DC);
    }

    for (i = 0; i < 8; i++) {
        GPIO_ResetBits(PORT_OLED, PIN_SCL);
        if (dat & 0x80u) {
            GPIO_SetBits(PORT_OLED, PIN_SDA);
        } else {
            GPIO_ResetBits(PORT_OLED, PIN_SDA);
        }
        GPIO_SetBits(PORT_OLED, PIN_SCL);
        dat = (uint8_t)(dat << 1);
    }

    GPIO_SetBits(PORT_OLED, PIN_DC);
}

static void send_page(uint8_t p)
{
    uint8_t n;

    wr((uint8_t)(0xB0 + p), CTRL_CMD);
    wr(0x00, CTRL_CMD);
    wr(0x10, CTRL_CMD);

    for (n = 0; n < COLS; n++) {
        wr(s_gram[p][n], CTRL_DATA);
    }
}

void oled_init(void)
{
    //初始化序列和常见那份 SSD1306 有几处不同(COM 方向、对比度、VCOMH),
    //照抄是为了屏幕方向和亮度都跟原来一致
    static const uint8_t seq[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x02,
        0xA1,
        0xC0,
        0xDA, 0x12,
        0x81, 0xEF,
        0xD9, 0xF1,
        0xDB, 0x30,
        0xA4,
        0xA6,
        0xAF
    };
    GPIO_InitTypeDef g;
    uint8_t i;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    g.GPIO_Pin = PIN_SCL | PIN_SDA | PIN_RES | PIN_DC;
    GPIO_Init(PORT_OLED, &g);

    GPIO_ResetBits(PORT_OLED, PIN_SCL);
    GPIO_SetBits(PORT_OLED, PIN_DC);

    //复位低电平保持一会儿再放开
    GPIO_ResetBits(PORT_OLED, PIN_RES);
    delay_ms(100);
    GPIO_SetBits(PORT_OLED, PIN_RES);

    for (i = 0; i < sizeof(seq); i++) {
        wr(seq[i], CTRL_CMD);
    }

    oled_clear();
    oled_flush();
}

void oled_clear(void)
{
    memset(s_gram, 0, sizeof(s_gram));
}

void oled_flush(void)
{
    uint8_t p;

    for (p = 0; p < 8; p++) {
        send_page(p);
    }
}

void oled_flush_row(void)
{
    uint8_t p = (uint8_t)(s_row * 2);

    send_page(p);
    send_page((uint8_t)(p + 1));

    s_row = (uint8_t)((s_row + 1) & 3);
}

static void draw_char(uint8_t x, uint8_t row, char c)
{
    uint8_t idx;
    uint8_t i;

    if (c < 0x20 || c > 0x7E) c = '?';
    if (x + 6 > COLS || row >= ROWS) return;

    idx = (uint8_t)(c - 0x20);

    //字库前 6 字节是上半,后 6 字节是下半
    for (i = 0; i < 6; i++) {
        s_gram[row * 2][x + i] = oled_font6x12[idx][i];
        s_gram[row * 2 + 1][x + i] = oled_font6x12[idx][i + 6];
    }
}

void oled_str(uint8_t x, uint8_t row, const char *s)
{
    //越界就停,不能让 x 一直加下去绕回左边
    while (*s && x + 6 <= COLS) {
        draw_char(x, row, *s);
        x = (uint8_t)(x + 6);
        s++;
    }
}

static uint8_t fmt_int(int32_t v, char *buf)
{
    char tmp[12];
    uint8_t n = 0;
    uint8_t i = 0;
    uint8_t neg = 0;
    uint32_t u;

    if (v < 0) {
        neg = 1;
        u = (uint32_t)(-v);
    } else {
        u = (uint32_t)v;
    }

    do {
        tmp[n++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while (u && n < sizeof(tmp));

    if (neg) buf[i++] = '-';
    while (n) {
        buf[i++] = tmp[--n];
    }
    buf[i] = 0;

    return i;
}

void oled_hex(uint8_t x, uint8_t row, uint32_t v)
{
    char b[10];
    uint8_t i;
    uint8_t n;

    for (i = 0; i < 8; i++) {
        n = (uint8_t)((v >> (28 - i * 4)) & 0x0Fu);
        b[i] = (char)((n < 10u) ? ('0' + n) : ('A' + n - 10u));
    }
    b[8] = 0;

    oled_str(x, row, b);
}

void oled_int(uint8_t x, uint8_t row, int32_t v)
{
    char b[14];

    fmt_int(v, b);
    oled_str(x, row, b);
}

void oled_fixed(uint8_t x, uint8_t row, int32_t value, uint8_t dec)
{
    char b[16];
    int32_t div = 1;
    int32_t ip;
    int32_t fp;
    uint8_t n;
    uint8_t k;

    if (dec > 3) dec = 3;

    for (k = 0; k < dec; k++) {
        div *= 10;
    }

    ip = value / div;
    fp = value % div;
    if (fp < 0) fp = -fp;

    n = fmt_int(ip, b);

    //绝对值小于 1 时整数部分是 0,fmt_int 不带负号,要补一个
    if (value < 0 && ip == 0) {
        b[0] = '-';
        b[1] = '0';
        n = 2;
    }

    if (dec) {
        b[n++] = '.';
        for (k = dec; k > 0; k--) {
            b[n + k - 1] = (char)('0' + (fp % 10));
            fp /= 10;
        }
        n = (uint8_t)(n + dec);
    }
    b[n] = 0;

    oled_str(x, row, b);
}
