// SSD1306 128x64。四线 SPI:PD7=SCL,PD6=SDA,PD5=RES,PD4=DC。
//
// 这块屏是 SPI 不是 I2C。软件翻转引脚,不加延时:GPIO 速度设成 2MHz,
// 靠引脚本身的翻转率限制速率,一直这么跑的。
#ifndef OLED_H
#define OLED_H

#include <stdint.h>

void oled_init(void);

void oled_clear(void);

//整屏刷新。只在初始化时用
void oled_flush(void);

//刷一行(2 页),hmi 任务每拍调一次,四行轮转
void oled_flush_row(void);

//x 是列(0~127),row 是行(0~3)
void oled_str(uint8_t x, uint8_t row, const char *s);

void oled_int(uint8_t x, uint8_t row, int32_t v);

//8 位十六进制,补零
void oled_hex(uint8_t x, uint8_t row, uint32_t v);

//value 已按 10^dec 放大过,dec 取 0~3
void oled_fixed(uint8_t x, uint8_t row, int32_t value, uint8_t dec);

#endif
