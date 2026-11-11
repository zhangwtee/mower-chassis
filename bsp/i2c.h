// 软件 I2C。SCL=PB9,SDA=PB8,主设备。
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(void);

//同一时刻只能有一个任务在总线上。整段事务前后必须成对调用
void i2c_lock(void);
void i2c_unlock(void);

//总线卡住时手动恢复(补 9 个时钟). 调用方要自己持锁
void i2c_recover(void);

//addr 是 8 位地址(含读写位). 返回 0 成功
uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);

//连续写 n 个字节(同一个寄存器/控制字节后面跟一长串数据). 返回 0 成功

//连续读 n 个字节. 返回 0 成功
uint8_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n);

#endif
