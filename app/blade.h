// 刀片继电器。
//
// 两套命令语义:
//   上位机 CMD=6:会持续刷新,所以按超时判定,不刷新就关
//   APP 按键 W/V/P:一次性字节,没法刷新,所以按开关式,一直保持到收到关命令
// 两条路都受 allowed 约束,失能/急停/欠压时一律强制关,且关掉之后再恢复使能也不会自己开。
#ifndef BLADE_H
#define BLADE_H

#include <stdint.h>

typedef struct {
    uint16_t age;
    uint8_t want;
    uint8_t sticky;
    uint8_t on;
} blade_t;

void blade_init(blade_t *b);

//cmd_valid 本拍收到命令,cmd_on 命令内容,sticky 1 表示开关式(不超时),allowed 当前是否允许
void blade_step(blade_t *b, uint16_t dt_ms, uint8_t cmd_valid, uint8_t cmd_on,
                uint8_t sticky, uint8_t allowed);

uint8_t blade_on(const blade_t *b);

#endif
