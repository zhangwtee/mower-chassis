// 串口协议链路。两个串口各自一个解析器,解析结果写成事件给控制环。
#ifndef LINK_H
#define LINK_H

#include <stdint.h>

typedef struct {
    uint8_t estop_req;
    uint8_t estop_release_req;
    uint8_t selftest_req;

    uint8_t cmd_new;
    float cmd_vx;
    float cmd_vz;

    uint8_t blade_valid;
    uint8_t blade_cmd;
    uint8_t blade_sticky;

    uint8_t servo_valid;
    uint16_t servo_us;
} link_events_t;

void link_init(void);

uint16_t link_bad_frames(void);

uint8_t link_dist_valid(void);
void link_poll_tick(void);

//任务里调,把两个串口收到的字节解析掉
void link_poll(void);

//一次性标志处理完清掉
void link_clear_once(void);

const link_events_t *link_events(void);

//刀片命令。sticky=1 用开关式(APP 按键),sticky=0 用刷新式(上位机 CMD=6)
void link_blade_cmd(uint8_t on, uint8_t sticky);

void link_selftest_req(void);

#endif
