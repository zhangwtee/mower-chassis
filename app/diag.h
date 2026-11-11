// 诊断信息。栈水位和运行时长,给上机时看。
//
// 栈深度没法静态算准,只能跑了之后读"历史最低剩余量"。这个数比任何估算都准,
// 所以上机时一定要看一眼:任何一个任务剩不到 1/4,就该把它的栈加大。
#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>

#define DIAG_CHASSIS    0
#define DIAG_TELEMETRY  1
#define DIAG_IMU        2
#define DIAG_HMI        3
#define DIAG_TASKS      4

#define DIAG_PAGES      3

//任务里每圈调一次,内部自己限速
void diag_kick(uint8_t idx);

//栈历史最低剩余量,单位是"字"(4 字节)
uint16_t diag_stack(uint8_t idx);

//上电后经过的秒数
uint32_t diag_uptime_s(void);
void diag_tick_1s(void);

//纯逻辑:下一页。放头文件里是为了 PC 测试能直接编
static inline uint8_t diag_page_next_of(uint8_t page)
{
    page++;
    if (page >= DIAG_PAGES) page = 0;
    return page;
}

void diag_page_next(void);
uint8_t diag_page(void);

#endif
