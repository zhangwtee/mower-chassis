// 看门狗监督。
//
// 光在空闲钩子里喂狗只能抓到"CPU 被霸占"和"死循环",抓不到"某一个任务卡死、
// 其他任务照常跑"——那种情况下空闲任务还在跑,狗被喂得饱饱的,板子却已经瘫了。
//
// 所以每个任务在循环里踢一下自己的位。看门狗只在"该踢的都踢过"之后才喂。
// 任何一个任务停了,位凑不齐,2 秒后复位。
//
// 位在每个喂狗窗口清一次:上电先让所有任务都跑过一轮,才允许第一次喂。
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

#define WD_BIT_CHASSIS      0x01u
#define WD_BIT_TELEMETRY    0x02u
#define WD_BIT_IMU          0x04u
#define WD_BIT_HMI          0x08u

#define WD_ALL              (WD_BIT_CHASSIS | WD_BIT_TELEMETRY | WD_BIT_IMU | WD_BIT_HMI)

//纯逻辑:该踢的都踢过就返回 1,并把位清掉;否则返回 0 且不动
//放头文件里是为了 PC 测试能直接编,不用拖 FreeRTOS 进来
static inline uint8_t wd_core(uint8_t *seen, uint8_t need)
{
    if ((*seen & need) != need) return 0;
    *seen = 0;
    return 1;
}

void wd_init(void);

//任务主循环里每圈调一次
void wd_kick(uint8_t bit);

//空闲钩子里调:该踢的齐了就喂狗并清位,不齐就不喂
void wd_poll(void);

#endif
