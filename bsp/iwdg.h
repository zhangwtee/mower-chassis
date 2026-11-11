// 独立看门狗。
//
// 喂狗点放在空闲任务钩子里,不是放在控制环里。
// 差别在于:控制环卡住的时候,空闲任务一定跑不到 —— 正好就是最需要复位的情况。
// 如果反过来喂在控制环里,控制环自己卡死了还在喂狗,看门狗就白装了。
//
// 默认关闭(USE_IWDG 0)。IWDG 一旦启动就停不下来,只有复位才能关;
// 超时值没验证过就打开,板子会反复重启而且救不回来。上机确认正常再开。
#ifndef IWDG_H
#define IWDG_H

#include <stdint.h>

#ifndef USE_IWDG
#define USE_IWDG        0
#endif

//超时时间毫秒。实际范围约 32ms ~ 32.7s(LSI 32kHz,256 分频)
void iwdg_init(uint32_t timeout_ms);

void iwdg_feed(void);

#endif
