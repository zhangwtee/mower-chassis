// 故障显示。
//
// 出异常(CoreFault/栈溢出/内存耗尽)时不能只是 for(;;) 挂着 —— 没有看门狗时
// 板子会静默死在原地,屏上还是最后那一帧,谁也看不出来出了什么事。
//
// 这里在异常上下文里直接把原因写到 OLED、点红灯、响蜂鸣,然后停住。
// 不用中断、不用调度器,只碰 GPIO 和 SPI 翻转,异常上下文里是安全的。
// 开了看门狗的话,停住 2 秒后会被复位,这一屏就是唯一的现场记录。
#ifndef FAULT_H
#define FAULT_H

//tag 最多 10 个字符,比如 "HARD" "MEM" "STK"
void fault_halt(const char *tag);

#endif
