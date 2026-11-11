// 速度限幅与平滑。纯计算,可 PC 测试。
#ifndef SPEED_SMOOTH_H
#define SPEED_SMOOTH_H

#include <stdint.h>

//每拍最多变化 step,到目标后锁住
float speed_smooth_step(float now, float target, float step);

//限幅,超出返回边界
float speed_limit(float v, float limit);

#endif
