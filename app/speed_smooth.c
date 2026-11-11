// 速度限幅和斜坡。一步跳到位会打滑
#include "speed_smooth.h"

float speed_smooth_step(float now, float target, float step)
{
    if (step < 0.0f) step = -step;

    if (now > target) {
        now -= step;
        if (now < target) now = target;
    } else if (now < target) {
        now += step;
        if (now > target) now = target;
    }

    return now;
}

float speed_limit(float v, float limit)
{
    if (limit < 0.0f) limit = -limit;

    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}
