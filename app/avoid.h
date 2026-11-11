// 超声波避障。纯逻辑,不碰硬件。
//
// 传感器编号按安装位置:A、B 一组,C、D 一组。哪边更近就往反方向转。
// 如果实车装反了,把 AVOID_AB_IS_RIGHT 改成 0。
#ifndef AVOID_H
#define AVOID_H

#include <stdint.h>

#define AVOID_SLOTS         6       //A~F
#define AVOID_CONFIRM       5       //连续几拍才认,滤掉抖动
#define AVOID_DEFAULT_M     0.50f
#define AVOID_MIN_M         0.10f
#define AVOID_MAX_M         2.00f

//A、B 在左侧为 1。A 是左方障碍物、D 是右方障碍物,
//所以默认 AB 在左。实车装反了改成 0。
#define AVOID_AB_IS_LEFT    1

typedef struct {
    uint8_t enable;
    float start_m;
    uint8_t count;          //总障碍连续计数
    uint8_t left_count;     //左侧单独计数
    uint8_t right_count;    //右侧单独计数
} avoid_t;

extern avoid_t g_avoid;

void avoid_init(avoid_t *a);

//dist 单位米,<=0 表示这一路没数据
//valid=0 表示超声波数据不新鲜,直接放行
void avoid_step(avoid_t *a, const float dist[AVOID_SLOTS], uint8_t valid,
                float *vx, float *vz);

void avoid_toggle(avoid_t *a);

void avoid_nearer(avoid_t *a, float delta);

#endif
