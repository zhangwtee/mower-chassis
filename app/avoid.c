// 避障。A/B 算左边、C/D 算右边,装反了过去看 AVOID_AB_IS_LEFT
#include "avoid.h"

#define TURN_BASE       0.78539816f     //PI/4
#define TURN_REF_VX     0.5f            //转向量按这个速度做基准
#define FORWARD_MIN     0.05f
#define INVALID_M       0.01f

//按 AB 的左右归属找"最左"和"最右"。左计数盯 A、右计数盯 D
#if AVOID_AB_IS_LEFT
#define SLOT_LEFT       0
#define SLOT_RIGHT      3
#else
#define SLOT_LEFT       3
#define SLOT_RIGHT      0
#endif

avoid_t g_avoid;

void avoid_init(avoid_t *a)
{
    a->enable = 0;
    a->start_m = AVOID_DEFAULT_M;
    a->count = 0;
    a->left_count = 0;
    a->right_count = 0;
}

void avoid_toggle(avoid_t *a)
{
    a->enable = a->enable ? 0u : 1u;
    a->count = 0;
}

void avoid_nearer(avoid_t *a, float delta)
{
    a->start_m += delta;

    if (a->start_m < AVOID_MIN_M) a->start_m = AVOID_MIN_M;
    if (a->start_m > AVOID_MAX_M) a->start_m = AVOID_MAX_M;
}


void avoid_step(avoid_t *a, const float dist[AVOID_SLOTS], uint8_t valid,
                float *vx, float *vz)
{
    uint8_t i;
    uint8_t any = 0;
    uint8_t blocked = 0;
    float group_ab;
    float group_cd;
    float turn;

    if (!a->enable || !valid) {
        a->count = 0;
        a->left_count = 0;
        a->right_count = 0;
        return;
    }

    //0 表示这一路没测到,不参与判断
    for (i = 0; i < AVOID_SLOTS; i++) {
        if (dist[i] > INVALID_M) {
            any = 1;
            if (dist[i] < a->start_m) {
                blocked = 1;
            }
        }
    }

    if (!any || !blocked) {
        a->count = 0;
        a->left_count = 0;
        a->right_count = 0;
        return;
    }

    //左右各自连续计数,用来判断"要转过去的那一侧是不是也堵着"
    if (dist[SLOT_LEFT] > INVALID_M && dist[SLOT_LEFT] < a->start_m) {
        if (a->left_count < 0xFFu) a->left_count++;
    } else {
        a->left_count = 0;
    }
    if (dist[SLOT_RIGHT] > INVALID_M && dist[SLOT_RIGHT] < a->start_m) {
        if (a->right_count < 0xFFu) a->right_count++;
    } else {
        a->right_count = 0;
    }

    if (a->count < AVOID_CONFIRM) {
        a->count++;
        //累加到门限的这一拍就动作,不是再多等一拍
        if (a->count < AVOID_CONFIRM) return;
    }

    //只处理"在往前走"的情况,后退的时候不去抢方向
    if (*vx <= FORWARD_MIN) {
        return;
    }

    group_ab = dist[0] + dist[1];
    group_cd = dist[2] + dist[3];

#if AVOID_AB_IS_LEFT
    //AB 在左侧:左侧更近就往右转(vz 负)
    if (group_ab < group_cd) {
        *vz = -TURN_BASE;
    } else {
        *vz = TURN_BASE;
    }
#else
    if (group_ab < group_cd) {
        *vz = TURN_BASE;
    } else {
        *vz = -TURN_BASE;
    }
#endif

    //要转过去的那一侧也堵着,就别转了,原地停住比拐进墙里好
    if (*vz > 0.0f && a->left_count >= AVOID_CONFIRM) {
        *vz = 0.0f;
        *vx = 0.0f;
        return;
    }
    if (*vz < 0.0f && a->right_count >= AVOID_CONFIRM) {
        *vz = 0.0f;
        *vx = 0.0f;
        return;
    }

    //转向量按当前前进速度缩放
    turn = (*vx / TURN_REF_VX) * TURN_BASE;
    if (turn > TURN_BASE) turn = TURN_BASE;
    *vz = (*vz > 0.0f) ? turn : -turn;
    *vx = 0.0f;
}
