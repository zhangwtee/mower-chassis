// 增量式 PI。输出限幅由调用方给，不清积分（增量式本身不会饱和）。
#ifndef PI_H
#define PI_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float bias;
    float last_bias;
    float out;
} pi_t;

void pi_init(pi_t *p, float kp, float ki);

void pi_reset(pi_t *p);

//在线改增益,不改动积分状态
void pi_set_gains(pi_t *p, float kp, float ki);

float pi_update(pi_t *p, float feedback, float target, float limit);

//驻车清残余，每次减 1，清零返回 1
int pi_clear_output(pi_t *p);

#endif
