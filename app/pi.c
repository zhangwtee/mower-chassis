// 增量式 PI,带输出限幅和积分限幅
#include "pi.h"

void pi_init(pi_t *p, float kp, float ki)
{
    p->kp = kp;
    p->ki = ki;
    pi_reset(p);
}

void pi_reset(pi_t *p)
{
    p->bias = 0.0f;
    p->last_bias = 0.0f;
    p->out = 0.0f;
}

void pi_set_gains(pi_t *p, float kp, float ki)
{
    p->kp = kp;
    p->ki = ki;
}

float pi_update(pi_t *p, float feedback, float target, float limit)
{
    p->bias = target - feedback;

    p->out += p->kp * (p->bias - p->last_bias) + p->ki * p->bias;

    if (p->out > limit) {
        p->out = limit;
    }
    if (p->out < -limit) {
        p->out = -limit;
    }

    p->last_bias = p->bias;
    return p->out;
}

int pi_clear_output(pi_t *p)
{
    if (p->out > 0.0f) {
        p->out -= 1.0f;
    }
    if (p->out < 0.0f) {
        p->out += 1.0f;
    }

    if (p->out < 2.0f && p->out > -2.0f) {
        p->out = 0.0f;
        return 1;
    }
    return 0;
}
