// 滑动平均。窗口 32,够压住电压的抖动
#include "filter.h"

void meanf_init(meanf_t *f, float first)
{
    uint8_t i;

    for (i = 0; i < MEANF_LEN; i++) {
        f->buf[i] = first;
    }
    f->idx = 0;
    f->sum = first * (float)MEANF_LEN;
    f->n = MEANF_LEN;
}

float meanf_push(meanf_t *f, float v)
{
    //窗口还没填满:先往里装,平均值按已填个数算。
    //这样即使调用方忘了 init,也不会出现"前 32 拍和 0 求平均"的爬坡
    if (f->n < MEANF_LEN) {
        f->buf[f->n] = v;
        f->sum += v;
        f->n++;
        if (f->n == MEANF_LEN) {
            f->idx = 0;
        }
        return f->sum / (float)f->n;
    }

    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = v;
    f->sum += v;
    f->idx = (uint8_t)((f->idx + 1) % MEANF_LEN);

    return f->sum / (float)MEANF_LEN;
}

float meanf_value(const meanf_t *f)
{
    if (f->n == 0u) {
        return 0.0f;
    }
    return f->sum / (float)f->n;
}
