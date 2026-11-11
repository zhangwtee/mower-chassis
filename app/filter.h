// 滑动平均。用来压电池电压的抖动。
#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

#define MEANF_LEN       32

typedef struct {
    float buf[MEANF_LEN];
    uint8_t idx;
    float sum;
    uint8_t n;      //已经填了几个。没调 init 时从 0 开始自己长
} meanf_t;

//把窗口填成同一个值。不调也能用:push 会自己从 1 个开始长到满
void meanf_init(meanf_t *f, float first);

float meanf_push(meanf_t *f, float v);

float meanf_value(const meanf_t *f);

#endif
