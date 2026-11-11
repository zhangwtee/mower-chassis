// 单击/双击/长按判定。纯逻辑,输入是"这一刻按键有没有按住",输出是一次性事件。
//
// 长按 500ms,双击窗口 50~300ms。
// 另外加了 30ms 最短按压滤机械抖动,没有这个的话抖一下就算单击。
#ifndef CLICK_H
#define CLICK_H

#include <stdint.h>

#define CLICK_MIN_MS        30
#define CLICK_LONG_MS       500
#define CLICK_DOUBLE_MS     300

typedef enum {
    CLICK_NONE = 0,
    CLICK_SINGLE,
    CLICK_DOUBLE,
    CLICK_LONG
} click_event_t;

typedef struct {
    uint8_t state;
    uint16_t t;
} clicker_t;

void click_init(clicker_t *c);

//每次事件只报一次,报完要等松手才会报下一次
click_event_t click_step(clicker_t *c, uint8_t pressed, uint16_t dt_ms);

#endif
