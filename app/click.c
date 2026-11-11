// µ¥»÷¡¢Ë«»÷¡¢³¤°´¡£´¿Âß¼­,²»Åö GPIO
#include "click.h"

#define ST_IDLE         0
#define ST_DOWN         1
#define ST_WAIT_SECOND  2
#define ST_SECOND       3
#define ST_WAIT_RELEASE 4

void click_init(clicker_t *c)
{
    c->state = ST_IDLE;
    c->t = 0;
}

click_event_t click_step(clicker_t *c, uint8_t pressed, uint16_t dt_ms)
{
    switch (c->state) {
    case ST_IDLE:
        if (pressed) {
            c->state = ST_DOWN;
            c->t = 0;
        }
        break;

    case ST_DOWN:
        c->t += dt_ms;
        if (!pressed) {
            //Ì«¶ÌµÄ°´Ñ¹µ±¶¶¶¯¶ªµô
            if (c->t < CLICK_MIN_MS) {
                c->state = ST_IDLE;
            } else {
                c->state = ST_WAIT_SECOND;
                c->t = 0;
            }
        } else if (c->t >= CLICK_LONG_MS) {
            c->state = ST_WAIT_RELEASE;
            return CLICK_LONG;
        }
        break;

    case ST_WAIT_SECOND:
        c->t += dt_ms;
        if (pressed) {
            c->state = ST_SECOND;
            c->t = 0;
        } else if (c->t > CLICK_DOUBLE_MS) {
            c->state = ST_IDLE;
            return CLICK_SINGLE;
        }
        break;

    case ST_SECOND:
        c->t += dt_ms;
        if (!pressed) {
            c->state = ST_IDLE;
            return CLICK_DOUBLE;
        }
        break;

    case ST_WAIT_RELEASE:
        if (!pressed) {
            c->state = ST_IDLE;
        }
        break;

    default:
        c->state = ST_IDLE;
        break;
    }

    return CLICK_NONE;
}
