// 刀片。刷新式控制,300ms 没新指令自己停
#include "blade.h"
#include "chassis_safety.h"

void blade_init(blade_t *b)
{
    b->age = 0;
    b->want = 0;
    b->sticky = 0;
    b->on = 0;
}

void blade_step(blade_t *b, uint16_t dt_ms, uint8_t cmd_valid, uint8_t cmd_on,
                uint8_t sticky, uint8_t allowed)
{
    if (cmd_valid) {
        b->want = cmd_on ? 1 : 0;
        b->sticky = sticky ? 1 : 0;
        b->age = 0;
    } else if (b->age < 0xFFFF) {
        b->age += dt_ms;
    }

    //不允许就立刻关,并把意愿清掉:恢复允许之后不会自己再开
    if (!allowed) {
        b->want = 0;
    }

    if (allowed && b->want && (b->sticky || b->age <= SAFETY_BLADE_AGE_MAX)) {
        b->on = 1;
    } else {
        b->on = 0;
    }
}

uint8_t blade_on(const blade_t *b)
{
    return b->on;
}
