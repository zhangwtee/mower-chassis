// 摇杆命令的超时。APP 不发心跳,只能这么兜
#include "joy_guard.h"

void joy_guard_init(joy_guard_t *j)
{
    j->last_seq = 0;
    j->age = 0;
    j->active = 0;
}

uint16_t joy_guard_age(const joy_guard_t *j)
{
    return j->age;
}

uint8_t joy_guard_step(joy_guard_t *j, uint32_t seq, uint8_t dir, uint16_t dt_ms)
{
    if (dir == 0u) {
        j->last_seq = seq;
        j->age = 0;
        j->active = 0;
        return 0;
    }

    if (seq != j->last_seq) {
        //有新输入,人还按着
        j->last_seq = seq;
        j->age = 0;
        j->active = 1;
        return 1;
    }

    if (!j->active) {
        return 0;
    }

    if (JOY_TIMEOUT_MS > 0) {
        if (j->age < 0xFFFFu) {
            j->age = (uint16_t)(j->age + dt_ms);
        }
        if (j->age > JOY_TIMEOUT_MS) {
            //太久没有新输入,当成链路已经断了
            j->active = 0;
            return 0;
        }
    }

    return 1;
}
