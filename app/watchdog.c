// 喂狗条件。几个任务都报到齐了才喂
#include "FreeRTOS.h"
#include "task.h"

#include "watchdog.h"
#include "iwdg.h"

static volatile uint8_t s_seen;
static uint8_t s_need;
static volatile uint8_t s_ready;


void wd_init(void)
{
    s_seen = 0;
    s_need = WD_ALL;
    s_ready = 0;
}

void wd_kick(uint8_t bit)
{
    s_seen |= bit;
}


void wd_poll(void)
{
    uint8_t ok;
    uint8_t seen;

    taskENTER_CRITICAL();

    seen = s_seen;
    ok = wd_core(&seen, s_need);
    if (ok) {
        s_seen = seen;
    }
    s_ready = ok;

    taskEXIT_CRITICAL();

    if (ok) {
        iwdg_feed();
    }
}
