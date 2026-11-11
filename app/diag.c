// 诊断页那几个数,任务栈水位也在这儿
#include "FreeRTOS.h"
#include "task.h"

#include "diag.h"

//每 100 圈采一次水位。采太勤没意义,这个量变化很慢
#define KICK_DIV    100

static uint16_t s_stack[DIAG_TASKS];
static uint8_t s_count[DIAG_TASKS];
static uint32_t s_uptime;
static uint8_t s_page;

void diag_kick(uint8_t idx)
{
    if (idx >= DIAG_TASKS) return;

    s_count[idx]++;
    if (s_count[idx] < KICK_DIV) return;

    s_count[idx] = 0;
    s_stack[idx] = (uint16_t)uxTaskGetStackHighWaterMark(NULL);
}

uint16_t diag_stack(uint8_t idx)
{
    if (idx >= DIAG_TASKS) return 0;
    return s_stack[idx];
}

void diag_tick_1s(void)
{
    s_uptime++;
}

uint32_t diag_uptime_s(void)
{
    return s_uptime;
}

void diag_page_next(void)
{
    s_page = diag_page_next_of(s_page);
}

uint8_t diag_page(void)
{
    return s_page;
}
