// 灯和蜂鸣器,按状态换闪法
#include "indicator.h"
#include "buzzer.h"
#include "chassis.h"

//闪烁相位每 3 拍翻一次,10Hz 下约 1.7Hz
#define BLINK_DIV       3
//故障时每 3 秒再响一次,提醒人还没清故障
#define NAG_TICKS       30

static uint16_t s_tick;
static uint8_t s_prev;
static uint16_t s_nag;

static uint8_t s_beep_left;
static uint8_t s_beep_on;
static uint8_t s_beep_t;


static void beep_start(uint8_t n)
{
    s_beep_left = n;
    s_beep_on = 0;
    s_beep_t = 0;
}

static void beep_tick(void)
{
    if (s_beep_left == 0u) {
        buzzer_set(0);
        return;
    }

    s_beep_t++;

    if (!s_beep_on) {
        if (s_beep_t >= 2u) {
            buzzer_set(1);
            s_beep_on = 1;
            s_beep_t = 0;
        }
    } else {
        if (s_beep_t >= 1u) {
            buzzer_set(0);
            s_beep_on = 0;
            s_beep_t = 0;
            s_beep_left--;
        }
    }
}

void indicator_init(void)
{
    s_tick = 0;
    s_prev = 0xFFu;
    s_nag = 0;
    s_beep_left = 0;
    s_beep_on = 0;
    s_beep_t = 0;

    led_set(LED_OFF);
    buzzer_set(0);
}

void indicator_step(uint8_t state)
{
    uint8_t blink;

    s_tick++;
    blink = (uint8_t)((s_tick / BLINK_DIV) & 1u);

    led_set(ind_color(state, blink));

    if (state != s_prev) {
        if (state == CHASSIS_FAULT) {
            beep_start(3);
        } else if (state == CHASSIS_ESTOP) {
            beep_start(1);
        } else if (state == CHASSIS_SELFTEST) {
            beep_start(1);
        } else if (state == CHASSIS_IDLE && s_prev == CHASSIS_SELFTEST) {
            //自检跑完回到待命,两声短的
            beep_start(2);
        }
    }

    if (state == CHASSIS_FAULT) {
        s_nag++;
        if (s_nag >= NAG_TICKS) {
            s_nag = 0;
            if (s_beep_left == 0u) {
                beep_start(1);
            }
        }
    } else {
        s_nag = 0;
    }

    s_prev = state;
    beep_tick();
}
