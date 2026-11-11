// APP 按键通道。裸字节,没协议
//
// 这儿没协议:裸字节,没帧头没校验,还会混进蓝牙模块自己的状态串。
// 所以顺序是死的 —— 先整行吞掉 +CONNECTING<<MAC / +CONNECTED / +DISC:SUCCESS
// 这类,再跳过 {...} 调参帧里头的字节,最后才认白名单。
// 第二步漏过一次:+DISC:SUCCESS 里的 S 被当成自检键,车当场自己动起来。
#include "stm32f4xx.h"

#include "app_key.h"
#include "app_param.h"
#include "avoid.h"
#include "link.h"

#define KEY_BLADE_ON    0x57    //'W'
#define KEY_BLADE_OFF1  0x56    //'V'
#define KEY_BLADE_OFF2  0x50    //'P'
#define KEY_SELFTEST    0x53    //'S'

#define KEY_SPEED_UP    0x58    //'X'
#define KEY_SPEED_DOWN  0x59    //'Y'

#define KEY_AVOID_TOG   0x6E
#define KEY_AVOID_UP    0x6C
#define KEY_AVOID_DOWN  0x6F
#define BT_LINE_MAX     48

//蓝牙模块状态串过滤器:见到 '+' 就把整行吞掉
static uint8_t bt_filter(uint8_t c)
{
    static uint8_t in_line;
    static uint8_t n;

    if (!in_line) {
        if (c == '+') {
            in_line = 1;
            n = 0;
            return 1;
        }
        return 0;
    }

    n++;
    if (c == 0x0A || n > BT_LINE_MAX) {
        in_line = 0;
    }
    return 1;
}

//"reset" 字符串软复位,给无线烧录用
static void reset_watch(uint8_t c)
{
    static const char k_reset[] = "reset";
    static uint8_t pos;

    if (c == (uint8_t)k_reset[pos]) {
        pos++;
        if (pos == 5u) {
            pos = 0;
            NVIC_SystemReset();
        }
    } else {
        pos = (c == (uint8_t)k_reset[0]) ? 1u : 0u;
    }
}

void app_key_init(void)
{
    app_param_init();
}

void app_key_feed(uint8_t c)
{
    if (bt_filter(c)) return;

    reset_watch(c);

    //调参帧:开帧、帧内、闭帧都由解析器管,期间不做按键判定
    if (c == '{' || app_param_in_frame()) {
        app_param_feed(c);
        return;
    }

    //摇杆:0x41~0x48 是 A~H 八个方向,其余(含 0x40)视为松开
    if (c >= 0x41 && c <= 0x48) {
        g_app_dir = (uint8_t)(c - 0x40);
        g_app_dir_seq++;
        return;
    }
    if (c >= 1 && c <= 8) {
        g_app_dir = c;
        g_app_dir_seq++;
        return;
    }
    g_app_dir = 0;

    switch (c) {
    case KEY_BLADE_ON:
        link_blade_cmd(1, 1);       //开关式:APP 没法持续刷新
        break;
    case KEY_BLADE_OFF1:
    case KEY_BLADE_OFF2:
        link_blade_cmd(0, 1);
        break;
    case KEY_SELFTEST:
        link_selftest_req();
        break;
    case KEY_SPEED_UP:
        if (g_tune.rc_speed_mm <= 3400u) {
            g_tune.rc_speed_mm = (uint16_t)(g_tune.rc_speed_mm + 100u);
        }
        break;
    case KEY_SPEED_DOWN:
        if (g_tune.rc_speed_mm >= 100u) {
            g_tune.rc_speed_mm = (uint16_t)(g_tune.rc_speed_mm - 100u);
        }
        break;
    case KEY_AVOID_TOG:
        avoid_toggle(&g_avoid);
        break;
    case KEY_AVOID_UP:
        avoid_nearer(&g_avoid, 0.1f);
        break;
    case KEY_AVOID_DOWN:
        avoid_nearer(&g_avoid, -0.1f);
        break;
    default:
        //翻页 m、回充 b、APP 显示回传还没做,先吞掉
        break;
    }
}
