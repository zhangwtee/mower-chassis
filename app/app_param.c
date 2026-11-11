// APP 调参帧。{...} 里是键值对,收到就写进运行时参数
//
// 调参帧:单个是 {<cmd>:<数字>},批量是 {#:n1:n2:...:n9}
// 下标有两处容易搞混,按最合理的形状实现,而且每个值都夹范围 ——
// 就算格式对不上,也不会把参数写成怪值。
#include "app_param.h"

#define FRAME_MAX       48

#define CMD_RC_SPEED    '0'
#define CMD_KP          '1'
#define CMD_KI          '2'
#define CMD_STEP        '3'
#define CMD_LINE_DIFF   '8'
#define CMD_DUMP        '#'
#define CMD_SAVE        'W'

#define PI_4            0.78539816f

tune_t g_tune;
volatile uint8_t g_app_dir;
volatile uint32_t g_app_dir_seq;
volatile uint8_t g_param_save_req;

static char s_frame[FRAME_MAX];
static uint8_t s_len;
static uint8_t s_open;

static uint16_t clamp_u16(uint32_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (uint16_t)v;
}

void app_param_init(void)
{
    g_tune.rc_speed_mm = 500;
    g_tune.kp = 300;
    g_tune.ki = 300;
    g_tune.step_mm = 20;
    g_tune.line_diff = 50;

    g_app_dir = 0;
    g_app_dir_seq = 0;
    g_param_save_req = 0;
    s_len = 0;
    s_open = 0;
}

uint8_t app_param_in_frame(void)
{
    return s_open;
}

void app_joystick(uint8_t dir, uint16_t rc_mm, float *vx, float *vz)
{
    float rc = (float)rc_mm / 1000.0f;
    float x = 0.0f;
    float z = 0.0f;

    switch (dir) {
    case 1: x = rc;   z = 0.0f;   break;   //A 前
    case 2: x = rc;   z = -PI_4;  break;   //B 右前
    case 3: x = 0.0f; z = -PI_4;  break;   //C 右
    case 4: x = -rc;  z = PI_4;   break;   //D 右后
    case 5: x = -rc;  z = 0.0f;   break;   //E 后
    case 6: x = -rc;  z = -PI_4;  break;   //F 左后
    case 7: x = 0.0f; z = PI_4;   break;   //G 左
    case 8: x = rc;   z = PI_4;   break;   //H 左前
    default: break;
    }

    //转向量按遥控速度基准缩放
    *vx = x;
    *vz = z * ((float)rc_mm / 500.0f);
}

//从 [from, to) 里取一个十进制数,遇到非数字就停
static uint16_t pick_number(const char *buf, uint8_t from, uint8_t to)
{
    uint32_t v = 0;
    uint8_t i;
    uint8_t got = 0;

    for (i = from; i < to; i++) {
        if (buf[i] >= '0' && buf[i] <= '9') {
            v = v * 10u + (uint32_t)(buf[i] - '0');
            got = 1;
            if (v > 100000u) v = 100000u;
        } else if (got) {
            break;
        }
    }
    return (uint16_t)v;
}

static void apply_one(char cmd, uint16_t v)
{
    switch (cmd) {
    case CMD_RC_SPEED:
        g_tune.rc_speed_mm = clamp_u16(v, 0u, 3500u);
        break;
    case CMD_KP:
        g_tune.kp = clamp_u16(v, 1u, 10000u);
        break;
    case CMD_KI:
        g_tune.ki = clamp_u16(v, 1u, 10000u);
        break;
    case CMD_STEP:
        g_tune.step_mm = clamp_u16(v, 1u, 200u);
        break;
    case CMD_LINE_DIFF:
        g_tune.line_diff = clamp_u16(v, 0u, 100u);
        break;
    case CMD_SAVE:
        //这里只挂个标志。真正擦写要 250ms 且中断全关,不能在中断里做
        g_param_save_req = 1;
        break;
    default:
        break;
    }
}

//批量帧:{#:rc:kp:ki:step:...:line_diff}
static void apply_dump(const char *buf, uint8_t n)
{
    uint16_t item[12];
    uint8_t cnt = 0;
    uint8_t i;
    uint8_t start = (buf[1] == 0x3A) ? 2u : 1u;

    for (i = start; i <= n && cnt < 12u; i++) {
        if (i == n || buf[i] == ':') {
            item[cnt++] = pick_number(buf, start, i);
            start = (uint8_t)(i + 1);
        }
    }

    if (cnt >= 1u) g_tune.rc_speed_mm = clamp_u16(item[0], 0u, 3500u);
    if (cnt >= 2u) g_tune.kp = clamp_u16(item[1], 1u, 10000u);
    if (cnt >= 3u) g_tune.ki = clamp_u16(item[2], 1u, 10000u);
    if (cnt >= 4u) g_tune.step_mm = clamp_u16(item[3], 1u, 200u);
    if (cnt >= 9u) g_tune.line_diff = clamp_u16(item[8], 0u, 100u);
}

uint8_t app_param_feed(uint8_t c)
{
    if (c == '{') {
        s_open = 1;
        s_len = 0;
        return 0;
    }

    if (!s_open) return 0;

    if (c == '}') {
        s_open = 0;

        if (s_len >= 3u && s_frame[0] == CMD_DUMP) {
            apply_dump(s_frame, s_len);
            return 1;
        }
        if (s_len >= 3u && s_frame[1] == ':') {
            apply_one(s_frame[0], pick_number(s_frame, 2, s_len));
            return 1;
        }
        return 0;
    }

    if (s_len < FRAME_MAX) {
        s_frame[s_len++] = (char)c;
    } else {
        //超长帧整帧丢掉,避免缓冲写穿
        s_open = 0;
        s_len = 0;
    }
    return 0;
}
