// 底盘状态机。六个状态,能自己走的只有 IDLE 和 RUN
#include "chassis.h"
#include "chassis_safety.h"

//轮子算停住的门限
#define ST_STOP_MPS         0.05f
//等轮子停下来的最长时间,100Hz 下 2 秒
#define ST_PENDING_MAX      200

//自检脚本。
//settle 是前多少拍不计入判定:电机从 0 加到目标值要时间,加速段会把
//最小值/最大值撑坏,不排掉就会每次自检都误判。
static const struct {
    uint16_t ticks;
    uint16_t settle;
    float vx;
    float vz;
    uint8_t check;
} k_st[] = {
    { 20,  0, 0.00f,  0.0f, 1 },   //静止,看计数是否接近 0
    { 60, 20, 0.15f,  0.0f, 2 },   //正走
    { 60, 20, -0.15f, 0.0f, 3 },   //倒走
    { 60, 20, 0.00f,  0.4f, 4 },   //原地转
    { 200, 0, 0.00f,  0.0f, 0 },   //舵机扫
    { 20,  0, 0.00f,  0.0f, 0 }    //停
};

#define ST_COUNT    ((uint8_t)(sizeof(k_st) / sizeof(k_st[0])))

static uint8_t wheels_stopped(float l, float r)
{
    if (l > ST_STOP_MPS || l < -ST_STOP_MPS) return 0;
    if (r > ST_STOP_MPS || r < -ST_STOP_MPS) return 0;
    return 1;
}

static void st_begin(chassis_t *c)
{
    c->st_step = 0;
    c->st_tick = 0;
    c->st_lo_l = 1e9f;
    c->st_hi_l = -1e9f;
    c->st_lo_r = 1e9f;
    c->st_hi_r = -1e9f;
}

static void st_track(chassis_t *c, float l, float r)
{
    if (l < c->st_lo_l) c->st_lo_l = l;
    if (l > c->st_hi_l) c->st_hi_l = l;
    if (r < c->st_lo_r) c->st_lo_r = r;
    if (r > c->st_hi_r) c->st_hi_r = r;
}

static uint8_t st_check(uint8_t kind, const chassis_t *c)
{
    switch (kind) {
    case 1:
        return (c->st_hi_l < ST_STOP_MPS && c->st_lo_l > -ST_STOP_MPS &&
                c->st_hi_r < ST_STOP_MPS && c->st_lo_r > -ST_STOP_MPS) ? 1 : 0;
    case 2:
        return (c->st_lo_l > 0.10f && c->st_hi_l < 0.25f &&
                c->st_lo_r > 0.10f && c->st_hi_r < 0.25f) ? 1 : 0;
    case 3:
        return (c->st_hi_l < -0.10f && c->st_hi_r < -0.10f) ? 1 : 0;
    case 4:
        return (c->st_hi_l < -0.05f && c->st_lo_r > 0.05f) ? 1 : 0;
    default:
        return 1;
    }
}

//舵机扫:1500 -> 500 -> 1500 -> 2500 -> 1500,每拍 20us,四段各 50 拍,共 2 秒。
//行程覆盖 500~2500 整段,
//2 秒扫完。自检只做一次,够看出卡不卡了。
#define SERVO_MID       1500
#define SERVO_LOW       500
#define SERVO_HIGH      2500
#define SERVO_STEP_US   20
#define SERVO_LEG       ((SERVO_MID - SERVO_LOW) / SERVO_STEP_US)

static uint16_t servo_sweep(uint16_t tick)
{
    if (tick < SERVO_LEG) {
        return (uint16_t)(SERVO_MID - tick * SERVO_STEP_US);
    }
    tick = (uint16_t)(tick - SERVO_LEG);
    if (tick < SERVO_LEG) {
        return (uint16_t)(SERVO_LOW + tick * SERVO_STEP_US);
    }
    tick = (uint16_t)(tick - SERVO_LEG);
    if (tick < SERVO_LEG) {
        return (uint16_t)(SERVO_MID + tick * SERVO_STEP_US);
    }
    tick = (uint16_t)(tick - SERVO_LEG);
    if (tick < SERVO_LEG) {
        return (uint16_t)(SERVO_HIGH - tick * SERVO_STEP_US);
    }
    return SERVO_MID;
}

void chassis_init(chassis_t *c)
{
    c->state = CHASSIS_INIT;
    c->uptime = 0;
    c->volt_low = 0;
    c->cmd_age = 0;
    c->cmd_vx = 0.0f;
    c->cmd_vz = 0.0f;
    c->servo_us = 1500;
    c->st_step = 0;
    c->st_tick = 0;
    c->st_lo_l = 1e9f;
    c->st_hi_l = -1e9f;
    c->st_lo_r = 1e9f;
    c->st_hi_r = -1e9f;
    c->st_pending = 0;
    c->st_pending_age = 0;
    blade_init(&c->blade);
}

void chassis_step(chassis_t *c, const chassis_in_t *in, uint16_t dt_ms, chassis_out_t *out)
{
    uint8_t volt_ok;
    uint8_t move_ok;
    uint8_t can_start;
    uint8_t state_ok;

    c->uptime += dt_ms;

    if (in->cmd_new) {
        c->cmd_age = 0;
        c->cmd_vx = in->cmd_vx;
        c->cmd_vz = in->cmd_vz;
    } else if (c->cmd_age < 0xFFFF) {
        c->cmd_age += dt_ms;
    }

    if (in->servo_valid) {
        c->servo_us = in->servo_us;
    }

    //电压要过滞回:低于 20V 锁住,高于 21V 才解除
    volt_ok = safety_volt_ok(in->voltage, &c->volt_low);
    move_ok = safety_move_ok(volt_ok, in->enable_switch, c->cmd_age);
    can_start = (uint8_t)(in->enable_switch && volt_ok);

    //急停优先。FAULT 是终态,不能被急停/解除绕出去
    if (in->estop_req && c->state != CHASSIS_FAULT) {
        c->state = CHASSIS_ESTOP;
    }

    switch (c->state) {
    case CHASSIS_INIT:
        if (c->uptime >= SAFETY_BOOT_MS) {
#if SELFTEST_ON_BOOT
            c->state = CHASSIS_SELFTEST;
            st_begin(c);
#else
            c->state = CHASSIS_IDLE;
#endif
        }
        break;

    case CHASSIS_IDLE:
        if (c->st_pending) {
            //等轮子停稳再进自检,否则"静止"和"正走"两项都会误判
            if (!can_start ||
                (in->cmd_new && (in->cmd_vx != 0.0f || in->cmd_vz != 0.0f))) {
                c->st_pending = 0;
            } else if (wheels_stopped(in->wheel_left, in->wheel_right)) {
                c->st_pending = 0;
                c->state = CHASSIS_SELFTEST;
                st_begin(c);
            } else if (c->st_pending_age < 0xFFFF) {
                c->st_pending_age++;
                if (c->st_pending_age > ST_PENDING_MAX) {
                    c->st_pending = 0;
                }
            }
        } else if (in->selftest_req && can_start) {
            c->state = CHASSIS_SELFTEST;
            st_begin(c);
        } else if (move_ok && (c->cmd_vx != 0.0f || c->cmd_vz != 0.0f)) {
            c->state = CHASSIS_RUN;
        }
        break;

    case CHASSIS_RUN:
        if (in->selftest_req) {
            //先停车,交给 IDLE 等轮子停下再进自检
            c->state = CHASSIS_IDLE;
            c->st_pending = 1;
            c->st_pending_age = 0;
        } else if (!move_ok || (c->cmd_vx == 0.0f && c->cmd_vz == 0.0f)) {
            //离开 RUN 就作废旧指令:欠压恢复、开关重新闭合都不能让它自己接着跑
            c->state = CHASSIS_IDLE;
            c->cmd_age = SAFETY_CMD_AGE_MAX + 1;
        }
        break;

    case CHASSIS_SELFTEST:
        //自检也要受使能开关和欠压管,断了就中止,不算通过也不算失败
        if (!can_start) {
            c->state = CHASSIS_IDLE;
            break;
        }

        if (c->st_tick >= k_st[c->st_step].settle) {
            st_track(c, in->wheel_left, in->wheel_right);
        }
        c->st_tick++;

        if (c->st_tick >= k_st[c->st_step].ticks) {
            if (!st_check(k_st[c->st_step].check, c)) {
                c->state = CHASSIS_FAULT;
                break;
            }
            c->st_step++;
            c->st_tick = 0;
            c->st_lo_l = 1e9f;
            c->st_hi_l = -1e9f;
            c->st_lo_r = 1e9f;
            c->st_hi_r = -1e9f;
            if (c->st_step >= ST_COUNT) {
                c->state = CHASSIS_IDLE;
            }
        }
        break;

    case CHASSIS_ESTOP:
        //同一拍还收到急停请求时不算解除
        if (in->estop_release_req && !in->estop_req && in->enable_switch) {
            c->state = CHASSIS_IDLE;
            c->cmd_age = SAFETY_CMD_AGE_MAX + 1;
        }
        break;

    case CHASSIS_FAULT:
    default:
        break;
    }

    out->state = c->state;
    out->move_allowed = 0;
    out->vx = 0.0f;
    out->vz = 0.0f;
    out->servo_us = c->servo_us;

    switch (c->state) {
    case CHASSIS_IDLE:
        out->move_allowed = 1;
        break;
    case CHASSIS_RUN:
        out->move_allowed = 1;
        out->vx = c->cmd_vx;
        out->vz = c->cmd_vz;
        break;
    case CHASSIS_SELFTEST:
        out->vx = k_st[c->st_step].vx;
        out->vz = k_st[c->st_step].vz;
        if (c->st_step == 4) {
            out->servo_us = servo_sweep(c->st_tick);
        }
        break;
    default:
        break;
    }

    state_ok = (c->state == CHASSIS_IDLE || c->state == CHASSIS_RUN) ? 1 : 0;
    blade_step(&c->blade, dt_ms, in->blade_valid, in->blade_cmd,
               in->blade_sticky, safety_blade_ok(state_ok, volt_ok));
    out->blade_on = blade_on(&c->blade);
}
