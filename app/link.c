// 下行帧到控制量的接线。遥测也从这儿过
#include "link.h"
#include "protocol.h"
#include "uart.h"
#include "can.h"
#include "app_state.h"

static protocol_t k_p1;
static protocol_t k_p3;
static link_events_t k_ev;

static volatile uint16_t s_dist_age = 0xFFFF;

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

//超声波板发两帧:A~D 在 0x101,E~F 在 0x102
static void on_can(uint16_t id, const uint8_t *d)
{
    if (id == CAN_ID_ULTRA_ABCD) {
        g_state.dist_mm[0] = be16(d + 0);
        g_state.dist_mm[1] = be16(d + 2);
        g_state.dist_mm[2] = be16(d + 4);
        g_state.dist_mm[3] = be16(d + 6);
        s_dist_age = 0;
    } else if (id == CAN_ID_ULTRA_EF) {
        g_state.dist_mm[4] = be16(d + 0);
        g_state.dist_mm[5] = be16(d + 2);
        s_dist_age = 0;
    }
}

uint16_t link_bad_frames(void)
{
    return (uint16_t)(k_p1.bad_frame + k_p3.bad_frame);
}

uint8_t link_dist_valid(void)
{
    return (uint8_t)(s_dist_age < 50u);
}

void link_poll_tick(void)
{
    if (s_dist_age < 0xFFFFu) s_dist_age++;
}

static void on_velocity(float vx, float vy, float vz, uint8_t src)
{
    (void)vy;
    (void)src;

    k_ev.cmd_new = 1;
    k_ev.cmd_vx = vx;
    k_ev.cmd_vz = vz;
}

//回充和对接暂不接,收到只记一条,不驱动电机
static void on_recharge(float vx, float vy, float vz, uint8_t nav, uint8_t src)
{
    (void)vx;
    (void)vy;
    (void)vz;
    (void)nav;
    (void)src;
}

static void on_dock(float vx, float vy, float vz)
{
    (void)vx;
    (void)vy;
    (void)vz;
}

static void on_servo(uint16_t us)
{
    k_ev.servo_valid = 1;
    k_ev.servo_us = us;
}

static void on_selftest(void)
{
    k_ev.selftest_req = 1;
}

//上位机 CMD=6:会持续刷新,按超时判定
static void on_blade(uint8_t on)
{
    link_blade_cmd(on, 0);
}

static void on_estop(void)
{
    k_ev.estop_req = 1;
}

static void on_estop_release(void)
{
    k_ev.estop_release_req = 1;
}

void link_blade_cmd(uint8_t on, uint8_t sticky)
{
    k_ev.blade_valid = 1;
    k_ev.blade_cmd = on;
    k_ev.blade_sticky = sticky;
}

void link_selftest_req(void)
{
    k_ev.selftest_req = 1;
}

static const protocol_sink_t k_sink = {
    on_velocity,
    on_recharge,
    on_dock,
    on_servo,
    on_selftest,
    on_blade,
    on_estop,
    on_estop_release
};

static void rx1(uint8_t c)
{
    protocol_feed_isr(&k_p1, c);
}

static void rx3(uint8_t c)
{
    protocol_feed_isr(&k_p3, c);
}

void link_init(void)
{
    protocol_init(&k_p1, &k_sink, PROTOCOL_SRC_USART);
    protocol_init(&k_p3, &k_sink, PROTOCOL_SRC_ROS);

    uart1_set_rx(rx1);
    uart3_set_rx(rx3);

    can_set_rx(on_can);

    link_clear_once();
}

void link_poll(void)
{
    protocol_poll(&k_p1);
    protocol_poll(&k_p3);
}

void link_clear_once(void)
{
    k_ev.estop_req = 0;
    k_ev.estop_release_req = 0;
    k_ev.selftest_req = 0;
    k_ev.cmd_new = 0;
    k_ev.blade_valid = 0;
    k_ev.blade_sticky = 0;
    k_ev.servo_valid = 0;
}

const link_events_t *link_events(void)
{
    return &k_ev;
}
