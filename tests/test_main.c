/* PC 上的单元测试，不需要硬件。
 *
 * protocol.c、telemetry.c 这些只 include stdint，不碰 STM32 头，拿 gcc 直接编就能跑。
 *
 * vectors.h 里是期望字节，由 gen_vectors.py 按上位机的组帧逻辑生成。
 * 所以"固件组出来的帧 == 上位机能解出来的帧"是跑出来的，不用人眼对。
 *
 * 跑法： powershell -ExecutionPolicy Bypass -File tests\run_tests.ps1
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "protocol.h"
#include "telemetry.h"
#include "kinematics.h"
#include "pi.h"
#include "chassis.h"
#include "chassis_safety.h"
#include "blade.h"
#include "ring.h"
#include "speed_smooth.h"
#include "filter.h"
#include "app_param.h"
#include "avoid.h"
#include "params.h"
#include "watchdog.h"
#include "click.h"
#include "park.h"
#include "diag.h"
#include "joy_guard.h"
#include "indicator.h"
#include "vectors.h"

static int g_fail;
static int g_pass;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (cond) {                                                       \
            g_pass++;                                                     \
        } else {                                                          \
            g_fail++;                                                     \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                 \
    } while (0)

static void dump_hex(const char *tag, const uint8_t *a, const uint8_t *b, int n)
{
    int i;

    printf("  %s\n    got: ", tag);
    for (i = 0; i < n; i++) {
        printf("%02X ", a[i]);
    }
    printf("\n    exp: ");
    for (i = 0; i < n; i++) {
        printf("%02X ", b[i]);
    }
    printf("\n");
}

/* ======================= 遥测帧 ======================= */

static void check_frame(const char *name, const uint8_t *got, const uint8_t *exp, int n)
{
    int ok = (memcmp(got, exp, (size_t)n) == 0);

    if (!ok) {
        dump_hex(name, got, exp, n);
    }
    CHECK(ok);
    if (ok) {
        printf("  ok   %s\n", name);
    }
}

static void test_telemetry(void)
{
    uint8_t buf[TELEMETRY_CHASSIS_LEN];
    int16_t dist[TELEMETRY_DISTANCE_SLOTS];
    telemetry_chassis_t in;

    printf("[telemetry] 底盘帧\n");

    in.flag_stop = 0u;
    in.vel_mmps[0] = 500;
    in.vel_mmps[1] = -250;
    in.vel_mmps[2] = 0;
    in.accel_raw[0] = 1;
    in.accel_raw[1] = -2;
    in.accel_raw[2] = 1671;
    in.gyro_raw[0] = -100;
    in.gyro_raw[1] = 0;
    in.gyro_raw[2] = 50;
    in.voltage_mv = 24000u;
    telemetry_build_chassis(buf, &in);
    check_frame("chassis_normal", buf, golden_chassis_normal, GOLDEN_CHASSIS_LEN);

    in.flag_stop = 1u;
    in.vel_mmps[0] = 0;
    in.vel_mmps[1] = 0;
    in.vel_mmps[2] = 0;
    in.accel_raw[0] = 0;
    in.accel_raw[1] = 0;
    in.accel_raw[2] = 0;
    in.gyro_raw[0] = 0;
    in.gyro_raw[1] = 0;
    in.gyro_raw[2] = 0;
    in.voltage_mv = 0u;
    telemetry_build_chassis(buf, &in);
    check_frame("chassis_stopped", buf, golden_chassis_stopped, GOLDEN_CHASSIS_LEN);

    in.flag_stop = 0u;
    in.vel_mmps[0] = 32767;
    in.vel_mmps[1] = -32768;
    in.vel_mmps[2] = 1000;
    in.accel_raw[0] = -32768;
    in.accel_raw[1] = 32767;
    in.accel_raw[2] = 0;
    in.gyro_raw[0] = 32767;
    in.gyro_raw[1] = -32768;
    in.gyro_raw[2] = 1;
    in.voltage_mv = 65535u;
    telemetry_build_chassis(buf, &in);
    check_frame("chassis_extremes", buf, golden_chassis_extremes, GOLDEN_CHASSIS_LEN);

    /* BCC 覆盖长度是这份协议最容易错的地方：22 字节，不是 24。
     * 用只覆盖到 9 或覆盖 24 的实现都会在这一条上挂掉。 */
    in.flag_stop = 0u;
    in.vel_mmps[0] = 500;
    in.vel_mmps[1] = -250;
    in.vel_mmps[2] = 0;
    in.accel_raw[0] = 1;
    in.accel_raw[1] = -2;
    in.accel_raw[2] = 1671;
    in.gyro_raw[0] = -100;
    in.gyro_raw[1] = 0;
    in.gyro_raw[2] = 50;
    in.voltage_mv = 24000u;
    telemetry_build_chassis(buf, &in);
    CHECK(telemetry_bcc(buf, TELEMETRY_CHASSIS_BCC_LEN) == buf[TELEMETRY_CHASSIS_BCC_LEN]);
    CHECK(telemetry_bcc(buf, TELEMETRY_CHASSIS_BCC_LEN) != telemetry_bcc(buf, 24u) ||
          buf[22] == telemetry_bcc(buf, 24u));

    printf("[telemetry] 超声波帧\n");

    dist[0] = 123; dist[1] = 456; dist[2] = 789; dist[3] = 1000;
    dist[4] = 1500; dist[5] = 2000; dist[6] = 0; dist[7] = 0;
    telemetry_build_distance(buf, dist);
    check_frame("distance_normal", buf, golden_distance_normal, GOLDEN_DISTANCE_LEN);

    dist[0] = 32767; dist[1] = -32768; dist[2] = 0; dist[3] = 1;
    dist[4] = -1; dist[5] = 32767; dist[6] = 0; dist[7] = 0;
    telemetry_build_distance(buf, dist);
    check_frame("distance_extremes", buf, golden_distance_extremes, GOLDEN_DISTANCE_LEN);

    memset(dist, 0, sizeof(dist));
    telemetry_build_distance(buf, dist);
    check_frame("distance_none", buf, golden_distance_none, GOLDEN_DISTANCE_LEN);

    /* 帧头帧尾 */
    CHECK(buf[0] == GOLDEN_DIST_HEADER);
    CHECK(buf[GOLDEN_DISTANCE_LEN - 1] == GOLDEN_DIST_TAIL);
}

/* ======================= 下行帧解析 ======================= */

typedef struct {
    int vel_calls, rech_calls, dock_calls, servo_calls, st_calls, blade_calls,
        estop_calls, rel_calls;
    float vx, vy, vz;
    float dock_vx;
    uint8_t nav, src, blade_on;
    uint16_t servo;
} rec_t;

static rec_t g_rec;

static void rec_reset(void)
{
    memset(&g_rec, 0, sizeof(g_rec));
}

static void s_vel(float vx, float vy, float vz, uint8_t src)
{
    g_rec.vel_calls++;
    g_rec.vx = vx;
    g_rec.vy = vy;
    g_rec.vz = vz;
    g_rec.src = src;
}

static void s_rech(float vx, float vy, float vz, uint8_t nav, uint8_t src)
{
    g_rec.rech_calls++;
    g_rec.vx = vx;
    g_rec.vy = vy;
    g_rec.vz = vz;
    g_rec.nav = nav;
    g_rec.src = src;
}

static void s_dock(float vx, float vy, float vz)
{
    g_rec.dock_calls++;
    g_rec.dock_vx = vx;
    g_rec.vy = vy;
    g_rec.vz = vz;
}

static void s_servo(uint16_t us)
{
    g_rec.servo_calls++;
    g_rec.servo = us;
}

static void s_st(void)
{
    g_rec.st_calls++;
}

static void s_blade(uint8_t on)
{
    g_rec.blade_calls++;
    g_rec.blade_on = on;
}

static void s_estop(void)
{
    g_rec.estop_calls++;
}

static void s_rel(void)
{
    g_rec.rel_calls++;
}

static const protocol_sink_t g_sink = {
    s_vel, s_rech, s_dock, s_servo, s_st, s_blade, s_estop, s_rel
};

/* 独立于 telemetry.c 组一帧，避免测试用被测代码自己造测试数据 */
static void mk_frame(uint8_t *f, uint8_t cmd, int16_t a, int16_t b, int16_t c)
{
    f[0] = PROTOCOL_HEADER;
    f[1] = cmd;
    f[2] = 0u;
    f[3] = (uint8_t)((uint16_t)a >> 8);
    f[4] = (uint8_t)a;
    f[5] = (uint8_t)((uint16_t)b >> 8);
    f[6] = (uint8_t)b;
    f[7] = (uint8_t)((uint16_t)c >> 8);
    f[8] = (uint8_t)c;
    f[9] = protocol_bcc(f, PROTOCOL_BCC_LEN);
    f[10] = PROTOCOL_TAIL;
}

/* 有些命令的载荷是"单个字节"而不是 int16（比如刀片取 buffer[3]），
 * 这种要用本函数构造，不能拿 mk_frame 的 int16 参数凑。 */
static void mk_frame_byte3(uint8_t *f, uint8_t cmd, uint8_t b3)
{
    mk_frame(f, cmd, 0, 0, 0);
    f[3] = b3;
    f[9] = protocol_bcc(f, PROTOCOL_BCC_LEN);
}

static void feed_all(protocol_t *p, const uint8_t *d, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        protocol_feed_isr(p, d[i]);
    }
    protocol_poll(p);
}

static void test_protocol(void)
{
    static protocol_t proto;
    uint8_t f[PROTOCOL_FRAME_LEN];
    uint8_t stream[64];
    int n;

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);

    printf("[protocol] 速度帧\n");

    rec_reset();
    mk_frame(f, 0u, 500, -250, 1000);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 1);
    CHECK(fabsf(g_rec.vx - 0.5f) < 1e-6f);
    CHECK(fabsf(g_rec.vy - (-0.25f)) < 1e-6f);
    CHECK(fabsf(g_rec.vz - 1.0f) < 1e-6f);
    CHECK(g_rec.src == PROTOCOL_SRC_ROS);
    CHECK(proto.bad_frame == 0u);

    printf("[protocol] 回充帧：CMD 1 允许导航、CMD 2 不允许\n");

    rec_reset();
    mk_frame(f, 1u, -100, 0, 200);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.rech_calls == 1);
    CHECK(g_rec.nav == 1u);
    CHECK(fabsf(g_rec.vx - (-0.1f)) < 1e-6f);

    rec_reset();
    mk_frame(f, 2u, 300, 0, 0);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.rech_calls == 1);
    CHECK(g_rec.nav == 0u);
    CHECK(fabsf(g_rec.vx - 0.3f) < 1e-6f);

    printf("[protocol] 舵机限幅\n");

    {
        static const struct { int16_t in; uint16_t exp; } cases[] = {
            { 1500, 1500u },
            { 500u, 500u },
            { 2500u, 2500u },
            { 0, 500u },        /* 越界下 */
            { 9999, 2500u },    /* 越界上 */
            { -100, 500u },     /* 负数 */
        };
        unsigned i;

        for (i = 0u; i < sizeof(cases) / sizeof(cases[0]); i++) {
            rec_reset();
            mk_frame(f, 4u, cases[i].in, 0, 0);
            feed_all(&proto, f, PROTOCOL_FRAME_LEN);
            CHECK(g_rec.servo_calls == 1);
            if (g_rec.servo != cases[i].exp) {
                printf("  servo in=%d exp=%u got=%u\n", cases[i].in, cases[i].exp, g_rec.servo);
            }
            CHECK(g_rec.servo == cases[i].exp);
        }
    }

    printf("[protocol] 自检 / 刀片 / 急停 / 解除\n");

    rec_reset();
    mk_frame(f, 5u, 0, 0, 0);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.st_calls == 1);

    rec_reset();
    mk_frame_byte3(f, 6u, 1u);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.blade_calls == 1 && g_rec.blade_on == 1u);

    rec_reset();
    mk_frame_byte3(f, 6u, 0u);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.blade_calls == 1 && g_rec.blade_on == 0u);

    /* 载荷非 0 就算开 */
    rec_reset();
    mk_frame_byte3(f, 6u, 0x7Fu);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.blade_calls == 1 && g_rec.blade_on == 1u);

    rec_reset();
    mk_frame(f, 7u, 0, 0, 0);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.estop_calls == 1);

    rec_reset();
    mk_frame(f, 8u, 0, 0, 0);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.rel_calls == 1);

    printf("[protocol] 逐字节喂入（模拟串口一次来一个字节）\n");

    rec_reset();
    mk_frame(f, 0u, 100, 200, 300);
    for (n = 0; n < (int)PROTOCOL_FRAME_LEN; n++) {
        protocol_feed_isr(&proto, f[n]);
        protocol_poll(&proto);
    }
    CHECK(g_rec.vel_calls == 1);
    CHECK(fabsf(g_rec.vx - 0.1f) < 1e-6f);

    printf("[protocol] 坏 BCC / 坏帧尾一律不派发\n");

    rec_reset();
    mk_frame(f, 0u, 500, 0, 0);
    f[9] = (uint8_t)(f[9] ^ 0xFFu);
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 0);
    CHECK(proto.bad_frame == 1u);

    rec_reset();
    mk_frame(f, 0u, 500, 0, 0);
    f[10] = 0x00u;
    feed_all(&proto, f, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 0);
    CHECK(proto.bad_frame == 2u);

    printf("[protocol] 帧前垃圾字节被跳过\n");

    rec_reset();
    mk_frame(f, 0u, 250, 0, 0);
    stream[0] = 0x00u;
    stream[1] = 0x55u;
    stream[2] = 0xAAu;
    memcpy(&stream[3], f, PROTOCOL_FRAME_LEN);
    feed_all(&proto, stream, 3 + (int)PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 1);
    CHECK(fabsf(g_rec.vx - 0.25f) < 1e-6f);

    printf("[protocol] 两帧背靠背\n");

    rec_reset();
    mk_frame(f, 0u, 100, 0, 0);
    memcpy(&stream[0], f, PROTOCOL_FRAME_LEN);
    mk_frame(f, 0u, 200, 0, 0);
    memcpy(&stream[PROTOCOL_FRAME_LEN], f, PROTOCOL_FRAME_LEN);
    feed_all(&proto, stream, 2 * (int)PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 2);

    printf("[protocol] 误码后重同步：坏帧里藏着下一帧的帧头，必须还能解出来\n");

    rec_reset();
    mk_frame(f, 0u, 750, 0, 0);
    /* 先给一个假帧头 + 5 个垃圾，紧接着一整个合法帧：
     * 组帧到 11 字节时校验必失败，resync 要在第 6 个位置找到真帧头。 */
    stream[0] = PROTOCOL_HEADER;
    for (n = 0; n < 5; n++) {
        stream[1 + n] = 0x11u;
    }
    memcpy(&stream[6], f, PROTOCOL_FRAME_LEN);
    feed_all(&proto, stream, 6 + (int)PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 1);
    CHECK(fabsf(g_rec.vx - 0.75f) < 1e-6f);
    CHECK(proto.bad_frame == 3u);

    printf("[protocol] 未知命令字不派发，也不算坏帧\n");

    {
        uint16_t before = proto.bad_frame;

        rec_reset();
        mk_frame(f, 9u, 0, 0, 0);          /* 校验对、帧尾对，只是命令号超范围 */
        feed_all(&proto, f, PROTOCOL_FRAME_LEN);
        CHECK(proto.bad_frame == before);  /* 不是坏帧 */
        CHECK(g_rec.vel_calls == 0);       /* 也不派发 */
        CHECK(g_rec.estop_calls == 0);
        CHECK(g_rec.blade_calls == 0);
    }

    printf("[protocol] 环形缓冲溢出要计数，不静默丢\n");

    {
        static protocol_t p2;
        int i;
        int total;
        int capacity;
        uint32_t before;

        protocol_init(&p2, &g_sink, PROTOCOL_SRC_USART);
        before = p2.overflow;

        /* 环形缓冲用 head==tail 表示空，所以可用容量是 RING-1 */
        capacity = (int)PROTOCOL_RX_RING - 1;
        total = (int)PROTOCOL_RX_RING + 10;

        for (i = 0; i < total; i++) {
            protocol_feed_isr(&p2, 0x00u);
        }
        CHECK(p2.overflow == before + (uint32_t)(total - capacity));
    }

    printf("[protocol] 换端口：src 要跟着走\n");

    {
        static protocol_t p3;
        protocol_init(&p3, &g_sink, PROTOCOL_SRC_USART);
        rec_reset();
        mk_frame(f, 0u, 100, 0, 0);
        feed_all(&p3, f, PROTOCOL_FRAME_LEN);
        CHECK(g_rec.vel_calls == 1);
        CHECK(g_rec.src == PROTOCOL_SRC_USART);
    }
}

/* ======================= 运动学 ======================= */

/* 轮距 0.435 m，胎径 0.180 m，编码器 4*500*27 */
static const float TRACK = 0.435f;
static const float CIRC = 0.5654867f;
static const float CPR = 54000.0f;

static void test_kinematics(void)
{
    float l, r, vx, vz;
    int i;

    printf("[kinematics] 逆解\n");

    kinematics_inverse(1.0f, 0.0f, TRACK, &l, &r);
    CHECK(fabsf(l - 1.0f) < 1e-6f && fabsf(r - 1.0f) < 1e-6f);

    /* Vz 为正 → 右轮快、左轮慢 */
    kinematics_inverse(0.0f, 1.0f, TRACK, &l, &r);
    CHECK(fabsf(l - (-0.2175f)) < 1e-6f);
    CHECK(fabsf(r - 0.2175f) < 1e-6f);

    kinematics_inverse(0.5f, 0.5f, TRACK, &l, &r);
    CHECK(fabsf(l - 0.39125f) < 1e-6f);
    CHECK(fabsf(r - 0.60875f) < 1e-6f);

    kinematics_inverse(-0.3f, -0.8f, TRACK, &l, &r);
    CHECK(fabsf(l - (-0.126f)) < 1e-6f);
    CHECK(fabsf(r - (-0.474f)) < 1e-6f);

    printf("[kinematics] 正解\n");

    kinematics_forward(1.0f, 1.0f, TRACK, &vx, &vz);
    CHECK(fabsf(vx - 1.0f) < 1e-6f && fabsf(vz) < 1e-6f);

    kinematics_forward(-0.2175f, 0.2175f, TRACK, &vx, &vz);
    CHECK(fabsf(vx) < 1e-6f && fabsf(vz - 1.0f) < 1e-6f);

    printf("[kinematics] 正逆解互为反函数\n");

    for (i = 0; i < 5; i++) {
        static const float vxs[] = { 0.0f, 0.5f, -0.5f, 1.2f, -0.25f };
        static const float vzs[] = { 0.0f, 0.3f, -0.3f, -1.0f, 0.8f };
        float l2, r2;

        kinematics_inverse(vxs[i], vzs[i], TRACK, &l2, &r2);
        kinematics_forward(l2, r2, TRACK, &vx, &vz);
        CHECK(fabsf(vx - vxs[i]) < 1e-5f);
        CHECK(fabsf(vz - vzs[i]) < 1e-5f);
    }

    printf("[kinematics] 编码器计数换算轮速\n");

    /* 100Hz 下 540 个脉冲 = 0.01 转 = 0.00565 m / 10ms = 0.5655 m/s */
    CHECK(fabsf(kinematics_counts_to_mps(540, CPR, CIRC, 100u) - CIRC) < 1e-5f);
    CHECK(fabsf(kinematics_counts_to_mps(-540, CPR, CIRC, 100u) + CIRC) < 1e-5f);
    CHECK(fabsf(kinematics_counts_to_mps(0, CPR, CIRC, 100u)) < 1e-9f);

    /* counts × 100 Hz ÷ CPR × 周长 */
    {
        int16_t c;
        for (c = -1000; c <= 1000; c += 250) {
            float old = (float)c * 100.0f / CPR * CIRC;
            float now = kinematics_counts_to_mps(c, CPR, CIRC, 100u);
            CHECK(fabsf(old - now) < 1e-6f);
        }
    }
}

/* ======================= 增量式 PI ======================= */

static void test_pi(void)
{
    pi_t pi;
    float out;
    int calls;

    printf("[pi] 前两步的增量公式\n");

    pi_init(&pi, 300.0f, 300.0f);

    /* 偏差 0.5 → 300*(0.5-0) + 300*0.5 = 300 */
    out = pi_update(&pi, 0.0f, 0.5f, 16800.0f);
    CHECK(fabsf(out - 300.0f) < 1e-3f);

    /* 偏差 0.4，上一步偏差 0.5 → +300*(0.4-0.5) + 300*0.4 = +90 → 390 */
    out = pi_update(&pi, 0.1f, 0.5f, 16800.0f);
    CHECK(fabsf(out - 390.0f) < 1e-2f);

    printf("[pi] 输出限幅\n");

    pi_reset(&pi);
    out = pi_update(&pi, 0.0f, 1000.0f, 16800.0f);
    CHECK(fabsf(out - 16800.0f) < 1e-3f);

    pi_reset(&pi);
    out = pi_update(&pi, 0.0f, -1000.0f, 16800.0f);
    CHECK(fabsf(out + 16800.0f) < 1e-3f);

    printf("[pi] 复位清空\n");

    pi_reset(&pi);
    CHECK(fabsf(pi.out) < 1e-9f && fabsf(pi.bias) < 1e-9f && fabsf(pi.last_bias) < 1e-9f);

    printf("[pi] 驻车清残余\n");

    pi_reset(&pi);
    pi.out = 5.0f;
    calls = 0;
    for (;;) {
        int done = pi_clear_output(&pi);
        calls++;
        if (done || calls > 100) {
            break;
        }
    }
    /* 5→4→3→2→1→0，第 4 次调用归零并返回 1 */
    CHECK(calls == 4);
    CHECK(fabsf(pi.out) < 1e-9f);

    pi_reset(&pi);
    pi.out = -5.0f;
    calls = 0;
    for (;;) {
        int done = pi_clear_output(&pi);
        calls++;
        if (done || calls > 100) {
            break;
        }
    }
    CHECK(calls == 4);
    CHECK(fabsf(pi.out) < 1e-9f);

    pi_reset(&pi);
    CHECK(pi_clear_output(&pi) == 1);
}

/* ======================= 安全判定 ======================= */

static void test_safety(void)
{
    printf("[safety] 运动允许的三个条件\n");

    uint8_t low = 0u;

    CHECK(safety_move_ok(1u, 1u, 0u) == 1u);
    CHECK(safety_move_ok(1u, 1u, 500u) == 1u);         /* 边界含等于 */
    CHECK(safety_move_ok(1u, 1u, 501u) == 0u);
    CHECK(safety_move_ok(0u, 1u, 0u) == 0u);           /* 欠压 */
    CHECK(safety_move_ok(1u, 0u, 0u) == 0u);           /* 开关断开 */

    printf("[safety] 电压滞回：低于 20 锁住，高于 21 才恢复\n");

    low = 0u;
    CHECK(safety_volt_ok(24.0f, &low) == 1u);
    CHECK(low == 0u);
    CHECK(safety_volt_ok(20.0f, &low) == 1u);          /* 边界含等于，不算欠压 */
    CHECK(safety_volt_ok(19.9f, &low) == 0u);          /* 欠压，锁住 */
    CHECK(low == 1u);
    CHECK(safety_volt_ok(20.5f, &low) == 0u);          /* 回到 20.5 仍不放行 */
    CHECK(low == 1u);
    CHECK(safety_volt_ok(21.0f, &low) == 0u);          /* 21.0 还不算“高于” */
    CHECK(safety_volt_ok(21.1f, &low) == 1u);          /* 过 21 才解除 */
    CHECK(low == 0u);
    CHECK(safety_volt_ok(20.5f, &low) == 1u);          /* 解除后 20.5 可以用 */

    printf("[safety] 刀片允许\n");

    CHECK(safety_blade_ok(1u, 1u) == 1u);
    CHECK(safety_blade_ok(0u, 1u) == 0u);
    CHECK(safety_blade_ok(1u, 0u) == 0u);
}

/* ======================= 刀片 ======================= */


static void test_blade(void)
{
    blade_t b;
    int i;

    printf("[blade] 刷新式:300ms 内不刷新就关\n");

    blade_init(&b);
    CHECK(blade_on(&b) == 0u);

    blade_step(&b, 10, 1u, 1u, 0u, 1u);
    CHECK(blade_on(&b) == 1u);

    for (i = 0; i < 29; i++) {
        blade_step(&b, 10, 0u, 0u, 0u, 1u);
    }
    CHECK(blade_on(&b) == 1u);

    blade_step(&b, 10, 0u, 0u, 0u, 1u);
    CHECK(blade_on(&b) == 1u);

    blade_step(&b, 10, 0u, 0u, 0u, 1u);
    CHECK(blade_on(&b) == 0u);

    printf("[blade] 刷新式:持续刷新就一直开\n");

    blade_init(&b);
    for (i = 0; i < 100; i++) {
        blade_step(&b, 10, 1u, 1u, 0u, 1u);
        CHECK(blade_on(&b) == 1u);
    }

    printf("[blade] 开关式:不刷新也保持(APP 按键)\n");

    blade_init(&b);
    blade_step(&b, 10, 1u, 1u, 1u, 1u);
    CHECK(blade_on(&b) == 1u);

    for (i = 0; i < 1000; i++) {
        blade_step(&b, 10, 0u, 0u, 1u, 1u);
    }
    CHECK(blade_on(&b) == 1u);

    blade_step(&b, 10, 1u, 0u, 1u, 1u);
    CHECK(blade_on(&b) == 0u);

    printf("[blade] 不允许时立刻关,恢复允许后不会自己开\n");

    blade_init(&b);
    blade_step(&b, 10, 1u, 1u, 1u, 1u);
    CHECK(blade_on(&b) == 1u);

    blade_step(&b, 10, 0u, 0u, 1u, 0u);
    CHECK(blade_on(&b) == 0u);

    for (i = 0; i < 100; i++) {
        blade_step(&b, 10, 0u, 0u, 1u, 1u);
    }
    CHECK(blade_on(&b) == 0u);

    blade_step(&b, 10, 1u, 1u, 1u, 1u);
    CHECK(blade_on(&b) == 1u);
}

/* ======================= 底盘状态机 ======================= */

/* 走 N 拍,一次性请求只在第一拍有效 */
static void ch_run(chassis_t *c, chassis_in_t *in, chassis_out_t *out, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        chassis_step(c, in, 10, out);
        in->cmd_new = 0u;
        in->estop_req = 0u;
        in->estop_release_req = 0u;
        in->selftest_req = 0u;
        in->servo_valid = 0u;
        in->blade_valid = 0u;
    }
}

static void ch_in_default(chassis_in_t *in)
{
    memset(in, 0, sizeof(*in));
    in->voltage = 24.0f;
    in->enable_switch = 1u;
}

/* 把自检脚本完整跑一遍。
 * 注意:前 settle 拍要喂从 0 爬到目标值的斜坡,模拟真实电机响应。
 * 第一版测试直接喂满值,掩盖了"加速段被算进判定"的 bug。 */
static void ch_selftest_run(chassis_t *c, chassis_in_t *in, chassis_out_t *out, int broken)
{
    static const int step_ticks[] = { 20, 60, 60, 60, 200, 20 };
    static const int settle[]     = {  0, 20, 20, 20,  0,  0 };
    int s, k;

    for (s = 0; s < 6; s++) {
        for (k = 0; k < step_ticks[s]; k++) {
            float ramp = 1.0f;

            if (k < settle[s]) {
                ramp = (float)k / (float)settle[s];
            }

            switch (s) {
            case 1:
                in->wheel_left = (broken ? -0.15f : 0.15f) * ramp;
                in->wheel_right = 0.15f * ramp;
                break;
            case 2:
                in->wheel_left = -0.15f * ramp;
                in->wheel_right = -0.15f * ramp;
                break;
            case 3:
                in->wheel_left = -0.087f * ramp;
                in->wheel_right = 0.087f * ramp;
                break;
            default:
                in->wheel_left = 0.0f;
                in->wheel_right = 0.0f;
                break;
            }

            chassis_step(c, in, 10, out);
            in->cmd_new = 0u;
            in->selftest_req = 0u;
        }
    }
}

static void test_chassis(void)
{
    static chassis_t c;
    static chassis_in_t in;
    static chassis_out_t out;

    printf("[chassis] 上电静默期内不动\n");

    chassis_init(&c);
    ch_in_default(&in);
    in.cmd_new = 1u;
    in.cmd_vx = 0.3f;
    ch_run(&c, &in, &out, 500);                    /* 5 秒 */
    CHECK(out.state == CHASSIS_INIT);
    CHECK(out.move_allowed == 0u);
    CHECK(out.vx == 0.0f);
    CHECK(out.blade_on == 0u);

    printf("[chassis] 静默期结束进自检,自检通过进 IDLE\n");

    ch_run(&c, &in, &out, 500);                    /* 累计 10 秒 */
    CHECK(out.state == CHASSIS_SELFTEST);

    ch_selftest_run(&c, &in, &out, 0);
    CHECK(out.state == CHASSIS_IDLE);
    CHECK(out.move_allowed == 1u);

    printf("[chassis] 自检失败进 FAULT 并锁存\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    CHECK(out.state == CHASSIS_SELFTEST);
    ch_selftest_run(&c, &in, &out, 1);             /* 有一个轮子反了 */
    CHECK(out.state == CHASSIS_FAULT);

    /* FAULT 之后发速度帧也不该动 */
    in.cmd_new = 1u;
    in.cmd_vx = 0.5f;
    ch_run(&c, &in, &out, 100);
    CHECK(out.state == CHASSIS_FAULT);
    CHECK(out.move_allowed == 0u);
    CHECK(out.vx == 0.0f);

    printf("[chassis] IDLE 收指令进 RUN\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    ch_selftest_run(&c, &in, &out, 0);
    CHECK(out.state == CHASSIS_IDLE);

    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    in.cmd_vz = 0.1f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);
    CHECK(out.move_allowed == 1u);
    CHECK(fabsf(out.vx - 0.4f) < 1e-6f);
    CHECK(fabsf(out.vz - 0.1f) < 1e-6f);

    printf("[chassis] 指令归零回 IDLE\n");

    in.cmd_new = 1u;
    in.cmd_vx = 0.0f;
    in.cmd_vz = 0.0f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_IDLE);

    printf("[chassis] 指令超时回 IDLE(500ms 仍在跑,510ms 超时)\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    ch_selftest_run(&c, &in, &out, 0);
    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);

    ch_run(&c, &in, &out, 50);                     /* 累计 500ms,边界含等于 */
    CHECK(out.state == CHASSIS_RUN);
    CHECK(out.move_allowed == 1u);

    ch_run(&c, &in, &out, 1);                      /* 510ms */
    CHECK(out.state == CHASSIS_IDLE);
    CHECK(out.move_allowed == 1u);                 /* IDLE 仍允许,只是速度为 0 */

    printf("[chassis] 欠压时不能进 RUN,已在 RUN 则退回 IDLE\n");

    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    in.voltage = 19.0f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_IDLE);

    in.voltage = 24.0f;
    in.cmd_new = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);

    in.voltage = 18.0f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_IDLE);
    CHECK(out.vx == 0.0f);

    printf("[chassis] 急停:任意状态进入,锁存,解除要开关闭合\n");

    in.voltage = 24.0f;
    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);

    in.estop_req = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_ESTOP);
    CHECK(out.move_allowed == 0u);
    CHECK(out.vx == 0.0f);

    /* 继续发速度帧,应该保持 ESTOP */
    in.cmd_new = 1u;
    ch_run(&c, &in, &out, 200);
    CHECK(out.state == CHASSIS_ESTOP);
    CHECK(out.move_allowed == 0u);

    /* 开关还断着,解除无效 */
    in.enable_switch = 0u;
    in.estop_release_req = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_ESTOP);

    /* 开关闭合后才能解除 */
    in.enable_switch = 1u;
    in.estop_release_req = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_IDLE);

    printf("[chassis] 解除后旧的指令不能把车重新带起来\n");

    ch_run(&c, &in, &out, 100);
    CHECK(out.state == CHASSIS_IDLE);
    CHECK(out.vx == 0.0f);

    printf("[chassis] 刀片只在 IDLE/RUN 允许,急停强制关\n");

    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    in.blade_valid = 1u;
    in.blade_cmd = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);
    CHECK(out.blade_on == 1u);

    in.estop_req = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_ESTOP);
    CHECK(out.blade_on == 0u);

    printf("[chassis] 同一拍收到急停和解除,急停优先\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    ch_selftest_run(&c, &in, &out, 0);
    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);

    in.estop_req = 1u;
    in.estop_release_req = 1u;          /* 两帧挤在同一拍里 */
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_ESTOP);  /* 不能被解除吃掉 */

    printf("[chassis] FAULT 是终态,急停和解除都绕不出去\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    ch_selftest_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_FAULT);

    in.estop_req = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_FAULT);  /* 不能变成 ESTOP */

    in.estop_release_req = 1u;
    in.enable_switch = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_FAULT);  /* 更不能借此回到 IDLE */

    printf("[chassis] 自检中途断使能开关,应中止回 IDLE 而不是 FAULT\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    CHECK(out.state == CHASSIS_SELFTEST);

    in.enable_switch = 0u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_IDLE);   /* 不是 FAULT,也不是继续自检 */

    /* 自检期间电机不应出力 */
    CHECK(out.vx == 0.0f);

    printf("[chassis] RUN 时触发自检:先停车,轮子停稳才真正开始\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    ch_selftest_run(&c, &in, &out, 0);
    CHECK(out.state == CHASSIS_IDLE);

    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    in.wheel_left = 0.4f;
    in.wheel_right = 0.4f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);

    /* 下发自检,此时车还在跑 */
    in.selftest_req = 1u;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_IDLE);   /* 先停车 */
    CHECK(out.vx == 0.0f);

    /* 轮子还在动,不能进自检 */
    ch_run(&c, &in, &out, 20);
    CHECK(out.state == CHASSIS_IDLE);

    /* 停稳之后才进 */
    in.wheel_left = 0.0f;
    in.wheel_right = 0.0f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_SELFTEST);

    printf("[chassis] 欠压恢复后不能自己接着跑,要新指令\n");

    chassis_init(&c);
    ch_in_default(&in);
    ch_run(&c, &in, &out, 1000);
    ch_selftest_run(&c, &in, &out, 0);
    CHECK(out.state == CHASSIS_IDLE);

    in.cmd_new = 1u;
    in.cmd_vx = 0.4f;
    ch_run(&c, &in, &out, 1);
    CHECK(out.state == CHASSIS_RUN);
}

/* ======================= 环形缓冲 ======================= */

static void test_ring(void)
{
    ring_t r;
    static uint8_t buf[16];
    uint8_t c;
    int i;

    printf("[ring] 基本读写\n");

    ring_init(&r, buf, 16);
    CHECK(ring_used(&r) == 0u);
    CHECK(ring_get(&r, &c) == 0u);

    CHECK(ring_put(&r, 0xA5u) == 1u);
    CHECK(ring_used(&r) == 1u);
    CHECK(ring_get(&r, &c) == 1u);
    CHECK(c == 0xA5u);
    CHECK(ring_used(&r) == 0u);

    printf("[ring] 先进先出\n");

    for (i = 0; i < 10; i++) {
        CHECK(ring_put(&r, (uint8_t)i) == 1u);
    }
    for (i = 0; i < 10; i++) {
        CHECK(ring_get(&r, &c) == 1u);
        CHECK(c == (uint8_t)i);
    }

    printf("[ring] 容量是 size-1,满了返回 0\n");

    ring_init(&r, buf, 16);
    for (i = 0; i < 15; i++) {
        CHECK(ring_put(&r, (uint8_t)i) == 1u);
    }
    CHECK(ring_used(&r) == 15u);
    CHECK(ring_put(&r, 0xFFu) == 0u);       /* 满 */
    CHECK(ring_used(&r) == 15u);

    printf("[ring] 回绕后仍然有序\n");

    for (i = 0; i < 8; i++) {
        CHECK(ring_get(&r, &c) == 1u);
    }
    CHECK(ring_used(&r) == 7u);
    for (i = 0; i < 8; i++) {                /* 再塞 8 个,越过物理末尾 */
        CHECK(ring_put(&r, (uint8_t)(0x80 + i)) == 1u);
    }
    CHECK(ring_used(&r) == 15u);
    for (i = 8; i < 15; i++) {               /* 先取出旧的 7 个 */
        CHECK(ring_get(&r, &c) == 1u);
        CHECK(c == (uint8_t)i);
    }
    for (i = 0; i < 8; i++) {
        CHECK(ring_get(&r, &c) == 1u);
        CHECK(c == (uint8_t)(0x80 + i));
    }
}

/* ======================= 速度限幅与平滑 ======================= */

static void test_speed_smooth(void)
{
    float v;
    int i;

    printf("[smooth] 每拍只走一步,到目标锁住\n");

    v = 0.0f;
    v = speed_smooth_step(v, 0.5f, 0.02f);
    CHECK(fabsf(v - 0.02f) < 1e-6f);

    for (i = 0; i < 24; i++) {
        v = speed_smooth_step(v, 0.5f, 0.02f);
    }
    CHECK(fabsf(v - 0.5f) < 1e-6f);          /* 25 拍到目标 */

    v = speed_smooth_step(v, 0.5f, 0.02f);
    CHECK(fabsf(v - 0.5f) < 1e-6f);          /* 不再变 */

    printf("[smooth] 反向与越界夹住\n");

    v = speed_smooth_step(0.0f, -0.5f, 0.02f);
    CHECK(fabsf(v + 0.02f) < 1e-6f);

    v = speed_smooth_step(0.49f, 0.5f, 0.1f);
    CHECK(fabsf(v - 0.5f) < 1e-6f);          /* 步进大于剩余距离 */

    v = speed_smooth_step(0.0f, 0.01f, 0.02f);
    CHECK(fabsf(v - 0.01f) < 1e-6f);

    v = speed_smooth_step(0.3f, 0.3f, 0.02f);
    CHECK(fabsf(v - 0.3f) < 1e-6f);          /* 已在目标 */

    printf("[smooth] 限幅\n");

    CHECK(fabsf(speed_limit(5.0f, 3.5f) - 3.5f) < 1e-6f);
    CHECK(fabsf(speed_limit(-5.0f, 3.5f) + 3.5f) < 1e-6f);
    CHECK(fabsf(speed_limit(1.2f, 3.5f) - 1.2f) < 1e-6f);
    CHECK(fabsf(speed_limit(3.5f, 3.5f) - 3.5f) < 1e-6f);
    CHECK(fabsf(speed_limit(-3.5f, 3.5f) + 3.5f) < 1e-6f);
    CHECK(fabsf(speed_limit(9.0f, -3.5f) - 3.5f) < 1e-6f);   /* limit 传负数也当正的 */
}

/* ======================= 摇杆命令守门 ======================= */

static void test_joy_guard(void)
{
    joy_guard_t j;
    uint32_t seq = 0;
    int i;

    printf("[joy] 松开时一律无效\n");

    joy_guard_init(&j);
    CHECK(joy_guard_step(&j, seq, 0u, 10u) == 0u);

    printf("[joy] 收到方向字节就生效,年龄清零\n");

    seq++;
    CHECK(joy_guard_step(&j, seq, 1u, 10u) == 1u);
    CHECK(joy_guard_age(&j) == 0u);

    printf("[joy] 没有新字节,年龄照涨,到点作废\n");

    /* 一路喂到刚好等于阈值:还有效 */
    for (i = 0; i < (int)(JOY_TIMEOUT_MS / 10u); i++) {
        CHECK(joy_guard_step(&j, seq, 1u, 10u) == 1u);
    }
    CHECK(joy_guard_age(&j) == JOY_TIMEOUT_MS);

    /* 再走一拍就超了 */
    CHECK(joy_guard_step(&j, seq, 1u, 10u) == 0u);

    printf("[joy] 作废之后再按一下能恢复\n");

    seq++;
    CHECK(joy_guard_step(&j, seq, 1u, 10u) == 1u);

    printf("[joy] 松开就把年龄清掉\n");

    CHECK(joy_guard_step(&j, seq, 0u, 10u) == 0u);
    CHECK(joy_guard_age(&j) == 0u);

    /* 松开后即使序号没变也不会又生效 */
    CHECK(joy_guard_step(&j, seq, 0u, 10u) == 0u);
    CHECK(joy_guard_step(&j, seq, 1u, 10u) == 0u);   /* 方向在但没新字节,且已 active=0 */

    printf("[joy] 序号回绕不出问题\n");

    joy_guard_init(&j);
    j.last_seq = 0xFFFFFFFFu;
    CHECK(joy_guard_step(&j, 0u, 2u, 10u) == 1u);    /* 0 != 0xFFFFFFFF,算新字节 */
}

/* ======================= 诊断翻页 ======================= */

static void test_diag(void)
{
    printf("[diag] 翻页到头回到第一页\n");

    CHECK(diag_page_next_of(0u) == 1u);
    CHECK(diag_page_next_of((uint8_t)(DIAG_PAGES - 1u)) == 0u);   /* 最后一页之后回 0 */

    printf("[diag] 页数改了也不会越界\n");

    {
        uint8_t page = 0u;
        int i;
        for (i = 0; i < 50; i++) {
            page = diag_page_next_of(page);
            CHECK(page < DIAG_PAGES);
        }
    }
}
/* ======================= 驻车释放 ======================= */

static void test_park(void)
{
    park_t p;

    printf("[park] 水平判定\n");

    /* 平放:ax=ay=0,az=1g */
    CHECK(park_is_flat(0, 0, 1671) == 1u);
    /* 前后倾 0.2g,约 11 度,不算平 */
    CHECK(park_is_flat(334, 0, 1671) == 0u);
    CHECK(park_is_flat(0, -334, 1671) == 0u);
    /* 侧倾一点点还算平 */
    CHECK(park_is_flat(100, 0, 1671) == 1u);
    /* 倒扣 */
    CHECK(park_is_flat(0, 0, -1671) == 0u);

    printf("[park] 平地静止 5 秒才释放\n");

    park_init(&p);
    CHECK(park_released(&p) == 0u);

    /* 4.9 秒,还没到 */
    {
        int i;
        for (i = 0; i < 489; i++) {
            park_step(&p, 1u, 1u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 0u);
        for (i = 0; i < 12; i++) {
            park_step(&p, 1u, 1u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 1u);
    }

    printf("[park] 不在平地不释放\n");

    park_init(&p);
    {
        int i;
        for (i = 0; i < 600; i++) {
            park_step(&p, 1u, 0u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 0u);
    }

    printf("[park] 没认到 IMU 不释放\n");

    park_init(&p);
    {
        int i;
        for (i = 0; i < 600; i++) {
            park_step(&p, 0u, 1u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 0u);
    }

    printf("[park] 有目标速度或有轮速就不算静止\n");

    park_init(&p);
    {
        int i;
        for (i = 0; i < 600; i++) {
            park_step(&p, 1u, 1u, 0.30f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 0u);
    }

    park_init(&p);
    {
        int i;
        for (i = 0; i < 600; i++) {
            park_step(&p, 1u, 1u, 0.0f, 0.30f, 10u);
        }
        CHECK(park_released(&p) == 0u);
    }

    printf("[park] 释放后一旦被推动就立刻收回\n");

    park_init(&p);
    {
        int i;
        for (i = 0; i < 600; i++) {
            park_step(&p, 1u, 1u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 1u);
        park_step(&p, 1u, 1u, 0.0f, 0.30f, 10u);
        CHECK(park_released(&p) == 0u);
    }

    printf("[park] 中途有一点动静就重新计时\n");

    park_init(&p);
    {
        int i;
        for (i = 0; i < 300; i++) {
            park_step(&p, 1u, 1u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 0u);
        park_step(&p, 1u, 1u, 0.2f, 0.0f, 10u);   /* 来了一条指令 */
        CHECK(park_released(&p) == 0u);
        for (i = 0; i < 300; i++) {
            park_step(&p, 1u, 1u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 0u);           /* 才 3 秒,不够 */
        for (i = 0; i < 220; i++) {
            park_step(&p, 1u, 1u, 0.0f, 0.0f, 10u);
        }
        CHECK(park_released(&p) == 1u);
    }
}

/* ======================= 按键判定 ======================= */

/* 以 100Hz 喂按键状态,返回这段时间内产生的事件数 */
static int click_run(clicker_t *c, uint8_t pressed, int ticks, click_event_t *last)
{
    int n = 0;
    int i;
    click_event_t e;

    for (i = 0; i < ticks; i++) {
        e = click_step(c, pressed, 10);
        if (e != CLICK_NONE) {
            n++;
            if (last) *last = e;
        }
    }
    return n;
}

static void test_click(void)
{
    clicker_t c;
    click_event_t e;
    int n;

    printf("[click] 抖一下不算单击\n");

    click_init(&c);
    n = click_run(&c, 1u, 2, NULL);      /* 按住 20ms,短于 30ms */
    n += click_run(&c, 0u, 40, &e);      /* 松开后等够窗口 */
    CHECK(n == 0);

    printf("[click] 正常单击\n");

    click_init(&c);
    n = click_run(&c, 1u, 5, NULL);      /* 50ms */
    n += click_run(&c, 0u, 1, NULL);
    CHECK(n == 0);                        /* 还要等满窗口 */
    n += click_run(&c, 0u, 32, &e);      /* 超过 300ms */
    CHECK(n == 1);
    CHECK(e == CLICK_SINGLE);

    printf("[click] 双击\n");

    click_init(&c);
    n = click_run(&c, 1u, 5, NULL);      /* 第一下 */
    n += click_run(&c, 0u, 3, NULL);     /* 松开 30ms */
    n += click_run(&c, 1u, 5, NULL);     /* 第二下 */
    n += click_run(&c, 0u, 1, &e);
    CHECK(n == 1);
    CHECK(e == CLICK_DOUBLE);

    printf("[click] 两次间隔太长就是两次单击\n");

    click_init(&c);
    n = click_run(&c, 1u, 5, NULL);
    n += click_run(&c, 0u, 40, &e);      /* 超过 300ms,判单击 */
    CHECK(n == 1);
    CHECK(e == CLICK_SINGLE);
    n = click_run(&c, 1u, 5, NULL);
    n += click_run(&c, 0u, 40, &e);
    CHECK(n == 1);
    CHECK(e == CLICK_SINGLE);

    printf("[click] 长按只报一次\n");

    click_init(&c);
    n = click_run(&c, 1u, 60, &e);       /* 按住 600ms */
    CHECK(n == 1);
    CHECK(e == CLICK_LONG);
    n = click_run(&c, 1u, 100, NULL);    /* 继续按住 */
    CHECK(n == 0);
    n = click_run(&c, 0u, 40, NULL);     /* 松手 */
    CHECK(n == 0);                        /* 松手不再补一个单击 */

    printf("[click] 松手后能重新开始\n");

    n = click_run(&c, 1u, 5, NULL);
    n += click_run(&c, 0u, 40, &e);
    CHECK(n == 1);
    CHECK(e == CLICK_SINGLE);
}

/* ======================= 发送缓冲与重同步边界 ======================= */

static void test_ring_free(void)
{
    static uint8_t buf[8];               /* 大小必须是 2 的幂 */
    ring_t r;
    uint8_t c;
    unsigned i;

    printf("[ring] 空闲量随进随出\n");

    ring_init(&r, buf, 8u);
    CHECK(ring_free(&r) == 7u);          /* 容量 = size-1,留一格区分满和空 */
    CHECK(ring_used(&r) == 0u);

    for (i = 0u; i < 7u; i++) {
        CHECK(ring_put(&r, (uint8_t)i) == 1u);
    }
    CHECK(ring_free(&r) == 0u);
    CHECK(ring_used(&r) == 7u);
    CHECK(ring_put(&r, 0xAAu) == 0u);   /* 满了 */

    CHECK(ring_get(&r, &c) == 1u);
    CHECK(ring_free(&r) == 1u);
    CHECK(ring_used(&r) == 6u);

    printf("[ring] 整帧放不下时 ring_free 能挡住\n");

    /* 模拟 uart_write 的判定:24 字节帧,缓冲只剩 7 格 */
    CHECK((uint32_t)ring_free(&r) < 24u);
    {
        uint8_t big[24];
        unsigned n = 0u;

        for (i = 0u; i < 24u; i++) big[i] = (uint8_t)i;

        /* 判定不通过就一个字节都不写 */
        if ((uint32_t)ring_free(&r) >= 24u) {
            for (i = 0u; i < 24u; i++) {
                if (ring_put(&r, big[i])) n++;
            }
        }
        CHECK(n == 0u);
        CHECK(ring_used(&r) == 6u);      /* 一个没动 */
    }
}

/* 重同步:喂一段"半截帧 + 完整帧",完整帧必须被解析出来 */
static void test_protocol_resync(void)
{
    static protocol_t proto;
    uint8_t a[PROTOCOL_FRAME_LEN];
    uint8_t b[PROTOCOL_FRAME_LEN];
    uint8_t stream[64];
    unsigned n;

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);

    printf("[resync] 坏 BCC 之后紧跟一条好帧\n");

    rec_reset();
    mk_frame(a, 0u, 500, 0, 0);
    a[9] ^= 0xFFu;                        /* 校验坏了 */
    mk_frame(b, 0u, 700, 0, 0);
    memcpy(stream, a, PROTOCOL_FRAME_LEN);
    memcpy(stream + PROTOCOL_FRAME_LEN, b, PROTOCOL_FRAME_LEN);
    feed_all(&proto, stream, PROTOCOL_FRAME_LEN * 2u);
    CHECK(g_rec.vel_calls == 1);          /* 只有好帧被接收 */
    CHECK(fabsf(g_rec.vx - 0.7f) < 1e-6f);
    CHECK(proto.bad_frame >= 1u);

    printf("[resync] 坏帧尾之后紧跟一条好帧\n");

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);
    rec_reset();
    mk_frame(a, 0u, 500, 0, 0);
    a[10] = 0x00u;                        /* 尾巴不对 */
    mk_frame(b, 0u, 900, 0, 0);
    memcpy(stream, a, PROTOCOL_FRAME_LEN);
    memcpy(stream + PROTOCOL_FRAME_LEN, b, PROTOCOL_FRAME_LEN);
    feed_all(&proto, stream, PROTOCOL_FRAME_LEN * 2u);
    CHECK(g_rec.vel_calls == 1);
    CHECK(fabsf(g_rec.vx - 0.9f) < 1e-6f);

    printf("[resync] 载荷里出现帧头字节也不能误判\n");

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);
    rec_reset();
    /* vx = 0x7B00,高字节正好等于帧头 0x7B */
    mk_frame(a, 0u, (int16_t)0x7B00, 0, 0);
    CHECK(a[3] == PROTOCOL_HEADER);
    feed_all(&proto, a, PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 1);
    CHECK(proto.bad_frame == 0u);

    printf("[resync] 半截帧 + 完整帧\n");

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);
    rec_reset();
    mk_frame(a, 0u, 500, 0, 0);
    mk_frame(b, 0u, 800, 0, 0);
    memcpy(stream, a, 5u);                /* 只发前 5 个字节 */
    memcpy(stream + 5u, b, PROTOCOL_FRAME_LEN);
    n = 5u + PROTOCOL_FRAME_LEN;
    feed_all(&proto, stream, n);
    CHECK(g_rec.vel_calls == 1);
    CHECK(fabsf(g_rec.vx - 0.8f) < 1e-6f);

    printf("[resync] 帧间夹垃圾字节\n");

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);
    rec_reset();
    mk_frame(b, 0u, 1200, 0, 0);
    stream[0] = 0x00u;
    stream[1] = 0xFFu;
    stream[2] = 0x7Bu;                    /* 假的帧头 */
    memcpy(stream + 3u, b, PROTOCOL_FRAME_LEN);
    feed_all(&proto, stream, 3u + PROTOCOL_FRAME_LEN);
    CHECK(g_rec.vel_calls == 1);
    CHECK(fabsf(g_rec.vx - 1.2f) < 1e-6f);

    printf("[resync] 两条好帧连着来,都要收到\n");

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);
    rec_reset();
    mk_frame(a, 0u, 300, 0, 0);
    mk_frame(b, 0u, 600, 0, 0);
    memcpy(stream, a, PROTOCOL_FRAME_LEN);
    memcpy(stream + PROTOCOL_FRAME_LEN, b, PROTOCOL_FRAME_LEN);
    feed_all(&proto, stream, PROTOCOL_FRAME_LEN * 2u);
    CHECK(g_rec.vel_calls == 2);
    CHECK(fabsf(g_rec.vx - 0.6f) < 1e-6f);   /* 后一条覆盖 */

    printf("[resync] 全是垃圾,不能误触发任何回调\n");

    protocol_init(&proto, &g_sink, PROTOCOL_SRC_ROS);
    rec_reset();
    for (n = 0u; n < 40u; n++) {
        stream[n] = (uint8_t)(n * 7u + 1u);   /* 不含 0x7B 的伪随机 */
        if (stream[n] == PROTOCOL_HEADER) stream[n] = 0x00u;
    }
    feed_all(&proto, stream, 40u);
    CHECK(g_rec.vel_calls == 0);
    CHECK(g_rec.estop_calls == 0);
    CHECK(g_rec.blade_calls == 0);
}

/* ======================= APP 调参帧与摇杆 ======================= */

static void feed_str(const char *s)
{
    while (*s) {
        app_param_feed((uint8_t)*s);
        s++;
    }
}

static void test_app_param(void)
{
    float vx, vz;
    uint16_t before;

    printf("[app] 摇杆八个方向\n");

    app_param_init();

    app_joystick(1u, 500u, &vx, &vz);
    CHECK(fabsf(vx - 0.5f) < 1e-5f && fabsf(vz) < 1e-5f);

    app_joystick(3u, 500u, &vx, &vz);
    CHECK(fabsf(vx) < 1e-5f && fabsf(vz + 0.78539816f) < 1e-5f);

    app_joystick(5u, 500u, &vx, &vz);
    CHECK(fabsf(vx + 0.5f) < 1e-5f && fabsf(vz) < 1e-5f);

    app_joystick(0u, 500u, &vx, &vz);
    CHECK(fabsf(vx) < 1e-5f && fabsf(vz) < 1e-5f);

    /* 速度基准变了,转向也跟着缩放 */
    app_joystick(8u, 1000u, &vx, &vz);
    CHECK(fabsf(vx - 1.0f) < 1e-5f);
    CHECK(fabsf(vz - 2.0f * 0.78539816f) < 1e-5f);

    printf("[app] 单参数帧与夹紧\n");

    app_param_init();
    CHECK(app_param_in_frame() == 0u);

    feed_str("{0:1000}");
    CHECK(app_param_in_frame() == 0u);
    CHECK(g_tune.rc_speed_mm == 1000u);

    feed_str("{0:99999}");
    CHECK(g_tune.rc_speed_mm == 3500u);

    feed_str("{1:500}");
    CHECK(g_tune.kp == 500u);

    feed_str("{2:250}");
    CHECK(g_tune.ki == 250u);

    feed_str("{3:50}");
    CHECK(g_tune.step_mm == 50u);

    feed_str("{8:80}");
    CHECK(g_tune.line_diff == 80u);

    feed_str("{8:500}");
    CHECK(g_tune.line_diff == 100u);

    printf("[app] 坏帧不改变参数\n");

    before = g_tune.kp;
    feed_str("{0}");
    feed_str("{}");
    feed_str("{zzz}");
    CHECK(g_tune.kp == before);

    printf("[app] 批量帧\n");

    app_param_init();
    feed_str("{#800:400:200:30:1:1:1:1:70}");
    CHECK(g_tune.rc_speed_mm == 800u);
    CHECK(g_tune.kp == 400u);
    CHECK(g_tune.ki == 200u);
    CHECK(g_tune.step_mm == 30u);
    CHECK(g_tune.line_diff == 70u);

    printf("[app] 帧内标志\n");

    app_param_init();
    app_param_feed('{');
    CHECK(app_param_in_frame() == 1u);
    app_param_feed('0');
    CHECK(app_param_in_frame() == 1u);
    app_param_feed('}');
    CHECK(app_param_in_frame() == 0u);
}

/* ======================= 看门狗监督 ======================= */

static void test_watchdog(void)
{
    uint8_t seen;

    printf("[wd] 位不齐不喂\n");

    seen = 0;
    CHECK(wd_core(&seen, WD_ALL) == 0u);
    CHECK(seen == 0u);

    seen = WD_BIT_CHASSIS | WD_BIT_TELEMETRY;
    CHECK(wd_core(&seen, WD_ALL) == 0u);
    CHECK(seen == (WD_BIT_CHASSIS | WD_BIT_TELEMETRY));   /* 没被清 */

    printf("[wd] 齐了才喂,喂完清位\n");

    seen = WD_ALL;
    CHECK(wd_core(&seen, WD_ALL) == 1u);
    CHECK(seen == 0u);

    /* 清掉之后马上再来一次,必须重新凑齐 */
    CHECK(wd_core(&seen, WD_ALL) == 0u);

    printf("[wd] 只盯需要的位\n");

    seen = WD_BIT_IMU | WD_BIT_HMI;
    CHECK(wd_core(&seen, WD_BIT_IMU | WD_BIT_HMI) == 1u);
    CHECK(seen == 0u);

    seen = WD_BIT_CHASSIS;
    CHECK(wd_core(&seen, WD_BIT_IMU | WD_BIT_HMI) == 0u);
    CHECK(seen == WD_BIT_CHASSIS);

    printf("[wd] 多余位不影响判定\n");

    seen = WD_ALL;
    CHECK(wd_core(&seen, WD_BIT_CHASSIS) == 1u);
}

/* ======================= 声光指示 ======================= */

static void test_indicator(void)
{
    printf("[ind] 每个状态的颜色\n");

    CHECK(ind_color(CHASSIS_INIT, 0u) == LED_WHITE);
    CHECK(ind_color(CHASSIS_IDLE, 0u) == LED_GREEN);
    CHECK(ind_color(CHASSIS_RUN, 0u) == LED_CYAN);
    CHECK(ind_color(CHASSIS_FAULT, 0u) == LED_RED);
    CHECK(ind_color(CHASSIS_FAULT, 1u) == LED_RED);      /* 故障不闪 */

    printf("[ind] 自检和急停是闪的\n");

    CHECK(ind_color(CHASSIS_SELFTEST, 1u) == LED_YELLOW);
    CHECK(ind_color(CHASSIS_SELFTEST, 0u) == LED_OFF);
    CHECK(ind_color(CHASSIS_ESTOP, 1u) == LED_PURPLE);
    CHECK(ind_color(CHASSIS_ESTOP, 0u) == LED_OFF);

    printf("[ind] 未知状态给白色,不是给黑屏\n");

    CHECK(ind_color(200u, 0u) == LED_WHITE);
}

/* ======================= 掉电参数 ======================= */

static void test_params(void)
{
    params_t p;
    params_t q;

    printf("[params] 默认值通过校验\n");

    params_defaults(&p);
    CHECK(params_check(&p) == 0u);
    CHECK(p.magic == PARAMS_MAGIC);
    CHECK(p.version == PARAMS_VERSION);

    printf("[params] 改一个字段 CRC 就对不上\n");

    q = p;
    q.kp = (uint16_t)(q.kp + 1u);
    CHECK(params_check(&q) == 3u);

    printf("[params] magic 和版本号不对\n");

    q = p;
    q.magic = 0x12345678u;
    CHECK(params_check(&q) == 1u);

    q = p;
    q.version = (uint16_t)(PARAMS_VERSION + 1u);
    CHECK(params_check(&q) == 2u);

    printf("[params] CRC 对但值荒唐也拒收\n");

    q = p;
    q.step_mm = 0u;
    q.crc = params_crc(&q);
    CHECK(params_check(&q) == 4u);

    q = p;
    q.rc_speed_mm = 9999u;
    q.crc = params_crc(&q);
    CHECK(params_check(&q) == 4u);

    q = p;
    q.avoid_mm = 50u;
    q.crc = params_crc(&q);
    CHECK(params_check(&q) == 4u);

    q = p;
    q.line_diff = 200u;
    q.crc = params_crc(&q);
    CHECK(params_check(&q) == 4u);

    printf("[params] CRC 对同样的数据稳定\n");

    q = p;
    CHECK(params_crc(&q) == params_crc(&p));
    CHECK(params_crc(&q) != 0u);
}

/* ======================= 超声波避障 ======================= */

static void test_avoid(void)
{
    avoid_t a;
    float vx, vz;
    float d[AVOID_SLOTS];
    int i;

    printf("[avoid] 关闭或无数据时放行\n");

    avoid_init(&a);
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 0.2f;
    vx = 0.5f;
    vz = 0.0f;
    avoid_step(&a, d, 1u, &vx, &vz);
    CHECK(fabsf(vx - 0.5f) < 1e-6f);

    a.enable = 1u;
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 0.0f;   /* 没模块,全 0 */
    vx = 0.5f;
    vz = 0.0f;
    avoid_step(&a, d, 1u, &vx, &vz);
    CHECK(fabsf(vx - 0.5f) < 1e-6f);

    vx = 0.5f;                                        /* 数据不新鲜 */
    vz = 0.0f;
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 0.2f;
    avoid_step(&a, d, 0u, &vx, &vz);
    CHECK(fabsf(vx - 0.5f) < 1e-6f);

    printf("[avoid] 连续 5 拍才确认\n");

    avoid_init(&a);
    a.enable = 1u;
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 1.0f;    /* 全程安全 */
    d[0] = 0.2f;                                       /* 左侧(A)近 */
    for (i = 0; i < 4; i++) {
        vx = 0.5f;
        vz = 0.0f;
        avoid_step(&a, d, 1u, &vx, &vz);
        CHECK(fabsf(vx - 0.5f) < 1e-6f);              /* 还没确认,不改写 */
    }
    vx = 0.5f;
    vz = 0.0f;
    avoid_step(&a, d, 1u, &vx, &vz);
    CHECK(fabsf(vx) < 1e-6f);                          /* 第 5 拍改写 */
    CHECK(vz < 0.0f);                                  /* 左侧近 → 往右转 */

    printf("[avoid] 哪边近就往反方向转\n");

    avoid_init(&a);                                    /* 左侧 AB 近 */
    a.enable = 1u;
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 1.0f;
    d[0] = 0.2f;
    d[1] = 0.2f;
    for (i = 0; i < 6; i++) {
        vx = 0.5f;
        vz = 0.0f;
        avoid_step(&a, d, 1u, &vx, &vz);
    }
    CHECK(vz < 0.0f);                                  /* 往右 */

    avoid_init(&a);                                    /* 右侧 CD 近 */
    a.enable = 1u;
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 1.0f;
    d[2] = 0.2f;
    d[3] = 0.2f;
    for (i = 0; i < 6; i++) {
        vx = 0.5f;
        vz = 0.0f;
        avoid_step(&a, d, 1u, &vx, &vz);
    }
    CHECK(vz > 0.0f);                                  /* 往左 */

    printf("[avoid] 要转过去的那侧也堵着,就不转\n");

    avoid_init(&a);                                    /* 四面都堵 */
    a.enable = 1u;
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 0.2f;
    for (i = 0; i < 6; i++) {
        vx = 0.5f;
        vz = 0.0f;
        avoid_step(&a, d, 1u, &vx, &vz);
    }
    CHECK(fabsf(vx) < 1e-6f);
    CHECK(fabsf(vz) < 1e-6f);                          /* 不硬拐 */

    printf("[avoid] 后退时不改写\n");

    avoid_init(&a);
    a.enable = 1u;
    for (i = 0; i < AVOID_SLOTS; i++) d[i] = 0.2f;
    for (i = 0; i < 6; i++) {
        vx = -0.5f;
        vz = 0.0f;
        avoid_step(&a, d, 1u, &vx, &vz);
    }
    CHECK(fabsf(vx + 0.5f) < 1e-6f);

    printf("[avoid] 距离可调且限幅\n");

    avoid_init(&a);
    CHECK(fabsf(a.start_m - AVOID_DEFAULT_M) < 1e-5f);

    avoid_nearer(&a, 0.1f);
    CHECK(fabsf(a.start_m - 0.6f) < 1e-5f);

    avoid_nearer(&a, -99.0f);
    CHECK(fabsf(a.start_m - AVOID_MIN_M) < 1e-5f);

    avoid_nearer(&a, 99.0f);
    CHECK(fabsf(a.start_m - AVOID_MAX_M) < 1e-5f);

    printf("[avoid] 开关\n");

    avoid_init(&a);
    CHECK(a.enable == 0u);
    avoid_toggle(&a);
    CHECK(a.enable == 1u);
    avoid_toggle(&a);
    CHECK(a.enable == 0u);
}


/* ======================= 滑动平均 ======================= */

static void test_filter(void)
{
    meanf_t f;
    int i;

    printf("[filter] 初始化后不被 0 拉低\n");

    meanf_init(&f, 24.0f);
    CHECK(fabsf(meanf_value(&f) - 24.0f) < 1e-4f);

    for (i = 0; i < 100; i++) {
        meanf_push(&f, 24.0f);
    }
    CHECK(fabsf(meanf_value(&f) - 24.0f) < 1e-4f);

    printf("[filter] 单点跌落后只被摊薄,不会瞬间掉下去\n");

    meanf_push(&f, 0.0f);
    /* 31 个 24 加 1 个 0,均值 = 744/32 */
    CHECK(fabsf(meanf_value(&f) - 23.25f) < 1e-3f);

    printf("[filter] 忘了调用 meanf_init 也不能从 0 爬\n");

    {
        meanf_t z;                    /* 不初始化,模拟调用方漏了 init */
        memset(&z, 0, sizeof(z));

        CHECK(fabsf(meanf_push(&z, 24.0f) - 24.0f) < 1e-4f);
        CHECK(fabsf(meanf_push(&z, 24.0f) - 24.0f) < 1e-4f);
        for (i = 0; i < 40; i++) {
            meanf_push(&z, 24.0f);
        }
        CHECK(fabsf(meanf_value(&z) - 24.0f) < 1e-4f);
    }

    printf("[filter] 连续输入后跟得上\n");

    for (i = 0; i < (int)MEANF_LEN; i++) {
        meanf_push(&f, 20.0f);
    }
    CHECK(fabsf(meanf_value(&f) - 20.0f) < 1e-3f);

}

int main(void)
{
    printf("=== mower-chassis 单元测试 ===\n\n");

    test_telemetry();
    printf("\n");
    test_protocol();
    printf("\n");
    test_kinematics();
    printf("\n");
    test_pi();
    printf("\n");
    test_safety();
    printf("\n");
    test_blade();
    printf("\n");
    test_chassis();
    printf("\n");
    test_ring();
    printf("\n");
    test_speed_smooth();
    printf("\n");
    test_filter();
    printf("\n");
    test_joy_guard();
    printf("\n");
    test_diag();
    printf("\n");
    test_park();
    printf("\n");
    test_click();
    printf("\n");
    test_protocol_resync();
    printf("\n");
    test_ring_free();
    printf("\n");
    test_app_param();
    printf("\n");
    test_avoid();
    printf("\n");
    test_params();
    printf("\n");
    test_watchdog();
    printf("\n");
    test_indicator();

    printf("\n=== 结果：%d 通过，%d 失败 ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}