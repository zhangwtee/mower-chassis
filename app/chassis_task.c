// 100Hz 主环
#include <string.h>

#include "app.h"
#include "watchdog.h"
#include "diag.h"
#include "app_state.h"
#include "chassis.h"
#include "chassis_task.h"
#include "chassis_safety.h"
#include "kinematics.h"
#include "link.h"
#include "pi.h"
#include "speed_smooth.h"
#include "filter.h"
#include "app_param.h"
#include "click.h"
#include "park.h"
#include "joy_guard.h"
#include "key.h"
#include "imu_task.h"
#include "avoid.h"

#include "adc.h"
#include "encoder.h"
#include "motor.h"
#include "relay.h"
#include "servo.h"

#define TASK_HZ         100
#define STEP_MS         10
#define TRACK_M         0.435f
#define CPR             54000.0f
#define CIRC_M          0.5654867f
#define KP              300.0f
#define KI              300.0f
#define SPEED_STEP      0.02f   //每拍最大变化,100Hz 下 2 m/s^2
#define SPEED_MAX       3.5f    //上限
#define SERVO_SLEW_US   40      //舵机每拍最多动多少 us

static chassis_t s_ch;
static pi_t s_pi_l;
static pi_t s_pi_r;
static float s_sx;
static float s_sz;
static clicker_t s_click;
static park_t s_park;
static joy_guard_t s_joy;
static uint16_t s_servo = 1500;
static meanf_t s_vf;

void chassis_ctx_init(void)
{
    click_init(&s_click);
    park_init(&s_park);
    joy_guard_init(&s_joy);
    chassis_init(&s_ch);
    pi_init(&s_pi_l, KP, KI);
    pi_init(&s_pi_r, KP, KI);
    s_sx = 0.0f;
    s_sz = 0.0f;

    //用第一次采样把滑动窗口填满。不填的话前 32 拍是在跟 0 求平均,
    //电压会从 0 爬到真值,这期间上位机看到的是"电池没电"
    meanf_init(&s_vf, adc_battery_volt());
}

void chassis_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    TickType_t prev = xTaskGetTickCount();
    uint16_t dt;
    click_event_t click;
    TickType_t now;
    chassis_in_t in;
    chassis_out_t out;
    uint8_t can_drive;

    (void)arg;

    for (;;) {

        wd_kick(WD_BIT_CHASSIS);

        diag_kick(DIAG_CHASSIS);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(STEP_MS));

        now = xTaskGetTickCount();
        dt = (uint16_t)(now - prev);
        prev = now;

        //卡了这么久(写 Flash 要关中断 250ms),编码器计数是跨整段累积的,
        //按 10ms 折算会把速度放大几倍。丢掉这一拍,清计数和 PI,重新起算
        if (dt > 30u) {
            (void)encoder_left();
            (void)encoder_right();
            pi_reset(&s_pi_l);
            pi_reset(&s_pi_r);
            s_sx = 0.0f;
            s_sz = 0.0f;
            motor_set(0, 0);
            continue;
        }
        if (dt == 0u) {
            dt = STEP_MS;
        }

        link_poll();
        link_poll_tick();

        memset(&in, 0, sizeof(in));
        //电压抖会让低压判定来回跳,先压一下再进状态机
        in.voltage = meanf_push(&s_vf, adc_battery_volt());
        in.enable_switch = enable_switch_ok();
        in.estop_req = link_events()->estop_req;
        in.estop_release_req = link_events()->estop_release_req;
        in.selftest_req = link_events()->selftest_req;
        if (joy_guard_step(&s_joy, g_app_dir_seq, g_app_dir, dt)) {
            //蓝牙摇杆按住期间由 APP 接管,松开后交回上位机
            app_joystick(g_app_dir, g_tune.rc_speed_mm, &in.cmd_vx, &in.cmd_vz);
            in.cmd_new = 1u;
        } else {
            in.cmd_new = link_events()->cmd_new;
            in.cmd_vx = link_events()->cmd_vx;
            in.cmd_vz = link_events()->cmd_vz;
        }
        in.blade_valid = link_events()->blade_valid;
        in.blade_cmd = link_events()->blade_cmd;
        in.blade_sticky = link_events()->blade_sticky;
        in.servo_valid = link_events()->servo_valid;
        in.servo_us = link_events()->servo_us;

        in.wheel_left = kinematics_counts_to_mps(encoder_left(), CPR, CIRC_M, TASK_HZ);
        in.wheel_right = kinematics_counts_to_mps(encoder_right(), CPR, CIRC_M, TASK_HZ);

        pi_set_gains(&s_pi_l, (float)g_tune.kp, (float)g_tune.ki);
        pi_set_gains(&s_pi_r, (float)g_tune.kp, (float)g_tune.ki);

        chassis_step(&s_ch, &in, dt, &out);
        link_clear_once();

        //用户按键:单击/双击/长按。双击重新标定陀螺零偏,只在停车时接受
        click = click_step(&s_click, key_pressed(), dt);
        if (click == CLICK_DOUBLE) {
            //双击重标定陀螺零偏,只在停车时接受
            if (s_ch.state == CHASSIS_IDLE) {
                imu_cal_restart();
            }
        } else if (click == CLICK_LONG) {
            //长按翻页,给第二页诊断信息用
            diag_page_next();
        }

        //IDLE 也要保持闭环:目标速度 0 由 PI 顶住,斜坡上才不溜车。
        //急停、故障、上电阶段才真正断电
        can_drive = (uint8_t)(out.state == CHASSIS_IDLE ||
                              out.state == CHASSIS_RUN ||
                              out.state == CHASSIS_SELFTEST);
        //驻车释放:平地静止 5 秒后放掉保持电流。斜坡、没 IMU、有目标速度时不释放
        if (out.state == CHASSIS_IDLE) {
            park_step(&s_park, g_state.imu_ok,
                      park_is_flat(g_state.accel[0], g_state.accel[1], g_state.accel[2]),
                      out.vx,
                      (in.wheel_left + in.wheel_right) * 0.5f,
                      dt);
        } else {
            park_init(&s_park);
    joy_guard_init(&s_joy);
        }

        if (can_drive && !park_released(&s_park)) {
            float tl, tr;
            float vx = out.vx;
            float vz = out.vz;
            float step = (float)g_tune.step_mm / 1000.0f;
            float dist[AVOID_SLOTS];
            uint8_t i;

            //超声波数据不新鲜时直接放行,不会因为没模块就卡住不动
            for (i = 0; i < AVOID_SLOTS; i++) {
                dist[i] = (float)g_state.dist_mm[i] / 1000.0f;
            }
            avoid_step(&g_avoid, dist, link_dist_valid(), &vx, &vz);

            //先限幅再平滑:一步跳变会让 PI 打满占空比,轮子窜、打滑、电流冲击
            s_sx = speed_smooth_step(s_sx, speed_limit(vx, SPEED_MAX), step);
            s_sz = speed_smooth_step(s_sz, speed_limit(vz, SPEED_MAX), step);

            kinematics_inverse(s_sx, s_sz, TRACK_M, &tl, &tr);

            //单轮速度可能比 vx 大,逆解之后还要再限一次
            tl = speed_limit(tl, SPEED_MAX);
            tr = speed_limit(tr, SPEED_MAX);
            motor_set((int16_t)pi_update(&s_pi_l, in.wheel_left, tl, (float)MOTOR_FULL_DUTY),
                      (int16_t)pi_update(&s_pi_r, in.wheel_right, tr, (float)MOTOR_FULL_DUTY));
        } else {
            s_sx = 0.0f;
            s_sz = 0.0f;
            pi_reset(&s_pi_l);
            pi_reset(&s_pi_r);
            motor_set(0, 0);
        }

        //舵机平滑:每拍最多动 SERVO_SLEW_US,别一步跳过去。
        //自检的扫描是 20us 一拍,这里 40us,不会拖慢它
        if (s_servo < out.servo_us) {
            uint16_t next = (uint16_t)(s_servo + SERVO_SLEW_US);
            s_servo = (next > out.servo_us) ? out.servo_us : next;
        } else if (s_servo > out.servo_us) {
            uint16_t next = (uint16_t)(out.servo_us + SERVO_SLEW_US);
            s_servo = (next > s_servo) ? out.servo_us : (uint16_t)(s_servo - SERVO_SLEW_US);
        }
        servo_set(s_servo);
        relay_set(out.blade_on);

        g_state.voltage = in.voltage;
        g_state.wheel_left = in.wheel_left;
        g_state.wheel_right = in.wheel_right;
        kinematics_forward(in.wheel_left, in.wheel_right, TRACK_M, &g_state.vx, &g_state.vz);
        g_state.flag_stop = (uint8_t)(can_drive ? 0 : 1);
        g_state.chassis_state = (uint8_t)out.state;
        g_state.enable_switch = in.enable_switch;
        g_state.cmd_age = s_ch.cmd_age;
    }
}
