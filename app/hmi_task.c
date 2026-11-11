// OLED。四页,长按用户键翻
#include "app.h"
#include "watchdog.h"
#include "diag.h"
#include "uart.h"
#include "link.h"
#include "app_state.h"
#include "hmi_task.h"

#include "app_param.h"
#include "avoid.h"
#include "params.h"
#include "indicator.h"
#include "chassis.h"
#include "oled.h"

static const char *state_name(uint8_t s)
{
    switch (s) {
    case CHASSIS_INIT:
        return "INIT";
    case CHASSIS_IDLE:
        return "IDLE";
    case CHASSIS_RUN:
        return "RUN ";
    case CHASSIS_SELFTEST:
        return "TEST";
    case CHASSIS_ESTOP:
        return "STOP";
    case CHASSIS_FAULT:
        return "FAIL";
    default:
        return "?   ";
    }
}

static uint8_t s_sec;

void hmi_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();

    (void)arg;

    for (;;) {

        wd_kick(WD_BIT_HMI);

        diag_kick(DIAG_HMI);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));

        //保存请求只在停车时执行:擦写要 250ms,期间中断全关,车不能在这种状态下跑
        if (g_param_save_req && g_state.chassis_state == CHASSIS_IDLE) {
            params_t p;

            p.rc_speed_mm = g_tune.rc_speed_mm;
            p.kp = g_tune.kp;
            p.ki = g_tune.ki;
            p.step_mm = g_tune.step_mm;
            p.line_diff = g_tune.line_diff;
            p.avoid_enable = g_avoid.enable;
            p.avoid_mm = (uint16_t)(g_avoid.start_m * 1000.0f);

            if (params_save(&p) == 0) {
                g_param_save_req = 0;
            }
        }

        //声光指示,和屏一起刷
        indicator_step((uint8_t)g_state.chassis_state);

        //10Hz 数到 10 就是 1 秒
        if (++s_sec >= 10u) {
            s_sec = 0;
            diag_tick_1s();
        }

        oled_clear();

        if (diag_page() == 1u) {
            //第二页:栈水位和运行时长。上机时重点看四个数,剩不到四分之一就要加栈
            oled_str(0, 0, "STK words");
            oled_str(0, 1, "CH");
            oled_int(18, 1, diag_stack(DIAG_CHASSIS));
            oled_str(54, 1, "TE");
            oled_int(72, 1, diag_stack(DIAG_TELEMETRY));
            oled_str(0, 2, "IM");
            oled_int(18, 2, diag_stack(DIAG_IMU));
            oled_str(54, 2, "HM");
            oled_int(72, 2, diag_stack(DIAG_HMI));
            oled_str(0, 3, "UP");
            oled_int(18, 3, (int32_t)diag_uptime_s());
            oled_str(66, 3, "D");
            oled_int(78, 3, (int32_t)uart_tx_drop());
            oled_str(96, 3, "B");
            oled_int(108, 3, (int32_t)link_bad_frames());

            oled_flush_row();
            continue;
        }

        if (diag_page() == 2u) {
            //第三页:安全链。重点是 AGE —— 上位机停发之后它应该一路涨到 500 然后车停
            oled_str(0, 0, "SAFETY");
            oled_str(0, 1, "AGE");
            oled_int(24, 1, (int32_t)g_state.cmd_age);
            oled_str(72, 1, "ms");
            oled_str(0, 2, "U");
            oled_fixed(12, 2, (int32_t)(g_state.voltage * 10.0f), 1);
            oled_str(60, 2, "EN");
            oled_int(78, 2, g_state.enable_switch);
            oled_str(0, 3, "FS");
            oled_int(18, 3, g_state.flag_stop);

            oled_flush_row();
            continue;
        }

        //第一行:状态和 IMU 是否认到
        oled_str(0, 0, state_name(g_state.chassis_state));
        oled_str(30, 0, "IMU");
        oled_str(54, 0, g_state.imu_ok ? "OK" : "--");

        //第二行:电池电压
        oled_str(0, 1, "U");
        oled_fixed(12, 1, (int32_t)(g_state.voltage * 10.0f), 1);

        //第三行:左右轮速 mm/s
        oled_str(0, 2, "L");
        oled_int(12, 2, (int32_t)(g_state.wheel_left * 1000.0f));
        oled_str(66, 2, "R");
        oled_int(78, 2, (int32_t)(g_state.wheel_right * 1000.0f));

        //第四行:失能标志
        oled_str(0, 3, "FS");
        oled_int(18, 3, g_state.flag_stop);

        oled_flush_row();
    }
}
