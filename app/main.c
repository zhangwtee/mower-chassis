// 入口。外设、任务、调度器,就这些
#include <string.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
#include "app_state.h"
#include "chassis.h"
#include "chassis_task.h"
#include "imu_task.h"
#include "hmi_task.h"
#include "app_key.h"
#include "link.h"
#include "telemetry_task.h"

#include "i2c.h"
#include "imu.h"
#include "adc.h"
#include "encoder.h"
#include "motor.h"
#include "relay.h"
#include "servo.h"
#include "oled.h"
#include "can.h"
#include "avoid.h"
#include "app_param.h"
#include "params.h"
#include "iwdg.h"
#include "key.h"
#include "led.h"
#include "buzzer.h"
#include "indicator.h"
#include "watchdog.h"
#include "delay.h"
#include "uart.h"

app_state_t g_state;


static void hw_init(void)
{
    //和 FreeRTOSConfig.h 里 configPRIO_BITS=4 对应
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init();

    //先起传感器和执行器,电机默认停
    motor_init();
    encoder_init();
    servo_init();
    relay_init();
    adc_init();

    memset(&g_state, 0, sizeof(g_state));

    //IMU 认不到时不复位,先按 0 跑,免得没有板子就起不来
    i2c_init();
    oled_init();
    if (imu_whoami() == IMU_WIA_VALUE) {
        g_state.imu_ok = (uint8_t)(imu_init() == 0);
    }

    key_init();
    led_init();
    buzzer_init();
    indicator_init();

    can_init();
    uart_init();
    link_init();

    //蓝牙 APP 只走按键,不接管运动
    app_key_init();
    uart4_set_rx(app_key_feed);

    avoid_init(&g_avoid);
    //读掉电参数,读不到或者校验不过就沿用默认值
    {
        params_t p;

        if (params_load(&p) == 0) {
            g_tune.rc_speed_mm = p.rc_speed_mm;
            g_tune.kp = p.kp;
            g_tune.ki = p.ki;
            g_tune.step_mm = p.step_mm;
            g_tune.line_diff = p.line_diff;
            g_avoid.enable = (uint8_t)p.avoid_enable;
            g_avoid.start_m = (float)p.avoid_mm / 1000.0f;
        }
    }
    chassis_ctx_init();

    //硬件全起来了再开狗。USE_IWDG 为 0 时这里是空调用
    wd_init();
    iwdg_init(2000u);
}

int main(void)
{
    hw_init();

    if (xTaskCreate(chassis_task, "chassis", 512, NULL, PRIO_CHASSIS, NULL) != pdPASS) {
        for (;;) {
        }
    }
    if (xTaskCreate(telemetry_task, "telem", 384, NULL, PRIO_TELEMETRY, NULL) != pdPASS) {
        for (;;) {
        }
    }
    if (xTaskCreate(imu_task, "imu", 384, NULL, PRIO_IMU, NULL) != pdPASS) {
        for (;;) {
        }
    }
    if (xTaskCreate(hmi_task, "hmi", 384, NULL, PRIO_HMI, NULL) != pdPASS) {
        for (;;) {
        }
    }

    vTaskStartScheduler();

    for (;;) {
    }
}
