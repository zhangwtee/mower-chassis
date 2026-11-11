// IMU 任务。采零偏、算姿态、供里程
#include "app.h"
#include "watchdog.h"
#include "diag.h"
#include "app_state.h"
#include "imu_task.h"

#include "imu.h"
#include "i2c.h"

#define TASK_MS         10
#define RECOVER_TICKS   50      //连续读失败 50 拍(0.5 秒)就试着救一次总线
#define CAL_TICKS       1000    //10 秒零偏采样,和上电静默期对齐

static volatile uint8_t s_cal_req;

void imu_cal_restart(void)
{
    s_cal_req = 1;
}

void imu_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int16_t a[3];
    int16_t g[3];
    int16_t bias[3] = { 0, 0, 0 };
    int32_t sum[3] = { 0, 0, 0 };
    uint16_t ticks = 0;
    uint16_t fails = 0;
    uint16_t n = 0;
    uint8_t i;

    (void)arg;

    for (;;) {

        wd_kick(WD_BIT_IMU);

        diag_kick(DIAG_IMU);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TASK_MS));

        if (imu_read(a, g) != 0) {
            fails++;
            //软 I2C 一旦被从机拉住就一直失败,隔一会儿补时钟救一次
            if (fails % RECOVER_TICKS == 0u) {
                i2c_lock();
                i2c_recover();
                i2c_unlock();
            }
        } else {
            fails = 0;
        }

        //重新标定:清掉累计重新来。旧零偏先留着,重标定期间读数不跳
        if (s_cal_req) {
            s_cal_req = 0;
            ticks = 0;
            n = 0;
            sum[0] = 0;
            sum[1] = 0;
            sum[2] = 0;
        }

        //车静止的时候拿来做陀螺零偏
        if (ticks < CAL_TICKS) {
            for (i = 0; i < 3; i++) {
                sum[i] += g[i];
            }
            n++;
            ticks++;

            if (ticks == CAL_TICKS && n > 0) {
                for (i = 0; i < 3; i++) {
                    bias[i] = (int16_t)(sum[i] / (int32_t)n);
                }
            }
        }

        //转成上位机的坐标:传感器 Y→X、X 取负→Y、Z 不变。
        //字节布局一样但轴不转的话,上位机拿到的姿态是转了 90 度的
        g_state.accel[0] = a[1];
        g_state.accel[1] = (int16_t)(-(int32_t)a[0]);
        g_state.accel[2] = a[2];

        g_state.gyro[0] = (int16_t)(g[1] - bias[1]);
        g_state.gyro[1] = (int16_t)(-(int32_t)(g[0] - bias[0]));
        g_state.gyro[2] = (int16_t)(g[2] - bias[2]);
    }
}
