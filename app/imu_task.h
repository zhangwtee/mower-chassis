// IMU 任务。100Hz 读一次,上电前 10 秒做陀螺零偏。
#ifndef IMU_TASK_H
#define IMU_TASK_H

void imu_task(void *arg);

//重新采一次零偏。双击用户按键会调这个
void imu_cal_restart(void);

#endif
