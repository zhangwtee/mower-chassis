// ICM20948。只为拿三轴加速度和角速度,陀螺 ±500dps,加速度 ±2g。
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

//ICM20948 的 WHO_AM_I 固定值
#define IMU_WIA_VALUE   0xEA

//返回 0 表示识别到芯片并初始化成功
uint8_t imu_init(void);

uint8_t imu_whoami(void);

//返回 0 成功,1 读失败(失败时 acc/gyr 填 0)
uint8_t imu_read(int16_t acc[3], int16_t gyr[3]);

#endif
