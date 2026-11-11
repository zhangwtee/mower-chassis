// ICM20948。V1.1 的板子用它,V1.0 是 MPU6050
#include "stm32f4xx.h"
#include "imu.h"
#include "i2c.h"
#include "delay.h"

#define IMU_ADDR        0xD0

#define REG_WIA         0x00
#define VAL_WIA         0xEA

#define REG_PWR_MGMT_1  0x06
#define REG_BANK_SEL    0x7F

#define BANK0           0x00
#define BANK2           0x20

//bank2
#define REG_GYRO_SMPLRT_DIV     0x00
#define REG_GYRO_CONFIG_1       0x01
#define REG_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ACCEL_CONFIG        0x14

#define GYRO_CFG_500DPS     0x33    //DLPF 开,±500dps
#define ACCEL_CFG_2G        0x31    //DLPF 开,±2g

//bank0
#define REG_ACCEL_XOUT_H    0x2D

static uint8_t bank(uint8_t b)
{
    return i2c_write_reg(IMU_ADDR, REG_BANK_SEL, b);
}

uint8_t imu_whoami(void)
{
    uint8_t v = 0;

    if (bank(BANK0) != 0) return 0;
    if (i2c_read_regs(IMU_ADDR, REG_WIA, &v, 1) != 0) return 0;

    return v;
}

uint8_t imu_init(void)
{
    //唤醒并选自动时钟
    if (bank(BANK0) != 0) return 1;
    if (i2c_write_reg(IMU_ADDR, REG_PWR_MGMT_1, 0x01) != 0) return 2;
    delay_ms(10);

    //采样率与量程
    if (bank(BANK2) != 0) return 3;
    if (i2c_write_reg(IMU_ADDR, REG_GYRO_SMPLRT_DIV, 0x07) != 0) return 4;
    if (i2c_write_reg(IMU_ADDR, REG_GYRO_CONFIG_1, GYRO_CFG_500DPS) != 0) return 5;
    if (i2c_write_reg(IMU_ADDR, REG_ACCEL_SMPLRT_DIV_2, 0x07) != 0) return 6;
    if (i2c_write_reg(IMU_ADDR, REG_ACCEL_CONFIG, ACCEL_CFG_2G) != 0) return 7;

    if (bank(BANK0) != 0) return 8;

    return 0;
}

uint8_t imu_read(int16_t acc[3], int16_t gyr[3])
{
    uint8_t b[12];
    uint8_t i;
    uint8_t err;

    i2c_lock();
    err = i2c_read_regs(IMU_ADDR, REG_ACCEL_XOUT_H, b, 12);
    i2c_unlock();

    if (err != 0) {
        for (i = 0; i < 3; i++) {
            acc[i] = 0;
            gyr[i] = 0;
        }
        return 1;
    }

    //加速度在前,角速度在后,都是大端
    for (i = 0; i < 3; i++) {
        acc[i] = (int16_t)(((uint16_t)b[i * 2] << 8) | b[i * 2 + 1]);
        gyr[i] = (int16_t)(((uint16_t)b[6 + i * 2] << 8) | b[6 + i * 2 + 1]);
    }

    return 0;
}
