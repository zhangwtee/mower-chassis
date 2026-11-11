// 两轮差速运动学。纯数学，不碰硬件。
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

//逆解：车体 Vx/Vz 换算左右轮线速度
void kinematics_inverse(float vx, float vz, float track_m,
                        float *left_mps, float *right_mps);

//正解：左右轮线速度换算车体 Vx/Vz
void kinematics_forward(float left_mps, float right_mps, float track_m,
                        float *vx, float *vz);

//编码器计数换算轮速。cpr 是电机转一圈的计数，含减速比和倍频
float kinematics_counts_to_mps(int16_t counts, float cpr, float circ_m, uint32_t rate_hz);

#endif
