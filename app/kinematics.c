// 两轮差速的正解和逆解,纯函数
#include "kinematics.h"

void kinematics_inverse(float vx, float vz, float track_m,
                        float *left_mps, float *right_mps)
{
    float half = vz * track_m * 0.5f;

    *left_mps = vx - half;
    *right_mps = vx + half;
}

void kinematics_forward(float left_mps, float right_mps, float track_m,
                        float *vx, float *vz)
{
    *vx = (left_mps + right_mps) * 0.5f;
    *vz = (right_mps - left_mps) / track_m;
}

float kinematics_counts_to_mps(int16_t counts, float cpr, float circ_m, uint32_t rate_hz)
{
    return (float)counts * (float)rate_hz / cpr * circ_m;
}
