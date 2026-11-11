// 电压判定和三个放行条件
#include "chassis_safety.h"

uint8_t safety_volt_ok(float voltage, uint8_t *latched)
{
    if (*latched) {
        //已经判过欠压,要明显高于阈值才放行
        if (voltage > SAFETY_VOLTAGE_RESUME) {
            *latched = 0;
        }
    } else if (voltage < SAFETY_VOLTAGE_MIN) {
        *latched = 1;
    }

    return (uint8_t)(*latched ? 0u : 1u);
}

uint8_t safety_move_ok(uint8_t volt_ok, uint8_t enable_switch, uint16_t cmd_age)
{
    if (!volt_ok) return 0;
    if (!enable_switch) return 0;
    if (cmd_age > SAFETY_CMD_AGE_MAX) return 0;
    return 1;
}

uint8_t safety_blade_ok(uint8_t state_ok, uint8_t volt_ok)
{
    if (!state_ok) return 0;
    if (!volt_ok) return 0;
    return 1;
}
