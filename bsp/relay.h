// 刀片继电器。PD14,高电平吸合。
#ifndef RELAY_H
#define RELAY_H

#include <stdint.h>

void relay_init(void);

void relay_set(uint8_t on);

#endif
