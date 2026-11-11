// 蜂鸣器。PD11,推挽。
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

//高电平响。实机是反的就把这个改成 0
#define BUZZER_ON_LEVEL     1

void buzzer_init(void);

void buzzer_set(uint8_t on);

#endif
