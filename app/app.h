// 应用层公共头。
#ifndef APP_H
#define APP_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define FW_VERSION_MAJOR        0
#define FW_VERSION_MINOR        1
#define FW_VERSION_PATCH        0

//任务优先级登记，新增任务先在这里占号
#define PRIO_HMI                1
#define PRIO_TELEMETRY          2
#define PRIO_BOOTSTRAP          2
#define PRIO_IMU                3
#define PRIO_CHASSIS            4

#endif
