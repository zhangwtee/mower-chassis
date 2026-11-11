/*
 * FreeRTOS 配置（割草机底盘）
 *
 * 只放"值"，不放解释性长文——每个值为什么是这个数，写在 docs/架构.md。
 * 改这里等同改实时性契约，改完必须重新量堆栈和抖动。
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include <stddef.h>

extern uint32_t SystemCoreClock;

/* ---- 调度器 ---- */
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_TICKLESS_IDLE                  0
#define configCPU_CLOCK_HZ                       (SystemCoreClock)
#define configTICK_RATE_HZ                       (1000)
#define configMAX_PRIORITIES                     (8)
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                  (12)
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1

/* ---- 同步原语（按需打开）---- */
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            0
#define configUSE_QUEUE_SETS                     0
#define configQUEUE_REGISTRY_SIZE                0

/* ---- 内存 ---- */
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configSUPPORT_STATIC_ALLOCATION          0
#define configTOTAL_HEAP_SIZE                    ((size_t)(24 * 1024))

/* ---- 钩子与调试 ---- */
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_IDLE_HOOK                                              1
#define configUSE_TICK_HOOK                      0
#define configUSE_APPLICATION_TASK_TAG           0
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

/* ---- 软件定时器：本工程不用，省 ~1 KB 堆和一个任务 ---- */
#define configUSE_TIMERS                         0

/* ---- 协程：不用 ---- */
#define configUSE_CO_ROUTINES                    0

/* ---- 可选 API ---- */
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark        1

/* ---- 中断优先级 ----
 * NVIC 分组为 Group_4（4 位抢占、0 位子优先级），与 app/main.c 里一致。
 * 调用 FreeRTOS API 的中断，抢占优先级必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY。
 */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS                          __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                          4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* ---- 异常向量名对接 ---- */
#define xPortPendSVHandler                       PendSV_Handler
#define vPortSVCHandler                          SVC_Handler

#endif /* FREERTOS_CONFIG_H */
