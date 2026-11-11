// 中断向量。异常一律进 fault_halt,别在这加逻辑
// 中断里只搬数据，解析和浮点放任务里。外设中断放在各自模块，不放这里。
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "fault.h"
#include "watchdog.h"
#include "task.h"

//异常都走 fault_halt:点红灯、屏上写 CFSR、然后停住
void NMI_Handler(void)
{
    fault_halt("NMI");
}

void HardFault_Handler(void)
{
    fault_halt("HARD");
}

void MemManage_Handler(void)
{
    fault_halt("MMU");
}

void BusFault_Handler(void)
{
    fault_halt("BUS");
}

void UsageFault_Handler(void)
{
    fault_halt("USAGE");
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        extern void xPortSysTickHandler(void);
        xPortSysTickHandler();
    }
}

//configCHECK_FOR_STACK_OVERFLOW=2 和 configUSE_MALLOC_FAILED_HOOK=1 要求提供这两个
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    fault_halt("STK");
}
void vApplicationMallocFailedHook(void)
{
    fault_halt("MEM");
}

//空闲任务里喂狗。任何任务霸占 CPU 或者死循环,这里就跑不到,看门狗复位
void vApplicationIdleHook(void)
{
    wd_poll();
}
