#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* STM32F407 当前系统时钟为 168 MHz */
#define configCPU_CLOCK_HZ                      168000000UL

/* FreeRTOS 每 1 ms 运行一次系统节拍 */
#define configTICK_RATE_HZ                      1000UL

/* 调度器 */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

/* 任务参数 */
#define configMAX_PRIORITIES                    6
#define configMINIMAL_STACK_SIZE                256
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0

/* 使用静态内存 */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION       0

/* 暂时不用的内核功能 */
#define configUSE_MUTEXES                       0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_TIMERS                        0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_NEWLIB_REENTRANT              0
#define configCHECK_FOR_STACK_OVERFLOW          0

/* 任务通知 */
#define configUSE_TASK_NOTIFICATIONS            1

/* 允许使用的 API */
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

/* Cortex-M4 中断优先级 */
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY         (15U << 4U)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (5U << 4U)

/* FreeRTOS 异常入口映射 */
// #define vPortSVCHandler                         SVC_Handler
// #define xPortPendSVHandler                      PendSV_Handler

/* 暂时停在死循环，最后再接 VOFA 错误码 */
#define configASSERT(condition) \
    do { \
        if (!(condition)) { \
            for (;;) {} \
        } \
    } while (0)

#endif