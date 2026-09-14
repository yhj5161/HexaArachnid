#ifndef __FREERTOS_CONFIG_H
#define __FREERTOS_CONFIG_H

/*
 * FreeRTOSConfig.h — HexaArachnid (STM32F407VE, Cortex-M4F, GCC)
 *
 * 内核版本：FreeRTOS-Kernel V11.1.0（目录 FreeRTOS/）
 * 移植层  ：portable/GCC/ARM_CM4F（硬件 FPU）
 * 内存管理：heap_4（支持 pvPortMalloc/vPortFree，带相邻块合并）
 *
 * 时钟约定：
 * - HCLK = 168MHz（SystemCoreClock 由 system_stm32f4xx.c 维护）
 * - SysTick 被内核占用作为 RTOS 节拍（1kHz / 1ms），
 *   裸机 Delay_us/Delay_ms 已改用 DWT->CYCCNT，二者互不干扰。
 *
 * 中断优先级约定（F407 为 4bit 优先级，0~15，数值越小越紧急）：
 * - 优先级 0~4：高于系统调用上限，ISR 内【禁止】调用任何 FreeRTOS API
 *   （含 FromISR 系列）。本工程的 TIM1/2/3 控制节拍、MPU6050 EXTI、
 *   USART 均在此范围 —— 它们只置标志位，由任务消费，时序不受内核影响。
 * - 优先级 5~15：可安全调用 xQueueSendFromISR / xTaskNotifyFromISR 等。
 *   需要"中断唤醒任务"的新外设中断必须配置在这个范围。
 */

#include <stdint.h>

/* 由 system_stm32f4xx.c 维护，启动文件 SystemInit() 已配置为 168MHz。 */
extern uint32_t SystemCoreClock;

/* ---------------- 内核基础配置 ---------------- */
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configUSE_TICKLESS_IDLE                  0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ( ( TickType_t ) 1000U )
#define configMAX_PRIORITIES                     7
#define configMINIMAL_STACK_SIZE                 ( ( uint16_t ) 128 )
#define configMAX_TASK_NAME_LEN                  16
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TASK_NOTIFICATIONS             1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES    1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_QUEUE_SETS                     0
#define configUSE_APPLICATION_TASK_TAG           0
#define configUSE_NEWLIB_REENTRANT               0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0

/* ---------------- 内存分配 ---------------- */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
/* heap_4 静态堆大小：F407VE 有 128K RAM，当前裸机占用约 6K，给内核 32K 充裕。 */
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 32U * 1024U ) )

/* ---------------- 钩子与诊断 ---------------- */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             1
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configASSERT( x )                                                       \
    if ( ( x ) == 0 )                                                           \
    {                                                                           \
        taskDISABLE_INTERRUPTS();                                               \
        for ( ; ; )                                                             \
        {                                                                       \
        }                                                                       \
    }

/* ---------------- 软件定时器 ---------------- */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( configMAX_PRIORITIES - 1U )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             ( configMINIMAL_STACK_SIZE * 2U )

/* ---------------- 中断优先级（F407：4bit，0~15） ---------------- */
#define configPRIO_BITS                          4U
/* 内核自身（SysTick/PendSV）使用最低优先级。 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15U
/* 允许调用 FromISR API 的最高优先级（数值）：5，即 0~4 为"内核不可见"区。 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5U
/* 以下两个宏写入 NVIC 寄存器，需左移到高 4bit，请勿修改算法。 */
#define configKERNEL_INTERRUPT_PRIORITY          \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8U - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8U - configPRIO_BITS ) )

/* ---------------- 向量表桥接 ----------------
 * 启动文件 startup_stm32f407vetx_gcc.c 中 SVC/PendSV/SysTick 均为 weak，
 * 这里把移植层 handler 直接重命名为标准向量名，链接时覆盖 weak 定义，
 * 无需修改启动文件。
 */
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
#define xPortSysTickHandler                      SysTick_Handler

/* ---------------- 可选功能裁剪 ---------------- */
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xResumeFromISR                   1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           0
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xEventGroupSetBitFromISR         1
#define INCLUDE_xTimerPendFunctionCall           1
#define INCLUDE_xTaskAbortDelay                  0
#define INCLUDE_xTaskGetHandle                   0
#define INCLUDE_xTaskResumeFromISR               1

#endif /* __FREERTOS_CONFIG_H */
