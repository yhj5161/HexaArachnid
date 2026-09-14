#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include <stdint.h>
#include "tim.h"
#include "usart.h"

extern uint32_t Timer_Bsp_t; // 程序运行的时间戳（s）
extern volatile uint8_t print_task_flag; // printf节拍-50ms
extern volatile uint8_t s_tick20ms;      // 20ms 步态节拍
extern volatile int16_t g_rxFrame[];     // UART4 解析出的本帧数字（协议 s12,-34,56e）
extern volatile uint8_t  g_rxFrameCount; // 本帧数字个数
extern volatile uint8_t  g_rxFrameReady; // 收到完整一帧置 1，OLED 消费后清 0

void Control_Task_Housekeeping_Callback(API_TIM_Id_t id);  /* TIM3: 按键/printf/时间戳 */
void Control_Task_USART_Callback(API_USART_Id_t id);      /* USART 中断分发：JY61P / SU-03T */

#endif /* __CONTROL_TASK_H */