#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "KEY.h"
#include "JY61P/JY61P.h"
#include "SU03T/SU03T.h"

/* 程序运行的时间戳（s） */
uint32_t Timer_Bsp_t = 0;

/* printf节拍 */
volatile uint8_t print_task_flag = 0;

/*
 * API_TIM3: 1ms -> Key + printf + time
 */
void Control_Task_Housekeeping_Callback(API_TIM_Id_t id)
{
	static uint8_t printf_tick = 0U;
	static uint16_t time_t = 0U;

	if (id != API_TIM3)
	{
		return;
	}

	Key_Tick();

	printf_tick++;
	time_t++;

	if (printf_tick >= 50U)
	{
		printf_tick = 0U;
		print_task_flag = 1U;
	}

	if (time_t >= 1000U)
	{
		time_t = 0U;
		Timer_Bsp_t++;
	}
}

/*
 * USART 中断回调：读取 RX 字节并分发到对应模块。
 * - USART1 → JY61P 姿态模块（主动上报，入环形缓冲）
 * - UART5  → SU-03T 语音模块（入环形缓冲）
 */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
	uint32_t data;
	uint8_t rxValid;

	data = 0U;
	rxValid = 0U;
	usart_irq_dispatch_by_id(id, &data, &rxValid);
	if (rxValid != 0U)
	{
		if (id == API_USART1)
		{
			JY61P_RxPush((uint8_t)data);
		}
		else if (id == API_USART5)
		{
			SU03T_RxPush((uint8_t)data);
		}
	}
}