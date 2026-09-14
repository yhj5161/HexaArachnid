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

/* 20ms 步态节拍：1ms 中断累加，满 20 由 ControlTask 消费 */
volatile uint8_t s_tick20ms = 0U;

/* UART4 帧解析 → OLED 显示。
 * 协议：s12,-34,56e —— 's' 包头、',' 分隔各数、'e' 包尾。
 * 非法帧整体丢弃；解析结果放 g_rxFrame[]，置 g_rxFrameReady=1，由 DisplayTask 消费后清 0。 */
#define RX_FRAME_MAX  10U              /* 一帧最多 10 个数 */
#define RX_NUM_MAX    15U              /* 每个数最多 15 个 ASCII 字符 */

static uint8_t s_rxState;              /* 0=等包头，1=收数，2=一帧完成等下一包头 */
static uint8_t s_rxCount;              /* 本帧已解析出几个数 */
static uint8_t s_rxBufLen;             /* 当前数的字符长度 */
static char    s_rxBuf[RX_NUM_MAX];    /* 当前数的 ASCII 缓冲 */
static int16_t s_rxNum[RX_FRAME_MAX];  /* 本帧解析出的数 */

volatile int16_t g_rxFrame[RX_FRAME_MAX];
volatile uint8_t g_rxFrameCount;
volatile uint8_t g_rxFrameReady;

/* 把 ASCII 数字缓冲解析成 int16_t（支持前导 '-'），非法字符按 0 处理 */
static int16_t RX_FrameParseInt(const char *buf, uint8_t len)
{
	uint8_t i = 0U;
	uint8_t neg = 0U;
	int16_t val = 0;

	if ((len > 0U) && (buf[0] == '-'))
	{
		neg = 1U;
		i = 1U;
	}
	for (; i < len; i++)
	{
		if ((buf[i] >= '0') && (buf[i] <= '9'))
		{
			val = (int16_t)(val * 10 + (buf[i] - '0'));
		}
		else
		{
			return 0;
		}
	}
	return neg ? (int16_t)(-val) : val;
}

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
	s_tick20ms++;

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
		else if (id == API_USART4)
		{
			/* UART4：帧解析状态机（协议 s12,-34,56e），非法帧整体丢弃 */
			uint8_t c = (uint8_t)data;
			uint8_t i;

			if (s_rxState == 0U)
			{
				/* 等包头 's' */
				if (c == 's')
				{
					s_rxState = 1U;
					s_rxCount  = 0U;
					s_rxBufLen = 0U;
				}
			}
			else if (s_rxState == 1U)
			{
				if (c == 'e')
				{
					/* 包尾：收下最后一个数，提交整帧 */
					if ((s_rxBufLen > 0U) && (s_rxCount < RX_FRAME_MAX))
					{
						s_rxNum[s_rxCount++] = RX_FrameParseInt(s_rxBuf, s_rxBufLen);
					}
					for (i = 0U; i < s_rxCount; i++)
					{
						g_rxFrame[i] = s_rxNum[i];
					}
					g_rxFrameCount = s_rxCount;
					g_rxFrameReady = 1U;
					s_rxState = 2U;
				}
				else if (c == ',')
				{
					/* 分隔符：收下当前数，开始下一个 */
					if ((s_rxBufLen > 0U) && (s_rxCount < RX_FRAME_MAX))
					{
						s_rxNum[s_rxCount++] = RX_FrameParseInt(s_rxBuf, s_rxBufLen);
					}
					else
					{
						s_rxState = 0U;   /* 空字段或个数超限，弃帧 */
					}
					s_rxBufLen = 0U;
				}
				else if (((c >= '0') && (c <= '9')) || (c == '-'))
				{
					if ((c == '-') && (s_rxBufLen != 0U))
					{
						s_rxState = 0U;   /* '-' 只能出现在数首 */
					}
					else if (s_rxBufLen < RX_NUM_MAX)
					{
						s_rxBuf[s_rxBufLen++] = (char)c;
					}
					else
					{
						s_rxState = 0U;   /* 单个数字过长，弃帧 */
					}
				}
				else
				{
					s_rxState = 0U;       /* 非法字符，弃帧 */
				}
			}
			else /* s_rxState == 2U：等下一帧包头 */
			{
				if (c == 's')
				{
					s_rxState = 1U;
					s_rxCount  = 0U;
					s_rxBufLen = 0U;
				}
			}
		}
	}
}