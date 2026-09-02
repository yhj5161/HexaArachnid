#include "SU03T.h"

#include "usart.h"
#include "My_Usart/My_Usart.h"

/* SU-03T 挂 UART5（PC12 TX / PD2 RX）。 */
#define SU03T_USART   UART5

/* 接收环形缓冲大小（字节）。 */
#define SU03T_RX_BUF_SIZE  128U

static uint8_t s_rx_buf[SU03T_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;   /* ISR 写 */
static uint16_t s_rx_tail = 0U;            /* 任务读 */

/* ===================== 接收（上半部 / 下半部分离） ===================== */

void SU03T_RxPush(uint8_t data)
{
	uint16_t next = (uint16_t)((s_rx_head + 1U) % SU03T_RX_BUF_SIZE);

	s_rx_buf[s_rx_head] = data;
	s_rx_head = next;

	/* 满则丢最老 1 字节，滑动窗口防死锁。 */
	if (next == s_rx_tail)
	{
		s_rx_tail = (uint16_t)((s_rx_tail + 1U) % SU03T_RX_BUF_SIZE);
	}
}

uint8_t SU03T_HasData(void)
{
	return (s_rx_tail != s_rx_head) ? 1U : 0U;
}

uint8_t SU03T_GetByte(uint8_t *byte)
{
	if (byte == 0)
	{
		return 0U;
	}

	if (s_rx_tail == s_rx_head)
	{
		return 0U;
	}

	*byte = s_rx_buf[s_rx_tail];
	s_rx_tail = (uint16_t)((s_rx_tail + 1U) % SU03T_RX_BUF_SIZE);
	return 1U;
}

void SU03T_Init(void)
{
	uint16_t i;

	s_rx_head = 0U;
	s_rx_tail = 0U;
	for (i = 0U; i < SU03T_RX_BUF_SIZE; i++)
	{
		s_rx_buf[i] = 0U;
	}
}

/* ===================== 发送 ===================== */

void SU03T_SendByte(uint8_t data)
{
	usart_send_byte(SU03T_USART, data);
}

void SU03T_SendString(const char *str)
{
	uint16_t i;

	if (str == 0)
	{
		return;
	}

	for (i = 0U; str[i] != '\0'; i++)
	{
		usart_send_byte(SU03T_USART, (uint8_t)str[i]);
	}
}