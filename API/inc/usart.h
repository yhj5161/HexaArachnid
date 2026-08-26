#ifndef __API_USART_H
#define __API_USART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 本工程固定为 STM32F407，直接包含其底层头文件。 */
#include "f407_usart.h"

/* F407 串口映射按正常顺序定义：API_USART1/2/3 对应底层串口 0/1/2。 */

typedef enum
{
	API_USART1 = 1U,
	API_USART2 = 2U,
	API_USART3 = 3U,
} API_USART_Id_t;

/* F407 底层串口编号。 */
#define API_USART_CORE_USART1  (0U)
#define API_USART_CORE_USART2  (1U)
#define API_USART_CORE_USART3  (2U)

typedef struct
{
	API_USART_Id_t id;
	uint8_t coreId;
	void *txPort;
	uint32_t txPin;
	void *rxPort;
	uint32_t rxPin;
} API_USART_Config_t;

typedef void (*API_USART_IrqHandler_t)(API_USART_Id_t id);

/*
 * 供应用层在串口中断里直接访问的最小寄存器视图（F407）。
 * 这样 main.c 可以直接写 USARTx_IRQHandler，而不用再走额外的 API 串口 IRQ 包装。
 */
typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} F407_USART_View_t;

#define USART1 ((F407_USART_View_t *)0x40011000UL)
#define USART2 ((F407_USART_View_t *)0x40004400UL)
#define USART3 ((F407_USART_View_t *)0x40004800UL)
#define USART4 ((F407_USART_View_t *)0x40004C00UL)
#define USART_SR_RXNE (1UL << 5)
#define USART_SR_TC   (1UL << 6)
#define USART_SR_TXE  (1UL << 7)

void API_USART_Register(const API_USART_Config_t *configTable, uint8_t count);
void API_USART_RegisterIrqHandler(API_USART_Id_t id, API_USART_IrqHandler_t handler);
void API_USART_HandleIrqByCoreId(uint8_t coreId);
/* 串口初始化接口：id 选择串口，baudRate 设置波特率。 */
void API_USART_Init(API_USART_Id_t id, uint32_t baudRate);
/* 串口发送 1 字节。 */
void API_USART_WriteByte(API_USART_Id_t id, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __API_USART_H */
