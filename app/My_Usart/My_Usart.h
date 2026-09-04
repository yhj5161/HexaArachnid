#ifndef __MY_USART_H
#define __MY_USART_H

/*
 * My_Usart 模块说明：
 * 1) 统一提供“应用层可直接调用”的串口发送/printf 接口；
 * 2) 保持与原标准库封装接近的函数名和调用方式；
 * 3) 底层适配当前工程 API 层(usart.h)与 F407 寄存器视图。
 */

#include "Enroll.h"
#include "usart.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 异步发送环形缓冲区大小（字节）。 */
#ifndef USART_TX_BUF_SIZE
#define USART_TX_BUF_SIZE 512U
#endif

/*
 * printf 默认输出串口（可在编译参数或上层头文件中重定义）。
 * 工程串口分工：
 *   USART1 (PB6/PB7, 115200) → JY61P 姿态模块（RX 入环形缓冲）
 *   USART3 (PD8/PD9, 115200)  → 调试打印（PRINTF_USART）
 *   UART4  (PA0/PA1, 115200)  → 备用调试口（后续调试用）
 *   UART5  (PC12/PD2, 115200) → SU-03T 离线语音模块
 */
#ifndef PRINTF_USART
#define PRINTF_USART USART3
#endif

/* CR1.TXEIE：发送寄存器空中断使能位。 */
#ifndef USART_CR1_TXEIE
#define USART_CR1_TXEIE (1UL << 7)
#endif

/*
 * USART_TypeDef 统一别名：本工程固定为 STM32F407，对应 F407_USART_View_t。
 */
typedef F407_USART_View_t USART_TypeDef;

/*
 * 发送单字节（优先异步，不行则退化阻塞发送）。
 * 参数：USARTx 选择串口实例，Byte 为待发送字节。
 */
void usart_send_byte(USART_TypeDef *USARTx, uint8_t Byte);

/*
 * 发送单字节（纯异步）。
 * 返回：1=入队成功，0=队列满或串口不支持。
 */
uint8_t usart_send_byte_async(USART_TypeDef *USARTx, uint8_t Byte);

/* 发送以 '\0' 结尾的字符串。 */
void usart_SendString(USART_TypeDef *USARTx, const char *String);

/* 发送 32 位无符号整数（十进制字符串形式）。 */
void usart_send_number(USART_TypeDef *USARTx, uint32_t Number);

/* 幂函数：返回 X^Y。 */
uint32_t usart_pow(uint32_t X, uint32_t Y);

/* 连续发送数组中的 Length 个字节。 */
void usart_send_array(USART_TypeDef *USARTx, uint8_t *Array, uint16_t Length);

/* printf 重定向接口：默认发送到 PRINTF_USART。 */
int fputc(int ch, FILE *f);

/*
 * 类 printf 串口输出。
 * 用法：usart_printf(USART1, "rpm=%d\r\n", rpm);
 */
void usart_printf(USART_TypeDef *USARTx, const char *format, ...);

/*
 * TXE 中断处理函数：
 * 需要在对应 USARTx_IRQHandler 的 TXE 分支里调用。
 */
void usart_tx_irq_handler(USART_TypeDef *USARTx);

/* 根据 API 串口 ID 处理 RX/TX 中断事件（由注册层回调触发）。 */
void usart_irq_dispatch_by_id(API_USART_Id_t id, uint32_t *rxData, uint8_t *rxValid);

#endif