#ifndef __SU03T_H
#define __SU03T_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SU-03T 离线语音模块驱动（接 UART5：TX=PC12，RX=PD2，115200bps）。
 *
 * 现状：(未完)
 * - 语音模块识别到指令后会通过 UART5 上报数据，具体帧格式由上位机
 *   配置决定（可以是 ASCII 命令名，也可以是十六进制码）。
 * - 本模块先做"字节级收发"：ISR 收进环形缓冲，上层按需取字节；
 *   等确认了 SU-03T 的配置格式后再补协议解析。
 *
 * 使用：
 *   中断里：SU03T_RxPush(byte);
 *   任务里：while (SU03T_GetByte(&b)) { ...处理/打印... }
 *   发指令：SU03T_SendByte(0xFF) / SU03T_SendString("...")
 */

/* 初始化：清空接收缓冲。 */
void SU03T_Init(void);

/* ISR 调用：推入 1 字节到环形缓冲（中断上半部）。 */
void SU03T_RxPush(uint8_t data);

/* 是否有数据可读。 */
uint8_t SU03T_HasData(void);

/*
 * 取 1 字节（中断下半部，任务里轮询）。返回 1 = 取到，0 = 空。
 */
uint8_t SU03T_GetByte(uint8_t *byte);

/* 发送 1 字节到语音模块（异步队列）。 */
void SU03T_SendByte(uint8_t data);

/* 发送字符串到语音模块。 */
void SU03T_SendString(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* __SU03T_H */