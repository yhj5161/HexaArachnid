#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 统一延时接口（对上层模块统一暴露）：
 * 1) Delay_us: 微秒级阻塞延时
 * 2) Delay_ms: 毫秒级阻塞延时
 * 3) Delay_s : 秒级阻塞延时
 *
 * 说明：
 * - 上层只需要 include 本头文件，不需要关心具体 MCU 型号。
 * - 本工程固定为 STM32F407，实现由 Core/STM32F407/f407_delay.c 提供。
 */
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

/*
 * 自由运行的微秒级时间戳（基于 Cortex-M4 DWT 周期计数器）。
 * - 首次调用时自动使能 DWT，之后返回自使能以来的微秒数。
 * - 用于测量脉冲宽度等场景：用 (后值 - 前值) 的无符号差，可免疫回绕。
 * - 典型用途：HC-SR04 超声波 Echo 高电平脉宽测量。
 */
uint32_t Micros(void);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */
