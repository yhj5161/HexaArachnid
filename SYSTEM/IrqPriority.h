#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 统一中断优先级管理
 *
 * 设计原则：
 * - 数字越小，优先级越高（NVIC 标准语义）
 * - 策略集中在这里，Core 层只接受优先级参数而不做决策
 * - 按 MCU 目标编译时自动选择正确的位宽范围
 *
 * 优先级分配：
 *  ┌──────────────────────────────────────────┐
 *  │  0  SysTick / 系统时钟                    │
 *  │  1  API_TIM 控制节拍 (1ms)               │ ← 控制回路的心脏
 *  │  2  (预留)                                │
 *  │  3  (预留)                                │
 *  │  4  USART (×3)                            │ ← 通信丢包可重传
 *  │  5+ 缺省                                  │
 *  └──────────────────────────────────────────┘
 *
 * 注：
 * - 控制任务在任务上下文执行，不在 ISR 里。
 * - 原 MPU6050 姿态外环、编码器已随轮式方案移除（2026-08-31），
 *   JY61P 姿态接入后再分配优先级。
 *
 * 本工程固定为 STM32F407（Cortex-M4, __NVIC_PRIO_BITS=4, 范围 0~15）。
 */

/* F407：4 bit 优先级 (0~15)，空间充足。 */
#define IRQ_PRIO_TIM_CTRL    1U   /* 最高实时：控制节拍       */
#define IRQ_PRIO_USART       4U   /* 中实时：串口通信          */
#define IRQ_PRIO_DEFAULT     5U   /* 低实时：缺省中断          */

/* STM32 使用 NVIC priority group 0 (全部 4bit 为抢占，无子优先级)，子优先级填 0 */

#endif /* __IRQ_PRIORITY_H */
