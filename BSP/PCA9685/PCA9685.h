#ifndef __PCA9685_H
#define __PCA9685_H

#include <stdint.h>
#include "API_I2C.h"

/*
 * PCA9685 16 路 12-bit PWM 驱动（软件 I2C）—— 蜘蛛舵机专用
 *
 * 用法：
 *   PCA9685_Handle_t pca1;
 *   PCA9685_Init(&pca1, API_I2C3, 0x40, 50.0f);   // I2C3, 地址 0x40, 50Hz
 *   PCA9685_SetServoAngle(&pca1, 0, 90.0f);         // 通道 0 → 90°
 *
 * 线程安全：每笔 I2C 事务内部自动 Lock/Unlock，各总线独立互斥。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* PCA9685 默认 7-bit I2C 地址（A0~A5 全部接地）。 */
#define PCA9685_DEFAULT_ADDR  0x40U

/* 舵机脉宽范围（μs），标准舵机 500~2500。 */
#define PCA9685_SERVO_MIN_US  500U
#define PCA9685_SERVO_MAX_US  2500U

/* 12-bit 分辨率上限。 */
#define PCA9685_PWM_RESOLUTION  4096U

/*
 * PCA9685 实例句柄：每个 PCA9685 绑定一条 I2C 总线和一个 7-bit 从机地址。
 */
typedef struct
{
	API_I2C_BusId_t i2cBus;   /* 绑定的软件 I2C 总线 */
	uint8_t        i2cAddr;   /* 7-bit I2C 地址（不含 R/W 位） */
	float          freq;      /* 当前 PWM 频率 (Hz) */
} PCA9685_Handle_t;

/*
 * 初始化 PCA9685：
 * - handle : 句柄指针（调用者分配）
 * - bus    : 软件 I2C 总线编号（如 API_I2C3）
 * - addr   : 7-bit I2C 地址（通常 0x40）
 * - freq   : PWM 频率 (Hz)，舵机用 50Hz
 *
 * 内部操作：复位 → 设 prescaler → 开自动增量 → 唤醒。
 */
void PCA9685_Init(PCA9685_Handle_t *handle, API_I2C_BusId_t bus, uint8_t addr, float freq);

/*
 * 原始 PWM 通道控制（12-bit）：
 * - channel : 0~15
 * - on      : 导通计数值 (0~4095)
 * - off     : 关断计数值 (0~4095)
 *
 * off=0 且 on≠0 时表示常高；off=4096 且 on=0 时表示常低。
 */
void PCA9685_SetPWM(PCA9685_Handle_t *handle, uint8_t channel, uint16_t on, uint16_t off);

/*
 * 舵机角度便捷接口：
 * - channel : 0~15
 * - angle   : 0.0° ~ 180.0°
 *
 * 内部映射：angle → 脉宽 (500~2500μs) → 12-bit 计数。
 * 超出范围自动钳位到 [0, 180]。
 */
void PCA9685_SetServoAngle(PCA9685_Handle_t *handle, uint8_t channel, float angle);

/*
 * 舵机脉宽直接控制（μs）：
 * - channel  : 0~15
 * - pulseUs  : 脉宽 μs（典型 500~2500，超出范围自动钳位）
 *
 * 校准 / 安装舵机时直接设脉宽比角度更精确。
 */
void PCA9685_SetServoPulseUs(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pulseUs);

/*
 * 一键设所有 16 路为同一脉宽（μs）—— 校准/安装时用。
 */
void PCA9685_SetAllPulseUs(PCA9685_Handle_t *handle, uint16_t pulseUs);

/*
 * 休眠 / 唤醒：
 * 休眠后所有 PWM 输出停止，芯片进入低功耗模式。
 */
void PCA9685_Sleep(PCA9685_Handle_t *handle);
void PCA9685_WakeUp(PCA9685_Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H */