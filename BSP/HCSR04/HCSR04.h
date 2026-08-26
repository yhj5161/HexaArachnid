#ifndef __HCSR04_H
#define __HCSR04_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HCSR04 模块说明（HC-SR04 超声波测距）：
 * 1) 引脚：Trig 输出触发脉冲，Echo 输入回波高电平，脉宽正比于距离；
 * 2) 计时：用 SYSTEM/Delay.h 的 Micros()（DWT 周期计数器）测 Echo 高电平微秒数；
 * 3) 资源：Trig/Echo 引脚由 Enroll 层按板级映射注册，BSP 只调 API_GPIO_*。
 *
 * 接线注意：HC-SR04 为 5V 器件，Echo 输出 5V，务必接 5V 容忍(FT)引脚；
 *           当前映射 Trig=PB6、Echo=PB7（见 Enroll/407_hw_config.h）。
 */

/* HC-SR04 逻辑编号（当前注册 1 个，可按需扩展）。 */
typedef enum
{
	HCSR04_1 = 0U
} HCSR04_Id_t;

/* 触发脉冲宽度（us）：HC-SR04 要求 >=10us。 */
#define HCSR04_TRIG_PULSE_US   (20U)
/* 等待 Echo 边沿的超时（us）：约对应 5m 量程，防止无回波时死等卡住主循环。 */
#define HCSR04_TIMEOUT_US      (30000U)
/* 测距失败/超时的返回值。 */
#define HCSR04_INVALID_DIST    (-1.0f)

/* HC-SR04 引脚资源映射：Trig 输出 + Echo 输入。 */
typedef struct
{
	/* Trig 触发脚（推挽输出）。 */
	void *trigPort;
	uint32_t trigPin;
	/* Echo 回波脚（输入）。 */
	void *echoPort;
	uint32_t echoPin;
} HCSR04_Config_t;

/* Enroll 层调用：注册当前板子的 HC-SR04 资源表。 */
void HCSR04_Register(const HCSR04_Config_t *configTable, uint8_t count);
/* BSP 层初始化：配置 Trig 为输出(默认低)、Echo 为输入。 */
void HCSR04_Init(HCSR04_Id_t id);
/* 单次测距：返回距离（cm），超时/未注册返回 HCSR04_INVALID_DIST。 */
float HCSR04_GetDistance(HCSR04_Id_t id);
/* 多次测距取平均：sampleCount 为采样次数（0 按 1 处理），全失败返回 HCSR04_INVALID_DIST。 */
float HCSR04_GetDistanceAvg(HCSR04_Id_t id, uint8_t sampleCount);

#ifdef __cplusplus
}
#endif

#endif /* __HCSR04_H */
