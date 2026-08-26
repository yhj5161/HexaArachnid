#include "HCSR04.h"
#include "gpio.h"
#include "Delay.h"

/* s_hcsr04Table: 已注册 HC-SR04 配置表，由 Enroll 层通过 HCSR04_Register 注入。 */
static const HCSR04_Config_t *s_hcsr04Table = 0;
/* s_hcsr04Count: 当前配置表中有效项数量。 */
static uint8_t s_hcsr04Count = 0U;

/* 在注册表中查找指定编号的 HC-SR04 配置。 */
static const HCSR04_Config_t *HCSR04_FindConfig(HCSR04_Id_t id)
{
	uint8_t index;

	if ((s_hcsr04Table == 0) || (s_hcsr04Count == 0U))
	{
		return 0;
	}

	for (index = 0U; index < s_hcsr04Count; ++index)
	{
		if (index == (uint8_t)id)
		{
			return &s_hcsr04Table[index];
		}
	}

	return 0;
}

/*
 * HCSR04_Register：把 Enroll 层提供的 HC-SR04 资源表存起来。
 * 这里不直接访问硬件，只负责保存"哪个 HC-SR04 对应哪组 Trig/Echo 引脚"。
 */
void HCSR04_Register(const HCSR04_Config_t *configTable, uint8_t count)
{
	if ((configTable == 0) || (count == 0U))
	{
		s_hcsr04Table = 0;
		s_hcsr04Count = 0U;
		return;
	}

	s_hcsr04Table = configTable;
	s_hcsr04Count = count;
}

/*
 * HCSR04_Init：配置 Trig 为推挽输出（默认低电平）、Echo 为输入。
 */
void HCSR04_Init(HCSR04_Id_t id)
{
	const HCSR04_Config_t *config;

	config = HCSR04_FindConfig(id);
	if (config == 0)
	{
		return;
	}

	API_GPIO_InitOutput(config->trigPort, config->trigPin);
	API_GPIO_Write(config->trigPort, config->trigPin, 0U); /* Trig 默认拉低 */
	API_GPIO_InitInput(config->echoPort, config->echoPin);
}

/*
 * HCSR04_GetDistance：单次测距。
 * 流程：Trig 拉高 20us 触发 -> 等 Echo 变高 -> 记录起点 -> 等 Echo 变低 -> 算脉宽。
 * 距离(cm) = 高电平时间(us) / 58（声速 340m/s 往返换算）。
 * 任一步超时/未注册返回 HCSR04_INVALID_DIST。
 */
float HCSR04_GetDistance(HCSR04_Id_t id)
{
	const HCSR04_Config_t *config;
	uint32_t waitStartUs;
	uint32_t echoStartUs;
	uint32_t echoWidthUs;

	config = HCSR04_FindConfig(id);
	if (config == 0)
	{
		return HCSR04_INVALID_DIST;
	}

	/* 1. 发出触发脉冲：Trig 拉高 >=10us 再拉低。 */
	API_GPIO_Write(config->trigPort, config->trigPin, 1U);
	Delay_us(HCSR04_TRIG_PULSE_US);
	API_GPIO_Write(config->trigPort, config->trigPin, 0U);

	/* 2. 等 Echo 变高（带超时，无符号差可免疫 Micros 回绕）。 */
	waitStartUs = Micros();
	while (API_GPIO_Read(config->echoPort, config->echoPin) == 0U)
	{
		if ((Micros() - waitStartUs) > HCSR04_TIMEOUT_US)
		{
			return HCSR04_INVALID_DIST;
		}
	}

	/* 3. 记录 Echo 拉高起点，等 Echo 变低（带超时）。 */
	echoStartUs = Micros();
	while (API_GPIO_Read(config->echoPort, config->echoPin) != 0U)
	{
		if ((Micros() - echoStartUs) > HCSR04_TIMEOUT_US)
		{
			return HCSR04_INVALID_DIST;
		}
	}
	echoWidthUs = Micros() - echoStartUs;

	/* 4. 换算成厘米。 */
	return (float)echoWidthUs / 58.0f;
}

/*
 * HCSR04_GetDistanceAvg：连续测 sampleCount 次取平均，自动剔除超时/失败的样本。
 * 全部失败返回 HCSR04_INVALID_DIST。
 */
float HCSR04_GetDistanceAvg(HCSR04_Id_t id, uint8_t sampleCount)
{
	float sum = 0.0f;
	float dist;
	uint8_t validCount = 0U;
	uint8_t i;

	if (sampleCount == 0U)
	{
		sampleCount = 1U;
	}

	for (i = 0U; i < sampleCount; ++i)
	{
		dist = HCSR04_GetDistance(id);
		if (dist >= 0.0f)
		{
			sum += dist;
			validCount++;
		}
	}

	if (validCount == 0U)
	{
		return HCSR04_INVALID_DIST;
	}

	return sum / (float)validCount;
}
