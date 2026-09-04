#include "PCA9685.h"
#include "Delay.h"

/* ===================== PCA9685 寄存器地址 ===================== */
#define PCA9685_REG_MODE1       0x00U
#define PCA9685_REG_MODE2       0x01U
#define PCA9685_REG_LED0_ON_L   0x06U
#define PCA9685_REG_ALL_LED_ON_L  0xFAU
#define PCA9685_REG_ALL_LED_OFF_L 0xFCU
#define PCA9685_REG_PRE_SCALE   0xFEU

/* MODE1 寄存器位定义 */
#define PCA9685_MODE1_RESTART   (1U << 7)
#define PCA9685_MODE1_EXTCLK    (1U << 6)
#define PCA9685_MODE1_AI        (1U << 5)  /* 自动增量 */
#define PCA9685_MODE1_SLEEP     (1U << 4)
#define PCA9685_MODE1_ALLCALL   (1U << 0)

/* MODE2 寄存器位定义 */
#define PCA9685_MODE2_OUTDRV    (1U << 2)  /* 推挽输出 */

/* 内部振荡器频率 (Hz) */
#define PCA9685_OSC_FREQ  25000000U

/* ===================== 内部辅助 ===================== */

/*
 * 写单个寄存器。
 */
static void PCA9685_WriteReg(PCA9685_Handle_t *handle, uint8_t reg, uint8_t val)
{
	API_I2C_Lock();
	API_I2C_SelectBus(handle->i2cBus);

	API_I2C_Start();
	API_I2C_SendByte((uint8_t)(handle->i2cAddr << 1));  /* 写地址 */
	API_I2C_Wait_Ack();
	API_I2C_SendByte(reg);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(val);
	API_I2C_Wait_Ack();
	API_I2C_Stop();

	API_I2C_Unlock();
}

/*
 * 读单个寄存器。
 */
static uint8_t PCA9685_ReadReg(PCA9685_Handle_t *handle, uint8_t reg)
{
	uint8_t val;

	API_I2C_Lock();
	API_I2C_SelectBus(handle->i2cBus);

	API_I2C_Start();
	API_I2C_SendByte((uint8_t)(handle->i2cAddr << 1));  /* 写地址 */
	API_I2C_Wait_Ack();
	API_I2C_SendByte(reg);
	API_I2C_Wait_Ack();

	API_I2C_Start();                                      /* 重复起始 */
	API_I2C_SendByte((uint8_t)((handle->i2cAddr << 1) | 1U)); /* 读地址 */
	API_I2C_Wait_Ack();
	val = API_I2C_ReceiveByte(0U);                        /* NACK → 结束 */
	API_I2C_Stop();

	API_I2C_Unlock();
	return val;
}

/* ===================== 公共 API ===================== */

/*
 * 初始化 PCA9685。
 */
void PCA9685_Init(PCA9685_Handle_t *handle, API_I2C_BusId_t bus, uint8_t addr, float freq)
{
	uint8_t mode1;
	uint8_t prescale;

	if (handle == 0)
	{
		return;
	}

	handle->i2cBus  = bus;
	handle->i2cAddr = addr;
	handle->freq    = freq;

	/* ---- 1. 进入休眠才能写 prescaler ---- */
	PCA9685_WriteReg(handle, PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP | PCA9685_MODE1_AI);

	/* ---- 2. 设 MODE2：推挽输出 ---- */
	PCA9685_WriteReg(handle, PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);

	/* ---- 3. 计算并写入 prescaler ---- */
	/* prescale = round(osc / (4096 * freq)) - 1 */
	prescale = (uint8_t)((float)PCA9685_OSC_FREQ / (PCA9685_PWM_RESOLUTION * freq) + 0.5f);
	if (prescale < 3U)
	{
		prescale = 3U;  /* 数据手册规定 prescale ≥ 3 */
	}
	PCA9685_WriteReg(handle, PCA9685_REG_PRE_SCALE, prescale);

	/* ---- 4. 退出休眠，等 500μs 让振荡器稳定 ---- */
	mode1 = PCA9685_ReadReg(handle, PCA9685_REG_MODE1);
	PCA9685_WriteReg(handle, PCA9685_REG_MODE1, mode1 & ~PCA9685_MODE1_SLEEP);
	Delay_us(500U);

	/* ---- 5. 发 RESTART 让内部 PWM 计数器重新同步 ---- */
	PCA9685_WriteReg(handle, PCA9685_REG_MODE1,
	                 (mode1 & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART | PCA9685_MODE1_AI);
}

/*
 * 设置单通道 PWM（12-bit）。
 */
void PCA9685_SetPWM(PCA9685_Handle_t *handle, uint8_t channel, uint16_t on, uint16_t off)
{
	uint8_t reg;

	if (handle == 0 || channel > 15U)
	{
		return;
	}

	reg = PCA9685_REG_LED0_ON_L + (uint8_t)(channel * 4U);

	API_I2C_Lock();
	API_I2C_SelectBus(handle->i2cBus);

	API_I2C_Start();
	API_I2C_SendByte((uint8_t)(handle->i2cAddr << 1));
	API_I2C_Wait_Ack();
	API_I2C_SendByte(reg);
	API_I2C_Wait_Ack();
	API_I2C_SendByte((uint8_t)(on & 0xFFU));       /* ON_L  */
	API_I2C_Wait_Ack();
	API_I2C_SendByte((uint8_t)(on >> 8U));          /* ON_H  */
	API_I2C_Wait_Ack();
	API_I2C_SendByte((uint8_t)(off & 0xFFU));       /* OFF_L */
	API_I2C_Wait_Ack();
	API_I2C_SendByte((uint8_t)(off >> 8U));         /* OFF_H */
	API_I2C_Wait_Ack();
	API_I2C_Stop();

	API_I2C_Unlock();
}

/*
 * 舵机角度 → 脉宽 → 12-bit 计数。
 */
void PCA9685_SetServoAngle(PCA9685_Handle_t *handle, uint8_t channel, float angle)
{
	uint32_t pulseUs;
	uint16_t count;

	if (handle == 0 || channel > 15U)
	{
		return;
	}

	/* 钳位 */
	if (angle < 0.0f)  { angle = 0.0f; }
	if (angle > 180.0f) { angle = 180.0f; }

	/* angle → 脉宽 (μs) */
	pulseUs = PCA9685_SERVO_MIN_US +
	          (uint32_t)((angle / 180.0f) * (float)(PCA9685_SERVO_MAX_US - PCA9685_SERVO_MIN_US));

	/* 脉宽 → 12-bit 计数 */
	count = (uint16_t)((float)pulseUs * (float)PCA9685_PWM_RESOLUTION * handle->freq / 1000000.0f);

	/* 写 PWM：ON=0, OFF=count */
	PCA9685_SetPWM(handle, channel, 0U, count);
}

/*
 * 脉宽 → 12-bit 计数（内部辅助）。
 */
static uint16_t PCA9685_PulseUsToCount(PCA9685_Handle_t *handle, uint16_t pulseUs)
{
	uint16_t clamped;

	if (pulseUs < PCA9685_SERVO_MIN_US)
	{
		clamped = PCA9685_SERVO_MIN_US;
	}
	else if (pulseUs > PCA9685_SERVO_MAX_US)
	{
		clamped = PCA9685_SERVO_MAX_US;
	}
	else
	{
		clamped = pulseUs;
	}

	return (uint16_t)((float)clamped * (float)PCA9685_PWM_RESOLUTION * handle->freq / 1000000.0f);
}

/*
 * 单通道舵机脉宽直接控制（μs）。
 */
void PCA9685_SetServoPulseUs(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pulseUs)
{
	uint16_t count;

	if (handle == 0 || channel > 15U)
	{
		return;
	}

	count = PCA9685_PulseUsToCount(handle, pulseUs);
	PCA9685_SetPWM(handle, channel, 0U, count);
}

/*
 * 一键设所有 16 路为同一脉宽（μs）。
 * 走 ALL_LED 寄存器，一笔 I2C 事务完成，比逐通道快 16 倍。
 */
void PCA9685_SetAllPulseUs(PCA9685_Handle_t *handle, uint16_t pulseUs)
{
	uint16_t count;

	if (handle == 0)
	{
		return;
	}

	count = PCA9685_PulseUsToCount(handle, pulseUs);

	API_I2C_Lock();
	API_I2C_SelectBus(handle->i2cBus);

	API_I2C_Start();
	API_I2C_SendByte((uint8_t)(handle->i2cAddr << 1));
	API_I2C_Wait_Ack();
	API_I2C_SendByte(PCA9685_REG_ALL_LED_ON_L);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(0x00U);              /* ALL_LED_ON_L  = 0 */
	API_I2C_Wait_Ack();
	API_I2C_SendByte(0x00U);              /* ALL_LED_ON_H  = 0 */
	API_I2C_Wait_Ack();
	API_I2C_SendByte((uint8_t)(count & 0xFFU));  /* ALL_LED_OFF_L */
	API_I2C_Wait_Ack();
	API_I2C_SendByte((uint8_t)(count >> 8U));    /* ALL_LED_OFF_H */
	API_I2C_Wait_Ack();
	API_I2C_Stop();

	API_I2C_Unlock();
}

/*
 * 休眠：设 SLEEP 位，500μs 后芯片进入低功耗。
 */
void PCA9685_Sleep(PCA9685_Handle_t *handle)
{
	uint8_t mode1;

	if (handle == 0)
	{
		return;
	}

	mode1 = PCA9685_ReadReg(handle, PCA9685_REG_MODE1);
	PCA9685_WriteReg(handle, PCA9685_REG_MODE1, mode1 | PCA9685_MODE1_SLEEP);
	Delay_us(500U);
}

/*
 * 唤醒：清 SLEEP 位，等振荡器稳定后发 RESTART。
 */
void PCA9685_WakeUp(PCA9685_Handle_t *handle)
{
	uint8_t mode1;

	if (handle == 0)
	{
		return;
	}

	mode1 = PCA9685_ReadReg(handle, PCA9685_REG_MODE1);
	PCA9685_WriteReg(handle, PCA9685_REG_MODE1, mode1 & ~PCA9685_MODE1_SLEEP);
	Delay_us(500U);
	PCA9685_WriteReg(handle, PCA9685_REG_MODE1,
	                 (mode1 & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART);
}