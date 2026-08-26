#include "delay.h"
#include "stm32f4xx.h"

/* 由 CMSIS 系统文件维护：表示当前 HCLK 频率（单位 Hz）。 */
extern uint32_t SystemCoreClock;

/*
 * F407 参考实现采用 SysTick 时钟源 HCLK/8。
 * facUs: 1us 对应的 SysTick 计数；facMs: 1ms 对应计数。
 */
static void F407_GetDelayFactor(uint32_t *facUs, uint32_t *facMs)
{
	uint32_t sysclkDiv8;

	sysclkDiv8 = SystemCoreClock / 8U;
	*facUs = sysclkDiv8 / 1000000U;
	if (*facUs == 0U)
	{
		*facUs = 1U;
	}
	*facMs = (*facUs) * 1000U;
}

/* 单次毫秒延时（受 24bit SysTick LOAD 限制）。 */
static void F407_DelayXms(uint32_t ms)
{
	uint32_t facUs;
	uint32_t facMs;
	uint32_t temp;

	if (ms == 0U)
	{
		return;
	}

	F407_GetDelayFactor(&facUs, &facMs);
	SysTick->LOAD = ms * facMs;
	SysTick->VAL = 0U;
	SysTick->CTRL = 0x01U;
	do
	{
		temp = SysTick->CTRL;
	} while (((temp & 0x01U) != 0U) && ((temp & (1U << 16U)) == 0U));
	SysTick->CTRL = 0U;
	SysTick->VAL = 0U;
}

void Delay_us(uint32_t us)
{
	uint32_t facUs;
	uint32_t facMs;
	uint32_t temp;
	uint32_t maxUsPerShot;

	if (us == 0U)
	{
		return;
	}

	F407_GetDelayFactor(&facUs, &facMs);
	maxUsPerShot = 0xFFFFFFU / facUs;
	if (maxUsPerShot == 0U)
	{
		maxUsPerShot = 1U;
	}

	while (us > 0U)
	{
		uint32_t thisUs;

		thisUs = (us > maxUsPerShot) ? maxUsPerShot : us;
		SysTick->LOAD = thisUs * facUs;
		SysTick->VAL = 0U;
		SysTick->CTRL = 0x01U;
		do
		{
			temp = SysTick->CTRL;
		} while (((temp & 0x01U) != 0U) && ((temp & (1U << 16U)) == 0U));
		SysTick->CTRL = 0U;
		SysTick->VAL = 0U;
		us -= thisUs;
	}
}

void Delay_ms(uint32_t ms)
{
	/* 与常见 F407 参考工程一致：每段 540ms，兼顾超频余量。 */
	while (ms > 540U)
	{
		F407_DelayXms(540U);
		ms -= 540U;
	}

	if (ms > 0U)
	{
		F407_DelayXms(ms);
	}
}

void Delay_s(uint32_t s)
{
	while (s > 0U)
	{
		Delay_ms(1000U);
		--s;
	}
}

/*
 * 使能 DWT 周期计数器（懒初始化，仅在首次调用 Micros 时执行一次）。
 * Cortex-M4 的 DWT->CYCCNT 以 HCLK 频率自由计数，用来做微秒级时间戳，
 * 不占用任何定时器外设。
 */
static void F407_DwtInitOnce(void)
{
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
	{
		return; /* 已使能 */
	}

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*
 * 返回自 DWT 使能以来的微秒数。
 * 说明：CYCCNT 为 32bit，168MHz 下约每 25.5s 回绕一次；
 * 测脉宽等场景用 (后值 - 前值) 的无符号差，可天然免疫回绕。
 */
uint32_t Micros(void)
{
	F407_DwtInitOnce();
	return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}
