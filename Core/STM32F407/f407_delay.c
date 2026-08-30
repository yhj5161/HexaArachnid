#include "delay.h"
#include "stm32f4xx.h"

/* 由 CMSIS 系统文件维护：表示当前 HCLK 频率（单位 Hz）。 */
extern uint32_t SystemCoreClock;

/*
 * 延时实现基于 Cortex-M4 DWT 周期计数器（DWT->CYCCNT），以 HCLK 频率计数。
 *
 * 为什么不用 SysTick：
 * FreeRTOS 接管 SysTick 作为内核节拍后，裸机延时若再去改写
 * SysTick->LOAD/CTRL 会破坏系统心跳。DWT 与 SysTick 完全独立，
 * 调度器运行前后都可安全使用，且微秒级分辨率更高（168MHz 下约 6ns）。
 *
 * 注意：这是忙等待（占着 CPU 空转）。RTOS 任务里的长延时请用
 * vTaskDelay() 把 CPU 让给其他任务；本接口保留给时序敏感的驱动
 * （软 I2C/SPI 位 bang、HC-SR04 触发脉宽等）使用。
 *
 * 单次延时上限：CYCCNT 为 32bit，168MHz 下约 25.5s 回绕一次，
 * 单次 Delay_us/Delay_ms 请勿超过该范围（Delay_s 内部已分段）。
 */

/* 使能 DWT 周期计数器（懒初始化，首次调用延时/时间戳时执行一次）。 */
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

void Delay_us(uint32_t us)
{
	uint32_t ticks;
	uint32_t start;

	if (us == 0U)
	{
		return;
	}

	F407_DwtInitOnce();
	ticks = us * (SystemCoreClock / 1000000U);
	start = DWT->CYCCNT;

	/* 无符号差值比较，天然免疫 CYCCNT 回绕。 */
	while ((DWT->CYCCNT - start) < ticks)
	{
	}
}

void Delay_ms(uint32_t ms)
{
	uint32_t ticks;
	uint32_t start;

	if (ms == 0U)
	{
		return;
	}

	F407_DwtInitOnce();
	/* 168MHz 下 ms * 168000 在 25.5s 内不溢出，常规毫秒延时远小于此。 */
	ticks = ms * (SystemCoreClock / 1000U);
	start = DWT->CYCCNT;

	while ((DWT->CYCCNT - start) < ticks)
	{
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
 * 返回自 DWT 使能以来的微秒数。
 * 说明：CYCCNT 为 32bit，168MHz 下约每 25.5s 回绕一次；
 * 测脉宽等场景用 (后值 - 前值) 的无符号差，可天然免疫回绕。
 */
uint32_t Micros(void)
{
	F407_DwtInitOnce();
	return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}
