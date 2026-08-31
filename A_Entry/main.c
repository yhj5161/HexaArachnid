/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"

/*API层 MCU片内外设*/
#include "usart.h"
#include "tim.h"
#include "pwm.h"
#include "adc.h"

/*app应用层*/
#include "My_Usart/My_Usart.h"
#include "API_I2C.h"
#include "API_SPI.h"
#include "Control_Task/Control_Task.h"

/*BSP硬件抽象层*/
#include "LED.h"
#include "KEY.h"
#include "OLED.h"
#include "HCSR04.h"

/*FreeRTOS 内核*/
#include "FreeRTOS.h"
#include "task.h"

/*
 * ======================== FreeRTOS 任务规划 ========================
 *
 * 时序模型：硬实时节拍仍由 TIM3 中断产生（ISR 只置标志位），
 * 任务按各自周期轮询消费标志 —— 与原裸机结构一一对应，行为完全一致。
 *
 * 为什么 ISR 不直接唤醒任务：TIM/USART 中断的优先级为 1~4
 * （见 SYSTEM/IrqPriority.h），高于 FreeRTOS 系统调用上限（5），
 * 这些 ISR 里禁止调用 FromISR 系列 API。这样做的好处是控制节拍永不
 * 被内核临界区屏蔽，硬实时性不受 RTOS 影响。
 *
 * ┌─────────────┬──────┬──────────────────────────────────────────┐
 * │ 任务        │ 优先级│ 职责                                     │
 * ├─────────────┼──────┼──────────────────────────────────────────┤
 * │ ControlTask │ +3   │ 按键响应、摄像头数据包处理（1ms 轮询）   │
 * │ DisplayTask │ +1   │ OLED 刷新 + 串口打印（50ms 周期）         │
 * └─────────────┴──────┴──────────────────────────────────────────┘
 *
 * 任务栈大小单位是 word（1 word = 4 字节），512 word = 2KB。
 */
#define TASK_PRIO_CONTROL     (tskIDLE_PRIORITY + 3U)
#define TASK_PRIO_DISPLAY     (tskIDLE_PRIORITY + 1U)

#define TASK_STACK_CONTROL    (512U)
#define TASK_STACK_DISPLAY    (512U)

/* 任务周期（ms），对应 pdMS_TO_TICKS 换算后的节拍数 */
#define TASK_PERIOD_CONTROL   (1U)    /* 1ms：轮询 TIM 节拍标志 */
#define TASK_PERIOD_DISPLAY   (50U)   /* 20Hz：OLED/串口打印 */

static void ControlTask(void *argument);
static void DisplayTask(void *argument);

int main(void)
{
/* 系统时钟配置初始化 */
	SYS_Init();
/* 注册层：注册相关资源，登记资源映射 */
	Enroll_USART_Register();				/* USART 资源注册 */
	Enroll_PWM_Register();					/* PWM 资源注册 */
	Enroll_ADC_Register();					/* ADC 资源注册 */
	Enroll_TIM_Register();					/* TIM 资源注册 */
	Enroll_I2C_Register();					/* I2C 资源注册 */
	Enroll_SPI_Register();					/* SPI 资源注册 */
	Enroll_LED_Register();					/* LED 资源注册 */
	Enroll_KEY_Register();					/* KEY 资源注册 */
	Enroll_OLED_Register();					/* OLED SPI 控制脚注册 */
	Enroll_HCSR04_Register();				/* HC-SR04 超声波 资源注册 */

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); /* USART 中断回调注册 */
	API_TIM_RegisterIrqHandler(API_TIM3, Control_Task_Housekeeping_Callback);    /* TIM3: Housekeeping */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1，波特率 115200
	API_USART_Init(API_USART2, 115200U); // 初始化 USART2，波特率 115200
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3，波特率 115200
	/* PWM 初始化（F407）:
	 * 例：API_PWM_TIM1 -> 50Hz, ARR=4000-1, PSC=840-1（驱动舵机常用 50Hz）
	 * 接蜘蛛舵机前请按 F407 定时器重新核算 ARR/PSC。
	 */
	API_PWM_Init(API_PWM_TIM1, 400U - 1U, 8U - 1U);
	API_ADC_Init(API_ADC1); // 初始化 ADC1
	API_TIM_Init(API_TIM3, 1U); /* TIM3: 杂务节拍，每 1ms */

/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	App_I2C_ScanOnce();					/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

/*BSP硬件抽象层初始化*/
	LED_Init(LED_LOW); // 初始化LED-低电平
	KEY_Init(); // 初始化按键
	OLED_Init(OLED_IF_SPI);		 		/* OLED_IF_I2C(4针) / OLED_IF_SPI(7针) */
	// HCSR04_Init(HCSR04_1); /* HC-SR04 超声波初始化（Trig=PB6, Echo=PB7） */

/* ======================== 创建任务，启动调度器 ======================== */
	(void)xTaskCreate(ControlTask, "control", TASK_STACK_CONTROL, NULL, TASK_PRIO_CONTROL, NULL);
	(void)xTaskCreate(DisplayTask, "display", TASK_STACK_DISPLAY, NULL, TASK_PRIO_DISPLAY, NULL);

	/* 启动调度器：从这里开始控制权交给 FreeRTOS，正常情况下不会返回。 */
	vTaskStartScheduler();

	/* 只有堆不足导致 Idle/Timer 任务创建失败才会走到这里。 */
	for (;;)
	{
	}
}

/*
 * 控制任务：1ms 轮询 TIM 中断置起的节拍标志。
 * 对应原裸机 while(1) 中的按键/PID/相机包处理。
 */
static void ControlTask(void *argument)
{
	(void)argument;

	for (;;)
	{
/* KEY测试 Key 0变成1 */
		key_Get();
		if (Key == 1U)
		{
			LED_Control(LED1, LED_HIGH);
		}
		if (Key == 2U)
		{
			LED_Control(LED2, LED_HIGH);
		}
		if (Key == 3U)
		{
			LED_Control(LED3, LED_HIGH);
		}
		if (Key == 4U)
		{
			LED_Control(LED1, LED_LOW);
			LED_Control(LED2, LED_LOW);
			LED_Control(LED3, LED_LOW);
		}

	/* 摄像头数据包接收示例：固定 3 个数据 s88,-93,104e */
		if (USART_DataTypeStruct.state == 2U)
		{
			uint8_t i;
			/* 1. 缓存解析结果到全局数组 */
			USART_Packet_Count = USART_DataTypeStruct.count;
			for (i = 0U; i < USART_Packet_Count; i++)
			{
				USART_Packet_Data[i] = USART_Deal(&USART_DataTypeStruct, (int8_t)i);
			}
			USART_DataTypeStruct.state = 0U;

			/* 2. 校验数据完整性并读取 */
			if (USART_Packet_Count == 3U)
			{
				int16_t cam_x = USART_Packet_Data[0];
				int16_t cam_y = USART_Packet_Data[1];
				int16_t cam_z = USART_Packet_Data[2];
				usart_printf(USART1, "Cam X=%d, Y=%d, Z=%d\r\n", cam_x, cam_y, cam_z);
			}
		}

	/* HC-SR04 超声波测距测试（单次 + 5 次平均） */
		// float dist = HCSR04_GetDistance(HCSR04_1);
		// float distAvg = HCSR04_GetDistanceAvg(HCSR04_1, 5U);
		// usart_printf(USART1, "dist=%.1f cm, avg=%.1f cm\r\n", dist, distAvg);

		vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_CONTROL));
	}
}

/*
 * 显示任务：OLED 刷新 + 串口打印，最低优先级。
 * OLED 走软件 SPI，放独立任务后不再拖累控制环。
 */
static void DisplayTask(void *argument)
{
	(void)argument;

	for (;;)
	{
	/* 串口数据打印（50ms 节拍标志由 TIM3 中断置起） */
		if (print_task_flag != 0U)
		{
			print_task_flag = 0U;
			// usart_printf(USART1, "key: %lu\r\n", Key);
			// usart_printf(USART1, "Timer_Bsp_t: %lu\r\n", Timer_Bsp_t);
		}

	/* OLED刷新 */
		OLED_Printf(0, 0, OLED_8X16, "%d", Timer_Bsp_t);
		OLED_Update();

		vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_DISPLAY));
	}
}

/* ======================== FreeRTOS 钩子函数 ======================== */

/*
 * 堆不足钩子：任务/队列/信号量创建失败（pvPortMalloc 返回 NULL）时触发。
 * 点亮 LED1 表示内存耗尽，便于无调试器时定位。
 */
void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	LED_Control(LED1, LED_HIGH);
	for (;;)
	{
	}
}

/*
 * 任务栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW=2）。
 * 点亮 LED2 表示有任务栈溢出，可通过调试器查看 pcTaskName 定位。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;
	taskDISABLE_INTERRUPTS();
	LED_Control(LED2, LED_HIGH);
	for (;;)
	{
	}
}
