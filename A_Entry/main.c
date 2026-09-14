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
#include "JY61P/JY61P.h"
#include "SU03T/SU03T.h"
#include "NRF24L01.h"
#include "PCA9685.h"
#include "hexapod.h"
#include "command.h"

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
 * │ ControlTask │ +3   │ 按键响应（1ms 轮询）                      │
 * │ SensorTask  │ +2   │ JY61P 姿态解析 + SU-03T 语音指令（5ms）   │
 * │ DisplayTask │ +1   │ OLED 刷新 + 串口打印（50ms 周期）         │
 * └─────────────┴──────┴──────────────────────────────────────────┘
 *
 * 任务栈大小单位是 word（1 word = 4 字节），512 word = 2KB。
 */
#define TASK_PRIO_CONTROL     (tskIDLE_PRIORITY + 3U)
#define TASK_PRIO_SENSOR      (tskIDLE_PRIORITY + 2U)
#define TASK_PRIO_DISPLAY     (tskIDLE_PRIORITY + 1U)

#define TASK_STACK_CONTROL    (512U)
#define TASK_STACK_SENSOR     (384U)
#define TASK_STACK_DISPLAY    (512U)

/* 任务周期（ms），对应 pdMS_TO_TICKS 换算后的节拍数 */
#define TASK_PERIOD_CONTROL   (1U)    /* 1ms：轮询 */
#define TASK_PERIOD_SENSOR    (5U)    /* 200Hz：JY61P 报文解析/语音 */
#define TASK_PERIOD_DISPLAY   (50U)   /* 20Hz：OLED/串口打印 */

static void ControlTask(void *argument);
static void SensorTask(void *argument);
static void DisplayTask(void *argument);

/*
 * ======================== 舵机布局（六足 × 3 关节/腿 = 18 舵机）========================
 *
 * PCA9685 #1 (I2C3, 0x40) — 左侧 3 条腿        PCA9685 #2 (I2C4, 0x40) — 右侧 3 条腿
 * ┌──────────┬──────┬──────┬──────┐              ┌──────────┬──────┬──────┬──────┐
 * │ 腿       │ Coxa │Femur │Tibia │              │ 腿       │ Coxa │Femur │Tibia │
 * ├──────────┼──────┼──────┼──────┤              ├──────────┼──────┼──────┼──────┤
 * │ Leg1 (左前)│ ch0  │ ch1  │ ch2  │              │ Leg4 (右前)│ ch0  │ ch1  │ ch2  │
 * │ Leg2 (左中)│ ch3  │ ch4  │ ch5  │              │ Leg5 (右中)│ ch3  │ ch4  │ ch5  │
 * │ Leg3 (左后)│ ch6  │ ch7  │ ch8  │              │ Leg6 (右后)│ ch6  │ ch7  │ ch8  │
 * └──────────┴──────┴──────┴──────┘              └──────────┴──────┴──────┴──────┘
 * ch9~15 空闲备用                                 ch9~15 空闲备用
 */
PCA9685_Handle_t g_pca1;  /* 左侧 PCA9685，I2C3 */
PCA9685_Handle_t g_pca2;  /* 右侧 PCA9685，I2C4 */

static Hexapod s_hexapod;
static MovementMode s_mode = MOVEMENT_STANDBY;  /* 当前运动模式 */

/*
 * NRF24L01 接收测试状态：
 * - 遥控器（F103 江协代码）每帧发 4 字节：d0=RV速度, d1=RH转向, d2=按键码, d3=LV倍率
 * - ControlTask 轮询收包后更新这里，DisplayTask 周期打印。
 */
static volatile uint8_t  s_nrfRxBuf[NRF24L01_RX_PACKET_WIDTH] = {0U, 0U, 0U, 0U};
static volatile uint32_t s_nrfRxCount;  /* 累计收包次数 */
static uint8_t s_nrfBlink;              /* LED 闪烁状态 */

int main(void)
{
/* 系统时钟配置初始化 */
	SYS_Init();
/* 注册层：注册相关资源，登记资源映射 */
	Enroll_USART_Register();				/* USART 资源注册（USART1/3 + UART5） */
	Enroll_PWM_Register();					/* PWM 资源注册 */
	Enroll_ADC_Register();					/* ADC 资源注册 */
	Enroll_TIM_Register();					/* TIM 资源注册 */
	Enroll_I2C_Register();					/* I2C 资源注册 */
	Enroll_SPI_Register();					/* SPI 资源注册 */
	Enroll_LED_Register();					/* LED 资源注册 */
	Enroll_KEY_Register();					/* KEY 资源注册 */
	Enroll_OLED_Register();					/* OLED SPI 控制脚注册 */
	Enroll_HCSR04_Register();				/* HC-SR04 超声波 资源注册 */
	Enroll_NRF24L01_Register();				/* NRF24L01 CE 控制脚注册 */

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); /* USART 中断回调：JY61P/SU-03T */
	API_TIM_RegisterIrqHandler(API_TIM3, Control_Task_Housekeeping_Callback);    /* TIM3: Housekeeping */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U);	/* USART1: JY61P 姿态模块（115200） */
	API_USART_Init(API_USART3, 115200U);	/* USART3: 调试打印 */
	API_USART_Init(API_USART4, 115200U);	/* UART4: 调试口（PA0=TX, PA1=RX，备用调试串口） */
	API_USART_Init(API_USART5, 115200U);	/* UART5: SU-03T 离线语音模块（115200） */
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
	NRF24L01_Init();					/* NRF24L01 收发初始化（SPI2 + CE=PC6） */

/*BSP硬件抽象层初始化*/
	LED_Init(LED_LOW); // 初始化LED-低电平
	KEY_Init(); // 初始化按键
	OLED_Init(OLED_IF_I2C);		 		/* OLED_IF_I2C(4针) / OLED_IF_SPI(7针) */
	JY61P_Init();						/* JY61P 姿态模块（数据在 SensorTask 解析） */
	SU03T_Init();						/* SU-03T 语音模块 */
	// HCSR04_Init(HCSR04_1); /* HC-SR04 超声波初始化（Trig=PA4, Echo=PA6） */

	/* PCA9685 舵机驱动初始化（2 块板，各走独立 I2C 总线） */
	PCA9685_Init(&g_pca1, API_I2C3, PCA9685_DEFAULT_ADDR, 50.0f);  /* 左侧：I2C3 */
	PCA9685_Init(&g_pca2, API_I2C4, PCA9685_DEFAULT_ADDR, 50.0f);  /* 右侧：I2C4 */

	/* 六足运动控制初始化（校准加载 + standby 姿态） */
	Hexapod_Init(&s_hexapod);
	Hexapod_SetSpeed(&s_hexapod, 1.0f);  /* [测试] 全速，改回 0.5f 恢复半速 */


/* ======================== 创建任务，启动调度器 ======================== */
	(void)xTaskCreate(ControlTask, "control", TASK_STACK_CONTROL, NULL, TASK_PRIO_CONTROL, NULL);
	(void)xTaskCreate(SensorTask, "sensor", TASK_STACK_SENSOR, NULL, TASK_PRIO_SENSOR, NULL);
	(void)xTaskCreate(DisplayTask, "display", TASK_STACK_DISPLAY, NULL, TASK_PRIO_DISPLAY, NULL);

	/* 启动调度器：从这里开始控制权交给 FreeRTOS，正常情况下不会返回。 */
	vTaskStartScheduler();

	/* 只有堆不足导致 Idle/Timer 任务创建失败才会走到这里。 */
	for (;;)
	{
	}
}

/*
 * 控制任务：1ms 轮询，当前职责为按键。
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
    if (s_mode == MOVEMENT_FORWARD)
    {
        s_mode = MOVEMENT_STANDBY;
    }
    else
    {
        s_mode = MOVEMENT_FORWARD;
    }
    Key = 0U;  /* ← 消费掉，不让下次循环重复触发 */
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

	/* NRF24L01 接收测试：遥控器 → F407，收到一帧就存全局 + LED2 翻转 */
		if (NRF24L01_Receive() == 1U)
		{
			uint8_t i;
			for (i = 0U; i < NRF24L01_RX_PACKET_WIDTH; i++)
			{
				s_nrfRxBuf[i] = NRF24L01_RxPacket[i];
			}
			s_nrfRxCount++;
			s_nrfBlink ^= 1U;
			LED_Control(LED2, s_nrfBlink ? LED_HIGH : LED_LOW);
		}

	/* 20ms 步态节拍：TIM3 1ms 中断累加 s_tick20ms，满 20 推进一帧 */
		if (s_tick20ms >= 20U)
		{
			s_tick20ms = 0U;
			Hexapod_ProcessMovement(&s_hexapod, s_mode, 20);
		}

	/* HC-SR04 超声波测距测试（单次 + 5 次平均） */
		// float dist = HCSR04_GetDistance(HCSR04_1);
		// float distAvg = HCSR04_GetDistanceAvg(HCSR04_1, 5U);
		// usart_printf(PRINTF_USART, "dist=%.1f cm, avg=%.1f cm\r\n", dist, distAvg);

		vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_CONTROL));
	}
}

/*
 * 传感器任务：JY61P 姿态 + SU-03T 语音。
 * - JY61P_Task() 解析 USART1 环形缓冲，主动上报，未就绪立即返回不阻塞。
 * - SU-03T 先把原始字节打出来，方便确认上位机配置的输出格式。
 */
static void SensorTask(void *argument)
{
	(void)argument;
	TickType_t lastWake = xTaskGetTickCount();

	for (;;)
	{
	/* JY61P 姿态解析 */
		JY61P_Task();
		(void)JY61P_GetUpdateFlags();

	/* SU-03T 语音：原始字节填充调试口（确认格式阶段，后续替换成协议解析） */
		{
			uint8_t b;
			while (SU03T_GetByte(&b) != 0U)
			{
				usart_printf(PRINTF_USART, "SV:0x%02X ", (unsigned int)b);
			}
		}

		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_PERIOD_SENSOR));
	}
}

/*
 * 显示任务：OLED 刷新 + 串口打印，最低优先级。
 * OLED 走软件 SPI，放独立任务后不再拖累控制任务。
 */
static void DisplayTask(void *argument)
{
	(void)argument;
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;

	for (;;)
	{/* JY61P 姿态角 */
		JY61P_GetAngle(&roll, &pitch, &yaw);
	/* 串口数据打印（50ms 节拍标志由 TIM3 中断置起） */
		if (print_task_flag != 0U)
		{
			print_task_flag = 0U;
			// usart_printf(PRINTF_USART, "key: %lu\r\n", Key);
			// usart_printf(PRINTF_USART, "Timer_Bsp_t: %lu\r\n", Timer_Bsp_t);
			usart_printf(USART4, "R=%.1f P=%.1f Y=%.1f\r\n", (double)roll, (double)pitch, (double)yaw);
		/* NRF24L01 收到数据打印（d0=速度 d1=转向 d2=按键 d3=倍率） */
			if (s_nrfRxCount != 0U)
			{
				usart_printf(USART4, "NRF cnt=%lu d0=%u d1=%u d2=%u d3=%u\r\n",
				             (unsigned long)s_nrfRxCount,
				             (unsigned int)s_nrfRxBuf[0], (unsigned int)s_nrfRxBuf[1],
				             (unsigned int)s_nrfRxBuf[2], (unsigned int)s_nrfRxBuf[3]);
			}
		}


	/* OLED刷新 */
		OLED_Printf(0, 0, OLED_8X16, "%d", Timer_Bsp_t);
		OLED_Printf(0, 16, OLED_8X16, "R%+.1f P%+.1f", (double)roll, (double)pitch);
		OLED_Printf(0, 32, OLED_8X16, "Y%+.1f", (double)yaw);
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