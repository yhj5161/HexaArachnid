# OmniLayer 工程架构深度解析

> 本文档旨在帮助在多个 Claude Code 对话中快速恢复对工程架构的完整认知。
> 每次重新开启对话后，Claude Code 只需阅读本文档即可快速理解项目设计原则与编码约定。

---

## 1. 项目元信息

| 项目 | 详情 |
|------|------|
| **名称** | OmniLayer（本仓库 QuadArachnid = 蜘蛛机器人固件） |
| **定位** | 分层嵌入式工程架构（**已固定为 STM32F407 单主控**） |
| **构建工具** | CMake + GCC ARM Embedded + OpenOCD |
| **IDE 兼容** | VS Code (主) + Keil MDK (保留兼容) |
| **分支策略** | `main` (裸机主线) / `FreeRTOS` (RTOS 方向) |
| **作者** | yhj5161 |
| **联系方式** | 2115951478@qq.com |

> **重要**：本工程最初是支持 STM32F103/F407/MSPM0G3507 的多 MCU 框架，**现已裁剪为 F407 单主控**。多 MCU 分发宏 `ENROLL_MCU_TARGET` 已移除，API 层直接调用 `F407_*` 底层实现。F103/G3507 的 Core、Drivers、板级配置、OpenOCD 配置均已删除。

---

## 2. 主控

| MCU | 架构 | 内核 | 构建预设 |
|-----|------|------|----------|
| STM32F407VET6 | ARM | Cortex-M4 + FPU | `Debug` |

单一主控，无多目标切换。编译/烧录直接用 `Debug` 预设（快捷键 `F7`/`F8`）。

---

## 3. 分层架构总览

```
┌────────────────────────────────────────────┐
│  A_Entry/      程序入口                     │  唯一 main.c
│  app/          应用层                       │  业务逻辑、控制算法、任务调度
│  - Control/    控制逻辑                     │
│  - Control_Task/ 任务调度+中断回调          │
│  - PID/        PID 控制器                   │
│  - Filter/     滤波器                       │
│  - My_Usart/   串口打印管理                 │
└────────────────────────────────────────────┘
              ↓ 调用
┌────────────────────────────────────────────┐
│  BSP/          板级支持层                   │  封装板载器件，提供稳定设备接口
│  - LED/KEY     IO 控制型外设                │
│  - OLED        显示屏 (I2C+SPI 双模式)      │
│  - MPU6050     6 轴传感器 + DMP             │
│  - TB6612      电机驱动                     │
│  - NRF24L01    2.4G 无线模块                │
│  - BMP280      气压传感器                   │
│  - QMC5883P    磁力计                       │
└────────────────────────────────────────────┘
              ↓ 依赖
┌────────────────────────────────────────────┐
│  Enroll/       注册层 (★核心特色)           │  硬件资源注册中心
│  - Enroll.h    对外接口                     │
│  - Enroll.c    注册实现 (X-Macro 展开)      │
│  - Enroll_Internal.h  内部依赖              │
│  - 407_hw_config.h  F407 板级映射表         │
└────────────────────────────────────────────┘
              ↓ 绑定
┌────────────────────────────────────────────┐
│  API/          片内外设抽象接口层            │  统一接口，直接对接 F407 Core
│  inc/ + src/   gpio/adc/pwm/tim/usart/exti  │
│  API_I2C/      I2C 协议层 (平台无关)        │
│  API_SPI/      SPI 协议层 (平台无关)        │
└────────────────────────────────────────────┘
              ↓ 调用
┌────────────────────────────────────────────┐
│  Core/         芯片底层实现                  │  仅剩 F407
│  STM32F407/    {src,inc}/f407_*.c,h         │
│  (另含: f407_soft_i2c, f407_soft_spi,       │
│         f407_delay.c, cmake/链接脚本)       │
└────────────────────────────────────────────┘
              ↓ 基于
┌────────────────────────────────────────────┐
│  Drivers/      驱动资源层                   │  启动文件、CMSIS/标准库
│  Drivers_STM32F4/  - std_periph 启动        │
└────────────────────────────────────────────┘
              ↓ 基础
┌────────────────────────────────────────────┐
│  SYSTEM/       系统层                       │  系统配置与初始化
│  sys.c/h      系统初始化 / 中断分发         │
│  Delay.h      统一延时接口                  │
│  BusRate.h    软件总线选择+速率集中配置     │
│  IrqPriority.h 统一中断优先级管理           │
└────────────────────────────────────────────┘
```

---

## 4. 核心设计模式

### 4.1 注册层模式 (Enroll — 最重要的设计)

注册层的本质是**用编译期 X-Macro 将"逻辑外设 ID"映射到"物理引脚+硬件实例"**。

**数据流：**
```
407_hw_config.h (板级映射宏)
    ↓ 定义 HW_xxx_MAP(X) 宏
Enroll.c (X-Macro 展开)
    ↓ 展开为结构体数组 s_xxxTable[]
Enroll_xxx_Register() (门面函数)
    ↓ 传入结构体数组 + 计数
API/BSP 的 Register() 函数
    ↓ 写入内部管理数组
后续 API/BSP 用逻辑 ID 操作
```

**示例 — F407 的 LED 注册：**

`407_hw_config.h` 定义映射宏：
```c
#define HW_LED_MAP(X) \
    X(LED1, GPIOE, GPIO_Pin_2) \
    X(LED2, GPIOE, GPIO_Pin_3) \
    X(LED3, GPIOE, GPIO_Pin_4)
```

`Enroll.c` 用 X-Macro 展开为配置表：
```c
#define ENROLL_LED_ITEM(id, port, pin) \
    { id, port, pin, ENROLL_GPIO_INIT_FN, ENROLL_GPIO_WRITE_FN },
static const LED_Config_t s_ledTable[] = {
    HW_LED_MAP(ENROLL_LED_ITEM)
};
```

优势：改板级接线/引脚只需改 `407_hw_config.h`，Enroll.c 与上层业务代码无需改动。

### 4.2 API 层直接对接 Core（原条件编译分发已移除）

历史上 API 层通过 `#if ENROLL_MCU_TARGET` 在多 MCU 间分发。**本工程已固定为 F407**，因此 API 层直接调用 F407 Core 实现，无任何条件编译：

```c
// API/inc/gpio.h — 接口声明 + 直接包含 F407 底层头
#include "f407_gpio.h"
void API_GPIO_Write(void *port, uint32_t pin, uint8_t level);

// API/src/gpio.c — 直接调用 F407 Core
void API_GPIO_Write(void *port, uint32_t pin, uint8_t level) {
    F407_GPIO_Write(port, pin, level);
}
```

**说明**：API 层仍然保留（App/BSP 依旧只调 `API_*`），但它现在是一层"直通"封装。价值在于统一的接口签名与集中管理，而非多态分发。

### 4.3 两阶段初始化模式 (Register → Init)

外设资源使用两阶段初始化：

```
Enroll_xxx_Register()  // 阶段1: 登记配置表 (填充内部数组)
        ↓
API_xxx_Init(id, ...)  // 阶段2: 激活硬件 (写寄存器、开启时钟)
```

这在 `main.c` 中体现得最明显：
```c
Enroll_USART_Register();           // 先登记 USART 配置表
API_USART_Init(API_USART1, 115200); // 再用逻辑 ID 初始化特定 USART
```

### 4.4 void *port 端口抽象

API 层统一用 `void *port` 传递 GPIO 端口指针，在 Core 层内部转回实际类型：
```c
gpioPort = (GPIO_TypeDef *)port; // F407 Core（GPIOA/GPIOB/...）
```
（该设计最初是为兼容不同 MCU 的端口类型，现虽只有 F407，但接口形态保留。）

### 4.5 软件总线 (bit-bang I2C/SPI) — 双分层架构

项目目前使用软件模拟的 I2C 和 SPI（非硬件外设），原因：
- 灵活性高，不受硬件 I2C/SPI 实例限制
- 引脚映射自由

**架构拆分**：协议逻辑与 GPIO 翻转分两层：

```
API/API_I2C/API_I2C.c          ← 协议逻辑 (平台无关, 始终编译)
    │  Start/Stop/SendByte/ReceiveByte/Wait_Ack/...
    │  通过 soft_i2c_hal.h 桥接 ↓
    │
Core/STM32F407/f407_soft_i2c.c  ← GPIO 翻转+延时
    │  直接寄存器访问: BSRR/BRR/...
```

```
API/API_SPI/API_SPI.c          ← 协议逻辑 (Start/Stop/SwapByte)
    │  通过 soft_spi_hal.h 桥接 ↓
Core/STM32F407/f407_soft_spi.c  ← GPIO 翻转+延时
```

**HAL 桥接接口** (`soft_i2c_hal.h`, `soft_spi_hal.h`)：
- **内部桥接**：API 协议层 ↔ Core 底层实现之间的桥梁
- **不对外暴露**：BSP/App 层不应直接引用，统一通过 `API_I2C.h` / `API_SPI.h` 操作
- 声明平台无关的底层原语函数（W_SCL, W_SDA, R_SDA, W_CS, W_SCK, delay_us 等），由 F407 Core 实现
- 接口里的 `iomux` 参数是为兼容 MSPM0 保留的，STM32 上一律传 0

**设计原则**：
- **所有 BSP 设备**只通过 `API_I2C.h` / `API_SPI.h` 的标准协议函数操作总线
- 总线选择+速率集中配置在 `SYSTEM/BusRate.h`，新增设备只需加两行宏

### 4.6 中断优先级统一管理 (IrqPriority.h)

对标 `BusRate.h` 的思路，`SYSTEM/IrqPriority.h` 集中管理所有 NVIC 中断优先级：

```
IrqPriority.h (策略层)  →  API/Core (机制层)  →  NVIC 硬件寄存器
  "谁比谁高"                   "怎么设"              "硬件执行"
```

**优先级分配 (当前)**：
| 优先级 | 中断源 | 理由 |
|:---:|--------|------|
| 0 | SysTick | 系统心跳 |
| 1 | API_TIM | 1ms 控制节拍，所有 PID 回路的心脏 |
| 2 | MPU6050 EXTI | 姿态数据，串级控制外环输入，实时性高于速度环 |
| 3 | 编码器 EXTI | 速度内环反馈 |
| 4 | USART ×3 | 通信（丢包可重传） |

**平台说明**：STM32F407 为 Cortex-M4，4bit NVIC → 0~15 级，每级独立。

**关键设计**：
- `IRQ_PRIO` 宏 = 抢占优先级（数字越小越高，高优先级 ISR 可打断低优先级）
- `IRQ_SUB` 宏 = 响应优先级/子优先级（仅同抢占优先级的中断同时到达时决定顺序，当前未使用填 0）
- PID 计算本身在 main loop 中执行，不是 ISR，无需 NVIC 优先级
- 改优先级只需改一行宏即可生效

**调用链（以 MPU6050 为例）**：
```
IrqPriority.h: #define IRQ_PRIO_MPU6050 2U
    → Enroll.c: API_EXTI_Init(id, trigger, IRQ_PRIO_MPU6050, IRQ_SUB_PRIO_MPU6050)
    → API/exti.c: API_EXTI_CoreInit(port, pin, ..., preemptPriority=2, subPriority=0)
    → Core/f407_exti.c: NVIC_SetPriority(irqn, 2) — 写入硬件寄存器
```

---

## 5. 当前 API 层支持的外设接口

| API 头文件 | 功能 | 说明 |
|-----------|------|------|
| `API/inc/gpio.h` | GPIO 输入/输出 | 直通 `F407_GPIO_*` |
| `API/inc/usart.h` | 串口通信 | 直通 `F407_USART_*` |
| `API/inc/pwm.h` | PWM 输出 | 直通 `F407_PWM_*` |
| `API/inc/tim.h` | 定时器中断 | 直通 `F407_TIM_*` |
| `API/inc/adc.h` | ADC 采集 | 直通 `F407_ADC_*` |
| `API/inc/exti.h` | 外部中断 | 直通 `F407_EXTI_*`（SYSCFG+EXTI+NVIC） |
| `API/inc/Encoder.h` | 编码器接口 | 直通 `F407_Encoder_*`（定时器硬件编码器模式，无需中断） |
| `API/API_I2C/API_I2C.h` | 软件 I2C 协议 (平台无关) | 经 `soft_i2c_hal` 接 `f407_soft_i2c.c` |
| `API/API_SPI/API_SPI.h` | 软件 SPI 协议 (平台无关) | 经 `soft_spi_hal` 接 `f407_soft_spi.c` |

**说明**：
- **Encoder**：F407 使用定时器硬件编码器模式（无需中断），port/pin 参数化——改 `407_hw_config.h` 即可换引脚，无需改 Core 代码。
- **I2C/SPI**：双分层架构——API 层负责协议逻辑（平台无关，始终编译），Core 层负责 GPIO 翻转+延时，中间通过 `soft_i2c_hal.h` / `soft_spi_hal.h` 桥接（内部接口，BSP 不接触）。

---

## 6. BSP 层当前支持的器件

| 模块 | 文件 | 接口类型 |
|------|------|---------|
| LED | `BSP/LED/LED.c` | GPIO 输出 |
| KEY | `BSP/KEY/KEY.c` | GPIO 输入 (消抖) |
| OLED | `BSP/OLED/OLED.c` | SPI / I2C 双模式 |
| MPU6050 | `BSP/MPU6050/MPU6050.c` | I2C + 外部中断 |
| MPU6050 DMP | `BSP/MPU6050/eMPL/` | InvenSense 官方 DMP 库 |
| TB6612 | `BSP/TB6612/TB6612.c` | PWM + GPIO |
| NRF24L01 | `BSP/NRF24L01/NRF24L01.c` | SPI |
| BMP280 | `BSP/BMP280/BMP280.c` | I2C |
| QMC5883P | `BSP/QMC5883P/QMC5883P.c` | I2C |

---

## 7. SYSTEM 层与 Core exti 文件角色说明

### F407 的 exti/sys 分工

| 文件 | 实际作用 |
|-----|------|
| `Core/STM32F407/src/f407_exti.c` | EXTI 初始化实现（SYSCFG+EXTI+NVIC） |
| `Core/STM32F407/inc/f407_exti.h` | EXTI 函数声明 |
| `Core/STM32F407/f407_delay.c` | 延时实现（Delay_us/ms/s） |
| （无时钟 sys.c） | 不需要 — STM32 启动时 `SystemInit()` 已配好时钟 |

**关键点**：STM32 的 `SystemInit()` 在启动文件中由 CMSIS 标准库自动调用，所以 Core 层不需要单独的时钟初始化代码，`SYSTEM/SYS_Init()` 对 F407 是空操作。

### SYSTEM 层 vs Core 层分工

```
SYSTEM/sys.c                          ← 平台门面层
  ├─ SYS_Init()                       → F407: 空（SystemInit 已做）
  ├─ SYS_EXTI_GetIrqn(port, pin)      → 引脚线号 → EXTIn_IRQn
  └─ SYS_EXTI_GetLineIndex(pin)       → 引脚掩码 → 0~15 线号

Core/STM32F407/src/f407_exti.c         ← EXTI 硬寄存器实现（SYSCFG 体系）
```

---

## 8. 关键文件索引

### 构建系统
| 文件 | 作用 |
|------|------|
| `CMakeLists.txt` | 统一构建入口（已内联 F407 平台配置） |
| `CMakePresets.json` | CMake 预设 (Debug/Release) |
| `gcc-arm-none-eabi.cmake` | ARM GCC 交叉编译工具链 |
| `.vscode/tasks.json` | VS Code 构建/烧录任务 |
| `.vscode/settings.json` | VS Code 配置 |

### 注册层 (最需要关注的目录)
| 文件 | 作用 |
|------|------|
| `Enroll/Enroll.h` | 注册层对外接口 |
| `Enroll/Enroll.c` | X-Macro 展开 + Register 门面函数 |
| `Enroll/Enroll_Internal.h` | 仅 Enroll.c 使用的内部依赖 |
| `Enroll/407_hw_config.h` | **F407 板级引脚映射（改接线/引脚看这里）** |

### SYSTEM 层与 Core exti 文件
| 文件 | 实际作用 |
|------|---------|
| `SYSTEM/sys.c` | SYS_Init/EXTI_GetIrqn/LineIndex |
| `SYSTEM/sys.h` | 系统初始化和 EXTI 辅助接口声明 |
| `SYSTEM/Delay.h` | 统一延时接口（Delay_us/ms/s） |
| `SYSTEM/BusRate.h` | 软件总线选择+速率集中配置 |
| `SYSTEM/IrqPriority.h` | NVIC 中断优先级统一管理 |
| `Core/STM32F407/src/f407_exti.c` | F407 EXTI 实现（SYSCFG+EXTI+NVIC） |

### 应用层核心
| 文件 | 作用 |
|------|------|
| `A_Entry/main.c` | 程序入口，完整的初始化流程 + 主循环 |
| `app/Control_Task/` | 控制任务调度 + 中断回调 |
| `app/PID/` | PID 控制器实现 |
| `app/Filter/` | 滤波器实现 |
| `API/API_I2C/` | 软件 I2C 协议层 (平台无关) |
| `API/API_SPI/` | 软件 SPI 协议层 (平台无关) |
| `Core/STM32F407/f407_soft_i2c/` | F407 I2C GPIO 翻转+延时 |
| `Core/STM32F407/f407_soft_spi/` | F407 SPI GPIO 翻转+延时 |
| `app/My_Usart/` | 串口打印封装 |

---

## 9. 开发约定与命名规范

### 函数命名
- `API_xxx_*` — API 层对外接口 (如 `API_GPIO_Write`)
- `F407_xxx_*` — F407 Core 层实现
- `Enroll_xxx_*` — 注册层门面函数
- `API_I2C_*` / `API_SPI_*` — 软件总线协议层
- `soft_i2c_hal_*` / `soft_spi_hal_*` — 总线 HAL 桥接接口 (由 Core 层实现)

### 文件组织
- 每个外设模块在自己的目录下，含 `.c` + `.h`
- API 层: `inc/` 放头文件，`src/` 放实现
- Core 层: `inc/` 放头文件，`src/` 放实现，`cmake/` 放链接脚本
- BSP 模块: 头文件与源文件平级放在模块目录

### 配置集中化
- 总线选择+速率收至 `SYSTEM/BusRate.h`
- 中断优先级收至 `SYSTEM/IrqPriority.h`
- 板级引脚映射收至 `Enroll/407_hw_config.h`

---

## 10. 新增器件 / 外设的接入步骤

本工程已固定 F407，新增器件（而非新 MCU）时遵循框架规范：

1. **BSP 层** — 在 `BSP/` 下新增器件目录，协议逻辑只调 `API_I2C.h`/`API_SPI.h`/`API_*`。
2. **Enroll 层** — 在 `407_hw_config.h` 新增 `HW_xxx_MAP` 板级映射宏（引脚/外设实例）。
3. **CMakeLists.txt** — 在 `BSP_LAYER_SOURCES` 与 `MCU_INCLUDE_DIRS` 登记新源文件与头文件目录。
4. **SYSTEM/BusRate.h** — 若是 I2C/SPI 设备，新增该设备的总线选择+速率宏。
5. **Core 层** — 仅当需要全新的 GPIO 翻转类底层（如新型总线）时才在 `Core/STM32F407/` 新增。

> 若未来确需支持另一款 MCU，请参考原始多 MCU 版 OmniLayer 框架仓库，重新引入 `ENROLL_MCU_TARGET` 分发与对应 Core/Drivers。

---

## 11. 当前工程状态

### 已完成 (近期)
- **精简为 F407 单主控**：删除 F103/G3507 的 Core/Drivers/板级配置/OpenOCD 配置，折叠所有 `#if ENROLL_MCU_TARGET` 分支为纯 F407，移除多 MCU 分发宏。构建产物与精简前逐字节一致（FLASH 53964 B / RAM 5584 B）。
- **I2C/SPI 架构重构 (V3.1)**：协议层 (`API/API_I2C`、`API/API_SPI`) + Core 底层 (`f407_soft_i2c`/`f407_soft_spi`) 双分层，经 `soft_i2c_hal.h`/`soft_spi_hal.h` 桥接。
- **BSP 统一到 API**：OLED/NRF24L01 等所有 BSP 设备只通过 `API_I2C.h` / `API_SPI.h` 操作总线。
- **OLED 驱动重构**：基于江协科技参考代码重写，I2C 模式走标准协议（SendByte + Wait_Ack）。
- **配置集中化**：总线选择+速率收至 `SYSTEM/BusRate.h`，中断优先级收至 `SYSTEM/IrqPriority.h`。
- **编码器 (Encoder)**：定时器硬件编码器模式，通用 port/pin 参数化（改 hw_config 即生效）。
- **速度环 PID**：基于现有 PID 库完成左右轮速度闭环控制
  - `PID_EncoderSpeed_t` 内部左右独立 `PID_TypeDef`，共用 kp/ki/kd
  - **关键经验**：ki 值需要乘以 `1/dt` 倍补偿 dt 因子（库中 error_sum 乘以 dt），否则 I 项积累过慢
  - 电机系统通常不需要 D 项（kd=0），微分噪声放大导致卡顿
  - `Out_max` 必须匹配 TB6612_MAX_DUTY（400），否则反积分饱和失效
- **中断优先级统一管理**：`SYSTEM/IrqPriority.h` 集中定义抢占/响应优先级宏。
- TB6612 电机驱动、KEY 驱动修复多路返回值/消抖逻辑。

### 注意事项
- FreeRTOS-LTS、USB 协议库源码不再同步上传至 GitHub。
- 工程保留 `MDK_ARM/MDK_ARM_F407` 用于 Keil IDE 兼容，但不保证最新。
- D 项（kd）在直流电机速度环中容易放大编码器量化噪声导致卡顿，通常设为 0。

---

## 12. 快速上手检查清单

为新对话恢复认知时，按以下顺序阅读关键文件：

1. ✅ 本文档 (docs/arch-guide.md) — 架构全貌
2. `README.md` — 项目简介与构建命令
3. `CMakeLists.txt` — 理解源文件组织（F407 配置已内联）
4. `Enroll/Enroll.h` — 注册层接口全览
5. `Enroll/407_hw_config.h` — 当前（唯一）板级映射
6. `A_Entry/main.c` — 典型的初始化流程
7. `SYSTEM/IrqPriority.h` — 中断优先级策略
8. `SYSTEM/BusRate.h` — 软件总线速率策略
9. 任一 `API/src/*.c` — 理解 API→Core 直通关系
10. 任一 `Core/STM32F407/src/*.c` — 理解 Core 层实现风格
