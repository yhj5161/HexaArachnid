# HexaArachnid — 六足蜘蛛机器人固件

STM32F407 + FreeRTOS 六足 hexapod 机器人固件工程，CMake + GCC + OpenOCD 统一构建烧录。

18 路舵机（6 腿 × 3 关节），2 块 PCA9685 I2C 驱动，FK/IK 运动学 + 14 种步态模式。

> 本工程为 F407 单主控专用，API 层直接对接 F407 Core 实现。架构核心理念："同一套业务代码，芯片变了只换 Core 层"。

## 🚀 项目定位

六足蜘蛛机器人一体化固件——从舵机驱动到步态引擎，从姿态传感器到无线遥控，全部集成在一个 FreeRTOS 工程里。

- 分层清晰，应用层不直接碰寄存器
- CMake 一键编译烧录，VS Code F7/F8
- 软 I2C/SPI 总线，引脚配置集中管理

## 🦿 运动控制架构

```
app/Hexa/
├─ config.h           # 连杆参数、安装位置
├─ base.h             # Point3D、Locations 基础类型
├─ servo.h/.c         # 舵机角度→脉宽映射
├─ leg.h/.c           # 单腿 FK/IK 运动学
├─ movement.h/.c      # 步态引擎（14 种模式 + 插值状态机）
├─ movement_table.h   # pathTool 生成的动作表（P1X~P6Z 宏）
├─ movement_profile.h/.c  # 各模式运动参数
├─ hexapod.h/.c       # 整机控制 + 校准持久化
└─ command.h/.c       # 字符串↔枚举指令解析
```

## 📁 项目结构

```text
HexaArachnid/
├─ A_Entry/                    # 程序入口 (main.c + FreeRTOS 任务)
├─ API/                        # MCU 片内外设抽象接口层
│  ├─ inc/                     # gpio/adc/pwm/tim/usart/exti
│  ├─ src/                     # API 实现
│  ├─ API_I2C/                 # 软 I2C 协议层（多总线互斥锁）
│  └─ API_SPI/                 # 软 SPI 协议层（模式 0）
├─ app/                        # 应用层
│  ├─ Hexa/                    # 六足运动控制库（FK/IK + 步态引擎）
│  ├─ Control_Task/            # 控制任务（按键/步态节拍/中断回调）
│  ├─ Filter/                  # 滤波器
│  ├─ PID/                     # PID 控制器
│  └─ My_Usart/                # 串口管理（收发缓冲 + USART1/3/4/5）
├─ BSP/                        # 板级支持层
│  ├─ PCA9685/                 # 16 路 PWM 舵机驱动（软 I2C, 50Hz）
│  ├─ JY61P/                   # 维特六轴姿态模块（串口主动上报）
│  ├─ SU03T/                   # 离线语音模块（环形缓冲）
│  ├─ NRF24L01/                # 2.4G 无线模块（软 SPI）
│  ├─ OLED/                    # I2C OLED 显示屏
│  ├─ KEY/                     # 按键扫描（消抖）
│  ├─ LED/                     # 状态指示灯
│  ├─ HCSR04/                  # 超声波测距
│  ├─ BMP280/                  # 气压传感器
│  └─ QMC5883P/                # 地磁传感器
├─ Core/STM32F407/             # F407 底层实现
├─ Drivers/Drivers_STM32F4/    # 启动文件、CMSIS、标准外设库
├─ Enroll/                     # 硬件资源注册（407_hw_config.h）
├─ FreeRTOS/                   # FreeRTOS V11.1.0 内核
├─ RTOS/                       # FreeRTOSConfig.h
├─ SYSTEM/                     # 系统初始化、延时、中断优先级、总线配置
├─ OpenOCD/                    # 烧录配置
└─ docs/                       # 文档
```

## 🏗️ 分层说明

| 层级 | 目录 | 职责 |
|---|---|---|
| 入口层 | `A_Entry/` | main.c + FreeRTOS 任务创建 |
| 应用层 | `app/` | 运动控制、步态引擎、任务逻辑 |
| 接口层 | `API/` | 统一外设接口 + I2C/SPI 协议层 |
| 板级层 | `BSP/` | 板载器件驱动封装 |
| 注册层 | `Enroll/` | 引脚映射与资源注册 |
| 核心层 | `Core/STM32F407/` | F407 底层 GPIO/TIM/USART/I2C/SPI 实现 |
| 系统层 | `SYSTEM/` | 时钟、延时、中断优先级、总线速率 |
| 驱动层 | `Drivers/` | 启动文件、CMSIS |

## 🦾 舵机布局

| 板 | I2C 总线 | 舵机 |
|----|---------|------|
| PCA9685 #1 | I2C3 (PC2/PC0) | 左侧 3 腿 × 3 关节 |
| PCA9685 #2 | I2C4 (PC4/PB0) | 右侧 3 腿 × 3 关节 |

## 🎮 串口分工（全 115200）

| 串口 | 引脚 | 用途 |
|------|------|------|
| USART1 | PB6/PB7 | JY61P 姿态模块 |
| USART3 | PD8/PD9 | 调试打印 |
| UART4 | PA0/PA1 | 备用调试口 |
| UART5 | PC12/PD2 | SU-03T 离线语音 |

## ⚙️ 构建与烧录

- `F7`：编译，`F8`：烧录
- 命令行：`cmake --preset Debug && cmake --build --preset Debug`

## 🎯 设计原则

- 业务逻辑尽量不直接操作寄存器
- 改板级接线优先改 `Enroll/407_hw_config.h`
- BSP 接口稳定，芯片变更只换 Core 层
- 性能优先——软 I2C/SPI 拆两层，协议复用、GPIO 零开销

## 📦 分支策略

| 分支 | 定位 |
|---|---|
| `main` | 开发主线（FreeRTOS + 六足运动控制） |

## 📮 联系

QQ 邮箱：2115951478@qq.com