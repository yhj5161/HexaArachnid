# QuadArachnid 交接文档

> 固件工程名：**OmniLayer**（分层架构框架）
> 机器人形态：蜘蛛型（四足）机器人
> 当前主控：**STM32F407（已精简为唯一主控，不再支持多 MCU 切换）**
> 文档日期：2026-08-26 ｜ 分支：`main`

---

## 1. 一句话现状

工程已切换到 **STM32F407** 作为目标，**编译、链接、烧录链路全部打通**（实测通过）。**本工程已进一步精简为 F407 单主控**：F103 与 G3507 的底层代码（Core/Drivers）、板级配置、OpenOCD 配置及构建/代码中的多 MCU 分支已全部移除（详见第 5 节）。但**应用层目前还是"平衡车/轮式"模板**（TB6612 直流电机 + 编码器 + MPU6050 姿态 + 速度环 PID），**蜘蛛机器人特有的腿部/舵机/步态/逆运动学代码尚未编写**，是接下来要做的核心工作。

---

## 2. 项目是什么

- OmniLayer 原本是一个**多 MCU、可迁移的分层嵌入式框架**，核心理念是"分层不是目的，隔离变化才是"。
- **本仓库 `QuadArachnid` 已将其裁剪为 F407 单主控专用**，用于蜘蛛机器人。多 MCU 分发（`ENROLL_MCU_TARGET`）已移除，API 层直接对接 F407 Core 实现。
- 详细架构设计请参阅 [`README.md`](../README.md) 与 [`docs/arch-guide.md`](arch-guide.md)。⚠️ 注意：这两份文档仍按"多 MCU 框架"描述，其中 F103/G3507 相关内容已不适用于本仓库，以本文档为准。

---

## 3. 分层速查（给接手人）

| 层级 | 目录 | 职责 |
|---|---|---|
| 入口层 | `A_Entry/main.c` | 唯一 main，初始化与主循环 |
| 应用层 | `app/` | 业务逻辑：Control / Control_Task / PID / Filter / My_Usart |
| 接口层 | `API/` | 统一片内外设接口（gpio/adc/pwm/tim/usart/exti/encoder）+ I2C/SPI 协议层（现直接对接 F407） |
| 板级层 | `BSP/` | 板载器件封装：OLED/MPU6050/QMC5883P/BMP280/NRF24L01/TB6612/LED/KEY |
| 注册层 | `Enroll/` | **核心思想**：把板级资源映射到 F407 引脚/外设实例，**改引脚/接线主要改这里** |
| 核心层 | `Core/STM32F407/` | F407 底层实现（仅剩这一套） |
| 系统层 | `SYSTEM/` | 时钟/中断分发/延时/总线速率/中断优先级 |
| 驱动资源层 | `Drivers/Drivers_STM32F4/` | F407 启动文件、CMSIS/标准库 |
| 中间件层 | `Middlewares/` | FreeRTOS、USB（**不上传仓库，需自行获取**） |

**上手最关键的文件**：
- [`Enroll/407_hw_config.h`](../Enroll/407_hw_config.h) — F407 的板级引脚/外设映射表，接线/改引脚看这里。

---

## 4. 构建 / 烧录

工具链：CMake + Ninja + gcc-arm-none-eabi + OpenOCD（VS Code / Trae）。**已固定为 F407，无需也无法再切换芯片。**

### 4.1 VS Code 快捷键

| 快捷键 | 作用 | 说明 |
|---|---|---|
| `F7` | 编译 | 走 `Debug` 预设，直接构建 F407 |
| `F8` | 烧录 | 先编译再 OpenOCD 烧录 |

> 原先的"选芯片编译/烧录/设默认芯片"快捷键（Ctrl+Shift+F1/F2/F3）已随多 MCU 支持一并移除。

### 4.2 命令行

```bash
cmake --preset Debug          # 配置
cmake --build --preset Debug  # 编译（产物 OmniLayer_F407.elf）
```

烧录/擦除走 OpenOCD 目标：`flash` / `erase`（配置在 `OpenOCD/F407_OpenOCD.cfg`）。

---

## 5. 最近一次改动（本次交接的变更）

主题：**把工程从"多 MCU 框架"精简为 F407 单主控专用**（删除 F103 + G3507 底层）。

**删除的文件/目录：**
| 项 | 内容 |
|---|---|
| `Core/STM32F103/` | F103 核心层（gpio/usart/tim/pwm/adc/exti/Encoder + soft_i2c/soft_spi + delay + 链接脚本） |
| `Core/MSPM0G3507/` | G3507 核心层（同上） |
| `Drivers/Drivers_STM32F1/` | F1 标准库 + 启动文件 |
| `Enroll/103_hw_config.h`、`G3507_hw_config.h`、`G3507_pinmux.h` | F103/G3507 板级映射 |
| `OpenOCD/F103_OpenOCD.cfg`、`G3507_OpenOCD.cfg` | F103/G3507 烧录配置 |
| `MDK_ARM/MDK_ARM_F103/` | Keil 的 F103 工程 |
| `.vscode/set-default-mcu-target.ps1` | 切换默认 MCU 的脚本（已无意义） |

**精简的共享代码（删除 `#if ENROLL_MCU_TARGET == F103/G3507` 分支，折叠为纯 F407）：**
- `Enroll/Enroll.h`：移除 MCU 目标常量与 `#if` 选配置，固定 `#include "407_hw_config.h"`。
- `API/inc/*.h` + `API/src/*.c`：删除 F103/G3507 的 `#include` 与函数分发分支，直接调 `F407_*`。
- `SYSTEM/sys.c`/`IrqPriority.h`/`BusRate.h`：删除 G3507 分支，保留 F407 配置。
- `app/My_Usart/*`、`app/Control_Task/*`、`A_Entry/main.c`：删除 G3507/F103 相关分支与注释。
- `BSP/MPU6050/MPU6050.h`：DMP 参数折叠为 STM32 值。

**构建系统：**
- `CMakeLists.txt`：移除 `MCU_TARGET`/AUTO 检测与 F103/G3507 分支，F407 配置直接内联；不再定义 `ENROLL_MCU_*` 宏。
- `CMakePresets.json`：删除 `Debug-F103/F407/G3507`，保留 `Debug`/`Release`。
- `.vscode/tasks.json`/`keybindings.json`：删除选芯片任务与快捷键，保留 `F7`编译/`F8`烧录。
- `.vscode/settings.json`/`c_cpp_properties.json`：删除 F103/G3507 相关路径。

**验证结果（实测，与精简前逐字节一致）：**
```
[42/42] Linking C executable artifacts/OmniLayer_F407.elf
FLASH: 53964 B / 512 KB (10.29%)
RAM:    5584 B / 128 KB (4.26%)
```

---

## 6. 当前应用层状态（重要）

[`A_Entry/main.c`](../A_Entry/main.c) 现在跑的是一套**平衡车/轮式机器人**的模板逻辑，**不是蜘蛛机器人代码**：

- 姿态：MPU6050 DMP 解算 Pitch/Roll/Yaw（OLED + 串口实时显示）。
- 运动：TB6612 直流电机 + 双编码器 + 速度环 PID。
- 通信：USART1/2/3，含摄像头数据包解析（`s88,-93,104e` 格式）。
- 调度：TIM1=PID 节拍、TIM2=编码器节拍、TIM3=杂务节拍，主循环按 flag 分时执行。

**结论**：蜘蛛机器人的**腿部舵机驱动、逆运动学（IK）、步态规划**等功能**尚未实现**，需要在现有框架上新增。当前 main.c 里大量测试代码被注释，可作为接入新功能的参考骨架。

---

## 7. 已知问题与注意事项

1. **已精简为 F407 单主控**：F103/G3507 的代码、配置与构建选项已全部移除，无法再切换芯片。若未来需要多 MCU，请参考原始 OmniLayer 框架仓库重新引入。
2. **Middlewares 不入库**：FreeRTOS-LTS、USB 协议栈等不上传 GitHub，需自行到官网获取（见 README 注意事项）。
3. **Keil 工程不同步**：`MDK_ARM/` 仅保留 `MDK_ARM_F407`，但不保证最新，主力环境是 VS Code + CMake；用 Keil 需自行补齐缺失配置。
4. **PWM 配置待核对**：`main.c` 中 `API_PWM_Init(API_PWM_TIM1, 400-1, 8-1)` 是旧 G3507 参数；F407 若要驱动舵机通常用 50Hz（建议 `ARR=4000-1, PSC=840-1`）。**接蜘蛛舵机前请按 F407 重新核算定时器参数。**
5. **分支策略**：`main` 为裸机主线，`FreeRTOS` 为 RTOS 主线；蜘蛛机器人若上 RTOS 需切到对应分支推进。

---

## 8. 下一步计划（蜘蛛机器人方向）

建议按此顺序推进：

1. **舵机驱动层**：确定舵机数量/控制方式（F407 定时器直接 PWM，或外挂 PCA9685），新增对应 BSP/Core 支持。
2. **单腿逆运动学（IK）**：建立腿部几何模型，实现"足端坐标 → 各关节角度"解算。
3. **步态规划**：实现三脚步态（tripod）等基础步态，协调四腿时序。
4. **姿态融合**：复用现有 MPU6050 DMP 姿态，做机身自平衡/姿态补偿。
5. **遥控/上位机**：基于 NRF24L01 或串口做运动指令下发。

> 新增器件时遵循框架规范：协议逻辑放 `API` 层、GPIO 翻转放 `Core` 层、板级映射写 `Enroll/407_hw_config.h`，并在 `CMakeLists.txt` 的 F407 分支登记新源文件。

---

## 9. 给接手人 / AI 的快速上手

1. 先读本文件 → 再看 [`README.md`](../README.md) → 深入看 [`docs/arch-guide.md`](arch-guide.md)。
2. 确认环境：装好 `gcc-arm-none-eabi`、`Ninja`、`OpenOCD`，VS Code 装 CMake Tools。
3. 按 `F7` 应能直接编译出 `OmniLayer_F407.elf`（Flash 占用约 10%）。
4. 改板级接线/引脚：改 `Enroll/407_hw_config.h`（本工程已固定 F407，无需切换芯片）。
5. 遇到问题先查第 7 节"已知问题"。

---

*维护联系：QQ 邮箱 2115951478@qq.com（见 README）*
