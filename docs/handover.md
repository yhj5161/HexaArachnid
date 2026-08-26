# QuadArachnid 交接文档

> 固件工程名：**OmniLayer**（多 MCU 分层架构框架）
> 机器人形态：蜘蛛型（四足）机器人
> 当前主控：**STM32F407**
> 文档日期：2026-08-26 ｜ 分支：`main` ｜ 最近提交：`0d867c5 更改了芯片配置`

---

## 1. 一句话现状

工程已切换到 **STM32F407** 作为默认目标，**编译、链接、烧录链路全部打通**（实测通过）。
但**应用层目前还是"平衡车/轮式"模板**（TB6612 直流电机 + 编码器 + MPU6050 姿态 + 速度环 PID），**蜘蛛机器人特有的腿部/舵机/步态/逆运动学代码尚未编写**，是接下来要做的核心工作。

---

## 2. 项目是什么

- OmniLayer 是一个**多 MCU、可迁移的分层嵌入式框架**，核心理念是"分层不是目的，隔离变化才是"——同一套业务代码，按目标芯片切换底层实现。
- 当前支持的芯片：**STM32F103 / STM32F407 / TI MSPM0G3507**。
- 本仓库 `QuadArachnid` 用这套框架来做蜘蛛机器人，主控定为 **F407**。
- 详细架构设计请参阅 [`README.md`](../README.md) 与 [`docs/arch-guide.md`](arch-guide.md)，本文档不重复，只讲"交接时需要知道的现状与操作"。

---

## 3. 分层速查（给接手人）

| 层级 | 目录 | 职责 | 换芯片时要不要改 |
|---|---|---|---|
| 入口层 | `A_Entry/main.c` | 唯一 main，初始化与主循环 | 一般不动 |
| 应用层 | `app/` | 业务逻辑：Control / Control_Task / PID / Filter / My_Usart | 不动 |
| 接口层 | `API/` | 统一片内外设接口（gpio/adc/pwm/tim/usart/exti/encoder）+ I2C/SPI 协议层 | 不动 |
| 板级层 | `BSP/` | 板载器件封装：OLED/MPU6050/QMC5883P/BMP280/NRF24L01/TB6612/LED/KEY | 按需增删 |
| 注册层 | `Enroll/` | **核心思想**：把板级资源映射到具体 MCU 引脚/外设实例 | **主要改这里** |
| 核心层 | `Core/` | 各 MCU 的底层实现（`STM32F103`/`STM32F407`/`MSPM0G3507`） | 换芯片换这套 |
| 系统层 | `SYSTEM/` | 时钟/中断分发/延时/总线速率/中断优先级 | 基本不动 |
| 驱动资源层 | `Drivers/` | 启动文件、CMSIS/标准库 | 换芯片换这套 |
| 中间件层 | `Middlewares/` | FreeRTOS、USB（**不上传仓库，需自行获取**） | — |

**上手最关键的两个文件**：
- [`Enroll/Enroll.h`](../Enroll/Enroll.h) — 第 28 行 `#define ENROLL_MCU_TARGET` 决定默认芯片（当前 = `ENROLL_MCU_F407`）。
- [`Enroll/407_hw_config.h`](../Enroll/407_hw_config.h) — F407 的板级引脚/外设映射表，接线/改引脚看这里。

---

## 4. 构建 / 烧录 / 切换芯片

工具链：CMake + Ninja + gcc-arm-none-eabi + OpenOCD（VS Code / Trae）。

### 4.1 VS Code 快捷键（推荐）

| 快捷键 | 作用 | 说明 |
|---|---|---|
| `F7` | 编译 | 走默认 `Debug` 预设（AUTO → 读 Enroll.h 的默认芯片） |
| `F8` | 烧录 | 先编译再 OpenOCD 烧录 |
| `Ctrl+Shift+F1` | 选芯片编译 | 弹下拉框，临时选 F103/F407/G3507，不改默认 |
| `Ctrl+Shift+F2` | 选芯片烧录 | 同上 |
| `Ctrl+Shift+F3` | **设定默认芯片** | 弹框选择后写回 `Enroll.h` 的 `ENROLL_MCU_TARGET` |

> 三个下拉框的默认项当前都已设为 **F407**，基本一路回车即可。

### 4.2 命令行

```bash
cmake --preset Debug          # 配置（AUTO 读 Enroll.h，当前解析为 F407）
cmake --build --preset Debug  # 编译
# 或用指定芯片的预设：Debug-F103 / Debug-F407 / Debug-G3507
```

烧录/擦除走 OpenOCD 目标：`flash` / `erase`（配置在 `OpenOCD/F407_OpenOCD.cfg`）。

### 4.3 切换默认芯片的两种方式

1. **快捷键**：`Ctrl+Shift+F3` 选目标（脚本 `.vscode/set-default-mcu-target.ps1` 自动改写 Enroll.h）。
2. **手动**：改 [`Enroll/Enroll.h`](../Enroll/Enroll.h) 第 28 行的 `ENROLL_MCU_TARGET` 为 `ENROLL_MCU_F103/F407/G3507`，再重新配置。

> 切换后务必重新 `configure`（`F7` 会自动先 configure），AUTO 模式每次配置都会重读 Enroll.h。

---

## 5. 最近一次改动（本次交接的变更）

主题：**把默认主控从 MSPM0G3507 切到 STM32F407**（对应提交 `0d867c5`）。

| 文件 | 改动 |
|---|---|
| [`Enroll/Enroll.h`](../Enroll/Enroll.h) | 第 28 行默认目标 `ENROLL_MCU_G3507` → `ENROLL_MCU_F407` |
| [`.vscode/tasks.json`](../.vscode/tasks.json) | 三个下拉框默认项 `Debug-G3507`/`ENROLL_MCU_G3507` → `Debug-F407`/`ENROLL_MCU_F407` |

**改动原因**：原默认目标是 G3507，但仓库里**没有 TI 的 SDK 源码**（`Drivers/Drivers_M0G3507/` 目录不存在），导致 CMake 报 `Cannot find source file ... startup_mspm0g350x_gcc.c`。而本项目主控就是 F407，故直接切到 F407。

**验证结果（实测）**：
```
Resolved MCU target: F407
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

1. **G3507 暂不可编译**：`Drivers/Drivers_M0G3507/`（TI SDK）不在仓库中。如需编译 G3507，须先从 TI 官网获取 MSPM0 SDK 并补到该目录。`.vscode/settings.json` 已把该路径排除索引。
2. **Middlewares 不入库**：FreeRTOS-LTS、USB 协议栈、TI SDK 等不上传 GitHub，需自行到官网获取（见 README 注意事项）。
3. **Keil 工程不同步**：`MDK_ARM/` 保留兼容但不保证最新，主力环境是 VS Code + CMake；用 Keil 需自行补齐缺失配置。
4. **PWM 配置待核对**：`main.c` 第 64 行 `API_PWM_Init(API_PWM_TIM1, 400-1, 8-1)` 是注释里标注的 G3507 参数；F407 若要驱动舵机通常用 50Hz（注释建议 `ARR=4000-1, PSC=840-1`）。**接蜘蛛舵机前请按 F407 重新核算定时器参数。**
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
4. 改板级接线/引脚：改 `Enroll/407_hw_config.h`；改默认芯片：`Ctrl+Shift+F3`。
5. 遇到问题先查第 7 节"已知问题"。

---

*维护联系：QQ 邮箱 2115951478@qq.com（见 README）*
