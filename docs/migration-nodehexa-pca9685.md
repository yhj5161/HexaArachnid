# NodeHexa → QuadArachnid：PCA9685 舵机驱动迁移交接文档

> 源工程（只读参考）：`D:\Desktop\NodeHexa`（ESP32 + Arduino + C++，六足）
> 目标工程（本次要改）：`C:\QuadArachnid`（STM32F407 + C + FreeRTOS，分层框架 OmniLayer）
> 本文档日期：2026-09-03
> 交接范围：**PCA9685 舵机驱动 BSP 层迁移 + 两路 I2C 总线规划 + 装机前舵机中位校准 + 冒烟测试**
> 交接对象：接手本工程的 AI（独立执行）；人负责按文核对硬件接线与机械安装。

> ⚠️ 本文档**不包含**运动控制算法（步态/运动学 FK·IK/动作表）的迁移——那是下一份文档的事。但第 8 节给出算法迁移必须保留的"角度→脉宽"映射与腿-板-通道表，避免返工。

---

## 1. 任务一句话

在 `QuadArachnid` 里新增一个符合本项目分层规范的 `BSP/PCA9685` 驱动（纯 C，跑在现有 `API_I2C` 软件 I2C 之上），让两块 PCA9685（每块各带 3 条腿，共 18 个 MG90S 舵机）能被 F407 控制，并用它完成"装机前把每个舵机打到 1500µs 物理中位"的校准与逐通道冒烟测试。

**验收标准（做完这些才算成功）：**

1. `BSP/PCA9685/PCA9685.c` 登记进 CMake 后，工程能编译链接通过（F7）。
2. 开机 I2C 扫描（现有 `App_I2C_ScanOnce()`）在 **API_I2C1 上恰好看到 1 个 0x40、API_I2C2 上恰好看到 1 个 0x40**。
3. `PCA9685_Init()` 对两块板都返回 true（能读到并写通 MODE1/PRE_SCALE）。
4. 一块板所有 16 路输出 1500µs（≈307 tick）后，接入的舵机全部转到物理中位、摇臂可 90° 垂直——此时执行"装机校准"。
5. 单通道 ±20° 摆动能驱动对应物理关节动，且逐通道都确认过（见 §9 冒烟表）。

---

## 2. 源工程里到底有什么可抄、有什么别抄

先读源工程这几个文件（只读，不改）：

| 源文件 | 内容 | 迁移时 |
|---|---|---|
| `firmware/src/servo.cpp` | PCA9685 频率、`us→tick` 换算、Servo 角度→脉宽映射、`hexapod2pwm()` 腿/关节→通道 | **通道表与角度映射抄进文档 §8**；Adafruit 调用不抄 |
| `firmware/lib/hal/pwm.cpp` | `hal::PCA9685` 只是 `Adafruit_PWMServoDriver` 的 `void*` 包装 | **不抄**（真驱动是 Arduino 库依赖 `adafruit/Adafruit PWM Servo Driver Library`，**没有随仓库提供，无源文件可拷**，我们按 §5 寄存器协议手写） |
| `firmware/src/leg.cpp` | 单腿 FK/IK、腿安装角 | 范围外（运动学文档） |
| `firmware/src/movement*.cpp/h`、`generated/movement_table.h` | 步态模式机 + 动作表 | 范围外 |
| `firmware/include/config.h` | 连杆长度、安装位置、`movementInterval=20` | 范围外；**警告见 §10.8** |

---

## 3. 目标工程的落点与既有约定（接手前必读）

在写任何代码前，先看这几个文件形成手感，文档后面的代码完全照此风格：

- `Enroll/407_hw_config.h` —— 板级引脚/外设映射表。**当前两路软件 I2C：I2C1=PB8/PB9（传感器），I2C2=PA7/PA5（OLED）**。
- `SYSTEM/BusRate.h` —— 所有 I2C/SPI 设备的**总线选择与速率档位集中在这里**（不要在 BSP 的 .h 里单独定义总线）。
- `BSP/BMP280/BMP280.h/.c` —— BSP 模板：`extern "C"` 包裹、8-bit 写地址宏、静态 `xxx_SelectI2CSpeed()`（`API_I2C_SelectBus + API_I2C_SetSpeed`）、用 `API_I2C_Start/SendByte/Wait_Ack/Stop` 拼事务。
- `BSP/OLED/OLED.c`（约 155–215 行）—— **FreeRTOS 环境下访问软件 I2C 的正确姿势**：每次访问 `API_I2C_Lock()` → `SelectBus` → 事务 → `API_I2C_Unlock()`。因为软件 I2C 的"当前总线/速率"是**全局状态**，多任务并发时必须在锁内重新选总线。
- `A_Entry/main.c` —— 启动流程：先一堆 `Enroll_xxx_Register()`，再 `API_I2C_Init()`，再各 BSP Init，最后建任务。当前是轮式机器人模板，PCA9685 是新增内容。
- `CMakeLists.txt` —— BSP 源文件加在 `BSP_LAYER_SOURCES`（127 行附近），头文件目录加在 `MCU_INCLUDE_DIRS`（141 行附近）。
- `API/API_I2C/API_I2C.h` —— API：`API_I2C_Lock/Unlock/SelectBus/SetSpeed/Start/Stop/SendByte/ReceiveByte/Wait_Ack`（**`Wait_Ack` 返回 0=收到 ACK，1=超时**）；总线枚举只有 `API_I2C1 / API_I2C2` 两路（`API_I2C_MAX=2`）。延时用 `#include "Delay.h"` 里的 `Delay_us()/Delay_ms()`。

---

## 4. 硬件与总线规划（先定，代码后写）

### 4.1 两块板怎么挂

- 用户硬件是两块 PCA9685，各带 3 条腿（共 6 腿 18 舵机），每块板用**独立的一套 I2C 引脚** → 正好对应工程的 `API_I2C1` 和 `API_I2C2`。
- **两块板的 A0~A5 地址脚全部接地 → 7-bit 地址都是 0x40（8-bit 写地址 0x80）**。分挂不同总线就不会冲突，**不需要焊接 SJ1 改地址跳帽**（源工程 `servo.cpp` 注释里说左侧板要跳帽是它俩同挂一条总线才需要，我们没有这个问题）。
- **红线：同一条总线上绝不能同时出现两个 0x40**（一旦接线串线/混插，两条总线都会乱）。

推荐默认分配（与源工程一致的分组含义）：

| 板 | 软件总线 | 引脚（当前定义） | 驱动哪几条腿 | 备注 |
|---|---|---|---|---|
| PCA9685-A | `API_I2C1` | PB8(SCL)/PB9(SDA) | leg0/1/2（右三腿：前/中/后） | 当前 BMP/QMC 也在这条总线，地址不同可共存（见 4.3） |
| PCA9685-B | `API_I2C2` | PA7(SCL)/PA5(SDA) | leg3/4/5（左三腿：后/中/前） | 当前 OLED 也在这条总线，地址不同可共存 |

> 如果你实际接线的引脚不是 PB8/PB9、PA7/PA5，就改 `Enroll/407_hw_config.h` 里 `HW_I2C1_SCL/SDA_PIN` 和 `HW_I2C2_SCL/SDA_PIN`，让软件映射**等于你的实物接线**。本工程固定 F407，`HW_I2C_COUNT` 保持 2。

### 4.2 供电与共地（机械/硬件红线）

- PCA9685 模块一般两个电源端子：**VCC（逻辑，接 F407 的 3.3V 或 5V，并共地）** 与 **V+（舵机电源，单独一路）**。
- **V+ 不要从 F407 的 5V 引脚取**。MG90S 单个堵转可到 ~0.6–0.8A，18 个峰值远超板载稳压能力；正常走动时每块板也建议给足 2A 以上。用 2S 锂电或独立 5V/6V UBEC，GND 与 F407 共地即可。
- **装机校准阶段可以先把 V+ 断开**：先用逻辑电 + I2C 扫出两块 0x40、把信号调通，再接 V+ 让舵机转中位、上摇臂。这样即使接错通道也不至于顶着电流乱转。

### 4.3 与现有 I2C 器件共存还是让位

现有 BMP280/QMC5883P 走 `API_I2C1`、OLED 走 `API_I2C2`。PCA9685(0x40) 与它们**地址不重叠，物理上可以共线**。两种策略任选：

- **保留策略（推荐先这样冒烟）**：其他器件照常初始化，PCA9685 只是多一个 0x40 从机。唯一代价是软件 I2C 带宽共享（见 §5.3 估算，完全够用）。
- **精简策略（最终六足形态）**：如果这些传感器/OLED 不再需要，可在 `main.c` 里注释对应 BSP Init，让总线更干净。**注意：注释 OLED 的 I2C 版本时，DisplayTask 里用到 OLED 的代码也要一起处理**，别只注释 Init 留下一堆调用报错。

> 不推荐把两块 PCA9685 硬挤到同一条总线然后把另一条空出来——软件 I2C 是逐位翻转 GPIO，串行执行，同挂一条总线不会并行变快，只会让单笔 16 通道刷新被拉长，还可能踩到 0x40 重地址。

---

## 5. PCA9685 寄存器协议与定标（核心参考，必须吃透）

### 5.1 关键寄存器

| 寄存器 | 地址 | 说明 |
|---|---|---|
| MODE1 | 0x00 | bit7=RESTART、bit5=AI(自增)、bit4=SLEEP、bit0=ALLCALL；复位值 0x21（AI+ALLCALL 开） |
| MODE2 | 0x01 | 复位值 0x04=OUTDRV（推挽输出）。**保持 OUTDRV，严禁置 INVRT(0x10)** |
| LED0_ON_L | 0x06 | 每通道 4 字节：`ON_L, ON_H, OFF_L, OFF_H`，LED0=0x06…LED15=0x5A |
| ALL_LED_ON_L | 0xFA | 全通道广播（本驱动不用） |
| PRE_SCALE | 0xFE | 分频 = round(25MHz /(4096×freq)) − 1 |
| TESTMODE | 0xFF | 别碰 |

### 5.2 频率与 tick 定标（⚠️ 与源工程有一个必须知道的差别）

内部振荡器 25MHz。目标 50Hz 舵机频率：

```
PRE_SCALE = round(25000000 / (4096 × 50)) − 1 = round(122.07) − 1 = 122 − 1 = 121
tick = (PRE_SCALE + 1) / 25MHz = 122 / 25e6 ≈ 4.88 µs
```

- 改 PRE_SCALE **必须先让芯片进 SLEEP**，写完再退出睡眠，最后写一次 RESTART 让振荡器重新跑（时序见 §7 `PCA9685_Init`，照 Adafruit 库的成熟流程）。
- **源工程用 `kTickUs = 5`（`servo.cpp` 注释自己也写了"~=5us"）做 `us/5` 换算**。这只是一种近似：1500µs 它发 300 tick，实际脉宽 300×4.88≈1465µs，比标称少约 35µs（≈3°，系统性的比例偏小 2.3%）。
- 我们**不复制这个误差**，用精确换算：`tick = round(µs × 4096 × 50 / 25e6) = round(µs × 0.2048)`。若你日后要"逐字节复刻源工程的脉宽"（一般没必要），才用 `tick = µs/5`。装机是**物理对中位**，2.3% 的量级在校准里被吸收，不影响。

常用换算表（tick≈4.88µs）：

| 脉宽 µs | 精确 tick | 源工程近似(÷5) |
|---|---|---|
| 500 | 102 | 100 |
| 1000 | 205 | 200 |
| **1500（中位）** | **307** | **300** |
| 2000 | 410 | 400 |
| 2500 | 512 | 500 |

**结论：文档里不要到处写死的 tick 常数（尤其别把 1500µs 记成 300 tick）。一律用 µs 表达目标，由 `PCA9685_UsToTicks()` 换算。**

### 5.3 单笔块写 vs 逐通道写（软件 I2C 带宽）

- 软件 I2C 逐位翻转 GPIO，每字节约 (8+1) 个位时隙，400kHz 档下理论 ~22.5µs/字节，加上函数调用开销实际更慢。
- 刷新一块板 16 通道 = 16×4=64 字节数据 + 地址 + 寄存器号 ≈ 1.5–4ms。**必须用一次事务的"块写"**（AI 自增，从 LED0_ON_L 连续写 64 字节），不要学 Adafruit 逐通道各开一笔 Start/Stop。
- 两板独立总线，刷新全部 18 舵机 ≈ 两笔块写 ≈ 3–8ms。源工程步态插值节拍 `movementInterval=20ms`，余量充足。

---

## 6. `SYSTEM/BusRate.h`：加集中配置

在文件里"总线选择"与"速率档位"两个区各加一段：

```c
/* --- PCA9685 舵机扩展板 ---
 * 板A(右三腿 leg0/1/2) 挂 API_I2C1；板B(左三腿 leg3/4/5) 挂 API_I2C2。
 * 两板 A0~A5 全接地，7-bit 地址均为 0x40（8-bit 写地址 0x80），
 * 因分挂不同总线互不冲突；同一条总线上不允许出现第二个 0x40。
 */
#define PCA9685_A_I2C_BUS    API_I2C1
#define PCA9685_B_I2C_BUS    API_I2C2
#define PCA9685_I2C_SPEED    API_I2C_SPEED_400K
```

> 若某条总线与现有设备冲突导致不稳定，可把速度降一档（`API_I2C_SPEED_200K`）——PCA9685 完全接受，代价是刷新略慢。

---

## 7. 新 BSP：`BSP/PCA9685/`（可直接粘贴）

风格对齐 BMP280/OLED：句柄携带"挂哪条总线 + 8-bit 写地址"；每次公开调用在锁内重新选总线；内部读写函数不重复加锁（锁由公开接口统一持有）。

### 7.1 `BSP/PCA9685/PCA9685.h`

```c
#ifndef __PCA9685_H
#define __PCA9685_H

#include <stdbool.h>
#include <stdint.h>

#include "API_I2C.h"
#include "BusRate.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== PCA9685 基础参数 =====
 * 两块板 A0~A5 全接地 → 7-bit 地址 0x40 → 8-bit 写地址 0x80。
 * 分挂 API_I2C1 / API_I2C2，互不冲突；同一条总线上绝不能出现两个 0x40。
 */
#define PCA9685_ADDR            (0x80U)
#define PCA9685_CH_MAX          (16U)

/* 寄存器地址 */
#define PCA9685_REG_MODE1       (0x00U)
#define PCA9685_REG_MODE2       (0x01U)
#define PCA9685_REG_LED0_ON_L   (0x06U)   /* 每通道 4 字节: ON_L, ON_H, OFF_L, OFF_H */
#define PCA9685_REG_PRE_SCALE   (0xFEU)

/* MODE1 位 */
#define PCA9685_MODE1_RESTART   (0x80U)
#define PCA9685_MODE1_AI        (0x20U)   /* 地址自增(默认开)，块写依赖它 */
#define PCA9685_MODE1_SLEEP     (0x10U)   /* 睡眠；改 PRE_SCALE 前必须先睡 */
#define PCA9685_MODE1_ALLCALL   (0x01U)

/* MODE2 位 */
#define PCA9685_MODE2_OUTDRV    (0x04U)   /* 推挽输出(默认)，接舵机保持此项 */
#define PCA9685_MODE2_INVRT     (0x10U)   /* 输出反相 —— 舵机严禁开启 */

/* 分频：25MHz 振荡器，50Hz → PRE_SCALE=121，tick≈4.88us */
#define PCA9685_PRESCALE_50HZ   (121U)

/* 舵机标称中位 */
#define PCA9685_US_CENTER       (1500U)

typedef struct
{
    API_I2C_BusId_t bus;   /* 该板挂哪一路: PCA9685_A_I2C_BUS / PCA9685_B_I2C_BUS */
    uint8_t writeAddr;     /* 8-bit 写地址，A0~A5 全接地 = 0x80 */
} PCA9685_Handle_t;

/* 把 us 换成 12-bit 计数值（tick≈4.88us @50Hz）：1500us→307，500us→102，2500us→512 */
uint16_t PCA9685_UsToTicks(uint16_t us);

/* 初始化单板到 50Hz 并确认在线（读到并写通寄存器），成功返回 true */
bool PCA9685_Init(PCA9685_Handle_t *h);
/* 只探测：该总线该地址上是否有器件应答 */
bool PCA9685_Probe(PCA9685_Handle_t *h);
/* 写单个通道：on/off 为 12-bit 计数值 */
bool PCA9685_SetPWM(PCA9685_Handle_t *h, uint8_t ch, uint16_t on, uint16_t off);
/* 便捷：单通道按 us 脉宽写（ON=0） */
bool PCA9685_SetServoUs(PCA9685_Handle_t *h, uint8_t ch, uint16_t us);
/* 块写 16 通道（推荐：一次 I2C 事务整板刷新，软件 I2C 才够快） */
bool PCA9685_SetAllUs(PCA9685_Handle_t *h, const uint16_t us[16]);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H */
```

### 7.2 `BSP/PCA9685/PCA9685.c`

```c
#include "PCA9685.h"
#include "Delay.h"

/* 选择本板挂的总线并设速（调用方须已持锁） */
static void PCA9685_SelectBus(PCA9685_Handle_t *h)
{
    API_I2C_SelectBus(h->bus);
    API_I2C_SetSpeed(PCA9685_I2C_SPEED);
}

/* 写 1 个寄存器（调用方须已持锁，单笔原子事务） */
static void pca9685WriteReg(PCA9685_Handle_t *h, uint8_t reg, uint8_t val)
{
    PCA9685_SelectBus(h);
    API_I2C_Start();
    API_I2C_SendByte(h->writeAddr);
    (void)API_I2C_Wait_Ack();
    API_I2C_SendByte(reg);
    (void)API_I2C_Wait_Ack();
    API_I2C_SendByte(val);
    (void)API_I2C_Wait_Ack();
    API_I2C_Stop();
}

/* 读 1 个寄存器（调用方须已持锁） */
static uint8_t pca9685ReadReg(PCA9685_Handle_t *h, uint8_t reg)
{
    uint8_t val = 0U;

    PCA9685_SelectBus(h);
    API_I2C_Start();
    API_I2C_SendByte(h->writeAddr);
    if (API_I2C_Wait_Ack() != 0U)
    {
        API_I2C_Stop();
        return val;
    }
    API_I2C_SendByte(reg);
    if (API_I2C_Wait_Ack() != 0U)
    {
        API_I2C_Stop();
        return val;
    }
    API_I2C_Start();                          /* 重复起始，切读方向 */
    API_I2C_SendByte((uint8_t)(h->writeAddr | 0x01U));
    if (API_I2C_Wait_Ack() != 0U)
    {
        API_I2C_Stop();
        return val;
    }
    val = API_I2C_ReceiveByte(0U);            /* 最后 1 字节回 NACK */
    API_I2C_Stop();
    return val;
}

/* us → 12-bit 计数值：round(us * 0.2048)，见文档 §5.2 */
uint16_t PCA9685_UsToTicks(uint16_t us)
{
    uint32_t t = ((uint32_t)us * 2048UL + 5000UL) / 10000UL;

    if (t > 4095UL)
    {
        t = 4095UL;
    }
    return (uint16_t)t;
}

bool PCA9685_Probe(PCA9685_Handle_t *h)
{
    uint8_t ack;

    if (h == NULL)
    {
        return false;
    }

    API_I2C_Lock();
    PCA9685_SelectBus(h);
    API_I2C_Start();
    API_I2C_SendByte(h->writeAddr);
    ack = API_I2C_Wait_Ack();
    API_I2C_Stop();
    API_I2C_Unlock();

    return (ack == 0U);
}

bool PCA9685_Init(PCA9685_Handle_t *h)
{
    uint8_t old;

    if (h == NULL)
    {
        return false;
    }

    API_I2C_Lock();
    PCA9685_SelectBus(h);

    /* 探测：地址不应答直接失败 */
    API_I2C_Start();
    API_I2C_SendByte(h->writeAddr);
    if (API_I2C_Wait_Ack() != 0U)
    {
        API_I2C_Stop();
        API_I2C_Unlock();
        return false;
    }
    API_I2C_Stop();

    /* 标准 50Hz 流程：读 MODE1 → 睡眠 → 写分频 → 退睡(确保 AI) → 写 MODE2 → RESTART */
    old = pca9685ReadReg(h, PCA9685_REG_MODE1);
    pca9685WriteReg(h, PCA9685_REG_MODE1, (uint8_t)(old | PCA9685_MODE1_SLEEP));
    pca9685WriteReg(h, PCA9685_REG_PRE_SCALE, PCA9685_PRESCALE_50HZ);
    pca9685WriteReg(h, PCA9685_REG_MODE1, (uint8_t)((old & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_AI));
    pca9685WriteReg(h, PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
    Delay_us(500U);
    pca9685WriteReg(h, PCA9685_REG_MODE1,
                    (uint8_t)((old & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_AI | PCA9685_MODE1_RESTART));
    Delay_us(500U);

    API_I2C_Unlock();
    return true;
}

bool PCA9685_SetPWM(PCA9685_Handle_t *h, uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t reg;
    uint8_t ok = 0U;

    if (h == NULL || ch >= PCA9685_CH_MAX)
    {
        return false;
    }
    reg = (uint8_t)(PCA9685_REG_LED0_ON_L + (uint8_t)(ch << 2U));

    API_I2C_Lock();
    PCA9685_SelectBus(h);
    API_I2C_Start();
    API_I2C_SendByte(h->writeAddr);
    ok |= API_I2C_Wait_Ack();
    API_I2C_SendByte(reg);
    ok |= API_I2C_Wait_Ack();
    API_I2C_SendByte((uint8_t)(on & 0xFFU));
    ok |= API_I2C_Wait_Ack();
    API_I2C_SendByte((uint8_t)(on >> 8U));
    ok |= API_I2C_Wait_Ack();
    API_I2C_SendByte((uint8_t)(off & 0xFFU));
    ok |= API_I2C_Wait_Ack();
    API_I2C_SendByte((uint8_t)(off >> 8U));
    ok |= API_I2C_Wait_Ack();
    API_I2C_Stop();
    API_I2C_Unlock();

    return (ok == 0U);
}

bool PCA9685_SetServoUs(PCA9685_Handle_t *h, uint8_t ch, uint16_t us)
{
    return PCA9685_SetPWM(h, ch, 0U, PCA9685_UsToTicks(us));
}

bool PCA9685_SetAllUs(PCA9685_Handle_t *h, const uint16_t us[16])
{
    uint8_t i;

    if (h == NULL || us == NULL)
    {
        return false;
    }

    API_I2C_Lock();
    PCA9685_SelectBus(h);
    API_I2C_Start();
    API_I2C_SendByte(h->writeAddr);
    if (API_I2C_Wait_Ack() != 0U)
    {
        API_I2C_Stop();
        API_I2C_Unlock();
        return false;
    }
    API_I2C_SendByte(PCA9685_REG_LED0_ON_L);
    if (API_I2C_Wait_Ack() != 0U)
    {
        API_I2C_Stop();
        API_I2C_Unlock();
        return false;
    }

    /* AI 自增：从 LED0_ON_L 起连续写 16 通道 × 4 字节，一笔事务刷新整板 */
    for (i = 0U; i < PCA9685_CH_MAX; i++)
    {
        uint16_t off = PCA9685_UsToTicks(us[i]);

        API_I2C_SendByte(0x00U);                /* ON_L  = 0 */
        (void)API_I2C_Wait_Ack();
        API_I2C_SendByte(0x00U);                /* ON_H  = 0 */
        (void)API_I2C_Wait_Ack();
        API_I2C_SendByte((uint8_t)(off & 0xFFU));  /* OFF_L */
        (void)API_I2C_Wait_Ack();
        API_I2C_SendByte((uint8_t)(off >> 8U));    /* OFF_H */
        (void)API_I2C_Wait_Ack();
    }
    API_I2C_Stop();
    API_I2C_Unlock();

    return true;
}
```

> 设计说明：公开接口 `Init/Probe/SetPWM/SetServoUs/SetAllUs` **自己持锁**，所以后续多个任务（控制任务刷腿、显示任务等）随便调用都安全；内部 `pca9685WriteReg/ReadReg` 只在 `Init` 的锁内被调用，故不再加锁。**不要在别处先 Lock 再调用这些公开接口**（会重复持锁）。

---

## 8. 腿 / 板 / 通道映射与"角度→脉宽"公式（算法迁移要用的底料，先存好）

这些来自源工程 `servo.cpp` + `leg.cpp`，是**给后续运动控制文档留的对照表**，也是装机冒烟时判断"哪条腿在动"的依据。

### 8.1 腿编号（源工程约定，配图见 `NodeHexa/firmware/src/main.cpp` 顶部注释）

| leg | 位置 | 安装角 |
|---|---|---|
| 0 | 右前 | 45° |
| 1 | 右中 | 0° |
| 2 | 右后 | −45°(315°) |
| 3 | 左后 | −135°(225°) |
| 4 | 左中 | 180° |
| 5 | 左前 | 135° |

### 8.2 板与通道（`hexapod2pwm()` 还原成两板的物理通道）

每块板每组 3 通道 = 1 条腿的 3 个关节；**块内通道规律：中腿用 ch2 组、前腿用 ch5 组、后腿用 ch8 组**（源工程 `case 1: return 2+part` / `case 0: return 5+part` / `case 2: return 8+part`）。

**PCA9685-A（API_I2C1，右三腿）**：

| leg | 关节 0(hip yaw) | 关节 1(thigh 大腿) | 关节 2(ankle 小腿) |
|---|---|---|---|
| 0 右前 | ch5 | ch6 | ch7 |
| 1 右中 | ch2 | ch3 | ch4 |
| 2 右后 | ch8 | ch9 | ch10 |

**PCA9685-B（API_I2C2，左三腿）**：

| leg | 关节 0 | 关节 1 | 关节 2 |
|---|---|---|---|
| 5 左前 | ch5 | ch6 | ch7 |
| 4 左中 | ch2 | ch3 | ch4 |
| 3 左后 | ch8 | ch9 | ch10 |

> 记法：每块板上 ch2=中腿、ch5=前腿、ch8=后腿；右板插右侧腿，左板插左侧腿。通道 0/1/11~15 空着。

### 8.3 关节 0/1/2 ↔ 物理意义与角度→脉宽公式（源工程 `Servo::setAngle` 原样翻译）

源工程映射（`servo.cpp` 103 行 `us = 1500 + (angle+offset)*(1000/90)`，`11.11µs/°`；offset 默认 0，可逐舵机校准）：

| part | 物理关节 | range_ | inverse_ | adjust_ | 软件角度 θ 对应 µs | 说明 |
|---|---|---|---|---|---|---|
| 0 | hip yaw（水平旋转，±45°） | 45 | 否 | 0 | `1500 + θ×11.11` | θ=0 ↔ 1500µs |
| 1 | thigh 大腿（±60°，**装反 + 内偏 15°**） | 60 | **是** | **15** | `1500 + (15 − θ)×11.11` | **θ=+15 ↔ 1500µs**；θ=0 ↔ 1667µs |
| 2 | ankle 小腿（±60°） | 60 | 否 | 0 | `1500 + θ×11.11` | θ=0 ↔ 1500µs |

- 软件角 θ 先被夹到 `[-range+adjust, range+adjust]`：hip=[−45,45]、thigh=[−45,75]、ankle=[−60,60]。
- 最终 µs 再夹到 [500, 2500]。

### 8.4 装机前校准到底各关节给多少 µs（你之前一直问的重点）

- **hip(0) 与 ankle(2)：物理中位就是软件 0°，1500µs。**
- **thigh(1)：软件 +15° 才落在 1500µs**（因为代码里大腿装反并内偏了 15°，这个 15° 由代码补偿）。也就是说：
  - 如果你想在装机时把"每条腿能活动的中间位置"对准打印件的设计姿态，**最稳的做法是驱动全部 18 个舵机到 1500µs，再按源工程同款 3D 打印件的装配说明，让大腿的摇臂在其物理中位（摇臂与舵机壳体成 90°）上紧**——软件里 thigh 的 +15°/反向是**在代码里处理的**，不要试图在装机时给 thigh 单独喂 1667µs 去"对准软件 0"。
  - 校准的完整口诀：**上电前全部舵机不接 V+ → 先 I2C 扫到两块板 → 上 V+ 前先跑一遍"全部 1500µs"让每颗舵机转物理中位 → 此时把摇臂以 90° 垂直位锁紧 → 完成，之后所有动作由代码按 §8.3 公式驱动，不要再手动调。**

---

## 9. `main.c` 集成 + 装机校准 + 冒烟测试步骤

### 9.1 在 `main.c` 里加句柄与初始化

全局（文件顶部附近，Task 之外）加两板句柄，供后续任务使用：

```c
/* PCA9685 舵机板句柄：板A=右三腿(leg0/1/2)，板B=左三腿(leg3/4/5)；两板都 0x40 分挂两总线 */
static PCA9685_Handle_t s_pca9685A = { PCA9685_A_I2C_BUS, PCA9685_ADDR };
static PCA9685_Handle_t s_pca9685B = { PCA9685_B_I2C_BUS, PCA9685_ADDR };
```

在 `main()` 的 BSP 初始化区（`OLED_Init` 附近，`API_I2C_Init()` 之后）加：

```c
    /* PCA9685 舵机扩展板：两块都初始化到 50Hz */
    if (PCA9685_Init(&s_pca9685A))
    {
        usart_printf(PRINTF_USART, "PCA9685-A(右腿) OK\r\n");
    }
    else
    {
        usart_printf(PRINTF_USART, "PCA9685-A(右腿) FAIL!\r\n");
    }
    if (PCA9685_Init(&s_pca9685B))
    {
        usart_printf(PRINTF_USART, "PCA9685-B(左腿) OK\r\n");
    }
    else
    {
        usart_printf(PRINTF_USART, "PCA9685-B(左腿) FAIL!\r\n");
    }
```

> 若工程里 `PRINTF_USART`/`usart_printf` 名字不同，照着附近现有打印语句抄。文件需 `#include "PCA9685.h"` 和 BusRate 已由它带入。

### 9.2 装机校准助手（临时函数，接到串口命令或按钮触发）

```c
/* 装机校准：把一块板 16 路全部打到 1500us 物理中位 */
static void PCA9685_CalCenterAll(void)
{
    uint16_t center[PCA9685_CH_MAX];
    uint8_t i;

    for (i = 0U; i < PCA9685_CH_MAX; i++)
    {
        center[i] = PCA9685_US_CENTER;
    }
    (void)PCA9685_SetAllUs(&s_pca9685A, center);
    (void)PCA9685_SetAllUs(&s_pca9685B, center);
    usart_printf(PRINTF_USART, "all servos -> 1500us center\r\n");
}
```

### 9.3 逐通道冒烟（找舵机、确认接线表 §8.2）

在 ControlTask 或临时循环里：先全部 1500µs，然后**一次只动一路**：把 ch 从 1500µs 摆到 1400µs（−10°，hip 约 −9°）停 1s 再回中位；人眼确认是"预期的那条腿的那个关节"在动。按表逐格打勾：

| 通道 | 预期动作 |
|---|---|
| A: ch5 / ch6 / ch7 | 右前腿 髋 / 大腿 / 小腿 |
| A: ch2 / ch3 / ch4 | 右中腿 髋 / 大腿 / 小腿 |
| A: ch8 / ch9 / ch10 | 右后腿 髋 / 大腿 / 小腿 |
| B: ch5 / ch6 / ch7 | 左前腿 髋 / 大腿 / 小腿 |
| B: ch2 / ch3 / ch4 | 左中腿 髋 / 大腿 / 小腿 |
| B: ch8 / ch9 / ch10 | 左后腿 髋 / 大腿 / 小腿 |

> 若发现"动了但方向反了 / 编号对不上"，**不要翻舵机线、不要改 §8.2 表**——源工程整套算法就按这张表写死。正确做法：把实际接成"ch2=中、ch5=前、ch8=后"的三腿对应到左/右板，让 leg 编号与实物一致；个别关节方向不对时，在后续算法移植的 Servo 结构里给该关节加 offset（源工程 `Servo::setParameter(offset)` 就是干这个的），而不是动线。

### 9.4 上电顺序建议（一次做对，少烧东西）

1. F407 上电（PCA9685 V+ 断开）。
2. 看串口：两板 Init OK / FAIL；I2C 扫描每总线一个 0x40。
3. 接 V+。跑 `PCA9685_CalCenterAll()`，观察 18 颗舵机都到中位、无抖动无异常发热。
4. 锁摇臂（90° 垂直），完成装机校准。
5. 逐通道冒烟打勾。

---

## 10. CMake 登记与坑清单

### 10.1 CMakeLists.txt（必改，否则链接报 undefined）

源文件列表 `BSP_LAYER_SOURCES`（约 127 行）末尾加：

```cmake
    ${CMAKE_SOURCE_DIR}/BSP/PCA9685/PCA9685.c
```

头文件目录 `MCU_INCLUDE_DIRS`（约 141 行）里加：

```cmake
    ${CMAKE_SOURCE_DIR}/BSP/PCA9685
```

### 10.2 坑清单（按踩坑概率排序）

1. **忘登记 CMake** → 链接 `undefined reference to PCA9685_Init`。先加文件再编译。
2. **同一条总线出现两个 0x40** → 两边都在抢应答，Init/Probe 时好时坏。用扫描命令确认"每总线恰好 1 个 0x40"。
3. **V+ 没接/接错共地** → PCA9685 逻辑正常（能被扫到）但舵机不动/乱抖。确认 V+ 电源与 F407 GND 共地。
4. **把 1500µs 死记成 300 tick** → 实际 307。一律走 `PCA9685_UsToTicks()`。
5. **改 PRE_SCALE 没进 SLEEP** → 分频写不进去，频率还是默认。照 §7 `Init` 流程。
6. **软件 I2C 全局状态被别的任务抢占** → 每笔访问必须在 `API_I2C_Lock()` 内且**锁内重新 SelectBus**（板 A/B 在两条总线上，最容易踩）。
7. **逐通道写导致刷新慢** → 高频刷新请用 `PCA9685_SetAllUs()` 块写；控制任务不要每 20ms 循环里逐通道 SetPWM 18 次。
8. **别用源工程 pathTool 直接重新生成动作表**：`workspace/pathTool/src/config.py` 的连杆/安装参数与 `firmware/include/config.h` **不一致**（29.87/22.41/55.41… vs 34.7/25/58…）。迁移运动控制前需先把两者对齐，否则生成的步态脚位是错的（属于下一份文档的警告，先记住）。
9. **买错舵机**：必须是 **MG90S 180° 位置舵机（模拟/数字都行）**，不是"360° 连续旋转"变体——连续旋转舵机给 1500µs 是停转而不是中位，整套系统直接废掉。
10. **C/C++ 边界**：本 BSP 是纯 C，已用 `extern "C"` 保护。日后算法迁移从 C++（NodeHexa）到 C，涉及 `.cpp` 结构体/类改写成 C 结构体 + 函数，另见后续文档，别在纯 C 文件里写 `new`/类。

---

## 11. 给接手 AI 的执行顺序（最短路径）

1. 读 §3 列的 5 个目标工程文件 + §5 寄存器协议。
2. 确认用户实物接线：两板各挂哪套引脚（默认 PB8/PB9 与 PA7/PA5，否则改 `407_hw_config.h`），两板 A0~A5 是否全接地。
3. 新建 `BSP/PCA9685/PCA9685.h/.c`（§7 全文粘贴），改 `BusRate.h`（§6）与 `CMakeLists.txt`（§10.1）。
4. `main.c` 加句柄 + Init（§9.1）。编译（F7）。串口应打印两板 OK，I2C 扫描每总线一个 0x40。
5. 加临时校准/冒烟代码（§9.2/9.3），接 V+ 后执行装机校准 + 逐通道打勾。
6. 冒烟全过后，把 §8 的映射表与 §8.3 公式写进本工程的 `app/Hexa/` 或留档，作为下一步运动控制迁移的输入。

## 12. 完成信号（对照 §1）

两板 Init OK、每总线恰好一个 0x40、全部舵机可中位校准、18 路逐通道冒烟与 §8.2 表全部吻合、工程 F7 编译零警告（除既有历史警告）。

---

*本文档是 NodeHexa → QuadArachnid 迁移的第 1 部分（PCA9685 驱动）。第 2 部分：FK/IK、Movement 步态机、PathTool 动作表生成链路迁移，另文交接。*
