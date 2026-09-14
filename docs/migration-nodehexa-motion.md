# NodeHexa → QuadArachnid：运动控制迁移交接文档

> 源工程：`D:\Desktop\NodeHexa`（ESP32 + C++，六足 hexapod）
> 目标工程：`C:\QuadArachnid`（STM32F407 + C + FreeRTOS，OmniLayer 分层框架）
> 本文档日期：2026-09-08
> 交接范围：**运动控制算法层（FK/IK + 步态引擎 + 动作表 + 指令解析 + 校准持久化）**
> 前置条件：PCA9685 驱动（`BSP/PCA9685/`）已完成，舵机装机校准已完成
> 接手对象：AI（独立执行）；人负责编译烧录和物理测试

> ⚠️ 本文档是迁移的第 2 部分。第 1 部分（PCA9685 驱动）见 `docs/migration-nodehexa-pca9685.md`。

---

## 1. 任务一句话

在 `QuadArachnid` 的 `app/Hexa/` 目录下，用纯 C 实现一套完整的六足运动控制库，然后在 `A_Entry/main.c` 的 FreeRTOS `ControlTask` 中集成 20ms 步态节拍，使机器人能响应串口/蓝牙/NRF/语音指令执行 14 种动作模式。

## 2. 源工程关键文件（只读参考）

| 源文件 | 核心内容 | 迁移目标 |
|---|---|---|
| `firmware/src/base.h` | Point3D、Locations、运算符重载 | `app/Hexa/base.h`（改为 C 结构体+函数） |
| `firmware/include/config.h` | 连杆长度、安装位置、时序参数 | `app/Hexa/config.h` |
| `firmware/src/leg.cpp/.h` | FK/IK、坐标旋转、moveTip | `app/Hexa/leg.c/.h` |
| `firmware/src/servo.cpp/.h` | 角度→脉宽、关节参数、offset | `app/Hexa/servo.c/.h` |
| `firmware/src/movement.cpp/.h` | MovementMode 枚举、MovementTable、Movement 插值状态机 | `app/Hexa/movement.c/.h` |
| `firmware/src/movement_profile.cpp/.h` | MovementMetrics 步态参数 | `app/Hexa/movement_profile.c/.h` |
| `firmware/src/hexapod.cpp/.h` | Hexapod 整机、processMovement、校准 | `app/Hexa/hexapod.c/.h` |
| `firmware/src/movements.cpp` | kStandby、P1X~P6Z 宏定义、include 生成的表 | 宏定义搬到 `app/Hexa/movement_table.h` 头部 |
| `firmware/src/generated/movement_table.h` | 13 种动作的足端轨迹表 | 用 pathTool 重新生成到 `app/Hexa/movement_table.h` |
| `workspace/pathTool/src/config.py` | pathTool 用的连杆参数（⚠️ 与 config.h 不一致） | 先对齐再生成 |
| `firmware/src/main.cpp` L1324-1346 | 字符串→枚举映射表 | `app/Hexa/command.c/.h` |

## 3. 目标工程约定（写代码前必读）

- 所有 `app/Hexa/` 下的文件是**纯 C**（不是 C++）。
- 头文件用 `#ifndef __HEXA_XXX_H` / `#define` 保护。
- 每个 .c 的第一行 `#include "xxx.h"`。
- 引用 BSP 用 `#include "PCA9685.h"` + `#include "BusRate.h"`。
- 浮点数学用 `#include <math.h>`（不是 `<cmath>`）。
- 延时用 `#include "Delay.h"` 的 `Delay_ms()`。
- CMake 登记：源文件加 `BSP_LAYER_SOURCES`（或者新建 `APP_LAYER_SOURCES` 变量），头文件目录加 `MCU_INCLUDE_DIRS`。
- 两块 PCA9685 的句柄已在 `main.c` 定义（`s_pca9685A` / `s_pca9685B`），`app/Hexa/servo.c` 通过 `extern` 引用它们。

## 4. 执行顺序（共 11 步）

### 4.1 创建目录

```
mkdir C:\QuadArachnid\app\Hexa
```

### 4.2 第 1 步：`app/Hexa/config.h`（连杆参数，纯常量）

直接从 `NodeHexa/firmware/include/config.h` 翻译，去掉 C++ 的 `namespace`，改为 `#define` 宏：

```c
#ifndef __HEXA_CONFIG_H
#define __HEXA_CONFIG_H

/* 连杆长度，单位 mm（与 firmware/include/config.h 保持一致） */
#define HEXA_LEG_ROOT_TO_J1      (19.4f)
#define HEXA_LEG_J1_TO_J2        (32.0f)
#define HEXA_LEG_J2_TO_J3        (43.8f)
#define HEXA_LEG_J3_TO_TIP       (90.05f)

/* 安装位置 */
#define HEXA_MOUNT_LR_X          (34.7f)
#define HEXA_MOUNT_OTHER_X       (25.0f)
#define HEXA_MOUNT_OTHER_Y       (58.0f)

/* 时序，单位 ms */
#define HEXA_MOVEMENT_INTERVAL   (20)
#define HEXA_MOVEMENT_SWITCH_DUR (150)

/* 速度 */
#define HEXA_DEFAULT_SPEED       (0.5f)
#define HEXA_MIN_SPEED           (0.25f)
#define HEXA_MAX_SPEED           (1.0f)

#endif
```

### 4.3 第 2 步：`app/Hexa/base.h`（基础类型）

把 C++ 的 `Point3D` 类和 `Locations` 类改为 C 结构体 + 函数：

```c
#ifndef __HEXA_BASE_H
#define __HEXA_BASE_H

typedef struct {
    float x, y, z;
} Point3D;

/* 6 足端坐标 = 1 帧 */
typedef struct {
    Point3D leg[6];
} Locations;

/* 向量运算（替代 C++ operator 重载） */
static inline Point3D p3d_sub(Point3D a, Point3D b) {
    Point3D r = { a.x - b.x, a.y - b.y, a.z - b.z };
    return r;
}
static inline Point3D p3d_mul(Point3D a, float b) {
    Point3D r = { a.x * b, a.y * b, a.z * b };
    return r;
}
static inline void p3d_addEq(Point3D *a, Point3D b) {
    a->x += b.x; a->y += b.y; a->z += b.z;
}
static inline int p3d_eq(Point3D a, Point3D b) {
    return (a.x == b.x && a.y == b.y && a.z == b.z);
}

/* Locations 运算 */
static inline Locations loc_sub(Locations a, Locations b) {
    Locations r;
    for (int i = 0; i < 6; i++) r.leg[i] = p3d_sub(a.leg[i], b.leg[i]);
    return r;
}
static inline Locations loc_mul(Locations a, float b) {
    Locations r;
    for (int i = 0; i < 6; i++) r.leg[i] = p3d_mul(a.leg[i], b);
    return r;
}
static inline void loc_addEq(Locations *a, Locations b) {
    for (int i = 0; i < 6; i++) p3d_addEq(&a->leg[i], b.leg[i]);
}

#endif
```

### 4.4 第 3 步：`app/Hexa/servo.h/.c`（舵机角度→脉宽）

**servo.h：**

```c
#ifndef __HEXA_SERVO_H
#define __HEXA_SERVO_H

#include <stdint.h>
#include <stdbool.h>

/* 关节参数：每个 joint 的 range/inverse/adjust 不同 */
typedef struct {
    uint8_t pcaCh;       /* PCA9685 通道号（0~15） */
    uint8_t board;       /* 0=板A(I2C1), 1=板B(I2C2) */
    float   range;       /* 角度范围：hip=45, thigh/ankle=60 */
    bool    inverse;     /* 是否反向：thigh=true */
    float   adjust;      /* 角度内偏：thigh=15°, 其他=0 */
    int     offset;      /* 校准偏移量 */
    float   angle;       /* 当前软件角度 */
} ServoJoint;

/* 初始化一个关节的参数（legIndex 0~5, partIndex 0~2） */
void ServoJoint_Init(ServoJoint *sj, int legIndex, int partIndex);

/* 设置软件角度 θ（°），自动转为 µs 并写入 PCA9685 */
void ServoJoint_SetAngle(ServoJoint *sj, float angle);

/* 获取当前角度 */
float ServoJoint_GetAngle(const ServoJoint *sj);

/* 设置校准偏移（update 是否立即生效） */
void ServoJoint_SetOffset(ServoJoint *sj, int offset, bool update);

/* 获取校准偏移 */
int ServoJoint_GetOffset(const ServoJoint *sj);

#endif
```

**servo.c 核心逻辑（照搬 NodeHexa `Servo::setAngle`）：**

```
角度 θ 先夹到 [-range+adjust, range+adjust]
θ -= adjust
if (inverse) θ = -θ
µs = 1500 + (θ + offset) * (1000.0f / 90.0f)
µs 夹到 [500, 2500]
tick = PCA9685_UsToTicks(µs)
PCA9685_SetPWM(handle, ch, 0, tick)
```

**通道映射表（照搬 NodeHexa `hexapod2pwm()`）：**

| leg | part 0 (hip) | part 1 (thigh) | part 2 (ankle) | 板 |
|-----|-------------|----------------|----------------|-----|
| 0 右前 | ch5 | ch6 | ch7 | A |
| 1 右中 | ch2 | ch3 | ch4 | A |
| 2 右后 | ch8 | ch9 | ch10 | A |
| 3 左后 | ch8 | ch9 | ch10 | B |
| 4 左中 | ch2 | ch3 | ch4 | B |
| 5 左前 | ch5 | ch6 | ch7 | B |

**关节参数表：**

| part | range | inverse | adjust | µs 公式 |
|------|-------|---------|--------|---------|
| 0 (hip) | 45 | false | 0 | 1500 + θ×11.11 |
| 1 (thigh) | 60 | **true** | **15** | 1500 + (15−θ)×11.11 |
| 2 (ankle) | 60 | false | 0 | 1500 + θ×11.11 |

### 4.5 第 4 步：`app/Hexa/leg.h/.c`（单腿 FK/IK）

**leg.h：**

```c
#ifndef __HEXA_LEG_H
#define __HEXA_LEG_H

#include "base.h"
#include "servo.h"

typedef struct {
    int         index;        /* 腿编号 0~5 */
    Point3D     mountPos;     /* 安装位置 */
    ServoJoint  joints[3];    /* 3 个关节 */
    Point3D     tipPos;       /* 当前足端世界坐标 */
    Point3D     tipPosLocal;  /* 当前足端局部坐标 */
    void (*localConv)(Point3D src, Point3D *dst);  /* 世界→局部 旋转函数 */
    void (*worldConv)(Point3D src, Point3D *dst);  /* 局部→世界 旋转函数 */
} HexaLeg;

/* 初始化一条腿（legIndex 0~5） */
void HexaLeg_Init(HexaLeg *leg, int legIndex);

/* 正运动学：3 关节角度 → 足端局部坐标 */
void HexaLeg_FK(float angles[3], Point3D *out);

/* 逆运动学：足端局部坐标 → 3 关节角度 */
void HexaLeg_IK(const Point3D *to, float angles[3]);

/* 世界坐标 → 局部坐标 */
void HexaLeg_WorldToLocal(const HexaLeg *leg, Point3D world, Point3D *local);

/* 局部坐标 → 世界坐标 */
void HexaLeg_LocalToWorld(const HexaLeg *leg, Point3D local, Point3D *world);

/* 移动足端到世界坐标目标 */
void HexaLeg_MoveTip(HexaLeg *leg, const Point3D *to);

#endif
```

**leg.c 核心逻辑：**

1. `HexaLeg_Init(leg, index)`：根据 legIndex 设置 mountPos 和旋转函数（6 种：rotate0/45/135/180/225/315），创建 3 个 ServoJoint。
2. `HexaLeg_IK(to, angles)`：照搬 `leg.cpp::_inverseKinematics()`
   - `angles[0] = atan2(y, x) * 180/M_PI`
   - `x = sqrt(x²+y²) − kLegJoint1ToJoint2`, `y = to.z`
   - 余弦定律：`lr² = x²+y²`, `ratio1 = (lr² + J2J3² − J3Tip²) / (2×J2J3×lr)`
   - `a1 = acos(clamp(ratio1, -1, 1))`, `a2 = acos(clamp(ratio2, -1, 1))`
   - `angles[1] = (atan2(y,x) + a1) × 180/M_PI`
   - `angles[2] = 90 − (a1 + a2) × 180/M_PI`
3. `HexaLeg_MoveTip(leg, to)`：World→Local → IK → 遍历 3 个 joint 调 `ServoJoint_SetAngle`。

**旋转函数表（legIndex → 安装角）：**

| leg | 安装角 | localConv | worldConv |
|-----|--------|-----------|-----------|
| 0 右前 | 45° | rotate315 | rotate45 |
| 1 右中 | 0° | rotate0 | rotate0 |
| 2 右后 | −45° | rotate45 | rotate315 |
| 3 左后 | −135° | rotate135 | rotate225 |
| 4 左中 | 180° | rotate180 | rotate180 |
| 5 左前 | 135° | rotate225 | rotate135 |

### 4.6 第 5 步：对齐 pathTool 配置 + 重新生成动作表

**⚠️ 必须先做这件事，否则生成的动作表用的是错误的连杆参数！**

当前不一致：
- `config.py`：kLegMountLeftRightX=29.87, kLegMountOtherX=22.41, kLegMountOtherY=55.41, kLegRootToJoint1=20.75, kLegJoint1ToJoint2=28, kLegJoint2ToJoint3=42.6, kLegJoint3ToTip=89.07
- `config.h`：kLegMountLeftRightX=34.7, kLegMountOtherX=25, kLegMountOtherY=58, kLegRootToJoint1=19.4, kLegJoint1ToJoint2=32, kLegJoint2ToJoint3=43.8, kLegJoint3ToTip=90.05

**以 `firmware/include/config.h` 为准**，把 `config.py` 的值改成：

```python
kLegMountLeftRightX = 34.7
kLegMountOtherX = 25.0
kLegMountOtherY = 58.0
kLegRootToJoint1 = 19.4
kLegJoint1ToJoint2 = 32.0
kLegJoint2ToJoint3 = 43.8
kLegJoint3ToTip = 90.05
```

然后运行：
```bash
cd D:\Desktop\NodeHexa\workspace\pathTool\src
python path_tool_cli.py --robot hexapod
```

生成的 `firmware/src/generated/movement_table.h` 复制到 `C:\QuadArachnid\app\Hexa\movement_table.h`。

### 4.7 第 6 步：`app/Hexa/movement.h/.c`（步态引擎）

**movement.h：**

```c
#ifndef __HEXA_MOVEMENT_H
#define __HEXA_MOVEMENT_H

#include "base.h"

typedef enum {
    MOVEMENT_STANDBY = 0,
    MOVEMENT_FORWARD,
    MOVEMENT_FORWARDFAST,
    MOVEMENT_BACKWARD,
    MOVEMENT_TURNLEFT,
    MOVEMENT_TURNRIGHT,
    MOVEMENT_SHIFTLEFT,
    MOVEMENT_SHIFTRIGHT,
    MOVEMENT_CLIMB,
    MOVEMENT_ROTATEX,
    MOVEMENT_ROTATEY,
    MOVEMENT_ROTATEZ,
    MOVEMENT_TWIST,
    MOVEMENT_BEATSWAY,
    MOVEMENT_TOTAL
} MovementMode;

typedef struct {
    const Locations *table;   /* 帧数组 */
    int   length;             /* 帧数 */
    int   stepDuration;       /* 每步时长 ms */
    const int *entries;       /* 起始帧索引数组 */
    int   entriesCount;       /* entries 长度 */
} MovementTable;

typedef struct {
    MovementMode mode;
    Locations    position;    /* 当前插值位置 */
    int          index;       /* 目标帧索引 */
    int          transiting;  /* 是否正在切换到新模式 */
    int          remainTime;  /* 当前帧剩余时间 ms */
    float        speed;       /* 速度倍率 0.25~1.0 */
} Movement;

/* 获取某个模式的 MovementTable */
const MovementTable *Movement_GetTable(MovementMode mode);

/* 初始化 Movement 状态机 */
void Movement_Init(Movement *mv, MovementMode mode);

/* 设置新模式 */
void Movement_SetMode(Movement *mv, MovementMode newMode);

/* 瞬切到新模式（不插值） */
void Movement_SnapToMode(Movement *mv, MovementMode newMode);

/* 推进 elapsed 毫秒，返回当前插值后的足端坐标 */
const Locations *Movement_Next(Movement *mv, int elapsed);

/* 速度控制 */
void Movement_SetSpeed(Movement *mv, float speed);
float Movement_GetSpeed(const Movement *mv);

#endif
```

**movement.c 核心逻辑（照搬 `movement.cpp`）：**

1. `kTable[MOVEMENT_TOTAL]`：按枚举顺序排列 14 个 `MovementTable` 指针（standbyTable → beatswayTable）。
2. `Movement_Next(mv, elapsed)`：
   ```
   table = kTable[mode]
   actualStep = table.stepDuration / speed
   if (remainTime <= 0) { index = (index+1) % length; remainTime = actualStep; }
   if (elapsed >= remainTime) elapsed = remainTime
   ratio = elapsed / remainTime
   position += (table[index] - position) * ratio
   remainTime -= elapsed
   return &position
   ```

### 4.8 第 7 步：`app/Hexa/movement_profile.h/.c`

```c
#ifndef __HEXA_MOVEMENT_PROFILE_H
#define __HEXA_MOVEMENT_PROFILE_H

#include "movement.h"

typedef struct {
    float distancePerCycle;  /* 每周期位移（米） */
    float rotationPerCycle;  /* 每周期旋转（度） */
    float cycleCount;        /* 周期数 */
} MovementMetrics;

const MovementMetrics *MovementProfile_Get(MovementMode mode);

#endif
```

数据直接抄 `movement_profile.cpp` 的 `kMetrics[]` 数组。

### 4.9 第 8 步：`app/Hexa/movement_table.h`（动作表 + P1X~P6Z 宏）

从 pathTool 生成后，需要在文件头部加上 P1X~P6Z 宏（这些宏原来在 `movements.cpp` 里定义，C 版本需要一起放进表文件）：

```c
/* 放在 movement_table.h 最前面 */
#ifndef HEXA_PX_MACROS
#define HEXA_PX_MACROS

#include "config.h"
#include <math.h>

#define SIN30   0.5f
#define COS30   0.866f
#define SIN45   0.7071f
#define COS45   0.7071f
#define SIN15   0.2588f
#define COS15   0.9659f

#define STANDBY_Z   (HEXA_LEG_J3_TO_TIP*COS15 - HEXA_LEG_J2_TO_J3*SIN30)
#define LEFTRIGHT_X (HEXA_MOUNT_LR_X + HEXA_LEG_ROOT_TO_J1 + HEXA_LEG_J1_TO_J2 + (HEXA_LEG_J2_TO_J3*COS30) + HEXA_LEG_J3_TO_TIP*SIN15)
#define OTHER_X     (HEXA_MOUNT_OTHER_X + (HEXA_LEG_ROOT_TO_J1 + HEXA_LEG_J1_TO_J2 + (HEXA_LEG_J2_TO_J3*COS30) + HEXA_LEG_J3_TO_TIP*SIN15)*COS45)
#define OTHER_Y     (HEXA_MOUNT_OTHER_Y + (HEXA_LEG_ROOT_TO_J1 + HEXA_LEG_J1_TO_J2 + (HEXA_LEG_J2_TO_J3*COS30) + HEXA_LEG_J3_TO_TIP*SIN15)*SIN45)

#define P1X  OTHER_X
#define P1Y  OTHER_Y
#define P1Z  (-STANDBY_Z)
#define P2X  LEFTRIGHT_X
#define P2Y  0.0f
#define P2Z  (-STANDBY_Z)
#define P3X  OTHER_X
#define P3Y  (-OTHER_Y)
#define P3Z  (-STANDBY_Z)
#define P4X  (-OTHER_X)
#define P4Y  (-OTHER_Y)
#define P4Z  (-STANDBY_Z)
#define P5X  (-LEFTRIGHT_X)
#define P5Y  0.0f
#define P5Z  (-STANDBY_Z)
#define P6X  (-OTHER_X)
#define P6Y  OTHER_Y
#define P6Z  (-STANDBY_Z)

#endif
```

然后紧跟 pathTool 生成的 `Locations` 数组和 `MovementTable` 声明。

**如果不用 pathTool 重新生成**，可以直接把 `NodeHexa/firmware/src/generated/movement_table.h` 复制过来，把 C++ 的 `namespace { }` 去掉，加上上面的宏头。但 **强烈建议先对齐 config.py 再重新生成**，否则连杆参数不匹配。

### 4.10 第 9 步：`app/Hexa/hexapod.h/.c`（整机）

```c
#ifndef __HEXA_HEXAPOD_H
#define __HEXA_HEXAPOD_H

#include "leg.h"
#include "movement.h"

typedef struct {
    HexaLeg   legs[6];
    Movement  movement;
    MovementMode mode;
} Hexapod;

void Hexapod_Init(Hexapod *hp);
void Hexapod_ProcessMovement(Hexapod *hp, MovementMode mode, int elapsed);
void Hexapod_SetSpeed(Hexapod *hp, float speed);

/* 校准接口 */
void Hexapod_CalibrationLoad(Hexapod *hp);
void Hexapod_CalibrationSave(const Hexapod *hp);
void Hexapod_CalibrationSet(Hexapod *hp, int leg, int part, int offset);
void Hexapod_CalibrationGet(const Hexapod *hp, int leg, int part, int *offset);

#endif
```

**hexapod.c 核心逻辑（照搬 `hexapod.cpp`）：**

1. `Hexapod_Init(hp)`：初始化 6 条腿 → 加载校准 → snapToMode(STANDBY)。
2. `Hexapod_ProcessMovement(hp, mode, elapsed)`：
   ```
   if (mode != hp->mode) { hp->mode = mode; Movement_SetMode(&hp->movement, mode); }
   Locations loc = *Movement_Next(&hp->movement, elapsed);
   for (i=0; i<6; i++) HexaLeg_MoveTip(&hp->legs[i], &loc.leg[i]);
   ```
3. 校准保存/加载：F407 内部 Flash（最后 1 个扇区，存 `int16_t offsets[18]`），不用 JSON（NodeHexa 用 SPIFFS JSON，F407 没有文件系统）。

### 4.11 第 10 步：`app/Hexa/command.h/.c`（指令解析）

```c
#ifndef __HEXA_COMMAND_H
#define __HEXA_COMMAND_H

#include "movement.h"

/* 字符串 → 枚举映射 */
MovementMode Command_Parse(const char *str);

/* 枚举 → 字符串 */
const char *Command_ToName(MovementMode mode);

#endif
```

映射表直接抄 `main.cpp` 1324~1346 行：

| 字符串 | 枚举 |
|---|---|
| "standby" | MOVEMENT_STANDBY |
| "forward" | MOVEMENT_FORWARD |
| "forwardfast" / "forward_fast" | MOVEMENT_FORWARDFAST |
| "backward" | MOVEMENT_BACKWARD |
| "turnleft" / "turn_left" | MOVEMENT_TURNLEFT |
| "turnright" / "turn_right" | MOVEMENT_TURNRIGHT |
| "shiftleft" / "shift_left" | MOVEMENT_SHIFTLEFT |
| "shiftright" / "shift_right" | MOVEMENT_SHIFTRIGHT |
| "climb" | MOVEMENT_CLIMB |
| "rotatex" / "rotate_x" | MOVEMENT_ROTATEX |
| "rotatey" / "rotate_y" | MOVEMENT_ROTATEY |
| "rotatez" / "rotate_z" | MOVEMENT_ROTATEZ |
| "twist" | MOVEMENT_TWIST |
| "beatsway" / "beat_sway" | MOVEMENT_BEATSWAY |

### 4.12 第 11 步：集成到 `main.c`

**修改 `A_Entry/main.c`：**

1. 文件顶部加：
```c
#include "hexapod.h"
#include "command.h"

static Hexapod s_hexapod;
static MovementMode s_mode = MOVEMENT_STANDBY;
static volatile uint8_t s_tick20ms = 0U;  /* TIM3 中断每 1ms 置起，20 次 = 20ms */
```

2. `main()` 的 BSP 初始化区（PCA9685_Init 之后）加：
```c
    Hexapod_Init(&s_hexapod);
```

3. `ControlTask` 里加 20ms 步态节拍：
```c
    if (s_tick20ms >= 20U) {
        s_tick20ms = 0U;
        Hexapod_ProcessMovement(&s_hexapod, s_mode, 20);
    }
```

4. 在串口/NRF/SU-03T 接收回调里，收到字符串后：
```c
    MovementMode newMode = Command_Parse(receivedString);
    if (newMode < MOVEMENT_TOTAL) {
        s_mode = newMode;
    }
```

5. 注册 TIM3 的 1ms 回调里加 `s_tick20ms++`（如果还没有）。

**CMake 登记：**

在 `CMakeLists.txt` 的 `APP_LAYER_SOURCES` 里加：
```cmake
    ${CMAKE_SOURCE_DIR}/app/Hexa/leg.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/servo.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/movement.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/movement_profile.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/hexapod.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/command.c
    ${CMAKE_SOURCE_DIR}/app/Hexa/calibration.c
```

头文件目录加：
```cmake
    ${CMAKE_SOURCE_DIR}/app/Hexa
```

## 5. 验收标准

1. 编译零错误（除既有历史警告）。
2. 上电后机器人自动进入 STANDBY 姿态（六条腿站稳）。
3. 串口发送 `"forward"` → 机器人前进，发送 `"standby"` → 停止。
4. 全部 14 种模式都能执行，每种模式的动作与 NodeHexa 原版肉眼一致。
5. 校准 offset 保存后断电重启不丢失。

## 6. 坑清单

1. **config.py 与 config.h 不一致**（§4.6）——必须先对齐再生成，否则步态脚位是错的。
2. **C++ → C 翻译**：`std::atan2` → `atan2f`，`std::acos` → `acosf`，`M_PI` 需要 `#include <math.h>`。
3. **P1X~P6Z 宏**：生成的动作表依赖这些宏，必须放在 `movement_table.h` 头部（§4.9）。
4. **MovementTable 声明的 extern 顺序**：`movement.c` 里 extern 声明 14 个 `xxxTable()`，`movement_table.h` 里定义它们。注意 C 不允许函数重载，每个 `xxxTable()` 返回对应 `MovementTable`。
5. **校准存储**：NodeHexa 用 SPIFFS JSON，F407 没有文件系统。改用内部 Flash 最后 1 个扇区存 `int16_t[18]`，注意擦除粒度（F407 扇区 16KB/64KB/128KB）。
6. **PCA9685 句柄**：`servo.c` 需要访问 `main.c` 里定义的 `s_pca9685A`/`s_pca9685B`，用 `extern` 声明。
7. **FreeRTOS 并发**：`ControlTask` 调用 `Hexapod_ProcessMovement` 会写 PCA9685（内部调 `PCA9685_SetPWM`），而 PCA9685 驱动已经自带 `API_I2C_Lock/Unlock`，所以不需要额外加锁。
8. **腿编号方向**：确保你的实物接线和 §4.4 通道表一致。不一致时改接线，不要改表（算法按表写死）。

---

*本文档是 NodeHexa → QuadArachnid 迁移的第 2 部分（运动控制）。迁移完成后，第 3 部分：电池电压监测、低电保护、多通道输入仲裁（蓝牙/NRF/语音优先级），另文交接。*