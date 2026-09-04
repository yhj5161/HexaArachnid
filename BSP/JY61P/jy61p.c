/*
 * jy61p.c — 维特智能 JY61P 六轴陀螺仪驱动
 *
 * 硬件背景：
 *   - 通信方式：UART (115200 bps)
 *   - 数据格式：16 进制原始字节，主动上报
 *   - 帧格式：0x55 + 类型(0x51/0x52/0x53) + 8字节数据 + 校验和 = 11 字节
 *
 * 协议数据包：
 *   0x51 — 加速度：AxL AxH AyL AyH AzL AzH TL TH  (量程 ±16g)
 *   0x52 — 角速度：WxL WxH WyL WyH WzL WzH VolL VolH  (量程 ±2000°/s)
 *   0x53 — 角度：  RollL RollH PitchL PitchH YawL YawH VL VH  (量程 ±180°)
 *
 * 架构设计（中断上半部 / 下半部分离）：
 *
 *   ┌─ USART ISR ─────────────────────────────┐
 *   │  JY61P_RxPush(byte)  →  环形缓冲区      │  ← 极快，只入队
 *   └──────────────────────────────────────────┘
 *                       ↓
 *   ┌─ 主循环 ────────────────────────────────┐
 *   │  JY61P_Task()  →  取字节 → 校验 → 浮点   │  ← 不在 ISR 上下文
 *   └──────────────────────────────────────────┘
 */

#include "jy61p.h"
#include "My_Usart/My_Usart.h"  /* usart_send_byte / USART3 */
#include "Delay.h"              /* Delay_ms                 */

/*===========================================================================
 * 环形缓冲区
 *===========================================================================*/
#define JY61P_RX_BUF_SIZE  128U

static uint8_t  s_rx_buf[JY61P_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;  /* ISR 写 */
static uint16_t s_rx_tail = 0U;           /* 主循环读 */

/*===========================================================================
 * 协议常量
 *===========================================================================*/
#define JY61P_HEADER       0x55U
#define JY61P_TYPE_ACC     0x51U
#define JY61P_TYPE_GYRO    0x52U
#define JY61P_TYPE_ANGLE   0x53U
#define JY61P_PACKET_LEN   11U

#define JY61P_ACC_SCALE    (16.0f / 32768.0f)
#define JY61P_GYRO_SCALE   (2000.0f / 32768.0f)
#define JY61P_ANGLE_SCALE  (180.0f / 32768.0f)
#define JY61P_TEMP_SCALE   (1.0f / 100.0f)

#define STATE_HEADER  0U
#define STATE_TYPE    1U
#define STATE_DATA    2U

/*===========================================================================
 * 全局传感器实例
 *===========================================================================*/
static JY61P_Data_t s_data;

/*===========================================================================
 * 内部辅助
 *===========================================================================*/

static int16_t JY61P_MakeShort(uint8_t low, uint8_t high)
{
    return (int16_t)(((int16_t)((uint16_t)high) << 8) | (uint16_t)low);
}

static uint8_t JY61P_Checksum(const uint8_t *buf)
{
    uint16_t sum = 0U;
    uint8_t  i;
    for (i = 0U; i < 10U; i++) { sum += (uint16_t)buf[i]; }
    return (uint8_t)(sum & 0xFFU);
}

static void JY61P_ParseAcc(const uint8_t *buf)
{
    s_data.acc_x = (float)JY61P_MakeShort(buf[2], buf[3]) * JY61P_ACC_SCALE;
    s_data.acc_y = (float)JY61P_MakeShort(buf[4], buf[5]) * JY61P_ACC_SCALE;
    s_data.acc_z = (float)JY61P_MakeShort(buf[6], buf[7]) * JY61P_ACC_SCALE;
    s_data.temp  = (float)JY61P_MakeShort(buf[8], buf[9]) * JY61P_TEMP_SCALE;
    s_data.update_flags |= JY61P_ACC_UPDATE;
}

static void JY61P_ParseGyro(const uint8_t *buf)
{
    s_data.gyro_x = (float)JY61P_MakeShort(buf[2], buf[3]) * JY61P_GYRO_SCALE;
    s_data.gyro_y = (float)JY61P_MakeShort(buf[4], buf[5]) * JY61P_GYRO_SCALE;
    s_data.gyro_z = (float)JY61P_MakeShort(buf[6], buf[7]) * JY61P_GYRO_SCALE;
    s_data.update_flags |= JY61P_GYRO_UPDATE;
}

static void JY61P_ParseAngle(const uint8_t *buf)
{
    s_data.roll  = (float)JY61P_MakeShort(buf[2], buf[3]) * JY61P_ANGLE_SCALE;
    s_data.pitch = (float)JY61P_MakeShort(buf[4], buf[5]) * JY61P_ANGLE_SCALE;
    s_data.yaw   = (float)JY61P_MakeShort(buf[6], buf[7]) * JY61P_ANGLE_SCALE;
    s_data.update_flags |= JY61P_ANGLE_UPDATE;
}

/*===========================================================================
 * JY61P_RxPush — ISR 调用：仅入队（中断上半部）
 *===========================================================================*/

void JY61P_RxPush(uint8_t data)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % JY61P_RX_BUF_SIZE);

    s_rx_buf[s_rx_head] = data;
    s_rx_head = next;

    /* 缓冲满 → 丢弃最老 1 字节（滑动窗口，防止死锁） */
    if (next == s_rx_tail)
    {
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % JY61P_RX_BUF_SIZE);
    }
}

/*===========================================================================
 * JY61P_Task — 主循环调用：解析已缓冲的字节（中断下半部）
 *===========================================================================*/

/*
 * activeType 决定解析哪种数据包：
 *   JY61P_TYPE_ANGLE (0x53) — 仅解析角度
 *   JY61P_TYPE_ACC   (0x51) — 仅解析加速度
 *   JY61P_TYPE_GYRO  (0x52) — 仅解析角速度
 *   0U                         — 解析全部
 */
static uint8_t s_activeType = 0;  /* 选择需要解析的数据包 */

void JY61P_SetActiveType(uint8_t packetType)
{
    s_activeType = packetType;
}

void JY61P_Task(void)
{
    static uint8_t state = STATE_HEADER;
    static uint8_t buf[11];
    static uint8_t index = 0U;
    static uint8_t ptype = 0U;

    uint8_t data;

    while (s_rx_tail != s_rx_head)
    {
        data       = s_rx_buf[s_rx_tail];
        s_rx_tail  = (uint16_t)((s_rx_tail + 1U) % JY61P_RX_BUF_SIZE);

        switch (state)
        {
        case STATE_HEADER:
            if (data == JY61P_HEADER)
            {
                buf[0] = data;
                index  = 1U;
                state  = STATE_TYPE;
            }
            break;

        case STATE_TYPE:
            if (data == JY61P_TYPE_ACC  ||
                data == JY61P_TYPE_GYRO ||
                data == JY61P_TYPE_ANGLE)
            {
                if (s_activeType == 0U || data == s_activeType)
                {
                    buf[1] = data;
                    ptype  = data;
                    index  = 2U;
                    state  = STATE_DATA;
                }
                else
                {
                    state = STATE_HEADER;  /* 不是需要的类型 → 跳过 */
                }
            }
            else
            {
                state = STATE_HEADER;
            }
            break;

        case STATE_DATA:
            buf[index] = data;
            index++;

            if (index >= JY61P_PACKET_LEN)
            {
                state = STATE_HEADER;

                if (JY61P_Checksum(buf) == buf[10])
                {
                    switch (ptype)
                    {
                    case JY61P_TYPE_ACC:   JY61P_ParseAcc(buf);   break;
                    case JY61P_TYPE_GYRO:  JY61P_ParseGyro(buf);  break;
                    case JY61P_TYPE_ANGLE: JY61P_ParseAngle(buf); break;
                    default: break;
                    }
                }
            }
            break;

        default:
            state = STATE_HEADER;
            break;
        }
    }
}

/*===========================================================================
 * 数据获取 API
 *===========================================================================*/

void JY61P_Init(void)
{
    uint16_t i;
    s_data.acc_x  = 0.0f;
    s_data.acc_y  = 0.0f;
    s_data.acc_z  = 0.0f;
    s_data.gyro_x = 0.0f;
    s_data.gyro_y = 0.0f;
    s_data.gyro_z = 0.0f;
    s_data.roll   = 0.0f;
    s_data.pitch  = 0.0f;
    s_data.yaw    = 0.0f;
    s_data.temp   = 0.0f;
    s_data.update_flags = 0U;

    s_rx_head = 0U;
    s_rx_tail = 0U;
    for (i = 0U; i < JY61P_RX_BUF_SIZE; i++) { s_rx_buf[i] = 0U; }
}

const JY61P_Data_t *JY61P_GetData(void)
{
    return &s_data;
}

uint8_t JY61P_GetUpdateFlags(void)
{
    uint8_t flags = s_data.update_flags;
    s_data.update_flags = 0U;
    return flags;
}

void JY61P_GetAcc(float *ax, float *ay, float *az)
{
    if (ax != 0) { *ax = s_data.acc_x; }
    if (ay != 0) { *ay = s_data.acc_y; }
    if (az != 0) { *az = s_data.acc_z; }
}

void JY61P_GetGyro(float *gx, float *gy, float *gz)
{
    if (gx != 0) { *gx = s_data.gyro_x; }
    if (gy != 0) { *gy = s_data.gyro_y; }
    if (gz != 0) { *gz = s_data.gyro_z; }
}

void JY61P_GetAngle(float *roll, float *pitch, float *yaw)
{
    if (roll  != 0) { *roll  = s_data.roll;  }
    if (pitch != 0) { *pitch = s_data.pitch; }
    if (yaw   != 0) { *yaw   = s_data.yaw;   }
}

float JY61P_GetTemp(void)
{
    return s_data.temp;
}

/*===========================================================================
 * 写操作（配置指令）
 *
 * JY61P 指令协议：
 *   帧格式：0xFF 0xAA ADDR DATAL DATAH（5 字节固定，无校验和）
 *   流程：解锁 → 延时 → 发指令 → 延时 → 保存
 *
 * 注意：
 *   - 所有数据均为 16 进制原始字节，不是 ASCII
 *   - 带长延时的函数会阻塞当前线程（通常在主循环初始化阶段调用一次即可）
 *===========================================================================*/

/* ── 指令常量 ── */
#define JY61P_CMD_HEADER1  0xFFU
#define JY61P_CMD_HEADER2  0xAAU
#define JY61P_REG_SAVE     0x00U   /* 保存寄存器        */
#define JY61P_REG_CALIB    0x01U   /* 校准寄存器        */
#define JY61P_REG_UNLOCK   0x69U   /* 解锁寄存器        */
#define JY61P_UNLOCK_VAL   0xB588U /* 解锁魔数          */

#define JY61P_USART         USART1  /* JY61P 串口号：接 USART1 (PB6/PB7, 115200) */

/* ── 发送 5 字节指令包 ── */
static void JY61P_SendCmd(uint8_t addr, uint16_t data)
{
    usart_send_byte(JY61P_USART, JY61P_CMD_HEADER1);
    usart_send_byte(JY61P_USART, JY61P_CMD_HEADER2);
    usart_send_byte(JY61P_USART, addr);
    usart_send_byte(JY61P_USART, (uint8_t)(data & 0xFFU));          /* DATAL */
    usart_send_byte(JY61P_USART, (uint8_t)((data >> 8) & 0xFFU));   /* DATAH */
}

/* ── 解锁 ── */
static void JY61P_Unlock(void)
{
    JY61P_SendCmd(JY61P_REG_UNLOCK, JY61P_UNLOCK_VAL);
}

/* ── 保存 ── */
static void JY61P_Save(void)
{
    JY61P_SendCmd(JY61P_REG_SAVE, 0x0000U);
}

/*===========================================================================
 * JY61P_ZAxisZero — Z 轴（偏航角）置零
 *
 * 需要六轴算法（上位机可切换）。
 * 九轴算法下为绝对角度，不能归零。
 * 阻塞约 3.5 秒。
 *===========================================================================*/
void JY61P_ZAxisZero(void)
{
    JY61P_Unlock();
    Delay_ms(200U);

    JY61P_SendCmd(JY61P_REG_CALIB, 0x0004U);  /* Z 轴归零 */
    Delay_ms(3000U);

    JY61P_Save();
}
