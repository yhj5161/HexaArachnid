#ifndef __HEXA_SERVO_H
#define __HEXA_SERVO_H

#include <stdint.h>
#include <stdbool.h>

/* 关节参数：每个 joint 的 range/inverse/adjust 不同 */
typedef struct {
	uint8_t pcaCh;       /* PCA9685 通道号（0~15） */
	uint8_t board;       /* 0=g_pca1(I2C3,右侧), 1=g_pca2(I2C4,左侧) */
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