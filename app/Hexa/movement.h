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