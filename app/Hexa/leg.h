#ifndef __HEXA_LEG_H
#define __HEXA_LEG_H

#include "base.h"
#include "servo.h"

typedef void (*RotateFunc)(Point3D src, Point3D *dst);

typedef struct {
	int         index;        /* 腿编号 0~5 */
	Point3D     mountPos;     /* 安装位置 */
	ServoJoint  joints[3];    /* 3 个关节 */
	Point3D     tipPos;       /* 当前足端世界坐标 */
	Point3D     tipPosLocal;  /* 当前足端局部坐标 */
	RotateFunc  localConv;    /* 世界→局部 旋转函数 */
	RotateFunc  worldConv;    /* 局部→世界 旋转函数 */
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