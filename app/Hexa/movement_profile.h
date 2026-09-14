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