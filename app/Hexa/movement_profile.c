#include "movement_profile.h"

static const MovementMetrics kMetrics[MOVEMENT_TOTAL] = {
	/* MOVEMENT_STANDBY     */ {0.0f, 0.0f, 1.0f},
	/* MOVEMENT_FORWARD     */ {0.050f, 0.0f, 2.0f},
	/* MOVEMENT_FORWARDFAST */ {0.100f, 0.0f, 2.0f},
	/* MOVEMENT_BACKWARD    */ {0.050f, 0.0f, 2.0f},
	/* MOVEMENT_TURNLEFT    */ {0.0f, 30.0f, 2.0f},
	/* MOVEMENT_TURNRIGHT   */ {0.0f, 30.0f, 2.0f},
	/* MOVEMENT_SHIFTLEFT   */ {0.050f, 0.0f, 2.0f},
	/* MOVEMENT_SHIFTRIGHT  */ {0.050f, 0.0f, 2.0f},
	/* MOVEMENT_CLIMB       */ {0.040f, 0.0f, 2.0f},
	/* MOVEMENT_ROTATEX     */ {0.0f, 15.0f, 2.0f},
	/* MOVEMENT_ROTATEY     */ {0.0f, 15.0f, 2.0f},
	/* MOVEMENT_ROTATEZ     */ {0.0f, 20.0f, 2.0f},
	/* MOVEMENT_TWIST       */ {0.0f, 15.0f, 2.0f},
	/* MOVEMENT_BEATSWAY    */ {0.0f, 18.0f, 2.0f},
};

const MovementMetrics *MovementProfile_Get(MovementMode mode)
{
	if (mode >= MOVEMENT_TOTAL) {
		static const MovementMetrics fallback = {0.0f, 0.0f, 1.0f};
		return &fallback;
	}
	return &kMetrics[mode];
}