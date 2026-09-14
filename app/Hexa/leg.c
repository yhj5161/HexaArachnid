#include "leg.h"
#include "config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- 旋转函数 ---- */
#define SIN45 0.7071f
#define COS45 0.7071f

static void rotate0(Point3D src, Point3D *dst) { *dst = src; }

static void rotate45(Point3D src, Point3D *dst) {
	dst->x = src.x * COS45 - src.y * SIN45;
	dst->y = src.x * SIN45 + src.y * COS45;
	dst->z = src.z;
}

static void rotate135(Point3D src, Point3D *dst) {
	dst->x = src.x * (-COS45) - src.y * SIN45;
	dst->y = src.x * SIN45 + src.y * (-COS45);
	dst->z = src.z;
}

static void rotate180(Point3D src, Point3D *dst) {
	dst->x = -src.x;
	dst->y = -src.y;
	dst->z = src.z;
}

static void rotate225(Point3D src, Point3D *dst) {
	dst->x = src.x * (-COS45) - src.y * (-SIN45);
	dst->y = src.x * (-SIN45) + src.y * (-COS45);
	dst->z = src.z;
}

static void rotate315(Point3D src, Point3D *dst) {
	dst->x = src.x * COS45 - src.y * (-SIN45);
	dst->y = src.x * (-SIN45) + src.y * COS45;
	dst->z = src.z;
}

static float clampUnit(float value)
{
	if (value < -1.0f) return -1.0f;
	if (value > 1.0f)  return 1.0f;
	return value;
}

/* ---- 公共接口 ---- */

void HexaLeg_Init(HexaLeg *leg, int legIndex)
{
	leg->index = legIndex;

	switch (legIndex) {
	case 0: /* 45° 右前 */
		leg->mountPos.x =  HEXA_MOUNT_OTHER_X;
		leg->mountPos.y =  HEXA_MOUNT_OTHER_Y;
		leg->mountPos.z =  0.0f;
		leg->localConv  = rotate315;
		leg->worldConv  = rotate45;
		break;
	case 1: /* 0° 右中 */
		leg->mountPos.x =  HEXA_MOUNT_LR_X;
		leg->mountPos.y =  0.0f;
		leg->mountPos.z =  0.0f;
		leg->localConv  = rotate0;
		leg->worldConv  = rotate0;
		break;
	case 2: /* -45° 右后 */
		leg->mountPos.x =  HEXA_MOUNT_OTHER_X;
		leg->mountPos.y = -HEXA_MOUNT_OTHER_Y;
		leg->mountPos.z =  0.0f;
		leg->localConv  = rotate45;
		leg->worldConv  = rotate315;
		break;
	case 3: /* -135° 左后 */
		leg->mountPos.x = -HEXA_MOUNT_OTHER_X;
		leg->mountPos.y = -HEXA_MOUNT_OTHER_Y;
		leg->mountPos.z =  0.0f;
		leg->localConv  = rotate135;
		leg->worldConv  = rotate225;
		break;
	case 4: /* 180° 左中 */
		leg->mountPos.x = -HEXA_MOUNT_LR_X;
		leg->mountPos.y =  0.0f;
		leg->mountPos.z =  0.0f;
		leg->localConv  = rotate180;
		leg->worldConv  = rotate180;
		break;
	case 5: /* 135° 左前 */
	default:
		leg->mountPos.x = -HEXA_MOUNT_OTHER_X;
		leg->mountPos.y =  HEXA_MOUNT_OTHER_Y;
		leg->mountPos.z =  0.0f;
		leg->localConv  = rotate225;
		leg->worldConv  = rotate135;
		break;
	}

	ServoJoint_Init(&leg->joints[0], legIndex, 0);
	ServoJoint_Init(&leg->joints[1], legIndex, 1);
	ServoJoint_Init(&leg->joints[2], legIndex, 2);

	leg->tipPos.x = 0.0f; leg->tipPos.y = 0.0f; leg->tipPos.z = 0.0f;
	leg->tipPosLocal.x = 0.0f; leg->tipPosLocal.y = 0.0f; leg->tipPosLocal.z = 0.0f;
}

void HexaLeg_FK(float angles[3], Point3D *out)
{
	float rad[3];
	float x;
	int i;

	for (i = 0; i < 3; i++)
		rad[i] = (float)M_PI * angles[i] / 180.0f;

	x = HEXA_LEG_J1_TO_J2 + cosf(rad[1]) * HEXA_LEG_J2_TO_J3
	    + cosf(rad[1] + rad[2] - (float)M_PI / 2.0f) * HEXA_LEG_J3_TO_TIP;

	out->x = HEXA_LEG_ROOT_TO_J1 + cosf(rad[0]) * x;
	out->y = sinf(rad[0]) * x;
	out->z = sinf(rad[1]) * HEXA_LEG_J2_TO_J3
	       + sinf(rad[1] + rad[2] - (float)M_PI / 2.0f) * HEXA_LEG_J3_TO_TIP;
}

void HexaLeg_IK(const Point3D *to, float angles[3])
{
	float x, y, ar, lr2, lr, ratio1, ratio2, a1, a2;

	x = to->x - HEXA_LEG_ROOT_TO_J1;
	y = to->y;

	angles[0] = atan2f(y, x) * 180.0f / (float)M_PI;

	x = sqrtf(x*x + y*y) - HEXA_LEG_J1_TO_J2;
	y = to->z;
	ar = atan2f(y, x);
	lr2 = x*x + y*y;
	lr = sqrtf(lr2);
	if (lr < 1e-4f) lr = 1e-4f;

	ratio1 = (lr2 + HEXA_LEG_J2_TO_J3*HEXA_LEG_J2_TO_J3 - HEXA_LEG_J3_TO_TIP*HEXA_LEG_J3_TO_TIP)
	         / (2.0f * HEXA_LEG_J2_TO_J3 * lr);
	ratio2 = (lr2 - HEXA_LEG_J2_TO_J3*HEXA_LEG_J2_TO_J3 + HEXA_LEG_J3_TO_TIP*HEXA_LEG_J3_TO_TIP)
	         / (2.0f * HEXA_LEG_J3_TO_TIP * lr);

	a1 = acosf(clampUnit(ratio1));
	a2 = acosf(clampUnit(ratio2));

	angles[1] = (ar + a1) * 180.0f / (float)M_PI;
	angles[2] = 90.0f - ((a1 + a2) * 180.0f / (float)M_PI);
}

void HexaLeg_WorldToLocal(const HexaLeg *leg, Point3D world, Point3D *local)
{
	Point3D tmp = p3d_sub(world, leg->mountPos);
	leg->localConv(tmp, local);
}

void HexaLeg_LocalToWorld(const HexaLeg *leg, Point3D local, Point3D *world)
{
	leg->worldConv(local, world);
	p3d_addEq(world, leg->mountPos);
}

static void HexaLeg_Move(const HexaLeg *leg, const Point3D *to)
{
	float angles[3];
	HexaLeg_IK(to, angles);
	ServoJoint_SetAngle((ServoJoint *)&leg->joints[0], angles[0]);
	ServoJoint_SetAngle((ServoJoint *)&leg->joints[1], angles[1]);
	ServoJoint_SetAngle((ServoJoint *)&leg->joints[2], angles[2]);
}

void HexaLeg_MoveTip(HexaLeg *leg, const Point3D *to)
{
	Point3D local;

	if (p3d_eq(*to, leg->tipPos))
		return;

	HexaLeg_WorldToLocal(leg, *to, &local);
	HexaLeg_Move(leg, &local);
	leg->tipPos = *to;
	leg->tipPosLocal = local;
}