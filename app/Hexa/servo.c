#include "servo.h"
#include "PCA9685.h"
#include <math.h>

/* 主控里定义的 PCA9685 句柄 */
extern PCA9685_Handle_t g_pca1;  /* I2C3, 右侧 legs 0/1/2 */
extern PCA9685_Handle_t g_pca2;  /* I2C4, 左侧 legs 3/4/5 */

/* 从 NodeHexa 的 hexapod2pwm() 搬过来 */
static int hexapod2pwm(int legIndex, int partIndex)
{
	switch(legIndex) {
	case 0: return 5 + partIndex;   /* 右前: ch5,6,7 → g_pca1 */
	case 1: return 2 + partIndex;   /* 右中: ch2,3,4 → g_pca1 */
	case 2: return 8 + partIndex;   /* 右后: ch8,9,10 → g_pca1 */
	case 3: return 16 + 8 + partIndex; /* 左后: ch24,25,26 → g_pca2 ch8,9,10 */
	case 4: return 16 + 2 + partIndex; /* 左中: ch18,19,20 → g_pca2 ch2,3,4 */
	case 5: return 16 + 5 + partIndex; /* 左前: ch21,22,23 → g_pca2 ch5,6,7 */
	default: return 0;
	}
}

void ServoJoint_Init(ServoJoint *sj, int legIndex, int partIndex)
{
	int pwmIdx = hexapod2pwm(legIndex, partIndex);

	sj->board   = (uint8_t)(pwmIdx < 16 ? 0 : 1);
	sj->pcaCh   = (uint8_t)(pwmIdx < 16 ? pwmIdx : pwmIdx - 16);
	sj->range   = (partIndex == 0) ? 45.0f : 60.0f;
	sj->inverse = (partIndex == 1) ? true : false;
	sj->adjust  = (partIndex == 1) ? 15.0f : 0.0f;
	sj->offset  = 0;
	sj->angle   = 0.0f;
}

void ServoJoint_SetAngle(ServoJoint *sj, float angle)
{
	PCA9685_Handle_t *pca;
	float a;
	int us;

	if (angle > sj->range + sj->adjust)
		angle = sj->range + sj->adjust;
	else if (angle < -sj->range + sj->adjust)
		angle = -sj->range + sj->adjust;

	sj->angle = angle;

	pca = (sj->board == 0U) ? &g_pca1 : &g_pca2;

	a = angle - sj->adjust;
	if (sj->inverse)
		a = -a;

	us = 1500 + (int)((a + (float)sj->offset) * (1000.0f / 90.0f));
	if (us > 2500) us = 2500;
	else if (us < 500) us = 500;

	PCA9685_SetServoPulseUs(pca, sj->pcaCh, (uint16_t)us);
}

float ServoJoint_GetAngle(const ServoJoint *sj)
{
	return sj->angle;
}

void ServoJoint_SetOffset(ServoJoint *sj, int offset, bool update)
{
	sj->offset = offset;
	if (update)
		ServoJoint_SetAngle(sj, sj->angle);
}

int ServoJoint_GetOffset(const ServoJoint *sj)
{
	return sj->offset;
}