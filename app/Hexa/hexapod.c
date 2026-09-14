#include "hexapod.h"
#include "servo.h"

#define CALIB_MAGIC  0xCA10U  /* 校准数据魔数 */
#define CALIB_SECTOR (7U)     /* F407 最后扇区（128KB 布局） */

/* 校准数据存储地址：F407 闪存扇区 7 起始 */
#define CALIB_ADDR   ((volatile uint32_t *)0x08060000U)

/* 校准保存结构：魔数 + 18 个 int16_t offset */
typedef struct {
	uint16_t magic;
	int16_t  offsets[18];
} CalibData;

void Hexapod_Init(Hexapod *hp)
{
	int i;

	for (i = 0; i < 6; i++)
		HexaLeg_Init(&hp->legs[i], i);

	Hexapod_CalibrationLoad(hp);

	Movement_Init(&hp->movement, MOVEMENT_STANDBY);
	Movement_SnapToMode(&hp->movement, MOVEMENT_STANDBY);
	hp->mode = MOVEMENT_STANDBY;
}

void Hexapod_ProcessMovement(Hexapod *hp, MovementMode mode, int elapsed)
{
	int i;
	const Locations *loc;

	if (hp->mode != mode) {
		hp->mode = mode;
		Movement_SetMode(&hp->movement, mode);
	}

	loc = (Locations *)Movement_Next(&hp->movement, elapsed);
	for (i = 0; i < 6; i++)
		HexaLeg_MoveTip(&hp->legs[i], &(*loc)[i]);
}

void Hexapod_SetSpeed(Hexapod *hp, float speed)
{
	Movement_SetSpeed(&hp->movement, speed);
}

/* ---- 校准接口（Flash 存储） ---- */

void Hexapod_CalibrationLoad(Hexapod *hp)
{
	const CalibData *calib = (const CalibData *)CALIB_ADDR;
	int i, leg, part;

	if (calib->magic != CALIB_MAGIC)
		return;

	for (i = 0; i < 18; i++) {
		leg  = i / 3;
		part = i % 3;
		ServoJoint_SetOffset(&hp->legs[leg].joints[part], (int)calib->offsets[i], false);
	}
}

void Hexapod_CalibrationSave(const Hexapod *hp)
{
	/* F407 内部 Flash 写入需要 HAL 或裸寄存器操作，此处留空。
	 * 校准数据由上位机通过串口指令读写，暂不实现自动保存。 */
	(void)hp;
}

void Hexapod_CalibrationSet(Hexapod *hp, int leg, int part, int offset)
{
	if (leg < 0 || leg >= 6 || part < 0 || part >= 3)
		return;
	ServoJoint_SetOffset(&hp->legs[leg].joints[part], offset, false);
}

void Hexapod_CalibrationGet(const Hexapod *hp, int leg, int part, int *offset)
{
	if (leg < 0 || leg >= 6 || part < 0 || part >= 3) {
		*offset = 0;
		return;
	}
	*offset = ServoJoint_GetOffset(&hp->legs[leg].joints[part]);
}