#include "movement.h"
#include "config.h"
#include "movement_table.h"
#include <stdlib.h>
#include <string.h>

static const MovementTable *kTable[MOVEMENT_TOTAL];
static int s_tablesInited = 0;

static void initTables(void)
{
	if (s_tablesInited) return;
	kTable[0]  = standbyTable();
	kTable[1]  = forwardTable();
	kTable[2]  = forwardfastTable();
	kTable[3]  = backwardTable();
	kTable[4]  = turnleftTable();
	kTable[5]  = turnrightTable();
	kTable[6]  = shiftleftTable();
	kTable[7]  = shiftrightTable();
	kTable[8]  = climbTable();
	kTable[9]  = rotatexTable();
	kTable[10] = rotateyTable();
	kTable[11] = rotatezTable();
	kTable[12] = twistTable();
	kTable[13] = beatswayTable();
	s_tablesInited = 1;
}

const MovementTable *Movement_GetTable(MovementMode mode)
{
	initTables();
	if (mode >= MOVEMENT_TOTAL) return kTable[MOVEMENT_STANDBY];
	return kTable[mode];
}

void Movement_Init(Movement *mv, MovementMode mode)
{
	initTables();
	mv->mode       = mode;
	mv->speed      = HEXA_DEFAULT_SPEED;
	mv->index      = 0;
	mv->remainTime = 0;
	memcpy(mv->position, kTable[MOVEMENT_STANDBY]->table[0], sizeof(Locations));
}

void Movement_SetMode(Movement *mv, MovementMode newMode)
{
	const MovementTable *table;
	int actualDuration, actualSwitchDuration;

	initTables();
	if (newMode >= MOVEMENT_TOTAL) return;

	table = kTable[newMode];
	if (!table->entries || table->entriesCount <= 0 || table->length <= 0) return;

	mv->mode = newMode;
	mv->index = table->entries[rand() % table->entriesCount];
	actualDuration = (int)((float)table->stepDuration / mv->speed);
	actualSwitchDuration = (int)((float)HEXA_MOVEMENT_SWITCH_DUR / mv->speed);
	mv->remainTime = actualSwitchDuration > actualDuration ? actualSwitchDuration : actualDuration;
}

void Movement_SnapToMode(Movement *mv, MovementMode newMode)
{
	const MovementTable *table;

	initTables();
	if (newMode >= MOVEMENT_TOTAL) newMode = MOVEMENT_STANDBY;

	table = kTable[newMode];
	if (!table->entries || table->entriesCount <= 0 || table->length <= 0) {
		mv->mode = newMode;
		mv->index = 0;
		mv->remainTime = 0;
		return;
	}

	mv->mode = newMode;
	mv->index = table->entries[0];
	if (mv->index < 0 || mv->index >= table->length) mv->index = 0;
	memcpy(mv->position, table->table[mv->index], sizeof(Locations));
	mv->remainTime = 0;
}

const Locations *Movement_Next(Movement *mv, int elapsed)
{
	const MovementTable *table = kTable[mv->mode];
	int actualStepDuration;
	float ratio;
	Locations diff;

	actualStepDuration = (int)((float)table->stepDuration / mv->speed);
	if (elapsed <= 0) elapsed = actualStepDuration;

	if (mv->remainTime <= 0) {
		mv->index = (mv->index + 1) % table->length;
		mv->remainTime = actualStepDuration;
	}
	if (elapsed >= mv->remainTime) elapsed = mv->remainTime;

	ratio = (float)elapsed / (float)mv->remainTime;
	loc_sub(&table->table[mv->index], &mv->position, &diff);
	loc_mul(&diff, ratio, &diff);
	loc_addEq(&mv->position, &diff);
	mv->remainTime -= elapsed;

	return &mv->position;
}

void Movement_SetSpeed(Movement *mv, float speed)
{
	if (speed < HEXA_MIN_SPEED) speed = HEXA_MIN_SPEED;
	else if (speed > HEXA_MAX_SPEED) speed = HEXA_MAX_SPEED;
	mv->speed = speed;
}

float Movement_GetSpeed(const Movement *mv)
{
	return mv->speed;
}