#include "command.h"
#include <string.h>

typedef struct {
	const char *name;
	MovementMode mode;
} CmdEntry;

static const CmdEntry kCmdTable[] = {
	{"standby",     MOVEMENT_STANDBY},
	{"forward",     MOVEMENT_FORWARD},
	{"forwardfast", MOVEMENT_FORWARDFAST},
	{"forward_fast",MOVEMENT_FORWARDFAST},
	{"backward",    MOVEMENT_BACKWARD},
	{"turnleft",    MOVEMENT_TURNLEFT},
	{"turn_left",   MOVEMENT_TURNLEFT},
	{"turnright",   MOVEMENT_TURNRIGHT},
	{"turn_right",  MOVEMENT_TURNRIGHT},
	{"shiftleft",   MOVEMENT_SHIFTLEFT},
	{"shift_left",  MOVEMENT_SHIFTLEFT},
	{"shiftright",  MOVEMENT_SHIFTRIGHT},
	{"shift_right", MOVEMENT_SHIFTRIGHT},
	{"climb",       MOVEMENT_CLIMB},
	{"rotatex",     MOVEMENT_ROTATEX},
	{"rotate_x",    MOVEMENT_ROTATEX},
	{"rotatey",     MOVEMENT_ROTATEY},
	{"rotate_y",    MOVEMENT_ROTATEY},
	{"rotatez",     MOVEMENT_ROTATEZ},
	{"rotate_z",    MOVEMENT_ROTATEZ},
	{"twist",       MOVEMENT_TWIST},
	{"beatsway",    MOVEMENT_BEATSWAY},
	{"beat_sway",   MOVEMENT_BEATSWAY},
};

#define CMD_COUNT (sizeof(kCmdTable) / sizeof(kCmdTable[0]))

MovementMode Command_Parse(const char *str)
{
	unsigned int i;

	if (str == 0) return MOVEMENT_TOTAL;

	for (i = 0U; i < CMD_COUNT; i++) {
		if (strcmp(str, kCmdTable[i].name) == 0)
			return kCmdTable[i].mode;
	}
	return MOVEMENT_TOTAL;
}

const char *Command_ToName(MovementMode mode)
{
	unsigned int i;

	for (i = 0U; i < CMD_COUNT; i++) {
		if (kCmdTable[i].mode == mode)
			return kCmdTable[i].name;
	}
	return "unknown";
}