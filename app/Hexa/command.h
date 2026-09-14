#ifndef __HEXA_COMMAND_H
#define __HEXA_COMMAND_H

#include "movement.h"

/* 字符串 → 枚举映射 */
MovementMode Command_Parse(const char *str);

/* 枚举 → 字符串 */
const char *Command_ToName(MovementMode mode);

#endif