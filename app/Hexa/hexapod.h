#ifndef __HEXA_HEXAPOD_H
#define __HEXA_HEXAPOD_H

#include "leg.h"
#include "movement.h"

typedef struct {
	HexaLeg     legs[6];
	Movement    movement;
	MovementMode mode;
} Hexapod;

void Hexapod_Init(Hexapod *hp);
void Hexapod_ProcessMovement(Hexapod *hp, MovementMode mode, int elapsed);
void Hexapod_SetSpeed(Hexapod *hp, float speed);

/* 校准接口 */
void Hexapod_CalibrationLoad(Hexapod *hp);
void Hexapod_CalibrationSave(const Hexapod *hp);
void Hexapod_CalibrationSet(Hexapod *hp, int leg, int part, int offset);
void Hexapod_CalibrationGet(const Hexapod *hp, int leg, int part, int *offset);

#endif