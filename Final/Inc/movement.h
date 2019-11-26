/**
******************************************************************************
  * @file           : movement.h
  * @brief          : Header for movement.c file.
  *                   This file contains variable and type defines related to robot movement
******************************************************************************
**/

#ifndef __MOVEMENT_H
#define __MOVEMENT_H

// Struct representing Robot position in 2D space
typedef struct {
	int x; // X-coordinate of robot in world
	int y; // Y-coordinate of robot in world
	double theta; // Current direction of robot (0-359 degrees)
}Robot;

// Struct representing one motor
typedef struct {
	char dir; //Direction of rotation: 1 = forward, 0=brake, -1 = reverse
	short encoder; // Current encoder reading
}Motor;

void encoderUpdate(Motor motor);

void robotPosUpdate(Robot robot);
