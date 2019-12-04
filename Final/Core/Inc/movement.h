/*
 * movement.h
 *
 *  Created on: Dec 3, 2019
 *      Author: Andrew Hoover
 */

// Prevent recursive inclusion
#ifndef __MOVEMENT_H
#define __MOVEMENT_H


#include "main.h"

// Robot movement defines
#define MAX_SPEED 65535
#define WHEEL_DIA 2.5
#define WHEEL_SEP 11
#define ARC 34.5575
#define ENC_FRAC 0.003125
#define RAD_ERR 0.01
#define POS_ERR 0.1
#define FORWARD 1
#define REVERSE -1
#define BRAKE 0

// Robot directional defines
#define FRONT 0
#define LEFT 1
#define REAR 2
#define RIGHT 3


// Motor struct
typedef struct{
	signed char dir;
	int encoder;
	TIM_HandleTypeDef* timer;
	int ch1;
	int ch2;
	arm_pid_instance_f32 rad_pid;
	arm_pid_instance_f32 pos_pid;
}Motor;

// 2-wheel drive robot struct
typedef struct{
	Motor left;
	Motor right;
	char cliff; // 1 = cliff detected. 0 = on ground
	double x;
	double y;
	double theta;
	char stop;
	char obstacle[3]; // 0:Left, 1:Center, 2:Right
}Robot;

// Function declarations
void pwm_gen(Motor* motor, int speed);
void update_pos(Robot* robot);
char turn(Robot* robot, float target);
char move_x(Robot* robot, float target);
char move_y(Robot* robot, float target);
int move_until(Robot* robot, int (*compare)(), int compare_value);
int turn_until(Robot* robot, int (*compare)());
void stop(Robot* robot);
int IR_dock(void);
int IR_align(void);
#endif

