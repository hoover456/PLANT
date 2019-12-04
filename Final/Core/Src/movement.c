/*
 * movement.c
 *
 *  Created on: Dec 3, 2019
 *      Author: Andrew Hoover
 */



#include "movement.h"

/**
 * Function to generate PWM signals to move motors. Should never be called directly, rather should be called by more general movement functions
 */
void pwm_gen(Motor* motor, int speed){

	// if dir is 0 or speed is 0 set to brake mode (both channels full duty cycle)
	if(speed == 0 || motor->dir == 0){
		__HAL_TIM_SET_COMPARE(motor->timer, motor->ch1, MAX_SPEED);
		__HAL_TIM_SET_COMPARE(motor->timer, motor->ch2, MAX_SPEED);
		return;
	}

	// Negative speed -> reverse direction and positive speed
	if(speed < 0){
		speed = -speed;
		motor->dir = -motor->dir;
	}
	if(speed > MAX_SPEED) speed = MAX_SPEED;

	// Set which channel gets PWM input based on dir
	if(motor->dir > 0){
		__HAL_TIM_SET_COMPARE(motor->timer, motor->ch1, speed);
		__HAL_TIM_SET_COMPARE(motor->timer, motor->ch2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(motor->timer, motor->ch1, 0);
		__HAL_TIM_SET_COMPARE(motor->timer, motor->ch2, speed);
	}

}

void update_pos(Robot* robot){
	float dLeft = robot->left.encoder * ENC_FRAC;
	float dRight = robot->right.encoder * ENC_FRAC;
	robot->left.encoder = 0;
	robot->right.encoder = 0;
	float phi = (dRight - dLeft) / WHEEL_SEP;
	float phi_2 = phi / 2;
	float dCenter = (dLeft + dRight) / 2;
	robot->x += dCenter * cos(robot->theta + phi_2);
	robot->y += dCenter * sin(robot->theta + phi_2);
	robot->theta += phi;
	robot->theta = fmod(robot->theta,(2*M_PI));
}

char move_x(Robot* robot, float target){
	double theta_target = target > 0 ? 0 : M_PI;
	float pos_error = target - robot->x;
	float rad_error = theta_target - robot->theta;

	float32_t left_pid = (arm_pid_f32(&robot->left.pos_pid, pos_error)); //+ arm_pid_f32(&robot->left.rad_pid, rad_error));
//	float32_t right_pid = (arm_pid_f32(&robot->left.pos_pid, pos_error) + arm_pid_f32(&robot->left.rad_pid, rad_error));
	int left_speed = (int) 	(arm_pid_f32(&robot->left.pos_pid, pos_error) + arm_pid_f32(&robot->left.rad_pid, rad_error));
	int right_speed = (int) 	(arm_pid_f32(&robot->right.pos_pid, pos_error) + arm_pid_f32(&robot->right.rad_pid, rad_error));

	robot->left.dir = 1;
	robot->right.dir = 1;
	pwm_gen(&robot->left, left_speed);
	pwm_gen(&robot->right, right_speed);

	return 0;
}

char move_y(Robot* robot, float target){
	return 0;
}

char turn(Robot* robot, float target){
	target = fmod(target,(2* M_PI));
	float error = robot->theta - target;
	if(fabs(error) > RAD_ERR){
		float K = error * 1000;
//		rad_integral+= 0.5;
		if(error > 0){
			robot->left.dir = 1;
			robot->right.dir = -1;
		} else {
			robot->left.dir = -1;
			robot->right.dir = 1;
		}
		pwm_gen(&robot->left, K);
		pwm_gen(&robot->right, K);
	} else {
//		rad_integral = 0;
		robot->left.dir = 0;
		robot->right.dir = 0;
	}
	return fabs(error) <= RAD_ERR;
}


/**
 * @Brief move the robot forward until compare returns compare value
 * Returns 1 when compare == compare_value, 0 otherwise (call in a loop).
 */
int move_until(Robot* robot, int (*compare)(), int compare_value){
	if(compare() != compare_value){
		robot->left.dir = FORWARD;
		robot->right.dir = FORWARD;
		pwm_gen(&robot->left, MAX_SPEED/2);
		pwm_gen(&robot->right, MAX_SPEED/2);
		return 0;
	} else {
		robot->left.dir = BRAKE;
		robot->right.dir = BRAKE;
		pwm_gen(&robot->left, 0);
		pwm_gen(&robot->right, 0);
		return 1;
	}
}

/**
 * @Brief turn the robot until compare returns 0
 * compare < 0 turns counterclockwise, >0 turns clockwise
 * Returns 1 when compare == compare_value, 0 otherwise (call in a loop).
 */
int turn_until(Robot* robot, int (*compare)()){
	int comp = compare();
	if(comp == 0){
		robot->left.dir = BRAKE;
		robot->right.dir = BRAKE;
		return 1;
	} else if(robot->left.dir == robot->right.dir){
		if(comp > 0){
			robot->left.dir = FORWARD;
			robot->right.dir = REVERSE;
			pwm_gen(&robot->left, MAX_SPEED/2);
			pwm_gen(&robot->right, MAX_SPEED/2);
			return 0;
		} else{
			robot->left.dir = REVERSE;
			robot->right.dir = FORWARD;
			pwm_gen(&robot->left, MAX_SPEED/2);
			pwm_gen(&robot->right, MAX_SPEED/2);
		}
	}
	return 0;
}

void stop(Robot* robot){
	robot->left.dir = BRAKE;
	robot->right.dir = BRAKE;
	pwm_gen(&robot->left, 0);
	pwm_gen(&robot->right, 0);
}
