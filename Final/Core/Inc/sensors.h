/*
 * sensors.h
 *
 *  Created on: Dec 3, 2019
 *      Author: Andrew Hoover and Phill Spiritoso
 */
#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"

// Light Sensing
void DMAInit(uint32_t* buffer, uint32_t length);
int check_light_direction(void);
int check_light_variance(void);

// Ultrasonics
void countUp(void);
void TIM6_UltraSonic_Handler(void);
void checkTurn(void);
void checkStraight(void);
void checkLeft(void);
void checkRight(void);
void determineDir(void);
void init_Left();
void init_Right();
void init_Straight();
#endif
