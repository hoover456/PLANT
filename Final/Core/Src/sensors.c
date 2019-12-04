/*
 * sensors.c
 *
 *  Created on: Dec 3, 2019
 *      Author: Andrew Hoover and Phill Spiritoso
 */
#include "sensors.h"
#include "main.h"

#define UltraLeft 1
#define UltraRight 2

int UltraTurn = 0;
int count = 0;
int left = 0;
int right = 0;
int LCheck = 0;
int RCheck = 0;
char obstacle_detected = 0;
extern Robot robot;


/**
 * Light Sensors
 * Author: Andrew Hoover
 */
void DMAInit(uint32_t* buffer, uint32_t length){
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM;

	DMA1_Channel1->CCR &= ~DMA_CCR_PL;
	DMA1_Channel1->CCR |= DMA_CCR_PL_1;

	DMA1_Channel1->CCR &= ~DMA_CCR_PSIZE;
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0; // 16bits

	DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE;
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0; // 16bits

	DMA1_Channel1->CCR &= ~DMA_CCR_PINC;

	DMA1_Channel1->CCR |= DMA_CCR_MINC;

	DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;

	DMA1_Channel1->CCR &= ~DMA_CCR_DIR;

	DMA1_Channel1->CNDTR = length;

	DMA1_Channel1->CPAR = (uint32_t) &(ADC1->DR);

	DMA1_Channel1->CMAR = (uint32_t) buffer;

	DMA1_CSELR->CSELR &= ~DMA_CSELR_C1S;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	ADC1->CFGR1 |= ADC_CFGR1_DMACFG;
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	ADC1->IER |= ADC_IER_EOSIE;
	ADC1->CR |= ADC_CR_ADEN;
	ADC1->CR |= ADC_CR_ADSTART;
}

/**
 *  @brief returns True if 1 direction of light is 25% brighter than the other 3, otherwise returns 0
 */
int check_light_variance(void){
	extern uint16_t buffer[];
	int front = (int)buffer[0];
	int left = (int)buffer[1];
	int rear = (int)buffer[2];
	int right = (int)buffer[3];

	int sum = front+left+rear+right;
	int avg =  (float)sum / 4;

	//4-way max
	int max = fmax(fmax(front, left), fmax(right, rear));


	float deviation = (float)(max - avg) / avg;
	if(deviation < 0.1)
		return 0;
	return 1;
}

/**
 * @brief returns -1 if left is the brightest dir, 1 if right or rear is the brightest, 0 otherwise
 */
int check_light_direction(void){
	extern uint16_t buffer[];
	int max = -1;
	int maxIndex = 0;

	for(int i = 0; i <= 3; i++){
		if(buffer[i] > max){
			max = buffer[i];
			maxIndex = i;
		}
	}

	if(maxIndex == FRONT) return 0;
	// If the left and right are too similar, move forward a bit
	if(maxIndex != REAR && abs(buffer[LEFT] - buffer[RIGHT]) < 200) return 0;
	return maxIndex == LEFT ? -1 : 1;
}

int IR_align(void){
	int IR_Pins = ((GPIOA->IDR>>10) & 7);
             	if(IR_Pins == 1 || IR_Pins == 5) return -1;
	if(IR_Pins == 2 || IR_Pins == 6) return 1;
	return 0;
}

int IR_dock(void){
	int IR_Pins = ((GPIOA->IDR>>10) & 7);
	return IR_Pins != 0;
}


/***
 * Ultrasonic Sensors
 * 	Author: Phill Spiritoso
 */
void TIM6_UltraSonic_Handler(void){
	if (UltraTurn > 0){
		checkTurn();
	}
	else if ((left == right) && (LCheck == RCheck)){
		checkStraight();
	}
	else if(left > 0){
		checkLeft();
	}
	else if(right > 0){
		checkRight();
	}
	else if ((RCheck > 0) && (LCheck > 0)){
		determineDir();
	}
	count = 0;
}
void checkTurn(void){
	if ((UltraTurn == UltraLeft) && (count > 100)){
		printf("Keep turning left\r\n");
	}
	else if ((UltraTurn == UltraRight) && (count > 100)){
		printf("Keep turning right\r\n");
	}
	else {
		printf("Good to go Forward\r\n");
		UltraTurn = 0;
	}
}
void checkStraight(void){
	if (count > 100){
		robot.obstacle[1] = 1;
		printf("checkDirection\r\n");
		left = 1;
		init_Left();
	} else {
		robot.obstacle[1] = 0;
	}
}
void checkLeft(void){
	left++;
	if (count > 100){
		robot.obstacle[0] = 1;
	} else robot.obstacle[0] = 0;
	if (left > 3){
		left = 0;
		right = 1;
		init_Right();
	}
	LCheck += count;
}
void checkRight(void){
	right++;
	if (count > 100){
		robot.obstacle[2] = 1;
	} else robot.obstacle[2] = 0;
	if (right > 3){
		left = 0;
		right = 0;
	}
	RCheck += count;
}
void determineDir(void){
	if (LCheck < RCheck){
		printf("Turn Left\r\n");
		UltraTurn = UltraLeft;
	}
	else {
		printf("Turn Right\r\n");
		UltraTurn = UltraRight;
	}
	init_Straight();
	LCheck = 0;
	RCheck = 0;
}
void init_Left(){
	GPIOA->ODR |= 1 << 4;
	GPIOC->ODR &= ~(1 << 13);
}
void init_Right() {
	GPIOA->ODR &= ~(1 << 4);
	GPIOC->ODR |= (1 << 13);
}
void init_Straight() {
	GPIOA->ODR &= ~(1 << 4);
	GPIOC->ODR &= ~(1 << 13);
}

void TIM2_conf(void){
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	GPIOA->MODER |= 2;
	GPIOA->AFR[0] |= 2;
	GPIOA->OSPEEDR |= 2;
	TIM2->PSC = 20;
	TIM2->ARR = 9;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM2->CCR1 = 9;
	TIM2->CCER |= 1;
	TIM2->CR1 |= 1;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
}

int obstacle_right(){
		return robot.obstacle[2];
}

int obstacle_left(){
		return robot.obstacle[0];
}

int ret_one(){
	return 1;
}
