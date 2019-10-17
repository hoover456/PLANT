/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 void checkTurn(void);
 void checkStraight(void);
 void checkLeft(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int count = 0;
int left = 0;
int right = 0;
int LCheck = 0;
int RCheck = 0;
int move = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void IR_LOCATE(void){
	while(1){
	int pins = ((GPIOA->IDR>>10) & 7);

	if(pins == 0){
		printf("0\r\n");
	}
	if(pins == 1){
		printf("1\r\n");
	}
	if(pins == 2){
		printf("2\r\n");
	}
	if(pins == 3){
		printf("3\r\n");
	}
	if(pins == 3){
		printf("3\r\n");
	}
	if(pins == 4){
		printf("4\r\n");
	}
	if(pins == 5){
		printf("5\r\n");
	}
	if(pins == 6){
		printf("6\r\n");
	}
	if(pins == 7){
		printf("7\r\n");
	}
	}
}


void checkTurn(void){
	if ((move == 1) && (count > 600)){
		printf("Keep turning left\r\n");
	}
	else if ((move == 2) && (count > 600)){
		printf("Keep turning right\r\n");
	}
	else {
		printf("Good to go Forward\r\n");
		move = 0;
	}
}
void checkStraight(void){
	if (count > 600){
		printf("checkDirection\r\n");
		left = 1;
		init_Left();
	}

}
void checkLeft(void){
	left++;
	if (count > 600){
		printf("Left Not good\r\n");
	}
	if (left > 3){
		left = 0;
		right = 1;
		init_Right();
	}
	LCheck += count;
}
void checkRight(void){
	right++;
	if (count > 600){
		printf("Right Not Good\r\n");

	}
	if (right > 3){
		left = 0;
		right = 0;
	}
	RCheck += count;
}
void determineDir(void){
	if (LCheck < RCheck){
		printf("Turn Left\r\n");
		move = 1;
	}
	else {
		printf("Turn Right\r\n");
		move = 2;
	}
	init_Straight();
	LCheck = 0;
	RCheck = 0;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim21;
extern TIM_HandleTypeDef htim22;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
	  printf("HARD FAULT\r\n");
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
	if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_1)){
		if (((GPIOA->IDR) > 1) & 1){
			count++;
		}
	}
  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
	// PIN 7
	if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_6)){
		printf("Push Button\r\n");
	}

	//PIN 9
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_9)){
		printf("FUCKING STOP\r\n");
		//do something here to stop motors
	}

	// PIN 11
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_11)){
		Left_Encoder_Interrupt_Handler();
	}
	// PIN 12
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_12))
	{
		Right_Encoder_Interrupt_Handler();
	}

	// PIN 13

	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
		Print_Encoder_Reading();
	}
  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles ADC, COMP1 and COMP2 interrupts (COMP interrupts through EXTI lines 21 and 22).
  */
void ADC1_COMP_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_COMP_IRQn 0 */
	if(ADC1->ISR & ADC_ISR_EOS){
		ADC_ConvCpltCallback();
	}
  /* USER CODE END ADC1_COMP_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_COMP_IRQn 1 */

  /* USER CODE END ADC1_COMP_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1/DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	if (move > 0){
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
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM21 global interrupt.
  */
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */
	TEN_KHZ_TIM_Interrupt_Handler();
  /* USER CODE END TIM21_IRQn 0 */
  HAL_TIM_IRQHandler(&htim21);
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}

/**
  * @brief This function handles TIM22 global interrupt.
  */
void TIM22_IRQHandler(void)
{
  /* USER CODE BEGIN TIM22_IRQn 0 */
//  TIM22_Interrupt_Handler();
  /* USER CODE END TIM22_IRQn 0 */
  HAL_TIM_IRQHandler(&htim22);
  /* USER CODE BEGIN TIM22_IRQn 1 */

  /* USER CODE END TIM22_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
