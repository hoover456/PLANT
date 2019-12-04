/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ultrasonic_B_Pin GPIO_PIN_13
#define Ultrasonic_B_GPIO_Port GPIOC
#define Ultrasonic_Trig_Pin GPIO_PIN_0
#define Ultrasonic_Trig_GPIO_Port GPIOA
#define Ultrasonic_Echo_Pin GPIO_PIN_1
#define Ultrasonic_Echo_GPIO_Port GPIOA
#define Ultrasonic_Echo_EXTI_IRQn EXTI0_1_IRQn
#define Ultrasonic_A_Pin GPIO_PIN_4
#define Ultrasonic_A_GPIO_Port GPIOA
#define LM_PWM_1_Pin GPIO_PIN_6
#define LM_PWM_1_GPIO_Port GPIOA
#define LM_PWM_2_Pin GPIO_PIN_7
#define LM_PWM_2_GPIO_Port GPIOA
#define RM_PWM_1_Pin GPIO_PIN_0
#define RM_PWM_1_GPIO_Port GPIOB
#define RM_PWM_2_Pin GPIO_PIN_1
#define RM_PWM_2_GPIO_Port GPIOB
#define L_MTR_ENC_Pin GPIO_PIN_11
#define L_MTR_ENC_GPIO_Port GPIOC
#define L_MTR_ENC_EXTI_IRQn EXTI4_15_IRQn
#define R_MTR_ENC_Pin GPIO_PIN_12
#define R_MTR_ENC_GPIO_Port GPIOC
#define R_MTR_ENC_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

typedef struct{
	signed char dir;
	int encoder;
	int ch1;
	int ch2;
}Motor;

typedef struct{
	Motor left;
	Motor right;
	double x;
	double y;
	double theta;
}Robot;

void pwm_gen(Motor* motor, int speed);
void update_pos(Robot* robot);
char turn(Robot* robot, float target);
char move_x(Robot* robot, float target);
char move_y(Robot* robot, float target);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
