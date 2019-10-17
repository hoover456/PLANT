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
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void init_Left(void);
void init_Right(void);
void init_Straight(void);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Right_Encoder_Interrupt_Handler(void);
void Left_Encoder_Interrupt_Handler(void);
void TEN_KHZ_TIM_Interrupt_Handler(void);
void Print_Encoder_Reading(void);
void TIM22_Interrupt_Handler(void);
void Read_Light_Sensors(void);
void DMA1_Channel1_Handler(void);
void ADC_ConvCpltCallback(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define soil_meter_power_Pin GPIO_PIN_5
#define soil_meter_power_GPIO_Port GPIOC
#define IR_CLIFF_SENSOR_Pin GPIO_PIN_9
#define IR_CLIFF_SENSOR_GPIO_Port GPIOA
#define IR_CLIFF_SENSOR_EXTI_IRQn EXTI4_15_IRQn
#define IR_FORWARD_Pin GPIO_PIN_10
#define IR_FORWARD_GPIO_Port GPIOA
#define IR_LEFT_Pin GPIO_PIN_11
#define IR_LEFT_GPIO_Port GPIOA
#define IR_RIGHT_Pin GPIO_PIN_12
#define IR_RIGHT_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
