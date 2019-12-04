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
#define ARM_MATH_CM0PLUS
#define ARM_MATH_CM0_FAMILY
#define __FPU_PRESENT 1
#include "arm_math.h"

#include "movement.h"
#include "sensors.h"
#include "math.h"

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
#define LS_FR_Pin GPIO_PIN_0
#define LS_FR_GPIO_Port GPIOC
#define LS_RI_Pin GPIO_PIN_1
#define LS_RI_GPIO_Port GPIOC
#define LS_RE_Pin GPIO_PIN_2
#define LS_RE_GPIO_Port GPIOC
#define LS_LE_Pin GPIO_PIN_3
#define LS_LE_GPIO_Port GPIOC
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
#define CLIFF_Pin GPIO_PIN_9
#define CLIFF_GPIO_Port GPIOA
#define CLIFF_EXTI_IRQn EXTI4_15_IRQn
#define IR_MI_Pin GPIO_PIN_10
#define IR_MI_GPIO_Port GPIOA
#define IR_LE_Pin GPIO_PIN_11
#define IR_LE_GPIO_Port GPIOA
#define IR_RI_Pin GPIO_PIN_12
#define IR_RI_GPIO_Port GPIOA
#define L_MTR_ENC_Pin GPIO_PIN_11
#define L_MTR_ENC_GPIO_Port GPIOC
#define L_MTR_ENC_EXTI_IRQn EXTI4_15_IRQn
#define R_MTR_ENC_Pin GPIO_PIN_12
#define R_MTR_ENC_GPIO_Port GPIOC
#define R_MTR_ENC_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
