/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SPEED 65535
#define WHEEL_DIA 2.5
#define WHEEL_SEP 11
#define ARC 34.5575
#define ENC_FRAC 0.003125
#define RAD_ERR 0.01
#define POS_ERR 0.1
#define UltraLeft 1
#define UltraRight 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
Motor left_motor;
Motor right_motor;
Robot robot;
float pos_integral;
float rad_integral;
int UltraTurn = 0;
int count = 0;
int left = 0;
int right = 0;
int LCheck = 0;
int RCheck = 0;
char obstacle_detected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // left motor
  extern Motor left_motor;
  left_motor.dir = 0;
  left_motor.encoder = 0;
  left_motor.ch1 = TIM_CHANNEL_1;
  left_motor.ch2 = TIM_CHANNEL_2;

  // right_motor
  extern Motor right_motor;
  right_motor.dir = 0;
  right_motor.encoder = 0;
  right_motor.ch1 = TIM_CHANNEL_3;
  right_motor.ch2 = TIM_CHANNEL_4;

  extern Robot robot;
  robot.left = left_motor;
  robot.right = right_motor;
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;

  rad_integral= 0;
  pos_integral = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char sw = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(sw){
	  case 0:
		  if(robot.x < 10)
			  move_x(&robot, 10.0);
		  else
			  sw = 1;
		  break;
	  case 1:
		  if(robot.x >= 0)
			  move_x(&robot, 0.0);
		  else
			  sw = 1;
		  break;
	  }

//	  switch(sw){
//	  	  case 0:
//			  if(robot.x < 36){
//				  robot.left.dir = 1;
//				  robot.right.dir = 1;
//				  pwm_gen(&robot.left, MAX_SPEED>>2);
//				  pwm_gen(&robot.right, MAX_SPEED>>2);
//			  } else{ sw = 1; }
//			  break;
//	  	  case 1:
//			  if(fabs(robot.theta - M_PI) > 0.1){
//				  robot.left.dir = 1;
//				  robot.right.dir = -1;
//				  pwm_gen(&robot.left, MAX_SPEED>>2);
//				  pwm_gen(&robot.right, MAX_SPEED>>2);
//			  } else { sw = 2;}
//			  break;
//	  	  case 2:
//	  		  if(robot.x > 0){
//				  robot.left.dir = 1;
//				  robot.right.dir = 1;
//				  pwm_gen(&robot.left, MAX_SPEED>>2);
//				  pwm_gen(&robot.right, MAX_SPEED>>2);
//	  		  } else {sw = 3;}
//	  		break;
//	  	  case 3:
//	  		  if(robot.theta > 0.1){
//				  robot.left.dir = 1;
//				  robot.right.dir = -1;
//				  pwm_gen(&robot.left, MAX_SPEED>>2);
//				  pwm_gen(&robot.right, MAX_SPEED>>2);
//			  } else{sw = 0;}
//	  		break;
//	  }


//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//	  HAL_Delay(500);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//	  HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim2, TIM3_TI1_GPIO) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim3, TIM3_TI1_GPIO) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2099;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 249;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ultrasonic_B_GPIO_Port, Ultrasonic_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ultrasonic_A_GPIO_Port, Ultrasonic_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Ultrasonic_B_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ultrasonic_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_Echo_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ultrasonic_Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_A_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ultrasonic_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L_MTR_ENC_Pin R_MTR_ENC_Pin */
  GPIO_InitStruct.Pin = L_MTR_ENC_Pin|R_MTR_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * Function to generate PWM signals to move motors. Should never be called directly. Use robot_move or robot_turn
 */
void pwm_gen(Motor* motor, int speed){
	if(speed == 0 || motor->dir == 0){
		__HAL_TIM_SET_COMPARE(&htim3, motor->ch1, 65535);
		__HAL_TIM_SET_COMPARE(&htim3, motor->ch2, 65535);
	}

	if(speed < 0){
		speed = -speed;
		motor->dir = -motor->dir;
	}
	if(speed > MAX_SPEED) speed = MAX_SPEED;
	if(motor->dir > 0){
		__HAL_TIM_SET_COMPARE(&htim3, motor->ch1, speed);
		__HAL_TIM_SET_COMPARE(&htim3, motor->ch2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, motor->ch1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, motor->ch2, speed);
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
	float error =  fabs(robot->x - target);
	if(fabs(robot->theta - theta_target) >= RAD_ERR){
		turn(robot, theta_target);
	} else {
		float K = error * 2500 + pos_integral;
		pos_integral++;
		if(error > POS_ERR){
			robot->left.dir = 1;
			robot->right.dir = 1;
		} else {
			pos_integral = 0;
			robot->left.dir = 0;
			robot->right.dir = 0;
		}
		pwm_gen(&robot->left, K);
		pwm_gen(&robot->right, K);
	}
	return error <= POS_ERR;
}

char move_y(Robot* robot, float target){
	return 0;
}

char turn(Robot* robot, float target){
	target = fmod(target,(2* M_PI));
	float error = robot->theta - target;
	if(fabs(error) > RAD_ERR){
		float K = error * 1000 + rad_integral;
		rad_integral+= 0.5;
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
		rad_integral = 0;
		robot->left.dir = 0;
		robot->right.dir = 0;
	}
	return fabs(error) <= RAD_ERR;

}
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
	if ((UltraTurn == UltraLeft) && (count > 300)){
		printf("Keep turning left\r\n");
	}
	else if ((UltraTurn == UltraRight) && (count > 300)){
		printf("Keep turning right\r\n");
	}
	else {
		printf("Good to go Forward\r\n");
		UltraTurn = 0;
	}
}
void checkStraight(void){
	if (count > 300){
		obstacle_detected = 1;
		if(state==0){
//			obstacle_detected = 1;
			printf("Stop\r\n");
		} else {
			printf("checkDirection\r\n");
			left = 1;
			init_Left();
		}
	} else {
		obstacle_detected = 0;
	}
}
void checkLeft(void){
	left++;
	if (count > 300){
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
	if (count > 300){
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
