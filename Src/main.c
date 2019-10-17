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
//#define ARM_MATH_CM0PLUS
//#define ARM_MATH_CM0_FAMILY
//#define __FPU_PRESENT 1
//#include "arm_math.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_DIAMETER 65.0 // Wheel Diameter in mm
#define WHEEL_CIRCUMFRENCE 204 // Wheel Circumfrence in mm
#define ENCODER_PPR 800 // 800 pulses per one wheel revolution.
#define MOTOR_MAX_SPEED 65535 // uint16_t max
// Directional Defines
#define TURN_LEFT 0
#define TURN_RIGHT 1
#define FORWARD 2
#define REVERSE 3
#define ROBOT_FRONT 0
#define ROBOT_LEFT 1
#define ROBOT_REAR 2
#define ROBOT_RIGHT 3

// printf redirect
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void myTim2Init(void);
void myDMAInit(uint32_t* buffer, uint32_t length);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Enable Left sensor
void init_Left(){
	GPIOA->ODR |= GPIO_PIN_4;
}
void init_Right(){
	GPIOA->ODR &= ~GPIO_PIN_4;
	GPIOA->ODR |= GPIO_PIN_8;
}
void init_Straight(){
	GPIOA->ODR &= ~GPIO_PIN_8;
}
void myTim2Init(void){
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
void myDMAInit(uint32_t* buffer, uint32_t length){
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
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//uint32_t nLoop=0;
//int r_enc_cnt = 0;
int r_dir; // set initial dir of both motors to forward
int l_dir;
int cms = 0;
int seconds = 0;
int r_enc_setpoint = 0;
int r_enc_currentPos = 0;
int l_enc_setpoint = 0;
int l_enc_currentPos = 0;
//arm_pid_instance_f32 R_PID;
//arm_pid_instance_f32 L_PID;
uint32_t nLoop=0;
int sense1 = 0;
int sense2 = 0;
int sense3 = 0;
int sense4 = 0;
int ten_hz_counter=0;
uint16_t ADC_Values[5], buffer[5];
int light_seeking;
int Kp = 100;
//int i = 0;
//int hold = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM22_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
void Left_Motor_PWM_Gen(int speed, int brake);
void Left_Motor_Position_Controller(void);
void Right_Motor_Position_Controller(void);
void Right_Motor_PWM_Gen(int speed, int brake);
int check_light(void);
int light_direction(void);
void turn_until_light(int dir, int compare_value);
void forward_until_light(int compare_value);
void IR_Locate(void);
//void Read_Light_Sensors(void);

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
//  R_PID.Kp = 500;
//  R_PID.Kd = 0;
//  R_PID.Ki = 0;
//  arm_pid_init_f32(&R_PID, 1);
//  L_PID.Kp = 500;
//  L_PID.Kd = 0;
//  L_PID.Ki = 0;
//  arm_pid_init_f32(&L_PID, 1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  MX_TIM3_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  r_enc_setpoint = 0;
  l_enc_setpoint = 0;

  printf("Hello...\r\n");
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//  HAL_ADC_Start_DMA(&hadc, (uint32_t*)buffer, 20);
  myTim2Init();
  TIM6->CR1 |= 1;
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim21);
  HAL_TIM_Encoder_Start(&htim22,TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COMPARE(&htim21, TIM_CHANNEL_1, 0);
  IR_LOCATE();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i = 0;
  int hold = 0;
  int dir = 0;
  // ADC DMA Config
  myDMAInit((uint32_t*) buffer, 5);
  ADC1->CFGR1 |= ADC_CFGR1_DMACFG;
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
  ADC1->CFGR1 |= ADC_CFGR1_CONT;
  ADC1->IER |= ADC_IER_EOSIE;
  ADC1->CR |= ADC_CR_ADEN;
  ADC1->CR |= ADC_CR_ADSTART;
  light_seeking = 1;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	printf("Test\r\n");
	if(light_seeking){
		dir = check_light();
//		printf("dir = %d\r\n", dir);
		if(dir != -1 && dir != ROBOT_FRONT){
//			printf("Turning to face light\r\n");
//			void (*light_direction)(void) = &light_direction;
			turn_until_light(dir==ROBOT_LEFT?TURN_LEFT:TURN_RIGHT, 0);
		} else {
//			printf("Moving to Light\r\n");
			forward_until_light(-1);
		}
	}
	i = TIM22->CNT;
	HAL_Delay(200);
//	printf("i = %d\r\n", i);
	if (hold > i){
		printf("turn left\r\n");
	  hold = i;
	}
	else if (hold < i) {
	  printf("turn right\r\n");
	  hold = i;
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  sConfigOC.Pulse = 9;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim6.Init.Period = 499;
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
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 209;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 0;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 65000;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim22, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim22, TIM3_TI1_GPIO) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */
  if (HAL_TIMEx_RemapConfig(&htim22, TIM22_TI1_GPIO) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE END TIM22_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(soil_meter_power_GPIO_Port, soil_meter_power_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : soil_meter_power_Pin */
  GPIO_InitStruct.Pin = soil_meter_power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(soil_meter_power_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_CLIFF_SENSOR_Pin */
  GPIO_InitStruct.Pin = IR_CLIFF_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_CLIFF_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_FORWARD_Pin IR_LEFT_Pin IR_RIGHT_Pin */
  GPIO_InitStruct.Pin = IR_FORWARD_Pin|IR_LEFT_Pin|IR_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Rotary_Encoder_PushButton_Pin */
  GPIO_InitStruct.Pin = Rotary_Encoder_PushButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART2 and Loop until the end of transmission */
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}

void Right_Encoder_Interrupt_Handler(void){
//	r_enc_cnt++;
	if(r_dir==1)
		r_enc_currentPos++;
	else
		r_enc_currentPos--;
}

void Left_Encoder_Interrupt_Handler(void){
	if(l_dir==1)
		l_enc_currentPos++;
	else
		l_enc_currentPos--;
}



void Right_Motor_PWM_Gen(int speed, int brake){
	// if brake is true, brake and ignore speed setting
	if (brake){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_SPEED);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_SPEED);
	} else {
		// Negative speed corresponds to opposite direction
		if(speed < 0){
			r_dir = 0;
			speed = -speed;
		} else
			r_dir = 1;
		// can't go faster than the PWM generator can handle (100% duty cycle)
		if(speed > MOTOR_MAX_SPEED)
			speed = MOTOR_MAX_SPEED;

		// r_dir == 1 corresponds to forward turn (PWM1 on, PWM2 off)
		if(r_dir==1){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
		}
	}
}

void Left_Motor_PWM_Gen(int speed, int brake){
	// if brake is true, brake and ignore speed setting
	if (brake){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_SPEED);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_SPEED);
	} else {
		// Negative speed corresponds to opposite direction
		if(speed < 0){
			l_dir = 0;
			speed = -speed;
		} else
			l_dir = 1;
		// can't go faster than the PWM generator can handle (100% duty cycle)
		if(speed > MOTOR_MAX_SPEED)
			speed = MOTOR_MAX_SPEED;

		// l_dir == 1 corresponds to forward turn (PWM1 on, PWM2 off)
		if(l_dir==1){
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
		}
	}
}

void Right_Motor_Position_Controller(void){
//	if(r_enc_setpoint < 0){
//		r_enc_setpoint = -r_enc_setpoint;
//		r_dir = 0;
//	} else {
//		r_dir = 1;
//	}
  	int r_pid_error = r_enc_setpoint - r_enc_currentPos; // Compute error
//  	int duty = (int)arm_pid_f32(&R_PID, r_pid_error); // Compute PID controller output
  	if(fabs(r_pid_error)  < 5)
  		Right_Motor_PWM_Gen(0, 1); // Switch to brake mode if within 0.5% off setpoint
  	else{
  		int duty = Kp * r_pid_error;
  		Right_Motor_PWM_Gen(duty, 0);
  	}

}

void Left_Motor_Position_Controller(void){
//	if(l_enc_setpoint < 0){
//			l_enc_setpoint = -l_enc_setpoint;
//			l_dir = 0;
//	} else {
//			l_dir = 1;
//	}
  	int l_pid_error = l_enc_setpoint - l_enc_currentPos; // Compute error
//  	int duty = (int)arm_pid_f32(&L_PID, l_pid_error); // Compute PID controller output
  	if(fabs(l_pid_error)  < 5)
  		Left_Motor_PWM_Gen(0, 1); // Switch to brake mode if within 0.5% off setpoint
  	else{
  		int duty = Kp * l_pid_error;
  		Left_Motor_PWM_Gen(duty, 0);
  	}

}

void move_robot(short dir, int speed){
	if(speed <= 0){
		Right_Motor_PWM_Gen(0,1);
		Left_Motor_PWM_Gen(0,1);
	} else {
		if(dir == TURN_LEFT){
			Right_Motor_PWM_Gen(speed,0);
			Left_Motor_PWM_Gen(-speed,0);
	} else if(dir == TURN_RIGHT){
			Right_Motor_PWM_Gen(-speed,0);
			Left_Motor_PWM_Gen(speed,0);
	} else if(dir == FORWARD){
		Right_Motor_PWM_Gen(speed,0);
		Left_Motor_PWM_Gen(speed,0);
	} else if(dir == REVERSE){
		Right_Motor_PWM_Gen(-speed,0);
		Left_Motor_PWM_Gen(-speed,0);
	}
	}
}

void TEN_KHZ_TIM_Interrupt_Handler(void){
//	ten_hz_counter++;
//	if(ten_hz_counter >= 10){
////		Right_Motor_Position_Controller();
////		Left_Motor_Position_Controller();
////		Read_Light_Sensors();
//		ten_hz_counter = 0;
//	}
}

void TIM22_Interrupt_Handler(void){
//	HAL_Delay(200);

}
// Debugging function - prints the right motor encoder reading when the blue button is pushed
void Print_Encoder_Reading(void)
{
	printf("Rotations: %d          \r", r_enc_currentPos);
	fflush(stdout);
//	r_enc_currentPos = 0;
}

void ADC_ConvCpltCallback(void)
{
	for(int i = 0; i < 5; i++){
		ADC_Values[i] = buffer[i];
	}
//	printf("%u, %u, %u, %u\r\n", ADC_Values[0], ADC_Values[1],ADC_Values[2], ADC_Values[3]);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
	printf("ERROR\r\n");
}

/**
 *  @brief This function computes the optimal light direction if 1 direction is 5% brighter than the other 3, otherwise returns -1
 */
int check_light(void){
//	return 0;
	int front = (int)ADC_Values[0];
	int left = (int)ADC_Values[1];
	int rear = (int)ADC_Values[2];
	int right = (int)ADC_Values[3];

	int sum = front+left+rear+right;
	int avg =  (float)sum / 4;

	int max = fmax(fmax(front, left), fmax(rear, right));

	float deviation = (float) (max-avg) / avg;
	if(deviation < 0.2)
		return -1;
	return light_direction();
}

/**
 * @brief Turn the robot until eval_fun returns a value equal to compare_value
 * @param dir: direction to turn, 0 = LEFT, 1 = RIGHT
 */
void turn_until_light(int dir, int compare_value){
//	move_robot(0,0);
//	int setpoint = 800;
	if(dir){
		move_robot(TURN_LEFT, 30000);
	} else {
		move_robot(TURN_RIGHT, 30000);
	}
}

/**
 * @brief This function moves the robot forward until eval_fun returns a value equal to compare_value
 */
void forward_until_light(int compare_value){
//	r_enc_currentPos = 0;
//	l_enc_currentPos = 0;
//	int setpoint = 800;
	if(check_light() == compare_value){
		move_robot(0, 0);
	} else {
		move_robot(FORWARD, 30000);
	}
}

int light_direction(void){
	uint16_t max = 0;
	int maxIndex = 0;
	for(int i = 0; i <= 3; i++){
		if(ADC_Values[i] > max){
			max = ADC_Values[i];
			maxIndex = i;
		}
	}
	return maxIndex;
}

void RTC_Init(void){
	LCD_RTC_Clock_Enable();
	RCC->CSR |= RCC_CSR_RTCEN;

	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	RTC->ISR |= RTC_ISR_INIT;
	while((RTC->ISR & RTC_ISR_INITF) == 0);
	RTC->CR &= ~RTC_CR_FMT;
	RTC->PRER |= (2<<7 - 1) << 16;
	RTC->PRER |= (2<<8 - 1);

	//call phills function to get time then write here.

	RTC->TR = 0<<22 | 1<<20 | 1<<16 | 3<<12| 2<<8; //time
	RTC->DR = 1<<20 | 6<<16 | 0<<12 | 5<<8 | 2<<4 | 7; //date, not used but initialized for reasons.

	RTC->ISR &= ~RTC_ISR_INIT;
	RTC->WPR = 0xFF;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	printf("ERRORRRRRR\r\n");
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
