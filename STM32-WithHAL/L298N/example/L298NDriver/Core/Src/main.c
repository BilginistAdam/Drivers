/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "L298N.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t u8_mode = 0;
uint8_t u8_dirChange = 0;

//L298N Motor
L298N_MotorHandler_t hmotor1;
L298N_MotorHandler_t hmotor2;

//Motor Speed
uint32_t u32_motor1_speed = L298N_MOTOR_SPEED_STOP;
uint32_t u32_motor2_speed = L298N_MOTOR_SPEED_STOP;

//Motor Direction
int8_t u8_motor1_direction = L298N_MOTOR_DIRECTION_STOP;
int8_t u8_motor2_direction = L298N_MOTOR_DIRECTION_STOP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void L298N_Init(void);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  L298N_Init();

  u32_motor1_speed = L298N_getMotorSpeed(&hmotor1);
  u32_motor2_speed = L298N_getMotorSpeed(&hmotor2);

  u8_motor1_direction = L298N_getMotorDirection(&hmotor1);
  u8_motor2_direction = L298N_getMotorDirection(&hmotor2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	//Select Mode
	if(u32_motor1_speed == L298N_MOTOR_SPEED_MAX){
		u8_mode = 0;
	}else if(u32_motor1_speed == L298N_MOTOR_SPEED_MIN){
		u8_mode = 1;
	}

	//Change Motor 1 Direction Condition
	if(u32_motor1_speed == L298N_MOTOR_SPEED_MIN){
		//Select Motor 1 Direction
		if(u8_motor1_direction == L298N_MOTOR_DIRECTION_STOP){
			u8_motor1_direction = L298N_MOTOR_DIRECTION_FORWARD;
			u8_dirChange = 1;
		}else if(u8_motor1_direction == L298N_MOTOR_DIRECTION_FORWARD){
			u8_motor1_direction = L298N_MOTOR_DIRECTION_BACKWARD;
			u8_dirChange = 1;
		}else if(u8_motor1_direction == L298N_MOTOR_DIRECTION_BACKWARD){
			u8_motor1_direction = L298N_MOTOR_DIRECTION_FORWARD;
			u8_dirChange = 1;
		}else{
			u8_motor1_direction = L298N_MOTOR_DIRECTION_STOP;
			u8_dirChange = 1;
		}

		//Change Motor 1 Direction
		if (u8_dirChange){
			L298N_setMotorDirection(&hmotor1, u8_motor1_direction);
			u8_dirChange = 0;
		}
	}

	//Change Motor 2 Direction Condition
	if(u32_motor2_speed == L298N_MOTOR_SPEED_MIN){
		//Select Motor 2 Direction
		if(u8_motor2_direction == L298N_MOTOR_DIRECTION_STOP){
			u8_motor2_direction = L298N_MOTOR_DIRECTION_FORWARD;
			u8_dirChange = 1;
		}else if(u8_motor2_direction == L298N_MOTOR_DIRECTION_FORWARD){
			u8_motor2_direction = L298N_MOTOR_DIRECTION_BACKWARD;
			u8_dirChange = 1;
		}else if(u8_motor2_direction == L298N_MOTOR_DIRECTION_BACKWARD){
			u8_motor2_direction = L298N_MOTOR_DIRECTION_FORWARD;
			u8_dirChange = 1;
		}else{
			u8_motor2_direction = L298N_MOTOR_DIRECTION_STOP;
			u8_dirChange = 1;
		}

		//Change Motor 2 Direction
		if (u8_dirChange){
			L298N_setMotorDirection(&hmotor2, u8_motor2_direction);
			u8_dirChange = 0;
		}
	}

	//Motor Speed Inc or Dec
	if (u8_mode){
		L298N_setMotorSpeed(&hmotor1, u32_motor1_speed);
		L298N_setMotorSpeed(&hmotor2, u32_motor2_speed);

		u32_motor1_speed++;
		u32_motor2_speed++;
	}else{
		L298N_setMotorSpeed(&hmotor1, u32_motor1_speed);
		L298N_setMotorSpeed(&hmotor2, u32_motor2_speed);

		u32_motor1_speed--;
		u32_motor2_speed--;
	}
	HAL_Delay(250);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|L298N_1_Pin|L298N_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L298N_3_Pin|L298N_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin L298N_1_Pin L298N_2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|L298N_1_Pin|L298N_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L298N_3_Pin L298N_4_Pin */
  GPIO_InitStruct.Pin = L298N_3_Pin|L298N_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void L298N_Init(void){
	L298N_MotorConfig_t L298N_MotorConfig[2] = {0};

	//Configure Motor 1
	L298N_MotorConfig[0].tim = &htim2;
	L298N_MotorConfig[0].channel = TIM_CHANNEL_1;
	L298N_MotorConfig[0].port[0] = L298N_1_GPIO_Port;
	L298N_MotorConfig[0].port[1] = L298N_2_GPIO_Port;
	L298N_MotorConfig[0].pin[0] = L298N_1_Pin;
	L298N_MotorConfig[0].pin[1] = L298N_2_Pin;

	//Configure Motor 2
	L298N_MotorConfig[1].tim = &htim2;
	L298N_MotorConfig[1].channel = TIM_CHANNEL_2;
	L298N_MotorConfig[1].port[0] = L298N_3_GPIO_Port;
	L298N_MotorConfig[1].port[1] = L298N_4_GPIO_Port;
	L298N_MotorConfig[1].pin[0] = L298N_3_Pin;
	L298N_MotorConfig[1].pin[1] = L298N_4_Pin;

	//Init Motor 1
	if(L298N_OK != L298N_MotorInit(&hmotor1,&L298N_MotorConfig[0])){
		Error_Handler();
	}

	//Init Motor 2
	if(L298N_OK != L298N_MotorInit(&hmotor2, &L298N_MotorConfig[1])){
		Error_Handler();
	}




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
