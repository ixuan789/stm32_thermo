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
#include "OLED.h"
#include <math.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
const float Rp=10000.0; //10K
const float T2 = (273.15+25.0);;//T2
const float Bx = 3950.0;//B
const float Ka = 273.15;
char DisplayCache1[16];
char DisplayCache2[16];
char DisplayCache3[16];
char DisplayCache4[16];
int frame;
int Emoji=1;
int UART=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
float GetTemp(float Rt)
{
	//eg. R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)),
	float temp;
	temp = Rt/Rp;
	temp = log(temp);//ln(Rt/Rp)
	temp/=Bx;//ln(Rt/Rp)/B
	temp+=(1/T2);
	temp = 1/(temp);
	temp-=Ka;
	return temp;
}


void RunMode(int mode)
{
	char TxCache[16];
	if(mode == 1)//ADC
	{
		OLED_ShowChinese(0,0,2,16);
		OLED_ShowChinese(16,0,3,16);
		OLED_ShowChinese(32,0,4,16);
		OLED_ShowString(30,20,DisplayCache1,24);
		OLED_ShowString(96,27,"ADC",16);
		OLED_ShowString(0,52,"UART OFF",12);
		if(UART)
		{
			OLED_ShowString(0,52,"UART  ON",12);
			sprintf(TxCache,"%s\r\n",DisplayCache1);
			HAL_UART_Transmit(&huart1,TxCache,6,100);
		}
	}
	if(mode == 2)//Volt
	{
		OLED_ShowChinese(0,0,5,16);
		OLED_ShowChinese(16,0,6,16);
		OLED_ShowString(30,20,DisplayCache2,24);
		OLED_ShowString(96,27,"V",16);
		OLED_ShowString(0,52,"UART OFF",12);
		if(UART)
		{
			OLED_ShowString(0,52,"UART  ON",12);
			sprintf(TxCache,"%s\r\n",DisplayCache2);
			HAL_UART_Transmit(&huart1,TxCache,7,100);
		}
	}

	if(mode == 3)//Register
	{
		OLED_ShowChinese(0,0,7,16);
		OLED_ShowChinese(16,0,4,16);
		OLED_ShowString(30,20,DisplayCache3,24);
		OLED_ShowChinese(96,27,8,16);
		OLED_ShowString(0,52,"UART OFF",12);
		if(UART)
		{
			OLED_ShowString(0,52,"UART  ON",12);
			sprintf(TxCache,"%s\r\n",DisplayCache3);
			HAL_UART_Transmit(&huart1,TxCache,6,100);
		}

	}
	if(mode == 4)//Temp
	{
		OLED_ShowChinese(0,0,0,16);
		OLED_ShowChinese(16,0,1,16);
		OLED_ShowString(30,20,DisplayCache4,24);
		OLED_ShowChinese(96,27,9,16);
		OLED_ShowChar(104,27,'C',16);
		OLED_ShowString(0,52,"UART OFF",12);
		if(UART)
		{
			OLED_ShowString(0,52,"UART  ON",12);
			sprintf(TxCache,"%s\r\n",DisplayCache4);
			HAL_UART_Transmit(&huart1,TxCache,7,100);
		}
	}
}

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);

	OLED_Init();
	OLED_DisplayTurn(1);
	OLED_ShowString(0,0,"Calibrating...",12);
	OLED_Refresh();
	int ADValue=0;
	int ADSwitch=0;
	int Mode=0;
	int preMode=0;
	float fADValue=0;
	int i=0;
	int y=0;
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(1000);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	HAL_NVIC_SetPriority(SysTick_IRQn,0,0U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		TIM2->CCR3 = (int)(-1.40877*(fADValue*10000/(3.3-fADValue)+140877));
		TIM3->CCR2 = (int)(-1.40877*(fADValue*10000/(3.3-fADValue)+140877));
  	  ADSwitch = HAL_ADC_GetValue(&hadc2);

  	  fADValue = 0;
	  for(y=0;y<1000;y++)
	  {
	  	  ADValue = 0;
		  for(i=0;i<100;i++)
		  {
			  ADValue += HAL_ADC_GetValue(&hadc1);
		  }
		  fADValue += (float)(ADValue/100)/4096*3.3;
	  }
	  fADValue /= 1000;
	  sprintf(DisplayCache1,"%d",ADValue/100); //ADC
	  sprintf(DisplayCache2,"%.3f",fADValue); //Volt
	  sprintf(DisplayCache3,"%.0f",fADValue*10000/(3.3-fADValue)); //Register
	  sprintf(DisplayCache4,"%.2f", GetTemp(fADValue/((3.3-fADValue)/10000))); //Temp
	  if(preMode != Mode)	OLED_Clear();
	  if(ADValue/100 >1800 && Emoji && Mode<5 )
	  {
		  OLED_ShowChar(72,0,'(',16);
		  OLED_ShowChinese(80,0,10,16);
		  OLED_ShowChar(96,0,'_',16);
		  OLED_ShowChinese(106,0,10,16);
		  OLED_ShowChar(122,0,')',16);
	  }
	  if(ADValue/100 <1800 && Emoji && Mode<5 )
	  {
		  OLED_ShowString(72,0,"(-",16);
		  OLED_ShowChinese(88,10,10,16);
		  OLED_ShowString(104,0,"-)",16);
	  }

	  if(ADSwitch <= 1024)
	  {
	  		preMode = Mode;
	  		Mode = 1;
	  }
	  if(ADSwitch > 1024 && ADSwitch < 2048)
	  {
		  	preMode = Mode;
		  	Mode = 2;
	  }
	  if(ADSwitch > 2048 && ADSwitch < 3072)
	  {
		  	preMode = Mode;
		  	Mode = 3;
	  }
	  if(ADSwitch > 3072 && ADSwitch < 4100)
	  {
		 	 preMode = Mode;
		 	 Mode = 4;
	  }
	  if(frame>0)
	  {
		  Mode = 5;
		  OLED_Refresh();
		  OLED_ShowString(0,0,"Calibrate Success!",12);
		  frame--;
		  HAL_Delay(10);
	  }
	  RunMode(Mode);
	  OLED_Refresh();


	  


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Red_Pin|CS_Pin|DC_Pin|RES_Pin
                          |MOSI_Pin|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : S2_Pin S3_Pin S4_Pin */
  GPIO_InitStruct.Pin = S2_Pin|S3_Pin|S4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Red_Pin CS_Pin DC_Pin RES_Pin
                           MOSI_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = Red_Pin|CS_Pin|DC_Pin|RES_Pin
                          |MOSI_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == S2_Pin)
	{
		Emoji = 1-Emoji;
		OLED_Clear();
		OLED_Refresh();
	}
	if(GPIO_Pin == S3_Pin)
	{
		UART = 1-UART;
		OLED_Clear();
		OLED_Refresh();
	}
	if(GPIO_Pin == S4_Pin)
	{
		OLED_Clear();

		HAL_ADC_Stop(&hadc1);
		OLED_ShowString(0,0,"Calibrate ADC1",12);
		HAL_ADCEx_Calibration_Start(&hadc1);
		HAL_ADC_Start(&hadc1);

		HAL_ADC_Stop(&hadc2);
		OLED_ShowString(0,0,"Calibrate ADC2",12);
		HAL_ADCEx_Calibration_Start(&hadc2);
		HAL_ADC_Start(&hadc2);

		OLED_Refresh();
		frame = 20;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
