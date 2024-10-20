/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

uint32_t ledRED_4 = 0;
uint32_t ledRED_3 = 0;
uint32_t ledRED_2 = 0;
uint32_t ledRED_1 = 0;
uint32_t ledRED_0 = 0;

uint32_t maxLedFound = 0;

uint32_t maxRED=10;
uint32_t maxIR=22;
uint32_t minRED=3;
uint32_t minIR=4;

uint32_t millisCounter = 0;
uint32_t millisAvergeCount = 20;
uint32_t averageRed;
uint32_t averageIR;
uint32_t n;

uint32_t averageLight;


float so2 = 0;

float averageTest;

float R=0;
float R_array[10];

float R_RED;
float R_IR;
float R_sum;

uint8_t flag=0;
uint8_t count=0;
ADC_ChannelConfTypeDef sConfig = {0};

int ADC_Read();

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_LPUART1_UART_Init(void);
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
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Turn off the RED LED
	  HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin, GPIO_PIN_RESET);
	  //Small Delay
	  HAL_Delay(10);
	  //Turn on the IR LED
	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
	  //Save old values of the IR LED
	  ledRED_2=ledRED_1;
	  ledRED_1=ledRED_0;
	  //**************** Reading ADC Value of IR LED ***********************************
	  //Reading light value is done in a way where we measure sample the value with adc in a defined time slot (millisAverageCount)

	  //Set millis counter to 0 and turn on the timer
	  millisCounter=0;
	  HAL_TIM_Base_Start_IT(&htim6);
	  //Make a reading every o.5 ms for millisAverageCount od millis seoconds. Usually for 20ms
	  do{
		averageRed += ADC_Read();
		HAL_Delay(0.5);
		n++;
	  }
	  while(millisCounter <= millisAvergeCount);

	  //We stop the timer
	  HAL_TIM_Base_Stop_IT(&htim6);
	  //Calculate the average value
	  ledRED_0 = averageRed/n - averageLight;
	  averageRed = 0;
	  n = 0;
	  //**********************************************************************************

	  //**************** Check if we got a new maximum in the past 3 readings ***********************************
	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
	  if(ledRED_2<ledRED_1 && ledRED_1> ledRED_0)
	  {
		  //Software check so new maximum is not lower that the current minimum
		  if(ledRED_1>minRED) maxRED=ledRED_1;

		  //We turn off the IR LED
		  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		  HAL_Delay(1);
		  //TUrn on the RED LED
		  HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin, GPIO_PIN_SET);

		  //Do the same avergae reading of the adc value in a "millisAverageCount" interval. (probably measure for 20ms)
		  millisCounter=0;
		  HAL_TIM_Base_Start_IT(&htim6);
		  do{
			  averageIR += ADC_Read();
			  HAL_Delay(0.5);
			  n++;
		 }while(millisCounter <= millisAvergeCount);

		 HAL_TIM_Base_Stop_IT(&htim6);
		 HAL_Delay(1);
		 //CHeck that the new maximum is not lower than the current minimum
		 if(averageIR/n > minIR) maxIR = averageIR/n;

		 //Reset all the values
		 averageIR=0;
		 n = 0;
		 maxLedFound=1;
		 //Turn off the LED
		 HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin, GPIO_PIN_RESET);
	  	 }

	  //**************** Check if we got a new minimum in the past 3 readings ***********************************
	  if(ledRED_2>ledRED_1 && ledRED_1<ledRED_0 && maxLedFound)
	  	{
		  //Software check so new minimum is not higher that the current maximum
		  if(ledRED_1<maxRED) minRED = ledRED_1;

		  //
		  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin, GPIO_PIN_SET);

		  //ADC Reading in the same principle as above
		  millisCounter=0;
		  HAL_TIM_Base_Start_IT(&htim6);
		  do{
			  averageIR += ADC_Read();
			  HAL_Delay(0.5);
			  n++;
		  }while(millisCounter <= millisAvergeCount);

		  HAL_TIM_Base_Stop_IT(&htim6);
		  HAL_Delay(1);
		  if(averageIR/n < maxIR) minIR = averageIR/n;

		  averageIR=0;
		  n = 0;
		  maxLedFound=0;
	  	}

	    //----Calculate R values for both LEDs
	  	R_RED = ((float)maxRED-(float)minRED)/(float)minRED;
	  	R_IR= ((float)maxIR-(float)minIR)/(float)minIR;

	  	//CHeck if finger is inserted (everyting above 2300 means no finger)
	  	if(ledRED_0<2300) {

	  		//CHeck that the Ratio makes sense
	  		//If the value is bellow 0.5, means that the reading is okay so we calculate the
	  		// R
	  		// New R is added to the array of past (10) arrays
	  		if(R_IR / R_RED < 0.5) {
	  			R_array[count] =  R_IR / R_RED ;
				if(count >=9) count = 0;
				else count ++;
	  		}



	  		//We calculate the R value by averaging the last 10 measurements
	  		R_sum = 0;
			for(uint8_t loop = 0; loop < 9; loop++) {
				R_sum = R_sum + R_array[loop];

			}
			R = R_sum/10.0;

			//******************* SO2 Calculation ****************************
			so2 = -10 * R + 100;
			//****************************************************************

			//**** We send the values via LPUART to the PC **************************
			uint8_t msg[16]="HEllo";
			sprintf(msg,"SO2: %d \n\r",(int)so2);
			HAL_UART_Transmit(&hlpuart1, msg, strlen(msg), 100);
			//*******************************************************************
	  	} else {
	  		//If there is no finger we display "no finger" text
	  		uint8_t msg[16]="No finger \n\r";
			HAL_UART_Transmit(&hlpuart1, msg, strlen(msg), 100);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  htim6.Init.Prescaler = 64000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_LED_Pin */
  GPIO_InitStruct.Pin = IR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IR_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_LED_Pin */
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 * Timer 6 callback function
 * Every xx u seconds callback function gets called
 * ADC measures voltage on photodiode (first RED then IR)
 * And then R value is computed from max in min values of both diodes.
 * @htim takes timer argument (htim6)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	millisCounter++;
	//HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

	//R =  R_IR / R_RED;

	//averaged saturation at any heart beat
	//sumSaturation += instantSaturation;
	//numBeat++;

}

int ADC_Read(){
	uint32_t adcValue = 0;
	sConfig.Channel = ADC_CHANNEL_1;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,5)==HAL_OK){
		adcValue=HAL_ADC_GetValue(&hadc1);
	}
	return adcValue;
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
