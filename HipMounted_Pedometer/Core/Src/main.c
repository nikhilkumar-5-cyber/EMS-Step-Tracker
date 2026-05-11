/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int isADCFinished = 0;
u_int32_t ADC_VAL[3]; // Store raw X, Y and Z values in a array
float adjVal[2][3] = { // Stores the adjustment value for +-x, +-y, +-z,
		{1, 1, 1},
		{1, 1, 1}
};

const float STM_res = 4095; // STM resolution
const float refV = 3.3; //[V]
const float sensitivity = 0.33; // [V/g]
// ADC value at 0g [half of 3.3 * max ADC value]
const float zero_gBias = (refV/2)*STM_res;
// The ADC value between a difference in 1 g - [(sensitivity/ref voltage)*(STM resolution)]
const float ADC_per_gVal = (sensitivity/refV)*(STM_res);

volatile int x;
volatile int y;
volatile int z;

volatile int stepCount = 0; // Stores the count of steps
volatile int prevStepCount; // Stores the previous step count
volatile int stepCountTimeDiff; // Difference in time when new step count is calculated
volatile float distanceTravelled = 0; // Distance travelled [m]

typedef enum {
	STATIC,
	WALKING,
	RUNNING
} WalkingPace;

WalkingPace pace = STATIC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void getValues(void);
static void stepTracking(void);
static void walkingPace(void);
static void ST_protocol(void);
static void calibration(void);
static void distanceTravelled(void);
static int ADC_to_mV(u_int32_t ADC_val);
static float ADC_to_g(u_int32_t ADC_val);
static int g_to_ADC(float g_val);
static u_int32_t get_ADC_Values(void);
static void HAL_ADC_ConvoCpltCallback(ADC_HandlerTypeDef *hadc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void getValues(void) { // Gets the ADC values and converts them to g values and adjusts them based on calibrated values
	get_ADC_Values();
	// Adjusts values based on calibrated values
	for (int i=0; i<3; i++) {
		// Checks if the g-val
		if (ADC_to_g(ADC_VAL[i])< 0) {
			ADC_VAL[i] = ADC_VAL[i]*adjVal[1][i];
		}
		else {
			ADC_VAL[i] = ADC_VAL[i]*adjVal[1][i];
		}
	}
	x = ADC_to_g(ADC_VAL[0]);
	y = ADC_to_g(ADC_VAL[1]);
	z = ADC_to_g(ADC_VAL[2]);
}


void walkingPace(void) {
	const int walkingFreqMax = 0; // FIX
	volatile int stepFrequency = (stepCount-prevStepCount)/stepCountTimeDiff;
	if (stepFrequency == 0) { // Checks if there hasn't been movement
		pace = STATIC;
		// Display Walking Pace as "Static/Stationary"
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin_7, GPIO_PIN_SET); // Yellow LED (stationary)
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin_9, GPIO_PIN_RESET); // Orange LED (walking)
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin_7, GPIO_PIN_RESET); // RED LED (running)
	}
	else if (stepFrequency < walkingFreqMax) { // Checks if Pace is walking
		pace = WALKING;
		// Display Walking Pace as "Static/Stationary"
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin_7, GPIO_PIN_RESET); // Yellow LED (stationary)
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin_9, GPIO_PIN_SET); // Orange LED (walking)
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin_7, GPIO_PIN_RESET); // RED LED (running)
	}
	else { // Pace is running
		pace = RUNNING;
		// Display Walking Pace as "Static/Stationary"
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin_7, GPIO_PIN_RESET); // Yellow LED (stationary)
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin_9, GPIO_PIN_RESET); // Orange LED (walking)
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin_7, GPIO_PIN_SET); // RED LED (running)

	}
}

void ST_protocol(void) {
	// Expected g change for X,Y and Z
	const float expected_X = (-0.4425/sensitivity); // -442.5 mV
	const float expected_Y = (0.4425/sensitivity); //  442.5 mV
	const float expected_Z = (0.75/sensitivity); // 750 mV

	// Convert the current X, Y, Z values to mV
	getValues();
	volatile float old_x = x;
	volatile float old_y = y;
	volatile float old_z = z;
	// Activate ADXL ST Pin
	HAL_GPIO_WritePin(GPIOB, GPIO_Pin_1, GPIO_PIN_SET);
	// get the new X,Y and Z values and convert them to mV
	getValues();
	volatile float new_x = x;
	volatile float new_y = y;
	volatile float new_z = z;
	// Disable ADXL ST Pin
	HAL_GPIO_WritePin(GPIOB, GPIO_Pin_1, GPIO_PIN_RESET);

	// Calculate the difference between old and new values
	volatile float x_diff = new_x - old_x;
	volatile float y_diff = new_y - old_y;
	volatile float z_diff = new_z - old_z;

	if ((x_diff >= expected_X+(0.1*expected_X) && x_diff <= expected_X-(0.1*expected_X)) &&
		(y_diff >= expected_Y+(0.1*expected_Y) && y_diff <= expected_Y-(0.1*expected_Y)) &&
		(z_diff >= expected_Z+(0.1*expected_Z) && z_diff <= expected_Z-(0.1*expected_Z))) {
		// Display "Working" on OLED (FIX)
	}
	else {
		// Display "Not Working" on OLED (FIX)
	}
}

void calibration(void) {
	HAL_Delay(500);
	int size = 3;
	float modVal;
	for (int i=0; i<size; i++) {
		modVal = ADC_to_g(ADC_VAL[i]);
		adjVal[0][i] = modVal;

		modVal = -ADC_to_g(ADC_VAL[i]);
		adjVal[1][i] = modVal;
	}

	// Display Calibration finished
	HAL_Delay(1000);
	// Display back to normal information
}

void distanceTravelled(void) {
	switch (pace) {
		case STATIC:
			distanceTravelled += 0;
		case WALKING:
			distanceTravelled += (stepCount-prevStepCount) * 1; // FIX
		case RUNNING:
			distanceTravelled += (stepCount-prevStepCount) * 3; // FIX
	}
}

int ADC_to_V(u_int32_t ADC_val) {
	// Converts value to mV
	int converted_val = ADC_val*(refV/STM_res);
	return converted_val;
}

float ADC_to_g(u_int32_t ADC_val) {
	// Converts ADC value to g
	float gVal = (ADC_to_V(ADC_val)-zero_gBias)/sensitivity;
	return gVal;
}

int g_to_ADC(float g_val) {
	// Converts g value to ADC
	int ADCval;
	if (g_val > 0) {
		ADCval = zero_gBias + (ADC_per_gVal*g_val);
	}
	if (g_val < 0) {
		ADCval = zero_gBias - (ADC_per_gVal*g_val);
	}
	return ADCval;
}

u_int32_t get_ADC_Values(void) {
	// Function reads the ADC values of X, Y and Z and puts it in the ADC_VAL array
	HAL_ADC_Start_DMA(&hadc1, ADC_VAL, 3);
	while (isADCFinished != 1) {}
	isADCFinshed = 0;
}

void HAL_ADC_ConvoCpltCallback(ADC_HandleTypeDef *hadc) {
	isADCFinished = 1;
}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  getValues(); // Gets the x,y and z values
	  stepTracking(); // Count steps
	  walkingPace(); // Determines Walking pace


	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_RESET) {// Check if the ST button is pressed
		  ST_protocol(); // Checks if the ST button is pressed and then checks if ADXL is working properly
	  }

	  if (/* condition */) {// Calibrates direction when button is pressed
		  calibration();
	  }

	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) {// Resets Counter when reset button is pressed
	  		stepCount = 0;
	  }

	  distanceTravelled(); // Calculates the distance travelled
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_ORANGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|ST_PIN_CONTROL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_ORANGE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_ORANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin ST_PIN_CONTROL_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|ST_PIN_CONTROL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_IN_ST_Pin Button_IN_RESET_Pin */
  GPIO_InitStruct.Pin = Button_IN_ST_Pin|Button_IN_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
