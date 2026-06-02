/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "self_test.h"
#include "walking_pace.h"
#include "calibration.h"
#include "OLED_format.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define NUM_SAMPLES 40 //Stores ~2x steps @20Hz

typedef struct {
	uint32_t begin;
	uint32_t peak_start;
	uint32_t end;
	uint32_t time;
} STEP_CLOCK_t;


typedef struct {
	double X;
	double Y;
	double Z;
	double magnitude;
} ADXL335_t;

extern volatile ADXL335_t RAW_SAMPLE;
extern ADXL335_t CALIB_SAMPLE;
extern ADXL335_t SAMPLE_BUFFER[NUM_SAMPLES];

extern volatile uint32_t stepCount;
extern volatile uint32_t indexVal; // Global index variable

extern uint16_t ADC_VAL[3];
extern float adjVal[2][3];

extern UART_HandleTypeDef huart2;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void get_ADC_Values(void);
void getValues(void);
double ADC_to_g(uint32_t ADC_val);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define Z_IN_Pin GPIO_PIN_0
#define Z_IN_GPIO_Port GPIOA
#define Y_IN_Pin GPIO_PIN_1
#define Y_IN_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define X_IN_Pin GPIO_PIN_4
#define X_IN_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_7
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define ST_PIN_CONTROL_Pin GPIO_PIN_1
#define ST_PIN_CONTROL_GPIO_Port GPIOB
#define Button_IN_CALI_Pin GPIO_PIN_14
#define Button_IN_CALI_GPIO_Port GPIOB
#define LED_ORANGE_Pin GPIO_PIN_9
#define LED_ORANGE_GPIO_Port GPIOA
#define LED_YELLOW_Pin GPIO_PIN_7
#define LED_YELLOW_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Button_IN_RESET_Pin GPIO_PIN_4
#define Button_IN_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
