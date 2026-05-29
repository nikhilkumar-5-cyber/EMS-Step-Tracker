/*
 * testComponents.c
 *
 *  Created on: 27 May 2026
 *      Author: Nikhi
 */

#include "testComponents.h"
#include "main.h"
#include "stdio.h"

char x_val[10];
char y_val[10];
char z_val[10];

void testLEDS(void) {
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	HAL_Delay(500);
}
void testButtons(void) {
	char reset[] = "Reset Button Pressed";
	char cali[] = "Calibration Button Pressed";
	if (HAL_GPIO_ReadPin(Button_IN_RESET_GPIO_Port, Button_IN_RESET_Pin) == GPIO_PIN_RESET) {
		HAL_UART_Transmit(&huart2, (uint8_t *)reset, sizeof(reset), HAL_MAX_DELAY); // Testing
		}
	if (HAL_GPIO_ReadPin(Button_IN_CALI_GPIO_Port, Button_IN_CALI_Pin) == GPIO_PIN_RESET) {
		HAL_UART_Transmit(&huart2, (uint8_t *)cali, sizeof(cali), HAL_MAX_DELAY); // Testing
	}
}
void testADXL(void) {
	float value = RAW_SAMPLE.X;
	int integer_part = (int)value;
	int decimal_part = (int)((value - integer_part) * 100);
	snprintf(x_val, sizeof(x_val), "X: %d.%02d\n", integer_part, decimal_part);
	HAL_UART_Transmit(&huart2, (uint8_t *)x_val, sizeof(x_val), HAL_MAX_DELAY); // Testing

	value = RAW_SAMPLE.Y;
	integer_part = (int)value;
	decimal_part = (int)((value - integer_part) * 100);
	snprintf(y_val, sizeof(y_val), "Y: %d.%02d\n", integer_part, decimal_part);
	HAL_UART_Transmit(&huart2, (uint8_t *)y_val, sizeof(y_val), HAL_MAX_DELAY); // Testing

	value = RAW_SAMPLE.Z;
	integer_part = (int)value;
	decimal_part = (int)((value - integer_part) * 100);
	snprintf(z_val, sizeof(z_val), "Z: %d.%02d\n", integer_part, decimal_part);
	HAL_UART_Transmit(&huart2, (uint8_t *)z_val, sizeof(z_val), HAL_MAX_DELAY); // Testing
}
