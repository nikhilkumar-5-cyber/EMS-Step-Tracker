/*
 * self_test.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

/* Includes */
#include "self_test.h"
#include "OLED_format.h"

/* Variable Definitions */


/* Function Implementations */
void ST_Protocol(void) {
	// Expected g change for X,Y and Z
	const float X_FACTORY_CHG = (-0.4425/sensitivity); // -442.5 mV
	const float Y_FACTORY_CHG = (0.4425/sensitivity); //  442.5 mV
	const float Z_FACTORY_CHG = (0.75/sensitivity); // 750 mV

	// Convert the current X, Y, Z values to mV
	get_ADC_Values();
	volatile float X_PRE_ST = RAW_SAMPLE.X;
	volatile float Y_PRE_ST = RAW_SAMPLE.Y;
	volatile float Z_PRE_ST = RAW_SAMPLE.Z;
	// Activate ADXL ST Pin
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	// get the new X,Y and Z values and convert them to mV
	get_ADC_Values();
	volatile float X_POST_ST = RAW_SAMPLE.X;
	volatile float Y_POST_ST = RAW_SAMPLE.Y;
	volatile float Z_POST_ST = RAW_SAMPLE.Z;
	// Disable ADXL ST Pin
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	// Calculate the difference between old and new values
	volatile float X_DELTA = X_POST_ST - X_PRE_ST;
	volatile float Y_DELTA = Y_POST_ST - Y_PRE_ST;
	volatile float Z_DELTA = Z_POST_ST - Z_PRE_ST;

	if ((X_DELTA >= X_FACTORY_CHG+(0.1*X_FACTORY_CHG) && X_DELTA <= X_FACTORY_CHG-(0.1*X_FACTORY_CHG)) &&
		(Y_DELTA >= Y_FACTORY_CHG+(0.1*Y_FACTORY_CHG) && Y_DELTA <= Y_FACTORY_CHG-(0.1*Y_FACTORY_CHG)) &&
		(Z_DELTA >= Z_FACTORY_CHG+(0.1*Z_FACTORY_CHG) && Z_DELTA <= Z_FACTORY_CHG-(0.1*Z_FACTORY_CHG))) {
		// Display "Working" on OLED
		ST_DISPLAY(true);
	}
	else {
		// Display "Not Working" on OLED
		ST_DISPLAY(false);
	}
}
