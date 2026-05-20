/*
 * calibration.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

/* Includes */
#include "main.h"
#include "math.h"
#include "calibration.h"
#include "OLED_format.h"
/* Variable Definitions */


/* Function Implementations */
void calibration(void) {
	Cali_Start_Display(); // Display going into calibration mode
	HAL_Delay(500);

	calibrationLoop(false);	// Calibrate positive directions
	calibrationLoop(true);	// Calibrate negative directions

	Cali_Finished_Display(); // Display Calibration finished
	HAL_Delay(1000);
	DEFAULT_DISPLAY();
}

void calibrationLoop(bool forNegative) {
	int directionsCompleted = 0; // Stores how many directions have been calibrated
	bool directionPassed = false; // checks if the direction has been calibrated
	float modVal;
	int adjIndexValue = forNegative;

	// Calibrate for directions
	while (directionsCompleted < 3) {
		// Display to tell the user to press the button to go through the direction
		// Display what direction to point the pedometer
		Cali_Display(forNegative, directionsCompleted);
		while (HAL_GPIO_ReadPin(Button_IN_CALI_GPIO_Port, Button_IN_CALI_Pin) != GPIO_PIN_RESET) {}
		getValues();
		modVal = ADC_to_g(ADC_VAL[directionsCompleted]);
		int absModVal = fabsf(modVal);
		if (absModVal >= 0.9 && absModVal <= 1.1) { // Check if the direction is pointed the right way
			adjVal[adjIndexValue][directionsCompleted] = modVal-1; // The difference from 1g
			directionPassed = true;
		}
		else {
			directionPassed = false;
		}

		if (directionPassed) {
			directionsCompleted++;
			directionPassed = false;
		}
		else {
			Cali_Error_Display(); // error message
			HAL_Delay(500);
		}
	}
}
