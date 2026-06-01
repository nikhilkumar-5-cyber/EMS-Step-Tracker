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
	HAL_Delay(2000);

	calibrationLoop(false);	// Calibrate positive directions
	calibrationLoop(true);	// Calibrate negative directions

	Cali_Finished_Display(); // Display Calibration finished
	HAL_Delay(1000);
	DEFAULT_DISPLAY(); // Go to the default display
}

void calibrationLoop(bool forNegative) {
	int directionsCompleted = 0; // Stores how many directions have been calibrated
	bool directionPassed = false; // checks if the direction has been calibrated
	float modVal; // Holds value that is read from the STM
	int adjIndexValue = forNegative; // Index value tells the AdjVal array where to store the adjustment value

	/* Calibrate for directions */
	while (directionsCompleted < 3) {
		/* Display to tell the user to press the button to go through the direction
		 * Display what direction to point the pedometer
		 * */
		Cali_Display(forNegative, directionsCompleted);

		/* Wait until the button is pressed*/
		while (HAL_GPIO_ReadPin(Button_IN_CALI_GPIO_Port, Button_IN_CALI_Pin) != GPIO_PIN_RESET) {}
		/* Get the latest X/Y/Z values and store them*/
		getValues();
		modVal = ADC_to_g(ADC_VAL[directionsCompleted]);

		/* Check if the direction is pointed the right way */
		float absModVal = fabsf(modVal);
		if (absModVal >= 0.9 && absModVal <= 1.1)
		{
			adjVal[adjIndexValue][directionsCompleted] = absModVal-1; // The difference from 1g
			directionPassed = true;
		}
		else
		{
			directionPassed = false;
		}

		/* Check if the direction is Calibrated */
		if (directionPassed) {
			/* Move on to next direction */
			directionsCompleted++;
			directionPassed = false;
		}
		else {
			/* Display Error Message */
			Cali_Error_Display(); // error message
			HAL_Delay(500);
		}
	}
}
