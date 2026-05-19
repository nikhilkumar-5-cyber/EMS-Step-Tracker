/*
 * calibration.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

/* Includes */
#include "calibration.h"

/* Variable Definitions */


/* Function Implementations */
void calibration(void) {
	HAL_Delay(500);
	int size = 3;
	float modVal;
	for (int i=0; i<size; i++) {
		modVal = ADC_to_g(ADC_VAL[i]);
		adjVal[0][i] = 1-modVal;

		modVal = -ADC_to_g(ADC_VAL[i]);
		adjVal[1][i] = -1-modVal;
	}

	// Display Calibration finished
	HAL_Delay(1000);
	// Display back to normal information
}
