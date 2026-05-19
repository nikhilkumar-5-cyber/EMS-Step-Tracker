/*
 * step_counting.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#include "magnitude.h"
#include "step_counting.h"
#include <stdbool.h>
#include "stm32g0xx_hal.h" //Included in main.h

/* Global Variables */

uint8_t vectorState = 0; //EXTERN
STEP_CLOCK_t timeCard = {0};
ADXL335_t startVector = {0};
ADXL335_t lastSamples[ISO_SAMPLES] = {0}; //Local processing buffer

/* INTERRUPT (Sample Acceleration >> Receive Magnitude) */

void update_lastSamples(ADXL335_t *dest) {

	for (uint8_t i = 0; i < ISO_SAMPLES; i++) {
		lastSamples[i] = SAMPLE_BUFFER[i]; //Update first (ISO_SMAPLES)
	}

}

void gait_cycle() {

	switch (vectorState) {

	case (0): //IDLE

		update_lastSamples(lastSamples); //Update local processing buffer
		uint8_t decreasing = 1; //Flag indicating: 0th > 1st > 2nd...

		for (uint8_t i = 0; i < ISO_SAMPLES-1; i++) {
			//Check for (ISO_SAMPLES)# of successive increases
			if (lastSamples[i].magnitude <= lastSamples[i+1].magnitude) {
				decreasing = 0;
				break;
			}
		}

		if (decreasing) {
		//Progress to MOVING
			vectorState = 1;
			startVector = lastSamples[0];
			timeCard.begin = HAL_GetTick();
		}

		break; /* CHANGE STATE: Increasing acceleration (1,2,3...)) >> (1) */

	/* MOVING */
	case (1):

		update_lastSamples(lastSamples); //Update local processing buffer

		if (HAL_GetTick() - timeCard.begin > MAX_MOVING_TIME) {
			vectorState = 0; //Return to IDLE
		}
		else if (lastSamples[0].magnitude > PEAK_THRESHOLD && lastSamples[1].magnitude > PEAK_THRESHOLD) { //2x Samples > Threshold
			vectorState = 2; //Progress to STEPPING
		}

		break; /* CHANGE STATE: Timeout >> (0); OR Break threshold >> (2) */

	/* STEPPING */
	case (2):

		update_lastSamples(lastSamples); //Update local processing buffer



		break; /* CHANGE STATE: Minimum Time (≥200ms) && (2-3 samples < threshold) >> 0 */
	}
}


