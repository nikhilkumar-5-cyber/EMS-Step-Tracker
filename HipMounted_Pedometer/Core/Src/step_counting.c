/*
 * step_counting.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#include "step_counting.h"

/* Global variables */
uint8_t vectorState = 0;

/* Local variables */
uint8_t referenceSamples = 0;
STEP_CLOCK_t timeCard = {0};
ADXL335_t startVector = {0};
ADXL335_t maxPeakVector = {0};
ADXL335_t peakSeries[PEAK_SAMPLES] ={0};
uint8_t peakSeriesIndex = 0;
ADXL335_t lastSamples[ISO_SAMPLES] = {0}; //Local processing buffer

bool bothAbove(double a, double b, double threshold) {

	return a > threshold && b > threshold;

}

void updateLastSamples(ADXL335_t *dest) {

	for (uint8_t i = 0; i < ISO_SAMPLES; i++) {
		lastSamples[i] = SAMPLE_BUFFER[i]; //Update first (ISO_SMAPLES)
	}
	//FIX: Subject to race-conditions
}

void gaitCycle() { //NOTE: call for every NEW sample

	switch (vectorState) {

	case (0): //IDLE

		updateLastSamples(lastSamples); //Update local processing buffer
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
			timeCard.begin = HAL_GetTick(); //Mark start time
		}

		break; /* CHANGE STATE: Increasing acceleration (1,2,3...)) >> (1) */

	/* MOVING */
	case (1):

		updateLastSamples(lastSamples); //Update local processing buffer

		if (HAL_GetTick() - timeCard.begin >= MAX_MOVING_TIME) {
			vectorState = 0; //Return to IDLE
		}
		else { //2x Samples > Threshold
			for (uint8_t i = 0; i < REF_SAMPLES-1; i++) {
				if (lastSamples[i].magnitude > lastSamples[i+1].magnitude && bothAbove(lastSamples[i].magnitude, lastSamples[i+1].magnitude, PEAK_THRESHOLD)) {
					//Flag or other condition
				}
			}

			//FIX: Change to REF_SAMPLES for loop
			vectorState = 2; //Progress to PEAKING
			timeCard.peak_start = HAL_GetTick();

			//Add previous samples for reference
			peakSeriesIndex = 0;
			memset(peakSeries, 0, sizeof(peakSeries)); //Clear buffer
			for (; peakSeriesIndex < REF_SAMPLES; peakSeriesIndex++) {
				peakSeries[peakSeriesIndex] = lastSamples[peakSeriesIndex];
			}
		}

		break; /* CHANGE STATE: Timeout >> (0); OR Break threshold >> (2) */

	/* PEAKING */
	case (2):

		//Dynamically copy SAMPLE_BUFF entries into peakSamples

		//Update maxPeakVector

		//Detect falling condition && below threshold

		if (HAL_GetTick() - timeCard.peak_start >= MIN_PEAK_TIME && lastSamples[0].magnitude <= PEAK_THRESHOLD && lastSamples[1].magnitude <= PEAK_THRESHOLD){
			vectorState = 0;
			timeCard.end = HAL_GetTick();
			timeCard.time = timeCard.end - timeCard.begin;
			stepCount++;
		}

		break; /* CHANGE STATE: Minimum Time (≥200ms) && (2-3 samples < threshold) >> 0 */
	}
}


