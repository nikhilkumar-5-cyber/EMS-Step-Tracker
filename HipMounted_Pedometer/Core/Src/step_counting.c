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
uint8_t aboveThreshold = 0;
STEP_CLOCK_t timeCard = {0};
ADXL335_t peakSeries[PEAK_SAMPLES] ={0};
uint8_t peakSeriesIndex = 0;
ADXL335_t lastSamples[ISO_SAMPLES] = {0}; //Local processing buffer

bool bothAbove(double a, double b, double threshold) {

	return a > threshold && b > threshold;

}

void updateLastSamples(ADXL335_t *dest) { //Called from gaitCycle()

	for (uint8_t i = 0; i < ISO_SAMPLES; i++) {
		lastSamples[i] = SAMPLE_BUFFER[i]; //Update FIFO processing buffer
	}

}

void gaitCycle() { //Called for each NEW sample; Prevents race-conditions

	switch (vectorState) {

	case (0): //IDLE::OK

		updateLastSamples(lastSamples); //Move samples into local processing buffer
		uint8_t increasing = 1;

		for (uint8_t i = 0; i < ISO_SAMPLES-1; i++) { //Check for (failed) increasing acceleration in FIFO (1st > 2nd > 3rd...)
			if (lastSamples[i].magnitude <= lastSamples[i+1].magnitude) {
				increasing = 0; //Not increasing; Remain in IDLE
				break;
			}
		}
		if (increasing) {
		//Progress to MOVING
			vectorState = 1;
			timeCard.begin = HAL_GetTick(); //Mark start time
		}

		break; /* CHANGE STATE: Increasing acceleration (1,2,3...)) >> (1) */


	case (1): //MOVING::OK

		updateLastSamples(lastSamples);  //Move samples into local processing buffer

		if (HAL_GetTick() - timeCard.begin >= MAX_MOVING_TIME) { //Check for timeout (random, short increase in acceleration)
			vectorState = 0; //Return to IDLE
		}

		else {
			for (uint8_t i = 0; i < REF_SAMPLES-1; i++) {
				if (bothAbove(lastSamples[i].magnitude, lastSamples[i+1].magnitude, PEAK_THRESHOLD)) { //Check set# of samples are above threshold
					aboveThreshold = 1;
				}
				else {
					aboveThreshold = 0;
					break; //Condition failed
				}
			}

			if (aboveThreshold) {
				//Reset index and clear buffer
				peakSeriesIndex = 0;
				memset(peakSeries, 0, sizeof(peakSeries));
				//Equate (REF_SAMPLES)x samples to kick-start peakSeries
				for (; peakSeriesIndex < REF_SAMPLES; peakSeriesIndex++) {
					peakSeries[peakSeriesIndex] = lastSamples[peakSeriesIndex]; //FIFO start from last element; Avoid push-back
				}
				//Update state variables
				vectorState = 2;
				timeCard.peak_start = HAL_GetTick();
			}
		}

		break; /* CHANGE STATE: Timeout >> (0); OR Break threshold >> (2) */

	case (2): //PEAKING::WIP

		/* Operating Principle: Detect a local peak, and a sharp decline */
		uint8_t peakDetected = 0;

		//Get the most recent sample (SAMPLE_BUFF)

		//Update maxPeakVector

		//Detect falling condition && below threshold

		if (peakDetected && HAL_GetTick() - timeCard.peak_start >= MIN_PEAK_TIME){
			vectorState = 0;
			timeCard.end = HAL_GetTick();
			timeCard.time = timeCard.end - timeCard.begin;
			stepCount++;
		}

		break; /* CHANGE STATE: Minimum Time (≥200ms) && (2-3 samples < threshold) >> 0 */
	}
}


