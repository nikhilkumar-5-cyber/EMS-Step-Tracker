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
uint8_t maxMagnitudeIndex = 0;
uint8_t peakSeriesIndex = 0;
//Custom types
STEP_CLOCK_t timeCard = {0};
ADXL335_t lastSamples[ISO_SAMPLES] = {0}; //Local processing buffer
ADXL335_t peakSeries[PEAK_SAMPLES] = {0};
ADXL335_t START_VECTOR = {0};
ADXL335_t END_VECTOR = {0};


void updateLastSamples(ADXL335_t *dest) { //Called from gaitCycle()

	for (uint8_t i = 0; i < ISO_SAMPLES; i++)
	{
		lastSamples[i] = SAMPLE_BUFFER[i];  //WARNING: Subject to race conditions (technically)
	}

}

void pushFront(ADXL335_t *dest, unsigned int size, ADXL335_t new_sample) {

	for (uint8_t i = size - 1; i > 0; i--)
	{
		dest[i] = dest[i-1];
	}
	dest[0] = new_sample;
}

ADXL335_t findMaxMagnitude(const ADXL335_t *buffer, unsigned int size) {

	ADXL335_t maxVector = {0};
	maxMagnitudeIndex = 0;

	for (uint8_t i = 1; i < size; i++)
	{
		if (buffer[i].magnitude > maxVector.magnitude)
		{
			maxVector = buffer[i];
			maxMagnitudeIndex = i;
		}
	}

	return maxVector;
}

void trackGaitPhase() { //Called for each NEW sample; Prevents race-conditions~

	switch (vectorState)
	{
	case (0): //IDLE::OK

		updateLastSamples(lastSamples); //Move samples into local processing buffer
		uint8_t increasing = 1;

		for (uint8_t i = 0; i < ISO_SAMPLES-1; i++)
		{ //Check for (failed) increasing acceleration in FIFO (1st > 2nd > 3rd...)
			if (lastSamples[i].magnitude <= lastSamples[i+1].magnitude)
			{
				increasing = 0; //Not increasing; Remain in IDLE
				break;
			}
		}
		if (increasing)
		{
		//Progress to MOVING
			vectorState = 1;
			timeCard.begin = HAL_GetTick(); //Mark start time
			START_VECTOR = lastSamples[0];
		}
		break; /* CHANGE STATE: Increasing acceleration (1,2,3...)) >> (1) */


	case (1): //MOVING::OK

		updateLastSamples(lastSamples);  //Move samples into local processing buffer

		if (HAL_GetTick() - timeCard.begin >= MAX_MOVING_TIME) //Check for timeout (random, short increase in acceleration)
		{
			vectorState = 0; //Return to IDLE
		}

		else
		{
			uint8_t aboveThreshold = 1;

			for (uint8_t i = 0; i < THRESHOLD_REF; i++)
			{
				if (lastSamples[i].magnitude <= PEAK_THRESHOLD) //Check set# of samples are above threshold
				{
					aboveThreshold = 0;
					break; //Condition failed
				}
			}

			if (aboveThreshold)
			{
				//Reset index and clear buffer
				peakSeriesIndex = 0;
				memset(peakSeries, 0, sizeof(peakSeries));
				//Equate (THRESHOLD_REF)x samples to kick-start peakSeries
				for (; peakSeriesIndex < THRESHOLD_REF; peakSeriesIndex++)
				{
					peakSeries[peakSeriesIndex] = lastSamples[peakSeriesIndex]; //FIFO start from last element; Avoid push-back
				}
				//Update state variables
				vectorState = 2;
				timeCard.peak_start = HAL_GetTick();
			}
		}
		break; /* CHANGE STATE: Timeout >> (0); OR Break threshold >> (2) */

	case (2): //PEAKING::OK?

		/* Operating Principle: Detect a local peak, and a sharp decline */
		uint8_t peakDetected = 1;

		//Most recent sample (SAMPLE_BUFF)
		pushFront(peakSeries, PEAK_SAMPLES, SAMPLE_BUFFER[0]); //WARNING: Subject to race conditions (technically)

		//Update maxPeakVector
		ADXL335_t MAX_VECTOR = findMaxMagnitude(peakSeries, PEAK_SAMPLES);

		//Detect falling condition
		for (uint8_t i = 1; i < POST_PEAK_DECREASE; i++)
		{
			if (peakSeries[maxMagnitudeIndex+i].magnitude >= lastSamples[(maxMagnitudeIndex+i)+1].magnitude) //FIFO comparison
			{
				peakDetected = 0; //Not decreasing significantly enough
			}
		}

		if (peakDetected && HAL_GetTick() - timeCard.peak_start >= MIN_PEAK_TIME) //FIX: Necessary waiting?
		{
			vectorState = 3;
		}

		break; /* CHANGE STATE: Minimum Time (≥200ms) && (2-3 samples < threshold) >> 0 */

	case (3): //DIVING::WIP

		updateLastSamples(lastSamples);  //Move samples into local processing buffer

		uint8_t belowThreshold = 1;

		for (uint8_t i = 0; i < THRESHOLD_REF; i++)
		{
			if (lastSamples[i].magnitude <= PEAK_THRESHOLD) //Check set# of samples are below threshold
			{
				belowThreshold = 0;
				break;
			}
		}

		if (belowThreshold == 1 || HAL_GetTick() - timeCard.begin > MAX_STEP_TIME) {
			timeCard.end = HAL_GetTick();
			END_VECTOR = lastSamples[0];
			stepCount++;
			vectorState = 0;
		}

		break;
	}

}


