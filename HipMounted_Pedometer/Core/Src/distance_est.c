/*
 * distance_est.c
 *
 *  Created on: Jun 2, 2026
 *      Author: noah
 */

/* Includes */
#include "distance_est.h"
#include "math.h"

/* Global variables */
double distanceTravelled = 0;
double avgStepSizeSum = 0;
double avgStepSize = 0;

/* Local variables */

void addDistance(ADXL335_t *peakVector, ADXL335_t *startVector, ADXL335_t *endVector) {

	double stepSize = fabs(endVector->magnitude - startVector->magnitude);

	avgStepSize += STEP_SIZE_ALPHA * (stepSize - avgStepSize); //Exponential Moving Average

	double alpha = 0; //Weighting * Peak(i)

	if (stepSize > avgStepSize) {

		alpha = 1.5;
	}
	else if (fabs(stepSize - avgStepSize) <= 0.1) {

		alpha = 1.0;
	}
	else {

		alpha = 0.5;
	}

	distanceTravelled += peakVector->magnitude * alpha;

}
