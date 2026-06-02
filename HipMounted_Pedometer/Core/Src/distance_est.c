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

	if (pace == WALKING) {
		distanceTravelled += 0.8;
	}

	else if (pace == RUNNING) {
		distanceTravelled += 1.2;
	}

}
