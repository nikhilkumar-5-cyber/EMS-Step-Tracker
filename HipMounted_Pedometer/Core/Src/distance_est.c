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

/* Local variables */

void addDistance() {

	if (pace == WALKING) {
		distanceTravelled += 0.8;
	}

	else if (pace == RUNNING) {
		distanceTravelled += 1.2;
	}

}
