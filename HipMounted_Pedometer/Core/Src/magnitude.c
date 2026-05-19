/*
 * magnitude.c
 *
 *  Created on: May 11, 2026
 *      Author: noah
 */

#include "magnitude.h"

/* Global Variables */

/* INTERRUPT (Sample Acceleration) */

void compute_Magnitude(ADXL335_t *sample) {

	//Square & sumXYZ
	double magSquared = pow(sample->X, 2) + pow(sample->Y, 2) + pow(sample->Z, 2) ;

	//sqrt()
	sample->magnitude = sqrt(magSquared);

}
