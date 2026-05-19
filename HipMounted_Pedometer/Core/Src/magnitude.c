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
	double magSquared = pow(sample->x, 2) + pow(sample->y, 2) + pow(sample->z, 2) ;

	//sqrt()
	sample->magnitude = sqrt(magSquared);

}
