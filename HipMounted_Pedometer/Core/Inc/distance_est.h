/*
 * distance_est.h
 *
 *  Created on: Jun 2, 2026
 *      Author: noah
 */

#ifndef INC_DISTANCE_EST_H_
#define INC_DISTANCE_EST_H_

#include "main.h"

/* Defines */
#define STEP_SIZE_ALPHA 0.1

/* Externs */
extern double distanceTravelled;
extern double avgStepSizeSum;
extern double avgStepSize;

/* Function prototypes */
void addDistance(ADXL335_t *peakVector, ADXL335_t *startVector, ADXL335_t *endVector);

#endif /* INC_DISTANCE_EST_H_ */
