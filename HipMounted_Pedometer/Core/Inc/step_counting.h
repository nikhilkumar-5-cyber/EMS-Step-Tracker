/*
 * step_counting.h
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#ifndef INC_STEP_COUNTING_H_
#define INC_STEP_COUNTING_H_

#include "main.h"
#include "magnitude.h"
#include <time.h>
#include <stdbool.h>
#include <string.h>

/* Defines */
#define PEAK_THRESHOLD 0.6 //acceleration [m/s^2]
#define PEAK_SAMPLES 10 //55% of sampling time
#define REF_SAMPLES 2 //Starting samples for peakSeries
#define MAX_MOVING_TIME 300 //50% of AVG_STEP [ms]
#define MIN_PEAK_TIME 200
#define ISO_SAMPLES 3 //Local processing buffer size

/* Externs */

extern uint8_t VECTOR_STATE;

/* Function prototypes */
bool bothAbove(double a, double b, double threshold);

void updateLastSamples(ADXL335_t *dest);

void gaitCycle(); //Control step-tracking states

#endif /* INC_STEP_COUNTING_H_ */
