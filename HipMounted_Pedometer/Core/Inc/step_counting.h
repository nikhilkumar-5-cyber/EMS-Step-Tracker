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
#define PEAK_THRESHOLD 1.0 //acceleration [m/s^2]
#define PEAK_SAMPLES 10 //55% of sampling time
#define POST_PEAK_DECREASE 3
#define REF_SAMPLES 2 //Starting samples for peakSeries
#define MAX_MOVING_TIME 300 //50% of AVG_STEP [ms]
#define MIN_PEAK_TIME 200
#define ISO_SAMPLES 3 //Local processing buffer size

/* Externs */

extern uint8_t VECTOR_STATE;

/* Function prototypes */
void updateLastSamples(ADXL335_t *dest); //Helper: Copies the most recent samples from SAMPLE_BUFFER into local processing buffer

bool bothAbove(double a, double b, double threshold); //Helper: Check (2) values are above a nominal value

void pushFront(ADXL335_t *dest, unsigned int size, ADXL335_t new_sample); //Helper: Push-back samples and equate [0] to new_sample

ADXL335_t findMaxMagnitude(const ADXL335_t *buffer, unsigned int size);

void trackGaitPhase(); //Step-Tracking state machine

#endif /* INC_STEP_COUNTING_H_ */
