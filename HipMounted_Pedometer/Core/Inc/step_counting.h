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

/* Defines */
#define PEAK_THRESHOLD 0.6 //acceleration [m/s^2]
#define PEAK_SERIES_SIZE 20 //~1x step-size MAX
#define MAX_MOVING_TIME 300 //50% of AVG_STEP [ms]
#define MIN_PEAK_TIME 200
#define ISO_SAMPLES 3 //Recent (3) samples

/* Externs */

extern uint8_t VECTOR_STATE;

/* Function prototypes */

void update_lastSamples(ADXL335_t *dest);

void gait_cycle(); //Control step-tracking states

#endif /* INC_STEP_COUNTING_H_ */
