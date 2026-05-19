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

/* Defines */
#define PEAK_THRESHOLD 0.6 //acceleration [m/s^2]
#define PEAK_SERIES_SIZE 20 //~1x step-size MAX
//#define MAX_WAITING_TIME
//#define MAX_STEP_TIME
#define MAG_SAMPLES 3 //Recent (3) samples


/* Externs */
typedef struct {
	uint32_t BEGIN;
	uint32_t END;
	uint32_t TIME;
} STEP_CLOCK;

extern uint8_t VECTOR_STATE;

/* Function prototypes */
ADXL335 new_Magnitudes(); //Retrieve last (MAG_SAMPLES) from sampling_Buffer; Copy and compute magnitudes.

void vector_tracking(); //Control step-tracking states

#endif /* INC_STEP_COUNTING_H_ */
