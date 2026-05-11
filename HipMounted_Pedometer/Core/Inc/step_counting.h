/*
 * step_counting.h
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#ifndef INC_STEP_COUNTING_H_
#define INC_STEP_COUNTING_H_

#include <time.h>
#include "magnitude.h"
/* Defines */
#define PEAK_THRESHOLD 0.6
#define PEAK_SERIES_SIZE 200
/* Externs */
typedef struct {
	uint32_t BEGIN;
	uint32_t END;
	uint32_t TIME;
} Timing;
extern float step_TIME;
extern float step_BEGIN;
extern float step_STOP;
extern uint8_t vector_state;
extern Timing STEP;
extern MPU6050_Accelerometer START_VECTOR;
extern MPU6050_Accelerometer PEAK_VECTOR;
extern MPU6050_Accelerometer STOP_VECTOR;
extern MPU6050_Accelerometer PEAK_SERIES[PEAK_SERIES_SIZE];
extern uint16_t TOTAL_STEPS;


#endif /* INC_STEP_COUNTING_H_ */
