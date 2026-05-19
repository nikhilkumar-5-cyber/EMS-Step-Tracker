/*
 * step_counting.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#include "magnitude.h"
#include "step_counting.h"

/* Global Variables */

uint8_t VECTOR_STATE = 0; //EXTERN
uint8_t magnitudeIndex = 0;


/* INTERRUPT (Sample Acceleration >> Receive Magnitude) */


ADXL335 new_magnitudes() {

	ADXL335 newMagnitudes[MAG_SAMPLES] = {0};

	//Check whether (MAG_SAMPLES)# of values have already been read
	for (uint8_t i = 0; i < MAG_SAMPLES; i++) {

		newMagnitudes[i] = (ADXL335){ //Equate most recent (MAG_SAMPLES) from SAMPLE_BUFFER (LIFO)
				.x = SAMPLE_BUFFER[i].x,
				.y = SAMPLE_BUFFER[i].y,
				.z = SAMPLE_BUFFER[i].z,

		};
		compute_Magnitude(&newMagnitudes[i]); //Store associated magnitude
	}
	return newMagnitudes; //COPY for processing/conditions
}

void gait_cycle() {

	switch (VECTOR_STATE) {

	case (0): //IDLE

		//if (MAG_SAMPLES) successive increases
				//VECTOR_STATE = 1; STEP_CLOCK.BEGIN;

		break; //Transition: Increasing acceleration (1,2,3...)

	case (1): //MOVING

		//if (CURRENT – STEP_CLOCK.BEGIN > 50%(AVG_STEP))
				//VECTOR_STATE = 0;

		//else if (0th and 1st (SAMPLE_BUFF) > Threshold)
				//VECTOR_STATE = 1;

		break; //Transition: Timeout (50% of AVG_STEP) or Break threshold

	case (2): //STEPPING



		break; //Transition: Timeout (≥200ms) && (2-3 samples < threshold)
	}
}
//	/* Pass 1,2,3 magnitudes */
//	switch (VECTOR_STATE) {
//		/* Start */
//		case (0):
//			/* Evaluate */
//			if ((MagSamples[0].Magnitude > MagSamples[1].Magnitude)
//				&& (MagSamples[1].Magnitude > MagSamples[2].Magnitude)
//				&& (MagSamples[2].Magnitude > MagSamples[3].Magnitude)) {
//				/*  */
//				START_VECTOR = MagSamples[3];
//				STEP.BEGIN = HAL_GetTick();
//				VECTOR_STATE++;
//			}
//			break;
//		/* Peak */
//		case (1):
//			/* Evaluate */
//			if (IIR[sample_count-1].Magnitude >= PEAK_THRESHOLD) {
//				VECTOR_STATE++;
//			}
//			break;
//		/* End(2) */
//		case (2):
//			/* Retains value across function calls */
//			static uint8_t peak_count;
//			static ADXL335 MAX_PEAK;
//			/* Store up to ~200 (2 seconds) */
//			PEAK_SERIES[peak_count] = IIR[sample_count-1];
//
//			if (IIR[sample_count-1].Magnitude < PEAK_THRESHOLD) {
//				/* Iterate & Return Max Value */
//				for (int i = 0; i <= peak_count; i++) {
//					ADXL335 MAX_SAMPLE = PEAK_SERIES[i];
//					if (MAX_SAMPLE.Magnitude > MAX_PEAK.Magnitude) {
//						MAX_PEAK = PEAK_SERIES[i];
//					}
//				PEAK_VECTOR = MAX_PEAK;
//				VECTOR_STATE++;
//				}
//			}
//			break;
//			/* End(2) */
//		case (3):
//			/* Evaluate */
//			if ((MagSamples[0].Magnitude > MagSamples[1].Magnitude)
//				&& (MagSamples[1].Magnitude > MagSamples[2].Magnitude)) {
//				/* */
//				STOP_VECTOR = MagSamples[2];
////				HAL_Delay(20);
//				STEP.END = HAL_GetTick();
//				STEP.TIME = STEP.END - STEP.BEGIN;
//				VECTOR_STATE = 0;
//				TOTAL_STEPS++;
//			}
//			break;
//	default:
//		//code
//	}
}

