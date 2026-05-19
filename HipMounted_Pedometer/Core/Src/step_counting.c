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


ADXL335 new_Magnitudes() {

	ADXL335 new_Magnitudes[MAG_SAMPLES] = {0};

	//Check whether (MAG_SAMPLES)# of values have already been read
	if (magnitudeIndex < MAG_SAMPLES) {

		new_Magnitudes[magnitudeIndex] = (ADXL335){ //Equate most recent (MAG_SAMPLES) from SAMPLE_BUFFER (LIFO)
				.x = SAMPLE_BUFFER[magnitudeIndex].x,
				.y = SAMPLE_BUFFER[magnitudeIndex].y,
				.z = SAMPLE_BUFFER[magnitudeIndex].z,
		};
		compute_Magnitude(new_Magnitudes[magnitudeIndex]); //Store associated magnitude

	}

	return new_Magnitudes; //COPY or HOLD?
}

void vector_tracking() {

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

