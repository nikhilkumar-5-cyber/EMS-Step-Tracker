/*
 * step_counting.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#include "magnitude.h"
#include "step_counting.h"
#include <stdbool.h>
#include "stm32g0xx_hal.h" //Included in main.h

/* Global Variables */

uint8_t vectorState = 0; //EXTERN
STEP_CLOCK_t timeCard = {0};
ADXL335_t startVector = {0};

/* INTERRUPT (Sample Acceleration >> Receive Magnitude) */

void gait_cycle() {

	switch (vectorState) {

	case (0): //IDLE

		ADXL335_t lastSamples[ISO_SAMPLES] = {0}; //Local processing buffer
		uint8_t decreasing = 1; //0th > 1st > 2nd...

		for (uint8_t i = 0; i < ISO_SAMPLES-1; i++) {
			//Check for (ISO_SAMPLES)# of successive increases
			if (lastSamples[i].magnitude <= lastSamples[i+1].magnitude) {
				decreasing = 0;
				break;
			}
		}

		if (decreasing) {
		//Progress to MOVING
			vectorState = 1;
			startVector = lastSamples[0];
			timeCard.BEGIN = HAL_GetTick();
		}
		break; //Transition: Increasing acceleration (1,2,3...))

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

