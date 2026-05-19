/*
 * magnitude.c
 *
 *  Created on: May 11, 2026
 *      Author: noah
 */

#include "magnitude.h"

/* Global Variables */

/* INTERRUPT (Sample Acceleration) */

void compute_Magnitude(ADXL335 *sample) {

	//Square & sumXYZ
	double magSquared = pow(sample->x, 2) + pow(sample->y, 2) + pow(sample->z, 2) ;

	//sqrt()
	sample->magnitude = sqrt(magSquared);

}


//	if (sample_count <= NUM_SAMPLES-1) {
//		/* Store sample[i] */
//		Sampling[sample_count].X = CALIB_SAMPLE.X;
//		Sampling[sample_count].Y = CALIB_SAMPLE.Y;
//		Sampling[sample_count].Z = CALIB_SAMPLE.Z;
//		/* Magnitude calculation */
//		Sampling[sample_count].Magnitude = (pow(Sampling[sample_count].X, 2) + pow(Sampling[sample_count].Y, 2) + pow(Sampling[sample_count].Z, 2));
//		Sampling[sample_count].Magnitude = sqrt(Sampling[sample_count].Magnitude);
//		/* Increment */
//		sample_count++;
//	}
//
//	else {
//		/* Rolling samples */
//		Sampling[0].X = Sampling[NUM_SAMPLES-1].X;
//		Sampling[0].Y = Sampling[NUM_SAMPLES-1].Y;
//		Sampling[0].Z = Sampling[NUM_SAMPLES-1].Z;
//		/* Reset count */
//		sample_count = 1;
//		/* Clear */
//		memset(&Sampling[1], 0, sizeof(Sampling[0]) * (NUM_SAMPLES - 1));
//	}
//
//	/* FIFO */
//	MagSamples[3] = MagSamples[2];
//	MagSamples[2] = MagSamples[1];
//	MagSamples[1] = MagSamples[0];
//	/* Pushes sample_count back (1) after incremented */
//	MagSamples[0] = Sampling[sample_count-1];
//}
