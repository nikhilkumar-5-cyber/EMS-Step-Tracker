/*
 * magnitude.c
 *
 *  Created on: May 11, 2026
 *      Author: noah
 */

#include "magnitude.h"

/* Global Variables */
MPU6050_Accelerometer Sampling[NUM_SAMPLES];
MPU6050_Accelerometer MagSamples[4];
uint8_t sample_count = 1;

/* INTERRUPT (Sample Acceleration) */

void conditions_INIT() {

	Sampling[0].X = 0;
	Sampling[0].Y = 0;
	Sampling[0].Z = 0;
	Sampling[0].Magnitude = 0;
}

void HPF_magnitiude_IT() {

	if (sample_count <= NUM_SAMPLES-1) {
		/* Store sample[i] */
		Sampling[sample_count].X;// = Acceleration.X;
		Sampling[sample_count].Y;// = Acceleration.Y;
		Sampling[sample_count].Z;// = Acceleration.Z;
		/* Magnitude calculation */
		Sampling[sample_count].Magnitude = (pow(Sampling[sample_count].X, 2) + pow(Sampling[sample_count].Y, 2) + pow(Sampling[sample_count].Z, 2));
		Sampling[sample_count].Magnitude = sqrt(Sampling[sample_count].Magnitude);
		/* Increment */
		sample_count++;
	}

	else {
		/* Rolling samples */
		Sampling[0].X = Sampling[NUM_SAMPLES-1].X;
		Sampling[0].Y = Sampling[NUM_SAMPLES-1].Y;
		Sampling[0].Z = Sampling[NUM_SAMPLES-1].Z;
		/* Reset count */
		sample_count = 1;
		/* Clear */
		memset(&Sampling[1], 0, sizeof(Sampling[0]) * (NUM_SAMPLES - 1));
	}

	/* FIFO */
	MagSamples[3] = MagSamples[2];
	MagSamples[2] = MagSamples[1];
	MagSamples[1] = MagSamples[0];
	/* Pushes sample_count back (1) after incremented */
	MagSamples[0] = Sampling[sample_count-1];
}
