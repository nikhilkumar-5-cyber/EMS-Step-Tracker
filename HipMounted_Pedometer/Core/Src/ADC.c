/*
 * ADC.c
 *
 *  Created on: 23 May 2026
 *      Author: Nikhi
 */

/* Includes */
#include "ADC.h"
#include "step_counting.h"

/* Variable Definitions */
int isADCFinished = 0;
const float STM_res = 4095; // STM resolution
const double refV = 3.3; //[V]
const double sensitivity = 0.34; // [V/g] 330 mV/g
const double zero_gBias = STM_res/2; // ADC value at 0g [max ADC value divided by 2]
const double ADC_per_gVal = (sensitivity/refV)*(STM_res); // The ADC value between a difference in 1 g - [(sensitivity/ref voltage)*(STM resolution)]

/* Function Implementations */
void getValues(void) {
	get_ADC_Values();
	/* Adjusts values based on calibrated values */
	double x_g = ADC_to_g(ADC_VAL[0]);
	double y_g = ADC_to_g(ADC_VAL[1]);
	double z_g = ADC_to_g(ADC_VAL[2]);

	RAW_SAMPLE.X = x_g + (x_g < 0 ? adjVal[1][0] : adjVal[0][0]);
	RAW_SAMPLE.Y = y_g + (y_g < 0 ? adjVal[1][1] : adjVal[0][1]);
	RAW_SAMPLE.Z = z_g + (z_g < 0 ? adjVal[1][2] : adjVal[0][2]);
	double magSquared = pow(RAW_SAMPLE.X, 2) + pow(RAW_SAMPLE.Y, 2) + pow(RAW_SAMPLE.Z, 2);
	RAW_SAMPLE.magnitude = pow(magSquared, 0.5);
	/* pushes last sample into sample buffer */
	pushFront(SAMPLE_BUFFER, NUM_SAMPLES, RAW_SAMPLE);
	indexVal++;
}

double ADC_to_V(uint32_t ADC_val) {
	/* Converts ADC value to V */
	double converted_val = ADC_val*(refV/STM_res);
	return converted_val;
}

double ADC_to_g(uint32_t ADC_val) {
	/* Converts ADC value to g */
	double gVal = (ADC_val-zero_gBias)/ADC_per_gVal;
	return gVal;
}

double g_to_ADC(double g_val) {
	/* Converts g value to ADC */
	float ADCval;
	if (g_val > 0) {
		ADCval = zero_gBias + (ADC_per_gVal*g_val);
	}
	if (g_val < 0) {
		ADCval = zero_gBias - (ADC_per_gVal*g_val);
	}
	return ADCval;
}

void get_ADC_Values(void) {
	/* Function reads the ADC values of X, Y and Z and puts it in the ADC_VAL array */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 3);
	while (isADCFinished != 1) {}
	isADCFinished = 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	/* Calls when the all ADC values are stored */
	isADCFinished = 1;
}
