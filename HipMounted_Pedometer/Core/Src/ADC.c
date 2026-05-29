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
const float refV = 3.3; //[V]
const float sensitivity = 0.33; // [V/g] 330 mV/g
const float zero_gBias = STM_res/2; // ADC value at 0g [max ADC value divided by 2]
const float ADC_per_gVal = (sensitivity/refV)*(STM_res); // The ADC value between a difference in 1 g - [(sensitivity/ref voltage)*(STM resolution)]

/* Function Implementations */
void getValues(void) {
	get_ADC_Values();
	/* Adjusts values based on calibrated values */
	for (int i=0; i<3; i++) {
		/* Checks if the g-val is positive or negative and then adjust value from calibrated values */
		if (ADC_to_g(ADC_VAL[i])< 0) {
			ADC_VAL[i] = ADC_VAL[i]+adjVal[1][i];
		}
		else {
			ADC_VAL[i] = ADC_VAL[i]-adjVal[1][i];
		}
	}
	/* Convert the X, Y and Z to g Values and Store them */
	RAW_SAMPLE.X = ADC_to_g(ADC_VAL[0]);
	RAW_SAMPLE.Y = ADC_to_g(ADC_VAL[1]);
	RAW_SAMPLE.Z = ADC_to_g(ADC_VAL[2]);
	/* pushes last sample into sample buffer */
	pushFront(SAMPLE_BUFFER, NUM_SAMPLES, RAW_SAMPLE);
	indexVal++;
}

int ADC_to_V(uint32_t ADC_val) {
	/* Converts ADC value to V */
	int converted_val = ADC_val*(refV/STM_res);
	return converted_val;
}

float ADC_to_g(uint32_t ADC_val) {
	/* Converts ADC value to g */
	float gVal = (ADC_to_V(ADC_val)-zero_gBias)/sensitivity;
	return gVal;
}

int g_to_ADC(float g_val) {
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
