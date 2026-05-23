/*
 * ADC.h
 *
 *  Created on: 23 May 2026
 *      Author: Nikhi
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

/* Includes */
#include "main.h"

/* Externs */
extern ADC_HandleTypeDef hadc1;
extern const float STM_res; // STM resolution
extern const float refV; //[V]
extern const float sensitivity;// [V/g] 330 mV/g
extern const float zero_gBias; // ADC value at 0g [max ADC value divided by 2]
extern const float ADC_per_gVal; // The ADC value between a difference in 1 g - [(sensitivity/ref voltage)*(STM resolution)]

/* Function Declarations */
void getValues(void);
int ADC_to_V(uint32_t ADC_val);
int g_to_ADC(float g_val);
void get_ADC_Values(void);


#endif /* INC_ADC_H_ */
