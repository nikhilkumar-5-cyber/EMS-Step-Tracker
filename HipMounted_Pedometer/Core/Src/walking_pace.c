/*
 * walking_pace.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

/* Includes */
#include "main.h"
#include "walking_pace.h"

/* Variable Definitions */
WalkingPace pace = STATIC;
volatile uint16_t prevStepCount; // Stores the previous step count
volatile uint32_t prevTime; // last time when pace was calculated
const float timeGap = 0.1; // Difference in time before getting new step count [s]
const int walkingFreqMax = 0; // FIX

/* Function Implementations */
void walkingPace(void) {
	if (HAL_GetTick()-prevTime < (timeGap*1e3)) {
		return;
	}
	prevTime = HAL_GetTick();
	volatile int stepFrequency = ((stepCount-prevStepCount)/timeGap)*10; // gets steps per second
	prevStepCount = stepCount;
	if (stepFrequency == 0) { // Checks if there hasn't been movement
		pace = STATIC;
		// Display Walking Pace as "Static/Stationary"
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Yellow LED (stationary)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Orange LED (walking)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // RED LED (running)
	}
	else if (stepFrequency < walkingFreqMax) { // Checks if Pace is walking
		pace = WALKING;
		// Display Walking Pace as "Static/Stationary"
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED (stationary)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); // Orange LED (walking)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // RED LED (running)
	}
	else { // Pace is running
		pace = RUNNING;
		// Display Walking Pace as "Static/Stationary"
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED (stationary)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Orange LED (walking)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // RED LED (running)

	}
}
