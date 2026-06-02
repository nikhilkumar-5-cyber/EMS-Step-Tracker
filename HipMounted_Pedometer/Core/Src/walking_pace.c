/*
 * walking_pace.c
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

/* Includes */
#include "main.h"
#include "walking_pace.h"
#include "step_counting.h"
#include "distance_est.h"

/* Variable Definitions */
WalkingPace pace = STATIC; // Stores the current walking pace
volatile uint16_t prevStepCount; // Stores the previous step count
volatile uint32_t prevTime; // last time when pace was calculated
volatile uint32_t prevTimeBlink; // Last time the LED was toggled
const float timeGap = 2; // Difference in time before getting new step count [s]
const int walkingFreqMax = 110; // FIX | Stores the max walking frequency (steps per minute)
const double runningThreshold= 1.2;

/* Function Implementations */
void walkingPace(void) {
	/* Only check walking pace every 100 ms */
	if (HAL_GetTick()-prevTime < (timeGap*1e3)) {
		return;
	}
	prevTime = HAL_GetTick();

	volatile double stepFrequency = ((stepCount-prevStepCount)/timeGap)*30; // gets steps per minute
	prevStepCount = stepCount;

	/* Turn OFF all LEDS */
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET); // Stationary LED
	HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET); // Walking LED
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET); // Running LED

	/* Compare calculated stepFrequency to get walking pace */
	if (lastSamples[0].magnitude > runningThreshold &&
		lastSamples[1].magnitude > runningThreshold &&
		lastSamples[2].magnitude > runningThreshold)
	{
		/* Set pace as Running and turn on its respective LED */
		pace = RUNNING;
		if (HAL_GetTick() - prevTimeBlink >= 250) {
			HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin); // flashing
			prevTimeBlink = HAL_GetTick();
		}
	}
	else if (stepFrequency <= 0) {
		/* Set pace as Static and turn on its respective LED */
		pace = STATIC;
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else if (stepFrequency <= walkingFreqMax) {
		/* Set pace as Walking and turn on its respective LED */
		pace = WALKING;
		HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
	}
	else {}
}
