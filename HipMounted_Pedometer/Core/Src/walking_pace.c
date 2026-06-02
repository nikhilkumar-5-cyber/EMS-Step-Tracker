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
WalkingPace pace = STATIC; // Stores the current walking pace
volatile uint16_t prevStepCount; // Stores the previous step count
volatile uint32_t prevTime; // last time when pace was calculated
const float timeGap = 0.1; // Difference in time before getting new step count [s]
const int walkingFreqMax = 3; // FIX | Stores the max walking frequency (steps per second)

/* Function Implementations */
void walkingPace(void) {
	/* Only check walking pace every 100 ms */
	if (HAL_GetTick()-prevTime < (timeGap*1e3)) {
		return;
	}
	prevTime = HAL_GetTick();

	volatile double stepFrequency = ((stepCount-prevStepCount)/timeGap)*10; // gets steps per second
	prevStepCount = stepCount;

	/* Turn OFF all LEDS */
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET); // Stationary LED
	HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET); // Walking LED
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET); // Running LED

	/* Compare calculated stepFrequency to get walking pace */
	if (stepFrequency <= 0) {
		/* Set pace as Static and turn on its respective LED */
		pace = STATIC;
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	}
	else if (stepFrequency <= walkingFreqMax) {
		/* Set pace as Walking and turn on its respective LED */
		pace = WALKING;
		HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
	}
	else {
		/* Set pace as Running and turn on its respective LED */
		pace = RUNNING;
		HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET); // flashing
	}
}
