/*
 * OLED_format.c
 *
 *  Created on: 19 May 2026
 *      Author: Nikhil
 */

/* Includes */
#include "OLED_format.h"

/* Variable Definitions */
char STEPS_TAKEN[5];
char WALKING_PACE[10];

/* Function Implementations */
void DEFAULT_DISPLAY() {
	ssd1306_Reset();
	// Constant char
	char STEP_text[] = "Total Steps: ";
	char PACE_text[] = "Walking Pace: ";

	// Display Step Count Title
	ssd1306_SetCursor(5, 5);
	ssd1306_WriteString(STEP_text, Font_7x10, White);
	// Display Walking Pace Title
	ssd1306_SetCursor(36, 40);
	ssd1306_WriteString(PACE_text, Font_7x10, White);
	ssd1306_UpdateScreen();
}

void display_STEP() {
	snprintf(STEPS_TAKEN, 5,"%", PRIu16, TOTAL_STEPS);
	ssd1306_SetCursor(86, 5);
	ssd1306_WriteString(STEPS_TAKEN, Font_7x10, White);
	ssd1306_UpdateScreen();
}

void display_WALKINGPACE() {
	snprintf(WALKING_PACE, sizeof(WALKING_PACE),"%s",);
	ssd1306_SetCursor(86, 40);
	ssd1306_WriteString(WALKING_PACE, Font_7x10, White);
	ssd1306_UpdateScreen();
}

void ST_DISPLAY(bool pass) {
	ssd1306_Reset();
	// Constant char
	char ST_pass_text[] = "Failed ST Protocol";
	char ST_fail_text[] = "Failed ST Protocol";

	// Display if ST protocol has passed
	ssd1306_SetCursor(5, 5);
	if (pass) {
		ssd1306_WriteString(ST_pass_text, Font_7x10, White);
	}
	else {
		ssd1306_WriteString(ST_fail_text, Font_7x10, White);
	}
	ssd1306_UpdateScreen();
}
