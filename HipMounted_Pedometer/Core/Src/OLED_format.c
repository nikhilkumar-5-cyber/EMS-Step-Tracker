/*
 * OLED_format.c
 *
 *  Created on: 19 May 2026
 *      Author: Nikhil
 */

/* Includes */
#include "OLED_format.h"
#include "ssd1306_fonts.h"
#include <inttypes.h>
#include <stdio.h>
#include "main.h"
#include "walking_pace.h"

/* Variable Definitions */
char STEPS_TAKEN[5];
char DISTANCE[8];
char WALKING_PACE[15];
char CALIBRATION[30];

const char* paceStrings[] = {"STATIC", "WALKING", "RUNNING"}; // To get the string for the Pace
const char* directionStrings[] = {"X", "Y", "Z"};

/* Function Implementations */
void DEFAULT_DISPLAY(void) {
	ssd1306_Fill(Black);
	// Constant char
	char STEP_text[] = "Total Steps: ";
	char DIST_text[] = "Distance: ";
	char PACE_text[] = "Pace: ";

	// Display Step Count Title
	ssd1306_SetCursor(5, 5);
	ssd1306_WriteString(STEP_text, Font_7x10, White);
	//Display Distance Title
	ssd1306_SetCursor(5, 30);
	ssd1306_WriteString(DIST_text, Font_7x10, White);
	// Display Walking Pace Title
	ssd1306_SetCursor(5, 60);
	ssd1306_WriteString(PACE_text, Font_7x10, White);

	ssd1306_UpdateScreen();
}

void display_STEP(void) {
	snprintf(STEPS_TAKEN, 5, PRIu16, stepCount);
	ssd1306_SetCursor(86, 5);
	ssd1306_WriteString(STEPS_TAKEN, Font_7x10, White);
	ssd1306_UpdateScreen();
}

void display_DISTANCE(void) {
	// FIX: - need global variable for distance
	snprintf(DISTANCE, sizeof(DISTANCE), "%d m", distanceTravelled);
	ssd1306_SetCursor(86, 30);
	ssd1306_WriteString(DISTANCE, Font_7x10, White);
	ssd1306_UpdateScreen();
}

void display_WALKINGPACE(void) {
	snprintf(WALKING_PACE, sizeof(WALKING_PACE), paceStrings[pace]);
	ssd1306_SetCursor(86, 60);
	ssd1306_WriteString(WALKING_PACE, Font_7x10, White);
	ssd1306_UpdateScreen();
}

void ST_DISPLAY(bool pass) {
	ssd1306_Fill(Black);
	// Constant char
	char ST_pass_text[] = "Passed ST Protocol";
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

void Cali_Start_Display(void) {
	char start_text[] = "Beginning Calibration";
	Display(start_text, Font_7x10);
}

void Cali_Display(bool isNegative, uint16_t direction) {
	if (!isNegative) {
		snprintf(CALIBRATION, sizeof(CALIBRATION), "Face the Arrow Down for +%s", directionStrings[direction]);
	}
	else {
		snprintf(CALIBRATION, sizeof(CALIBRATION), "Face the Arrow Down for -%s", directionStrings[direction]);
	}
}
void Cali_Error_Display(void) {
	char Cali_Error_text[] = "Calibration Failed";
	Display(Cali_Error_text, Font_7x10);
}

void Cali_Finished_Display(void) {
	char Cali_Finished_text[] = "Calibration Completed";
	Display(Cali_Finished_text, Font_7x10);
}

void Display(char* str, SSD1306_Font_t Font) {
	ssd1306_Fill(Black);
	ssd1306_SetCursor(5, 5);
	ssd1306_WriteString(str, Font, White);
	ssd1306_UpdateScreen();
}
