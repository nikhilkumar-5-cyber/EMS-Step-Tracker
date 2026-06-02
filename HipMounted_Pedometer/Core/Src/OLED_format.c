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
#include "distance_est.h"
#include "step_counting.h"

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
	/* Constant chars */
	char STEP_text[] = "Total Steps: ";
	char DIST_text[] = "Distance: ";
	char PACE_text[] = "Pace: ";

	/* Display Step Count Title */
	ssd1306_SetCursor(5, 5);
	ssd1306_WriteString(STEP_text, Font_6x8, White);
	display_STEP();
	/* Display Distance Title */
	ssd1306_SetCursor(5, 25);
	ssd1306_WriteString(DIST_text, Font_6x8, White);
	display_DISTANCE();
	/* Display Walking Pace Title */
	ssd1306_SetCursor(5, 45);
	ssd1306_WriteString(PACE_text, Font_6x8, White);
	display_WALKINGPACE();
	ssd1306_UpdateScreen();
}

void UPDATE_DEFAULT_DISPLAY(void) {
	DEFAULT_DISPLAY();
	display_STEP();
	display_DISTANCE();
	display_WALKINGPACE();
	ssd1306_UpdateScreen();
}

void display_STEP(void) {
	/* Combine step count into the char array and then display it */
	snprintf(STEPS_TAKEN, 5, "%" PRIu32, stepCount);
	ssd1306_SetCursor(86, 5);
	ssd1306_WriteString(STEPS_TAKEN, Font_6x8, White);
}

void display_DISTANCE(void) {
	/* Combine distance into the char array and then display it */
	// FIX: - need global variable for distance
	snprintf(DISTANCE, sizeof(DISTANCE), "%.1fm", distanceTravelled); //truncates to 1dp
	ssd1306_SetCursor(65, 25);
	ssd1306_WriteString(DISTANCE, Font_6x8, White);
}

void display_WALKINGPACE(void) {
	/* Combine walking pace into the char array and then display it */
	snprintf(WALKING_PACE, sizeof(WALKING_PACE), paceStrings[pace]);
	ssd1306_SetCursor(40, 45);
	ssd1306_WriteString(WALKING_PACE, Font_6x8, White);
}

void ST_DISPLAY(bool pass) {
	ssd1306_Fill(Black);
	/* Constant chars */
	char ST_pass_text[] = "Passed ST Protocol\n";
	char ST_fail_text[] = "Failed ST Protocol\n";

	/* Display if ST protocol has passed or failed */
	ssd1306_SetCursor(5, 5);
	if (pass) {
		ssd1306_WriteString(ST_pass_text, Font_6x8, White);
		HAL_UART_Transmit(&huart2, (uint8_t *)ST_pass_text, sizeof(ST_pass_text), HAL_MAX_DELAY); // Testing
	}
	else {
		ssd1306_WriteString(ST_fail_text, Font_6x8, White);
		HAL_UART_Transmit(&huart2, (uint8_t *)ST_fail_text, sizeof(ST_fail_text), HAL_MAX_DELAY); // Testing
	}
	ssd1306_UpdateScreen();
}

void Cali_Start_Display(void) {
	char start_text[] = "Calibration Starting";
	Display(start_text, Font_6x8);
	HAL_UART_Transmit(&huart2, (uint8_t *)start_text, sizeof(start_text), HAL_MAX_DELAY); // Testing
}

void Cali_Display(bool isNegative, uint16_t direction) {
	/* Display correct direction */
	ssd1306_Fill(Black);
	ssd1306_SetCursor(5, 5);
	ssd1306_WriteString("Face the Arrow Down", Font_6x8, White);
	if (!isNegative) {
		snprintf(CALIBRATION, sizeof(CALIBRATION), "for +%s", directionStrings[direction]);
	}
	else {
		snprintf(CALIBRATION, sizeof(CALIBRATION), "for -%s", directionStrings[direction]);
	}
	ssd1306_SetCursor(5, 15);
	ssd1306_WriteString(CALIBRATION, Font_6x8, White);
	HAL_UART_Transmit(&huart2, (uint8_t *)CALIBRATION, sizeof(CALIBRATION), HAL_MAX_DELAY); // Testing

	/* Display Arrow */
	Arrow_Display(isNegative, direction);
}
void Cali_Error_Display(void) {
	char Cali_Error_text[] = "Calibration Failed";
	Display(Cali_Error_text, Font_7x10);
	HAL_UART_Transmit(&huart2, (uint8_t *)Cali_Error_text, sizeof(Cali_Error_text), HAL_MAX_DELAY); // Testing
}

void Cali_Finished_Display(void) {
	char Cali_Finished_text[] = "Calibration Completed";
	Display(Cali_Finished_text, Font_6x8);
	HAL_UART_Transmit(&huart2, (uint8_t *)Cali_Finished_text, sizeof(Cali_Finished_text), HAL_MAX_DELAY); // Testing
}

void Display(char* str, SSD1306_Font_t Font) {
	ssd1306_Fill(Black);
	ssd1306_SetCursor(5, 5);
	ssd1306_WriteString(str, Font, White);
	ssd1306_UpdateScreen();
}

void Arrow_Display(bool isNegative, uint16_t direction) {
	/* Display Arrow */
	uint8_t x1, y1, x2, y2;

	switch (direction) {
	case 0: // X
		// Draw a vertical Line
		x1 = 64; y1 = 25;
		x2 = 64; y2 = 55;
		ssd1306_SetCursor(25, 50); // For arrow body
		ssd1306_Line(x1, y1, x2, y2, White);
		if (isNegative) {
			ssd1306_SetCursor(x1-3, y2);
			ssd1306_Line(60, 55, 64, 60, White);
			ssd1306_Line(64, 60, 68, 55, White);
		}
		else {
			ssd1306_SetCursor(x2-3, y1);
			ssd1306_Line(60, 25, 64, 30, White);
			ssd1306_Line(64, 30, 68, 25, White);
		}
		break;
	case 1: // Y
		// Draw a Horizontal Line
		x1 = 25; y1 = 40;
		x2 = 55; y2 = 40;
		ssd1306_SetCursor(25, 50); // For arrow body
		ssd1306_Line(x1, y1, x2, y2, White);
		if (isNegative) {
			ssd1306_SetCursor(x2, y2-3);
			ssd1306_WriteString(">", Font_6x8, White);
		}
		else {
			ssd1306_SetCursor(x1, y2-3);
			ssd1306_WriteString("< ", Font_6x8, White);
		}
		break;
	case 2: // Z
		// Draw a Horizontal Line
		if (!isNegative) {
			ssd1306_SetCursor(15, 40);
			ssd1306_WriteString("Face Screen Up", Font_6x8, White);
		}
		else {
			ssd1306_SetCursor(15, 40);
			ssd1306_WriteString("Face Screen Down", Font_6x8, White);
		}
		break;
	}
	ssd1306_UpdateScreen();
}
