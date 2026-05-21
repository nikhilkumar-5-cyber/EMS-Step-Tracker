/*
 * OLED_format.h
 *
 *  Created on: 19 May 2026
 *      Author: Nikhil
 */

#ifndef INC_OLED_FORMAT_H_
#define INC_OLED_FORMAT_H_

/* Includes */
#include "stdbool.h"
#include "ssd1306.h"

/* Function Declarations */

void DEFAULT_DISPLAY(void);
void display_STEP(void);
void display_DISTANCE(void);
void display_WALKINGPACE(void);
void ST_DISPLAY(bool pass);
void Cali_Start_Display(void);
void Cali_Display(bool isNegative, uint16_t direction);
void Cali_Error_Display(void);
void Cali_Finished_Display(void);

/**
 * @brief Helper Function To display just a line on the OLED, automatically resets and updates screen
 */
void Display(char* str, SSD1306_Font_t Font);

/**
 * @brief Helper Function To display a arrow based on direction (+-X, +-Y, +-Z)
 */
void Arrow_Display(bool isNegative, uint16_t direction);

#endif /* INC_OLED_FORMAT_H_ */
