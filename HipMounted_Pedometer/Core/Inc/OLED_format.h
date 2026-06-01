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

/* Functions for Default UI/Display */

/*
 * @brief Default UI that sets the titles for information that will be display
 * */
void DEFAULT_DISPLAY(void);

/*
 * @brief Updates Default display with real time values
 * */
void UPDATE_DEFAULT_DISPLAY(void);

/*
 * @brief Displays the number of steps taken next to its title
 * */
void display_STEP(void);

/*
 * @brief Displays the distance next to its title
 * */
void display_DISTANCE(void);

/*
 * @brief Displays the walking pace next to its title
 * */
void display_WALKINGPACE(void);

/* Functions for Self-Test protcol */
void ST_DISPLAY(bool pass);

/*Functions for Calibration Display  */

/*
 * @brief Initial message when entering calibration mode
 * */
void Cali_Start_Display(void);

/*
 * @brief Display that tells the user to point the pedometer the right direction
 * */
void Cali_Display(bool isNegative, uint16_t direction);

/*
 * @brief Error Message when it isn't pointed in the right direction
 * */
void Cali_Error_Display(void);

/*
 * @brief Message when calibration finishes
 * */
void Cali_Finished_Display(void);

/* Helper Functions */

/**
 * @brief Helper Function To display just a line on the OLED, automatically resets and updates screen
 */
void Display(char* str, SSD1306_Font_t Font);

/**
 * @brief Helper Function To display a arrow based on direction (+-X, +-Y, +-Z)
 */
void Arrow_Display(bool isNegative, uint16_t direction);

#endif /* INC_OLED_FORMAT_H_ */
