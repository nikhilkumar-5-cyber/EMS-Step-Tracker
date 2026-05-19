/*
 * OLED_format.h
 *
 *  Created on: 19 May 2026
 *      Author: Nikhil
 */

#ifndef INC_OLED_FORMAT_H_
#define INC_OLED_FORMAT_H_

/* Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdbool.h"
#include <inttypes.h>
#include <stdio.h>

/* Function Declarations */

void DEFAULT_DISPLAY(void);
void display_STEP(void);
void display_WALKINGPACE(void);
void ST_DISPLAY(bool pass);


#endif /* INC_OLED_FORMAT_H_ */
