/*
 * calibration.h
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

/* Includes */
#include "stdbool.h"

/* Function Declarations */

/*
 * @brief Calibrates all six directions
 * */
void calibration(void);

/*
 * @brief Helper function to calibrate X,Y and Z values based on direction (+/-)
 * */
void calibrationLoop(bool forPositive);

#endif /* INC_CALIBRATION_H_ */
