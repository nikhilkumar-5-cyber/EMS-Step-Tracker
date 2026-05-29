/*
 * walking_pace.h
 *
 *  Created on: Mar 11, 2026
 *      Author: noah
 */

#ifndef INC_WALKING_PACE_H_
#define INC_WALKING_PACE_H_

/* Includes */

/* Externs */
typedef enum {
	STATIC,
	WALKING,
	RUNNING
} WalkingPace;

extern WalkingPace pace;

/* Function Declarations */
void walkingPace(void);

#endif /* INC_WALKING_PACE_H_ */
