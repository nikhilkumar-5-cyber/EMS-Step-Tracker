
#ifndef INC_MAGNITUDEL_H_
#define INC_MAGNITUDE_H_

/* Includes */
#include <stdint.h>
#include "main.h"

/* Defines */

/* Externs */

/* Function Declarations */
void compute_Magnitude(ADXL335 *sample); //Dynamically computes magnitude for every sample; circular buffer of NUM_SAMPLES

#endif /* INC_MAGNITUDE_H_ */
