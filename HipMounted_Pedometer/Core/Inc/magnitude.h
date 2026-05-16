
#ifndef INC_MAGNITUDEL_H_
#define INC_MAGNITUDE_H_

/* Includes */
#include <stdint.h>
#include "main.h"

/* Defines */
#define NUM_SAMPLES 80
#define NUM_MAGS 4
/* Externs */
extern ADXL335 IIR[NUM_SAMPLES];
extern ADXL335 Sampling[NUM_SAMPLES];
extern ADXL335 MagSamples[NUM_MAGS];
extern uint8_t sample_count;

/* Function Declarations */
void conditions_INIT();

void HPF_magnitiude_IT();

#endif /* INC_MAGNITUDE_H_ */
