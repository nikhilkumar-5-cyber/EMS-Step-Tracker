
#ifndef INC_MAGNITUDEL_H_
#define INC_MAGNITUDE_H_

/* Includes */

/* Defines */
#define NUM_SAMPLES 80
/* Externs */
typedef struct {
	double X;
	double Y;
	double Z;
	double Magnitude;
} MPU6050_Accelerometer;

extern MPU6050_Accelerometer IIR[NUM_SAMPLES];
extern MPU6050_Accelerometer Sampling[NUM_SAMPLES];
extern MPU6050_Accelerometer MagSamples[4];
extern uint8_t sample_count;

/* Function Declarations */
void conditions_INIT();

void HPF_magnitiude_IT();

#endif /* INC_MAGNITUDE_H_ */
