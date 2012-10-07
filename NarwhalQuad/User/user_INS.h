
#ifndef user_INS_h
#define user_INS_h


#include <narwhal_top.h>
#include <ukf.h>

extern OS_STK *narwhalINSStack;

#define MARG_NUM	9	/*number of sensors*/

/*Gyroscope/acclemeter array values in ADC table*/
#define GYROX		0
#define GYROY		1
#define GYROZ		2
#define ACCX		3
#define ACCY		4
#define ACCZ		5

#define TEMPST		6

#define AVERAGE_SAMPLES	12
#define TEMP_CALIBRATION_SAMPLE 25 /*100 samples*/

#define STEADY_STATE_STD_ACC 0.00050f

typedef struct INS_Orientation
{
	float gyrox[AVERAGE_SAMPLES];
	float gyroy[AVERAGE_SAMPLES];
	float gyroz[AVERAGE_SAMPLES];
	float accx[AVERAGE_SAMPLES];
	float accy[AVERAGE_SAMPLES];
	float accz[AVERAGE_SAMPLES];
	float magx[AVERAGE_SAMPLES];
	float magy[AVERAGE_SAMPLES];
	float magz[AVERAGE_SAMPLES];
	float temperatureST[AVERAGE_SAMPLES];

	float avg_gyrox_sum;
	float avg_gyroy_sum;
	float avg_gyroz_sum;
	float avg_accx_sum;
	float avg_accy_sum;
	float avg_accz_sum;
	float avg_magx_sum;
	float avg_magy_sum;
	float avg_magz_sum;
	float avg_temperatureST_sum;

	float avg_gyrox;
	float avg_gyroy;
	float avg_gyroz;
	float avg_accx;
	float avg_accy;
	float avg_accz;
	float avg_magx;
	float avg_magy;
	float avg_magz;
	float avg_temperatureST;

	uint32_t dt;
	//uint32_t lastSample;

} INS_Orientation;

typedef struct Calibration_SampleSpace
{
	float calibration_temperature;
	uint32_t calibration_samplecnt;
	float gyro_x_samplespace[AVERAGE_SAMPLES*TEMP_CALIBRATION_SAMPLE];
	float gyro_y_samplespace[AVERAGE_SAMPLES*TEMP_CALIBRATION_SAMPLE];
	float gyro_z_samplespace[AVERAGE_SAMPLES*TEMP_CALIBRATION_SAMPLE];

} Calibration_SampleSpace;

extern INS_Orientation OrientationValues;
extern void INS_Sensor_ValueCalibrate(U64 ticks);
extern void INS_Sensor_Static(void);

void INS_Init_Task(void);

#endif
