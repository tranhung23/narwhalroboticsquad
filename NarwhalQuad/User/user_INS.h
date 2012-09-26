#include <narwhal_top.h>

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


typedef struct INS_Orientation{
	float gyrox;
	float gyroy;
	float gyroz;
	float accx[AVERAGE_SAMPLES];
	float accy[AVERAGE_SAMPLES];
	float accz[AVERAGE_SAMPLES];
	float magx[AVERAGE_SAMPLES];
	float magy[AVERAGE_SAMPLES];
	float magz[AVERAGE_SAMPLES];
	float temperatureST[AVERAGE_SAMPLES];

	float avg_accx;
	float avg_accy;
	float avg_accz;
	float avg_magx;
	float avg_magy;
	float avg_magz;
	float avg_temperatureST;






}INS_Orientation;

static INS_Orientation OrientationValues;
static void INS_Sensor_ValueCalibrate(U64 ticks);

void INS_Init_Task(void);