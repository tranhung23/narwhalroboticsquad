/*
 * Control loop of the drone
 */

#include "user_INS.h"
#include <narwhal_ADC.h>
#include <narwhal_I2C.h>
#include <sensors.h>

OS_STK *narwhalINSStack;
#define DEBUG 1

//Shoudl be ready for analog and digital readings with a few changes
void INS_Init_Task(void)
{
	U64 timestamp = CoGetOSTime();
	StatusType result;
	U64 ticks = 0;
	sync_printf("Init INS starting \r\n");

	while (1)
	{
		//TODO: wait for UKF filter completation.

		/*New values values in in, update pitch yaw roll throttle*/
		if (CoAcceptSingleFlag(I2C1_FLAG) == E_OK)
		{
			OrientationValues.magx = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[1]) | (MARG_SENSORS[MAG]->SensorRawValue[0] << 8))));
			OrientationValues.magy = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[3]) | (MARG_SENSORS[MAG]->SensorRawValue[2] << 8))));
			OrientationValues.magz = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[5]) | (MARG_SENSORS[MAG]->SensorRawValue[4] << 8))));

#ifdef DEBUG
			//async_printf("mag x: %f, y: %f, z: %f, %lld\r\n", x, y, z, CoGetOSTime());
#endif
		}
		// Update the magnometer at 150 hz.
		if (ticks % 7 == 0)
		{
			timestamp = CoGetOSTime();
			Sensors_Read_async(MAG);
		}

		/*wait for ADC to finish processing*/
		result = CoWaitForSingleFlag(ADC_FLAG, 0);
		/*Convert all the ADC values to understandalbe values*/
		INS_Sensor_ValueCalibrate(ticks);

		/*PID Loop update process*/

	}
}

static void INS_Sensor_ValueCalibrate(U64 ticks)
{
	/*Current values are now in volts*/

	float ax, ay, az, gx, gy, gz, tempst;
	uint32_t currAvgIdx = ticks % AVERAGE_SAMPLES;

	ax = (double) FilteringADCData.adcSum[ACCX] * ADC_DIVISOR;
	ay = (double) FilteringADCData.adcSum[ACCY] * ADC_DIVISOR;
	az = (double) FilteringADCData.adcSum[ACCZ] * ADC_DIVISOR;
	gx = (double) FilteringADCData.adcSum[GYROX] * ADC_DIVISOR;
	gy = (double) FilteringADCData.adcSum[GYROY] * ADC_DIVISOR;
	gz = (double) FilteringADCData.adcSum[GYROZ] * ADC_DIVISOR;
	tempst = (double)FilteringADCData.adcSum[TEMPST] * ADC_DIVISOR_TEMP;



	//p[IMU_GYO_DEGLSB]

	OrientationValues.accx = ax + p[IMU_ACC_BIAS_X];
	OrientationValues.accy = ay + p[IMU_ACC_BIAS_Y];
	OrientationValues.accz = az + p[IMU_ACC_BIAS_Z];
	OrientationValues.gyrox = gx + p[IMU_GYO_BIAS_X];
	OrientationValues.gyroy = gy + p[IMU_GYO_BIAS_Y];
	OrientationValues.gyroz = gz + p[IMU_GYO_BIAS_Z];
	OrientationValues.temperatureST = (tempst - ADC_TEMP_REF)/ADC_MVC + ADC_TEMP_SHIFT;




#ifdef DEBUG
	async_printf("%f %f %f %f %f %f %f\r\n", OrientationValues.accx, OrientationValues.accy, OrientationValues.accz, OrientationValues.gyrox, OrientationValues.gyroy, OrientationValues.gyroz, OrientationValues.temperatureST);
	//async_printf("Acc x: %f, y: %f, z: %f, GYRO x: %f, y: %f, z: %f\r\n", OrientationValues.accx, OrientationValues.accy, OrientationValues.accz,OrientationValues.gyrox,OrientationValues.gyroy,OrientationValues.gyroz);
#endif
}

static void INS_Sensor_Calibrate(void)
{

}
