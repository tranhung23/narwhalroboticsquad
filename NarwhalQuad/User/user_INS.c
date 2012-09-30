/*
 * Control loop of the drone
 */

#include "user_INS.h"
#include <narwhal_ADC.h>
#include <narwhal_I2C.h>
#include <sensors.h>

OS_STK *narwhalINSStack;
#define DEBUG 1

struct INS_Orientation OrientationValues;

//Shoudl be ready for analog and digital readings with a few changes
void INS_Init_Task(void)
{
	StatusType result;
	U64 ticks = 0;
	sync_printf("Init INS starting \r\n");

	//TODO: Init task

	while (1)
	{
		//TODO: wait for UKF filter completation.

		// Update the magnometer at 150 hz.
		if (ticks % 7 == 0)
		{
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
	StatusType result;
	float ax, ay, az, gx, gy, gz, mx, my, mz, tempst;
	uint32_t currAvgIdx = ticks % AVERAGE_SAMPLES;

	/*New values values in in, update pitch yaw roll throttle*/
	if (CoAcceptSingleFlag(I2C1_FLAG) == E_OK)
	{
		/* use a union*/
		mx = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[1]) | (MARG_SENSORS[MAG]->SensorRawValue[0] << 8))));
		my = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[3]) | (MARG_SENSORS[MAG]->SensorRawValue[2] << 8))));
		mz = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[5]) | (MARG_SENSORS[MAG]->SensorRawValue[4] << 8))));

		/*TODO: Calculate temp biases*/
		mx = mx + p[IMU_MAG_BIAS_X];
		my = my + p[IMU_MAG_BIAS_Y];
		mz = mz + p[IMU_MAG_BIAS_Z];

		/*circular average buffer*/
		OrientationValues.avg_magx_sum -= OrientationValues.magx[currAvgIdx];
		OrientationValues.avg_magy_sum -= OrientationValues.magy[currAvgIdx];
		OrientationValues.avg_magz_sum -= OrientationValues.magz[currAvgIdx];

		OrientationValues.avg_magx_sum += mx;
		OrientationValues.avg_magy_sum += my;
		OrientationValues.avg_magz_sum += mz;

		OrientationValues.magx[currAvgIdx] = mx;
		OrientationValues.magy[currAvgIdx] = my;
		OrientationValues.magz[currAvgIdx] = mz;

		/*get the average of the current sample*/
		OrientationValues.avg_magx = OrientationValues.avg_magx_sum / AVERAGE_SAMPLES;
		OrientationValues.avg_magy = OrientationValues.avg_magy_sum / AVERAGE_SAMPLES;
		OrientationValues.avg_magz = OrientationValues.avg_magz_sum / AVERAGE_SAMPLES;

#ifdef DEBUG
		//async_printf("mag x: %f, y: %f, z: %f, %lld\r\n", x, y, z, CoGetOSTime());
#endif
	}

	ax = (double) FilteringADCData.adcSum[ACCX] * ADC_DIVISOR;
	ay = (double) FilteringADCData.adcSum[ACCY] * ADC_DIVISOR;
	az = (double) FilteringADCData.adcSum[ACCZ] * ADC_DIVISOR;
	gx = (double) FilteringADCData.adcSum[GYROX] * ADC_DIVISOR;
	gy = (double) FilteringADCData.adcSum[GYROY] * ADC_DIVISOR;
	gz = (double) FilteringADCData.adcSum[GYROZ] * ADC_DIVISOR;
	tempst = (double) FilteringADCData.adcSum[TEMPST] * ADC_DIVISOR_TEMP;

	//TODO: temperature calibration data
	/*now we have voltages, do temperature calibration*/
	ax = ax + p[IMU_ACC_BIAS_X];
	ay = ay + p[IMU_ACC_BIAS_Y];
	az = az + p[IMU_ACC_BIAS_Z];
	tempst = (tempst - ADC_TEMP_REF) / ADC_MVC + ADC_TEMP_SHIFT;

	/*subtract current average index from average values*/
	OrientationValues.avg_accx_sum -= OrientationValues.accx[currAvgIdx];
	OrientationValues.avg_accy_sum -= OrientationValues.accy[currAvgIdx];
	OrientationValues.avg_accz_sum -= OrientationValues.accz[currAvgIdx];
	OrientationValues.avg_temperatureST_sum -= OrientationValues.temperatureST[currAvgIdx];

	/*add current value to average*/
	OrientationValues.avg_accx_sum += ax;
	OrientationValues.avg_accy_sum += ay;
	OrientationValues.avg_accz_sum += az;
	OrientationValues.avg_temperatureST_sum += tempst;

	/*set current average index with current value*/
	OrientationValues.accx[currAvgIdx] = ax;
	OrientationValues.accy[currAvgIdx] = ay;
	OrientationValues.accz[currAvgIdx] = az;
	OrientationValues.temperatureST[currAvgIdx] = tempst;

	OrientationValues.avg_accx = OrientationValues.avg_accx_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_accy = OrientationValues.avg_accy_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_accz = OrientationValues.avg_accz_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_temperatureST = OrientationValues.avg_temperatureST_sum / AVERAGE_SAMPLES;

#ifdef DEBUG
	async_printf("%f %f %f %f %f %f %f\r\n", OrientationValues.accx, OrientationValues.accy, OrientationValues.accz, OrientationValues.gyrox, OrientationValues.gyroy, OrientationValues.gyroz, OrientationValues.temperatureST);
	//async_printf("Acc x: %f, y: %f, z: %f, GYRO x: %f, y: %f, z: %f\r\n", OrientationValues.accx, OrientationValues.accy, OrientationValues.accz,OrientationValues.gyrox,OrientationValues.gyroy,OrientationValues.gyroz);
#endif
}

static void INS_Sensor_Calibrate(void)
{

}
