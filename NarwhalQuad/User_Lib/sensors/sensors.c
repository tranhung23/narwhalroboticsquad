/*
 * Sensors.c
 *
 *  Created on: Mar 8, 2012
 *      Author: GrubyGrub
 */

//TODO: This file is a mess, it needs to be re-written

#include <narwhal_I2C.h>

#include "sensors.h"
#include "../sensors/ITG3200.h"
#include "../sensors/LSM303.h"
#include "../sensors/L3G3200.h"

#include "../sensors/ADXL345.h"
#include "../sensors/HMC5883L.h"


static MARG_SENSOR_TypeDef Magnetometer;
static MARG_SENSOR_TypeDef Acclermeter;
static MARG_SENSOR_TypeDef Gyroscope;
volatile int CurrentSensor = -1;

MARG_SENSOR_TypeDef * MARG_SENSORS[SENSORn] = { &Magnetometer, &Acclermeter, &Gyroscope };

void Sensors_Init(uint8_t SensorType)
{
	switch (SensorType)
	{
	/*ITG-3200 GYRO*/
	case GYRO:

		MARG_SENSORS[GYRO]->ReadLength = 6;
		MARG_SENSORS[GYRO]->SensorType = GYRO;
		MARG_SENSORS[GYRO]->SensorValueUpdated = 0;

		/*ITG3200*/

		I2C_WriteDeviceRegister(I2C_COM1, ITG3200_ADDRESS, PWR_MGM_ITG, 0x80, SETTING_DATA_LENGTH);
		I2C_WriteDeviceRegister(I2C_COM1, ITG3200_ADDRESS, SMPLRT_DIV_ITG, 0x00, SETTING_DATA_LENGTH);
		I2C_WriteDeviceRegister(I2C_COM1, ITG3200_ADDRESS, DLPF_FS_ITG, 0x1C, SETTING_DATA_LENGTH); /* 5 hz low pass filter*/
		I2C_WriteDeviceRegister(I2C_COM1, ITG3200_ADDRESS, INT_CFG_ITG, 0x05, SETTING_DATA_LENGTH);
		/*
		 I2C_WriteDeviceRegister(I2C_COM1,L3G3200_ADDRESS,CTRL_REG4_G,0x10,SETTING_DATA_LENGTH);
		 I2C_WriteDeviceRegister(I2C_COM1,L3G3200_ADDRESS,CTRL_REG1_G,0xFF,SETTING_DATA_LENGTH);
		 */
		break;

	case ACC:

		MARG_SENSORS[ACC]->ReadLength = 6;
		MARG_SENSORS[ACC]->SensorType = ACC;
		MARG_SENSORS[ACC]->SensorValueUpdated = 0;

		/*ADXL345*/
		I2C_WriteDeviceRegister(I2C_COM1, ADXL345_ADDRESS, Register_DataFormat, 0x0B, SETTING_DATA_LENGTH);/*16 G*/
		I2C_WriteDeviceRegister(I2C_COM1, ADXL345_ADDRESS, Register_DataBW, 0xD, SETTING_DATA_LENGTH);
		I2C_WriteDeviceRegister(I2C_COM1, ADXL345_ADDRESS, Register_PowerControl, 0x00, SETTING_DATA_LENGTH);
		I2C_WriteDeviceRegister(I2C_COM1, ADXL345_ADDRESS, Register_PowerControl, 0x10, SETTING_DATA_LENGTH);
		I2C_WriteDeviceRegister(I2C_COM1, ADXL345_ADDRESS, Register_PowerControl, 0x08, SETTING_DATA_LENGTH);

		/*LSM303 Acclemeter*/
		//I2C_WriteDeviceRegister(I2C_COM1,LSM303_ACC_ADDRESS,CTRL_REG4_A,0x0,SETTING_DATA_LENGTH);
		//I2C_WriteDeviceRegister(I2C_COM1,LSM303_ACC_ADDRESS,CTRL_REG1_A,0x3F,SETTING_DATA_LENGTH);
		break;

		/*LSM303 MAG*/
	case MAG:

		MARG_SENSORS[MAG]->ReadLength = 6;
		MARG_SENSORS[MAG]->SensorType = MAG;
		MARG_SENSORS[MAG]->SensorValueUpdated = 0;

		//I2C_WriteDeviceRegister(I2C_COM1,LSM303_MAG_ADDRESS,CRA_REG_M,0x18,SETTING_DATA_LENGTH);
		//I2C_WriteDeviceRegister(I2C_COM1,LSM303_MAG_ADDRESS,CRB_REG_M,0x20,SETTING_DATA_LENGTH);
		//I2C_WriteDeviceRegister(I2C_COM1,LSM303_MAG_ADDRESS,MR_REG_M,0x0,SETTING_DATA_LENGTH);

		I2C_WriteDeviceRegister(I2C_COM1, HMC5883L_ADDRESS, MODE_REG_M, 0x00, SETTING_DATA_LENGTH); //cont measurement
		I2C_WriteDeviceRegister(I2C_COM1, HMC5883L_ADDRESS, CTRL_REG1_M, 0x18, SETTING_DATA_LENGTH); // max hertz
		I2C_WriteDeviceRegister(I2C_COM1, HMC5883L_ADDRESS, CTRL_REG2_M, 0x00, SETTING_DATA_LENGTH); // max gain

		break;
	}
}

void Sensors_Read(uint8_t SensorType)
{
	int g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z;
	switch (SensorType)
	{
	case GYRO:
		I2C_ReadDeviceRegister(I2C_COM1, ITG3200_ADDRESS, GYRO_XOUT_H_ITG | (1 << 7), MARG_SENSORS[GYRO]->ReadLength, (uint32_t) MARG_SENSORS[GYRO]->SensorRawValue);
		//I2C_ReadDeviceRegister_async(I2C_COM1,L3G3200_ADDRESS, OUT_X_L_G | (1 << 7) ,MARG_SENSORS[GYRO]->ReadLength,(uint32_t)MARG_SENSORS[GYRO]->SensorRawValue);

		g_x = ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[1]) | (MARG_SENSORS[GYRO]->SensorRawValue[0] << 8))));
		g_y = ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[3]) | (MARG_SENSORS[GYRO]->SensorRawValue[2] << 8))));
		g_z = ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[5]) | (MARG_SENSORS[GYRO]->SensorRawValue[4] << 8))));
		sync_printf("GYRO: %d %d %d\r\n", g_x, g_y, g_z);

		break;
		/*init ITG3200*/
	case ACC:

		//I2C_ReadDeviceRegister(I2C_COM1,LSM303_ACC_ADDRESS, OUT_X_H_A | (1 << 7) ,MARG_SENSORS[ACC]->ReadLength,(uint32_t)MARG_SENSORS[ACC]->SensorRawValue);
		I2C_ReadDeviceRegister(I2C_COM1, ADXL345_ADDRESS, Register_DataX_L | (1 << 7), MARG_SENSORS[ACC]->ReadLength, (uint32_t) MARG_SENSORS[ACC]->SensorRawValue); //ADXL345

		a_x = ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[0]) | (MARG_SENSORS[ACC]->SensorRawValue[1] << 8))));
		a_y = ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[2]) | (MARG_SENSORS[ACC]->SensorRawValue[3] << 8))));
		a_z = ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[4]) | (MARG_SENSORS[ACC]->SensorRawValue[5] << 8))));

		sync_printf("ACC: %d %d %d\r\n", a_x, a_y, a_z);
		break;
	case MAG:

		//I2C_ReadDeviceRegister(I2C_COM1,LSM303_MAG_ADDRESS, OUT_X_H_M | (1 << 7) ,MARG_SENSORS[MAG]->ReadLength,(uint32_t)MARG_SENSORS[MAG]->SensorRawValue);
		I2C_ReadDeviceRegister(I2C_COM1, HMC5883L_ADDRESS, OUT_X_H_M | (1 << 7), MARG_SENSORS[MAG]->ReadLength, (uint32_t) MARG_SENSORS[MAG]->SensorRawValue);

		m_x = ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[1]) | (MARG_SENSORS[MAG]->SensorRawValue[0] << 8))));
		m_y = ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[3]) | (MARG_SENSORS[MAG]->SensorRawValue[2] << 8))));
		m_z = ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[5]) | (MARG_SENSORS[MAG]->SensorRawValue[4] << 8))));

		sync_printf("%d %d %d\r\n", m_x, m_y, m_z);
		break;
	default:
		I2C_ReadDeviceRegister(I2C_COM1, ITG3200_ADDRESS, GYRO_XOUT_H_ITG | (1 << 7), MARG_SENSORS[GYRO]->ReadLength, (uint32_t) MARG_SENSORS[GYRO]->SensorRawValue);
		//I2C_ReadDeviceRegister_async(I2C_COM1,L3G3200_ADDRESS, OUT_X_L_G | (1 << 7) ,MARG_SENSORS[GYRO]->ReadLength,(uint32_t)MARG_SENSORS[GYRO]->SensorRawValue);

		g_x = ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[1]) | (MARG_SENSORS[GYRO]->SensorRawValue[0] << 8))));
		g_y = ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[3]) | (MARG_SENSORS[GYRO]->SensorRawValue[2] << 8))));
		g_z = ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[5]) | (MARG_SENSORS[GYRO]->SensorRawValue[4] << 8))));

		I2C_ReadDeviceRegister(I2C_COM1, ADXL345_ADDRESS, Register_DataX_L | (1 << 7), MARG_SENSORS[ACC]->ReadLength, (uint32_t) MARG_SENSORS[ACC]->SensorRawValue); //ADXL345

		a_x = ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[0]) | (MARG_SENSORS[ACC]->SensorRawValue[1] << 8))));
		a_y = ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[2]) | (MARG_SENSORS[ACC]->SensorRawValue[3] << 8))));
		a_z = ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[4]) | (MARG_SENSORS[ACC]->SensorRawValue[5] << 8))));

		I2C_ReadDeviceRegister(I2C_COM1, HMC5883L_ADDRESS, OUT_X_H_M | (1 << 7), MARG_SENSORS[MAG]->ReadLength, (uint32_t) MARG_SENSORS[MAG]->SensorRawValue);

		m_x = ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[1]) | (MARG_SENSORS[MAG]->SensorRawValue[0] << 8))));
		m_y = ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[3]) | (MARG_SENSORS[MAG]->SensorRawValue[2] << 8))));
		m_z = ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[5]) | (MARG_SENSORS[MAG]->SensorRawValue[4] << 8))));


		async_printf("%d %d %d %d %d %d %d %d %d\r\n", a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z);

		break;

	}
}

int Sensors_Process(uint8_t SensorType, float dt)
{
	switch (SensorType)
	{
	case GYRO:
		if (MARG_SENSORS[GYRO]->SensorValueUpdated)
		{
			/*ITG 3200*/
			float x = (float) ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[1]) | (MARG_SENSORS[GYRO]->SensorRawValue[0] << 8))));
			float y = (float) ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[3]) | (MARG_SENSORS[GYRO]->SensorRawValue[2] << 8))));
			float z = (float) ((int) ((int16_t) ((MARG_SENSORS[GYRO]->SensorRawValue[5]) | (MARG_SENSORS[GYRO]->SensorRawValue[4] << 8))));

			//float x = (float)((int)((int16_t)((MARG_SENSORS[GYRO]->SensorRawValue[0]) | (MARG_SENSORS[GYRO]->SensorRawValue[1] <<8))));
			//float y = (float)((int)((int16_t)((MARG_SENSORS[GYRO]->SensorRawValue[2]) | (MARG_SENSORS[GYRO]->SensorRawValue[3] <<8))));
			//float z = (float)((int)((int16_t)((MARG_SENSORS[GYRO]->SensorRawValue[4]) | (MARG_SENSORS[GYRO]->SensorRawValue[5] <<8))));

			MARG_SENSORS[GYRO]->SensorValueUpdated = -1;
			//async_printf("GYRO: %f %f %f\r\n",x, y, z);
			//process_gyro(x, y, z, dt);
			return 1;
		}
		break;
	case ACC:

		if (MARG_SENSORS[ACC]->SensorValueUpdated)
		{
			float x = (float) ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[0]) | (MARG_SENSORS[ACC]->SensorRawValue[1] << 8))));
			float y = (float) ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[2]) | (MARG_SENSORS[ACC]->SensorRawValue[3] << 8))));
			float z = (float) ((int) ((int16_t) ((MARG_SENSORS[ACC]->SensorRawValue[4]) | (MARG_SENSORS[ACC]->SensorRawValue[5] << 8))));
			MARG_SENSORS[ACC]->SensorValueUpdated = -1;

			//async_printf("ACC: %f %f %f\r\n", x, y, z);
			//process_acc(x, y, z);
			return 1;
		}
		break;
	case MAG:
		if (MARG_SENSORS[MAG]->SensorValueUpdated)
		{
			float x = (float) ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[1]) | (MARG_SENSORS[MAG]->SensorRawValue[0] << 8))));
			float y = (float) ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[3]) | (MARG_SENSORS[MAG]->SensorRawValue[2] << 8))));
			float z = (float) ((int) ((int16_t) ((MARG_SENSORS[MAG]->SensorRawValue[5]) | (MARG_SENSORS[MAG]->SensorRawValue[4] << 8))));
			MARG_SENSORS[MAG]->SensorValueUpdated = -1;

			//async_printf("MAG: %f %f %f\r\n",x, y, z);
			//process_mag(x, y, z);
			return 1;
		}
		break;
	default:
		return -1;
	}
}


void Sensors_Read_async(uint8_t SensorType)
{
	if (DMA_GetCmdStatus(I2C1_DMA_STREAM_RX))
	{
		return;
	}

	switch (SensorType)
	{

	/*ITG-3200*/
	case GYRO:
		I2C_ReadDeviceRegister_async(I2C_COM1, ITG3200_ADDRESS, GYRO_XOUT_H_ITG | (1 << 7), MARG_SENSORS[GYRO]->ReadLength, (uint32_t) MARG_SENSORS[GYRO]->SensorRawValue);
		//I2C_ReadDeviceRegister_async(I2C_COM1,L3G3200_ADDRESS, OUT_X_L_G | (1 << 7) ,MARG_SENSORS[GYRO]->ReadLength,(uint32_t)MARG_SENSORS[GYRO]->SensorRawValue);//L3G

		break;
		/*BOTH BELOW ARE LSM*/
	case ACC:
		//I2C_ReadDeviceRegister_async(I2C_COM1,LSM303_ACC_ADDRESS, OUT_X_H_A | (1 << 7) ,MARG_SENSORS[ACC]->ReadLength,(uint32_t)MARG_SENSORS[ACC]->SensorRawValue); //LSM303
		I2C_ReadDeviceRegister_async(I2C_COM1, ADXL345_ADDRESS, Register_DataX_L | (1 << 7), MARG_SENSORS[ACC]->ReadLength, (uint32_t) MARG_SENSORS[ACC]->SensorRawValue); //ADXL345
		break;

	case MAG:
		//I2C_ReadDeviceRegister_async(I2C_COM1,LSM303_MAG_ADDRESS, OUT_X_H_M | (1 << 7) ,MARG_SENSORS[MAG]->ReadLength,(uint32_t)MARG_SENSORS[MAG]->SensorRawValue);
		I2C_ReadDeviceRegister_async(I2C_COM1, HMC5883L_ADDRESS, OUT_X_H_M | (1 << 7), MARG_SENSORS[MAG]->ReadLength, (uint32_t) MARG_SENSORS[MAG]->SensorRawValue);
		break;

	}
}

