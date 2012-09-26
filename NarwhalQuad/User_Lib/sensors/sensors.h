/*
 * sensors.h
 *
 *  Created on: Mar 8, 2012
 *      Author: GrubyGrub
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include <narwhal_top.h>

#define SENSORn 3
#define SETTING_DATA_LENGTH 0x01

typedef enum
{
	MAG,
	ACC,
	GYRO
}MARG_SENSOR_TYPE_TypeDef;

typedef struct
{
	uint16_t ReadLength;
	uint8_t SensorRawValue[6];
	uint8_t SensorValueUpdated;
	MARG_SENSOR_TYPE_TypeDef SensorType;
	float x;
	float y;
	float z;

}MARG_SENSOR_TypeDef;

extern MARG_SENSOR_TypeDef * MARG_SENSORS[SENSORn];

void Sensors_Init(uint8_t SensorType);
void Sensors_Read_async(uint8_t SensorType);
void Sensors_Read(uint8_t SensorType);
int Sensors_Process(uint8_t SensorType, float dt);


#endif /* SENSORS_H_ */
