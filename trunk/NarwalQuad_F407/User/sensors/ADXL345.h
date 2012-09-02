/*
 * ADXL345.h
 *
 *  Created on: Feb 15, 2012
 *      Author: GrubyGrub
 */

#ifndef ADXL345_H_
#define ADXL345_H_


#define ADXL345_ADDRESS (0xA6) //0x1D


#define ADXL345_DEVID 0x00
#define ADXL345_THRESH_TAP 0x1D
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_TAP_AXES 0x2a

/*
 Interrupt bit position
 */
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_SINGLE_TAP 0x06
#define ADXL345_DOUBLE_TAP 0x05

#define Register_PowerControl 0x2D
#define Register_DataBW 0x2C
#define Register_DataFormat 0x31
#define Register_DataX_L 0x32
#define Register_DataX_H 0x33
#define Register_DataY_L 0x34
#define Register_DataY_H 0x35
#define Register_DataZ_L 0x36
#define Register_DataZ_H 0x37

#endif /* ADXL345_H_ */
