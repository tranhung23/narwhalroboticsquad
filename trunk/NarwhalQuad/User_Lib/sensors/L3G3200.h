/*
 * L3G3200.h
 *
 *  Created on: Feb 15, 2012
 *      Author: GrubyGrub
 */

#ifndef L3G3200_H_
#define L3G3200_H_

#define L3G3200_ADDRESS		(0xD3)

#define WHO_AM_I_G 0x0F

#define CTRL_REG1_G 0x20
#define CTRL_REG2_G 0x21
#define CTRL_REG3_G 0x22
#define CTRL_REG4_G 0x23
#define CTRL_REG5_G 0x24

#define REFERENCE_G 0x25
#define OUT_TEMP_G 0x26

#define STATUS_REG_G 0x27

#define OUT_X_L_G 0x28
#define OUT_X_H_G 0x29
#define OUT_Y_L_G 0x2A
#define OUT_Y_H_G 0x2B
#define OUT_Z_L_G 0x2C
#define OUT_Z_H_G 0x2D

#define FIFO_CTRL_REG_G 0x2Eb
#define FIFO_SRC_REG_G 0x2F

#define INT1_CFG_G 0x30
#define INT1_SRC_G 0x31
#define INT1_TSH_XH_G 0x32
#define INT1_TSH_XL_G 0x33
#define INT1_TSH_YH_G 0x34
#define INT1_TSH_YL_G 0x35
#define INT1_TSH_ZH_G 0x36
#define INT1_TSH_ZL_G 0x37
#define INT1_DURATION_G 0x38

#endif /* L3G3200_H_ */
