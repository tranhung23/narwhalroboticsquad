/*
 * HMC5883L.h
 *
 *  Created on: Feb 15, 2012
 *      Author: GrubyGrub
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_


#define HMC5883L_ADDRESS	(0x3D)


#define CTRL_REG1_M 		(0x00)
#define CTRL_REG2_M 		(0x01)
#define MODE_REG_M 			(0x02)
#define STATUS_REG_M 		(0x09)

#define OUT_X_H_M  			(0x03)
#define OUT_X_L_M  			(0x04)
#define OUT_Y_H_M  			(0x05)
#define OUT_Y_L_M  			(0x06)
#define OUT_Z_H_M  			(0x07)
#define OUT_Z_L_M  			(0x08)


#endif /* HMC5883L_H_ */
