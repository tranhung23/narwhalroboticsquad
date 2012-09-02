/*
 * ITG3200.h
 *
 *  Created on: Feb 15, 2012
 *      Author: GrubyGrub
 */

#ifndef ITG3200_H_
#define ITG3200_H_

#define ITG3200_ADDRESS		(0xD0)

/*ITG3200*/
#define WHO_AM_I_ITG 0x00

#define SMPLRT_DIV_ITG 0x15

#define DLPF_FS_ITG 0x16

#define INT_CFG_ITG 0x17

#define INT_STATUS_ITG 0x1A

#define TEMP_OUT_H_ITG 0x1B
#define TEMP_OUT_L_ITG 0x1C

#define GYRO_XOUT_H_ITG 0x1D
#define GYRO_XOUT_L_ITG 0x1E
#define GYRO_YOUT_H_ITG 0x1F
#define GYRO_YOUT_L_ITG 0x20
#define GYRO_ZOUT_H_ITG 0x21
#define GYRO_ZOUT_L_ITG 0x22

#define PWR_MGM_ITG 0x3E

#endif /* ITG3200_H_ */
