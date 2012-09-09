/*
 * ekf.h
 *
 * Created: 9/17/2011 2:17:38 PM
 *  Author: GrubyGrub
 */

#include "../quad_lib/config.h"
#ifndef EKF_H_
#define EKF_H_

void ekf_init(void);
void process_gyro(float gx, float gy, float gz, float dT);
void process_mag(float mx, float my, float mz);
void process_acc(float ax, float ay, float az);

void process_magaccvector(void);

void ekf_print(void);

#endif /* EKF_H_ */
