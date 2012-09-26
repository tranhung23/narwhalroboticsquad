/*
 * pwm.h
 *
 *  Created on: Sep 9, 2012
 *      Author: GrubyGrub
 */
#include "stm32f4xx.h"
#ifndef PWM_H_
#define PWM_H_

enum pwmDirections
{
    PWM_OUTPUT = 1, PWM_INPUT
};

typedef void pwmCallback_t(uint32_t, uint8_t);

typedef struct
{
    volatile uint32_t *ccr;
    volatile uint32_t *cnt;
    pwmCallback_t *callback;
    uint32_t period;
    int8_t direction;
} pwmPortStruct_t;

extern pwmPortStruct_t *pwmInitOut(uint8_t pwmPort, uint32_t period, uint32_t inititalValue, int8_t ESC32Mode);
extern pwmPortStruct_t *pwmInitIn(uint8_t pwmPort, uint16_t polarity, uint32_t period, pwmCallback_t callback);

#endif /* PWM_H_ */
