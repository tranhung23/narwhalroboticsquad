/*
 * stm32f4_timer.h
 *
 *  Created on: Apr 18, 2012
 *      Author: GrubyGrub
 */

#ifndef STM32F4_MICRO_TIMER_H_
#define STM32F4_MICRO_TIMER_H_

#include <narwhal_top.h>

#define uTicks()	MICROS_TIMER->CNT

typedef void timerCallback_t(int);

typedef struct microsTimerEvents {
    timerCallback_t *alarm1Callback, *alarm2Callback, *alarm3Callback;
    int alarm1Parameter, alarm2Parameter, alarm3Parameter;

    uint32_t timerStart;
} microsTimerEvents;

extern void usTimerInit(void);
extern void usTimerStart(void);
extern uint32_t usTimerStop(void);
extern void timerCancelAlarm1(void);
extern void timerCancelAlarm2(void);
extern void timerCancelAlarm3(void);
extern void timerSetAlarm1(int32_t us, timerCallback_t *callback, int parameter);
extern void timerSetAlarm2(int32_t us, timerCallback_t *callback, int parameter);
extern void timerSetAlarm3(int32_t us, timerCallback_t *callback, int parameter);



#endif /* STM32F4_TIMER_H_ */
