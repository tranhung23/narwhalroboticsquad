/*Snippits of this file were taken from autoquad
 */

#include <narwhal_top.h>
#include <CoOS.h>
#include <stdlib.h>

#define	UTIL_STACK_CHECK	16		// uncomment to allow system to self check for stack overflows

#define UTIL_CCM_HEAP_SIZE	(0x2000)	//  32KB

#define UTIL_ISR_DISABLE	__asm volatile ( "CPSID   F\n")
#define UTIL_ISR_ENABLE		__asm volatile ( "CPSIE   F\n")

#define yield(n)    CoTickDelay(n);

#define	AQ_NOP			{__asm volatile ("nop\n\t");}
#define	AQ_4_NOPS		{AQ_NOP; AQ_NOP; AQ_NOP; AQ_NOP;}
#define	AQ_16_NOPS		{AQ_4_NOPS; AQ_4_NOPS; AQ_4_NOPS; AQ_4_NOPS;}
#define	AQ_64_NOPS		{AQ_16_NOPS; AQ_16_NOPS; AQ_16_NOPS; AQ_16_NOPS;}
#define	AQ_256_NOPS		{AQ_64_NOPS; AQ_64_NOPS; AQ_64_NOPS; AQ_64_NOPS;}

RCC_ClocksTypeDef rccClocks;


extern OS_STK *narwhalStackInit(uint16_t size);
extern void *narwhalDataCalloc(uint16_t count, uint16_t size);
extern void narwhalFree(void *ptr, size_t count, size_t size);
extern void *narwhalCalloc(size_t count, size_t size);
extern void utilStackCheck(void);

extern void info(void);
extern float constrainFloat(float i, float lo, float hi);
extern int constrainInt(int i, int lo, int hi);
extern void delay(unsigned long t);
extern void delayMicros(unsigned long t);
