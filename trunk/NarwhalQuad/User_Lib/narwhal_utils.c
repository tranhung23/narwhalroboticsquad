

/*Portions of this file taken from Autoquad*/

#include "narwhal_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t heapUsed, heapHighWater, dataSramUsed;

uint32_t *ccmHeap[UTIL_CCM_HEAP_SIZE] __attribute__ ((section(".ccm")));


#ifdef UTIL_STACK_CHECK
int32_t numStacks;
void *stackPointers[UTIL_STACK_CHECK] __attribute__((section(".ccm")));
uint16_t stackSizes[UTIL_STACK_CHECK] __attribute__((section(".ccm")));
uint16_t stackFrees[UTIL_STACK_CHECK] __attribute__((section(".ccm")));

void utilStackCheck(void) {
    char s[48];
    int i, j;

    for (i = 0; i < numStacks; i++) {
	for (j = 0; j < stackSizes[i]; j++)
	    if (*(char *)(stackPointers[i]+j) != 0xFF)
		break;
	stackFrees[i] = j;
	if (j < 16) {
	    sync_printf("Potential stack overflow [%d]!\n", i);
	}
    }
}
#endif

void *narwhalCalloc(size_t count, size_t size) {
    char *addr;

    addr = calloc(count, size);

    heapUsed += count * size;
    if (heapUsed > heapHighWater)
	heapHighWater = heapUsed;

    if (addr == 0)
    	sync_printf("Out of heap memory!\n");

    return addr;
}

void narwhalFree(void *ptr, size_t count, size_t size) {
    if (ptr) {
	free(ptr);
	heapUsed -= count * size;
    }
}

// allocates memory from 64KB CCM
void *narwhalDataCalloc(uint16_t count, uint16_t size) {
    uint32_t words;

    // round up to word size
    words = (count*size + sizeof(int)-1) / sizeof(int);

    if ((dataSramUsed + words) > UTIL_CCM_HEAP_SIZE) {
    	sync_printf("Out of data SRAM!\n");
    }
    else {
	dataSramUsed += words;
    }

    return (void *)(ccmHeap + dataSramUsed - words);
}

// size in words
OS_STK *narwhalStackInit(uint16_t size) {
    OS_STK *sp;

    // use memory in the CCM
    sp = (OS_STK *)narwhalDataCalloc(1, size*4);

    // fill memory with pattern to ease overflow detection
    memset(sp, 0xFF, size*4);

#ifdef UTIL_STACK_CHECK
    stackPointers[numStacks] = sp;
    stackSizes[numStacks] = size*4;
    numStacks++;
#endif

    return sp;
}

//TODO: FIX THIS
void delayMicros(unsigned long t) {
//    t = t + timerMicros();
//
//    while (timerMicros() < t)
//	AQ_64_NOPS;
}

// delay for given milli seconds
void delay(unsigned long t) {
    delayMicros(t * 1000);
}

int constrainInt(int i, int lo, int hi) {
    if (i < lo)
       return	lo;
    if (i > hi)
       return	hi;

    return i;
}

float constrainFloat(float i, float lo, float hi) {
    if (i < lo)
       return	lo;
    if (i > hi)
       return	hi;

    return i;
}

void info(void) {
    char s[96];

    //sync_printf(s, "AQ S/N: %08X-%08X-%08X\n", flashSerno(2), flashSerno(1), flashSerno(0));

    yield(100);

    //sync_printf(s, "Mavlink SYS ID: %d\n", flashSerno(0) % 250);

    yield(100);

    sync_printf("SYS Clock: %u MHz\n", rccClocks.SYSCLK_Frequency / 1000000);

    yield(100);

    sync_printf("%u/%u heap used/high water\n", heapUsed, heapHighWater);

    yield(100);

    sync_printf("%u of %u CCM heap used\n", dataSramUsed * sizeof(int), UTIL_CCM_HEAP_SIZE * sizeof(int));

    yield(100);
}
