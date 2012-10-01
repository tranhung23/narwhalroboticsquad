/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <stm32f4xx_usart.h>
#include <narwhal_driver_config.h>
#include <sys/types.h>
#include <sys/stat.h>

#undef errno
extern int errno;
extern int _end;

caddr_t _sbrk(int incr)
{
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;

	if (heap == NULL)
	{
		heap = (unsigned char *) &_end;
	}
	prev_heap = heap;

	heap += incr;

	return (caddr_t) prev_heap;
}

int link(char *old, char *new)
{
	return -1;
}

int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;

}

int _read(int file, char *ptr, int len)
{

	while (USART_GetFlagStatus(COM1, USART_FLAG_RXNE) == RESET)
		;
	*ptr = (uint8_t) USART_ReceiveData(COM1);

#ifdef USART_ECHO
	USART_SendData(COM1, (uint16_t)(*ptr));
	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(COM1, USART_FLAG_TC) == RESET)
#endif
		return len;
}

int _write(int file, char *ptr, int len)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	int counter;

	counter = len;
	for (; counter > 0; counter--)
	{
		if (*ptr == 0)
			break;
		USART_SendData(COM1, (uint16_t)(*ptr));
		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(COM1, USART_FLAG_TC) == RESET)
			;
		ptr++;
	}
	return len;
}

void abort(void)
{
	/* Abort called */
	while (1)
		;
}

/* --------------------------------- End Of File ------------------------------ */
