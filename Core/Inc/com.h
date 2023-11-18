#ifndef COM_H_
#define COM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ctype.h"
#include "stdarg.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "string.h"

    extern int cprintf(void *p, const char *format, ...);

    extern void cgets(void *p, char *str, char end);

    extern void cgets_s(void *p, char *str, int s);

    extern void cgets(void *p, char *str, char end);

    extern char cgetc(void *p);


#ifdef __cplusplus
}
#endif

#endif
