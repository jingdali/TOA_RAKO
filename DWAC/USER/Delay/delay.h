#ifndef __Delay_H
#define __Delay_H
#include "sys.h"

void delay_init(void);
void Delay_us(__IO uint32_t nTime);
void Delay_ms(__IO uint32_t nTime);
extern __IO uint32_t msec;
extern __IO uint16_t usec;

#endif
