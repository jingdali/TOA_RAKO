#ifndef __Delay_H
#define __Delay_H
#include "stdint.h"
#include "stm32f0xx_hal.h"
#define delay_us Delay_us
#define delay_ms Delay_ms
void delay_init(void);
void Delay_us(volatile uint32_t nus);
void Delay_ms(volatile uint16_t nms);
void get_msticks(unsigned long *timestamp);
void HAL_Delay(__IO uint32_t Delay);
//extern __IO uint16_t msec;
//extern __IO uint16_t usec;

#endif
