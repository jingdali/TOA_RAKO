#ifndef __LED_H
#define __LED_H	 
#include "stm32f0xx.h"

#define LED0(x) {(x)?(GPIOD->BSRR = 0x0004):(GPIOD->BSRR = 0x00040000);}
#define LED1(x) {(x)?(GPIOC->BSRR = 0x0800):(GPIOC->BSRR = 0x08000000);}
#define LED2(x) {(x)?(GPIOC->BSRR = 0x1000):(GPIOC->BSRR = 0x10000000);}	
void LED_Init(void);
		 				    
#endif
