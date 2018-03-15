#ifndef __USART_H
#define __USART_H

#include "stm32f0xx_hal.h"
#include "main.h"

#define USART_REC_LEN 128
#define EN_USART1_RX 			1		
	  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 
extern uint16_t USART_RX_STA;     
extern UART_HandleTypeDef huart1;


void USART_IRQ_Config(void);
void USART1_IRQHandler(void);
#endif