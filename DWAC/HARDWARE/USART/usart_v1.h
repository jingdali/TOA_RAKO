#ifndef __USART_V1_H
#define __USART_V1_H

#include "stm32f0xx_usart.h"

#define USART_REC_LEN 128
//#define TXBUFFERSIZE   128
//#define RXBUFFERSIZE   128
/* pravite */
//extern uint8_t TxBuffer[TXBUFFERSIZE];
//extern uint8_t RxBuffer[RXBUFFERSIZE];
//extern uint8_t NbrOfDataToTransfer;
//extern uint8_t NbrOfDataToRead;
//extern __IO uint8_t TxCount; 
//extern __IO uint16_t RxCount;
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记
void usart_trans(uint8_t *p,uint16_t len);
void USART_Config(void);
void USART1_IRQHandler(void);
#endif
