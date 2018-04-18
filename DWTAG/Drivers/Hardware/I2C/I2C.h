#ifndef __I2C_H
#define __I2C_H
//IIC所有操作函数
#include "stm32f0xx_hal.h"
typedef uint8_t  u8;
#define PIN_CLK MPU_SCL_Pin
#define PIN_SDA MPU_SDA_Pin
#define IIC_GPIOx MPU_SCL_GPIO_Port

#define IIC_CLK_H() IIC_GPIOx->BSRR = PIN_CLK
#define IIC_CLK_L() IIC_GPIOx->BRR = PIN_CLK
#define IIC_SDA_H() IIC_GPIOx->BSRR = PIN_SDA
#define IIC_SDA_L() IIC_GPIOx->BRR = PIN_SDA
#define SDA_READ() (IIC_GPIOx->IDR & PIN_SDA)

void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

int I2C_ReadRegister_9250(unsigned char Address, unsigned char RegisterAddr, 
                                          unsigned short RegisterLen, unsigned char *RegisterValue);
int I2C_WriteRegister_9250(unsigned char Address, unsigned char RegisterAddr, 
                                           unsigned short RegisterLen, const unsigned char *RegisterValue);  


//----------------------------------------------

#endif
