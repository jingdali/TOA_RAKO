#ifndef __DECA_SPI_H
#define __DECA_SPI_H
#include <string.h>
#include "stm32f0xx_hal.h"

#include "deca_device_api.h"
#define DWSPIx hspi2.Instance
#define DWSPIGPIOx GPIOB


#define DWNSS_RESET()	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port,SPI2_NSS_Pin,GPIO_PIN_RESET)
#define DWNSS_SET()   HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port,SPI2_NSS_Pin,GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

 void SPI_SendData8(SPI_TypeDef* SPIx, uint8_t Data);
 uint8_t SPI_ReceiveData8(SPI_TypeDef* SPIx);
#endif
