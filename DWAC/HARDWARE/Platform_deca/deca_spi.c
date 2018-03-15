#include <string.h>
#include "stm32f0xx.h"
#include "deca_spi.h"
#include "deca_device_api.h"

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    //((GPIO_TypeDef *) GPIOA_BASE)->BRR = SPIx_CS;
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    for(i=0; i<headerLength; i++)
    {
    	SPI_SendData8(SPI2, headerBuffer[i]);
			//SPI1->DR = headerBuffer[i];
    	while ((SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)) == (uint16_t)RESET);

    	SPI_ReceiveData8(SPI2);
    }

    for(i=0; i<bodylength; i++)
    {
     	SPI_SendData8(SPI2, bodyBuffer[i]);
			//SPI1->DR = bodyBuffer[i];
			while ((SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)) == (uint16_t)RESET);
    	//while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		SPI_ReceiveData8(SPI2);
	}

    //SPIx_CS_GPIO->BSRR = SPIx_CS;
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
    decamutexoff(stat) ;

    return 0;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    /* Wait for SPIx Tx buffer empty */
    //while (port_SPIx_busy_sending());

    //SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPI_SendData8(SPI2, headerBuffer[i]);
			while ((SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)) == (uint16_t)RESET);
     	//while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

     	readBuffer[0]=SPI_ReceiveData8(SPI2);  // Dummy read as we write the header
    }

    for(i=0; i<readlength; i++)
    {
    	SPI_SendData8(SPI2, 0);  // Dummy write as we read the message body
			while ((SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)) == (uint16_t)RESET);
			
    	//while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
 
	   	readBuffer[i]=SPI_ReceiveData8(SPI2);//port_SPIx_receive_data(); //this clears RXNE bit
    }

   // SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
    return 0;
}	

