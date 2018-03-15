
#include "deca_spi.h"


int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
//	int i=0;

    decaIrqStatus_t  stat ;
		uint8_t temp[200]={0};
		memcpy(temp,headerBuffer,headerLength);
		memcpy(&temp[headerLength],bodyBuffer,bodylength);

    stat = decamutexon() ;
	

		DWNSS_RESET();
		
    //SPIx_CS_GPIO->BRR = SPIx_CS;
		HAL_SPI_Transmit(&hspi2,temp,headerLength+bodylength,500);
		HAL_SPIEx_FlushRxFifo(&hspi2);
	
//    for(i=0; i<headerLength; i++)
//    {
//    	SPI_SendData8(DWSPIx, headerBuffer[i]);
//			//SPI1->DR = headerBuffer[i];
////    	while ((SPI_I2S_GetFlagStatus(DWSPIx, SPI_I2S_FLAG_RXNE)) == (uint16_t)RESET);
//			while((DWSPIx->SR & SPI_FLAG_RXNE) ==RESET)
//			{}
//    	SPI_ReceiveData8(DWSPIx);
//    }

//    for(i=0; i<bodylength; i++)
//    {
//     	SPI_SendData8(DWSPIx, bodyBuffer[i]);
//			//SPI1->DR = bodyBuffer[i];
//			while((DWSPIx->SR & SPI_FLAG_RXNE) == RESET)
//			{}
//    	//while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

//		SPI_ReceiveData8(DWSPIx);
//	}

    //SPIx_CS_GPIO->BSRR = SPIx_CS;
		DWNSS_SET();
    decamutexoff(stat) ;

    return 0;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
//	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;
		DWNSS_RESET();
    /* Wait for SPIx Tx buffer empty */
    //while (port_SPIx_busy_sending());

    //SPIx_CS_GPIO->BRR = SPIx_CS;
		HAL_SPI_Transmit(&hspi2,(uint8_t *)headerBuffer,headerLength,500);
		HAL_SPIEx_FlushRxFifo(&hspi2);
		HAL_SPI_Receive(&hspi2,(uint8_t *)readBuffer,readlength,500);
//    for(i=0; i<headerLength; i++)
//    {
//    	SPI_SendData8(DWSPIx, headerBuffer[i]);
//			while((DWSPIx->SR & SPI_FLAG_TXE) == RESET)
//			{
//			}
//			while((DWSPIx->SR & SPI_FLAG_RXNE) == RESET)
//			{
//			}
//     	//while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

//     	readBuffer[0]=SPI_ReceiveData8(DWSPIx);  // Dummy read as we write the header
//    }

//    for(i=0; i<readlength; i++)
//    {
//    	SPI_SendData8(DWSPIx, 0);  // Dummy write as we read the message body
//			while((DWSPIx->SR & SPI_FLAG_TXE) == RESET)
//			{
//			}
//			while((DWSPIx->SR & SPI_FLAG_RXNE) == RESET)
//			{
//			}
//			
//    	//while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
// 
//	   	readBuffer[i]=SPI_ReceiveData8(DWSPIx);//port_SPIx_receive_data(); //this clears RXNE bit
//    }

   // SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;
	  DWNSS_SET();
    return 0;
}	

void SPI_SendData8(SPI_TypeDef* SPIx, uint8_t Data)
{
  uint32_t spixbase = 0x00;

  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  *(__IO uint8_t *) spixbase = Data;
}

uint8_t SPI_ReceiveData8(SPI_TypeDef* SPIx)
{
  uint32_t spixbase = 0x00;
  
  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  return *(__IO uint8_t *) spixbase;
}
