//////////////////////////////////////////////
//  * 硬件连接 ----------------------------
//  *         | P??-CE          :24L01-CE |
//  *         | P??-IRQ  :      24L01-IRQ |
//  *         | P??-CS  :        24L01-CS  |
//  *         | P??-SPI1-SCK  : 24L01-CLK |
//  *         | P??-SPI1-MISO : 24L01-DO  |
//  *         | P??-SPI1-MOSI : 24L01-DIO |
//  *          ----------------------------
//  * 库版本  ：ST3.0.0
//  *
////////////////////////////////////////////

#include  "rf24l01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x68,0x86,0x66,0x88,0x28}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x68,0x86,0x66,0x88,0x28}; //发送地址
uint8_t SPI_RF_SendByte(uint8_t temp);
uint8_t nrfsta=0;//0:rece 1: trans
uint8_t TransCop=0;
/**************************************
* 函数名   : SPI_RF_Init
* 描述     : RF24L01初始化
* 输入参数 : 无
* 返回值   : 无
**************************************/


/**************************************
* 函数名   : NRF24L01_Write_Reg
* 描述     : 焊?4L01的寄存器写值（一个字节）
* 输入参数 : uint8_t reg,uint8_t value
*            reg    要写的寄存器地址  
*            value  给寄存器写的值
* 返回值   : 一个字节的状态值 
**************************************/
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;

	SPI_RF_CS_LOW() ;	 //CSN=0;
	
	status=SPI_RF_SendByte(reg);
	SPI_RF_SendByte(value);
	
//	HAL_SPI_TransmitReceive(&hspi1,&reg,&status,1,500);
//	HAL_SPI_Transmit(&hspi1,&value,1,500);
//	HAL_SPIEx_FlushRxFifo(&hspi1);

	SPI_RF_CS_HIGH();   //CSN=1;
	return status;
}

/*************************************************/
/* 函数功能：读24L01的寄存器值 （一个字节）      */
/* 入口参数：reg  要读的寄存器地址               */
/* 出口参数：value 读出寄存器的值                */
/*************************************************/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	SPI_RF_CS_LOW() ; //CSN=0; 
//	status=SPI_RF_SendByte(reg);
	value=SPI_RF_SendByte(0XFF);
  
//  HAL_SPI_Transmit(&hspi1,&reg,1,500);//发送寄存器值(位置),并读取状态值
//	HAL_SPIEx_FlushRxFifo(&hspi1);
//  HAL_SPI_Receive(&hspi1,&value,1,500);
	
	SPI_RF_CS_HIGH();  //CSN=1;
	return value;
}

/*********************************************/
/* 函数功能：读24L01的寄存器值（多个字节）   */
/* 入口参数：reg   寄存器地址                */
/*           *pBuf 读出寄存器值的存放数组    */
/*           len   数组字节长度              */
/* 出口参数：status 状态值                   */
/*********************************************/
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,i;
	SPI_RF_CS_LOW() ;//CSN=0


	status=SPI_RF_SendByte(reg);
	for(i=0;i<len;i++)pBuf[i]=SPI_RF_SendByte(0XFF);//读出数据
	
//	HAL_SPI_TransmitReceive(&hspi1,&reg,&status,1,500);
//	HAL_SPI_Receive(&hspi1,pBuf,len,500);
	SPI_RF_CS_HIGH(); //CSN=1
  return status;        //返回读到的状态值
}

/**********************************************/
/* 函数功能：给24L01的寄存器写值（多个字节）  */
/* 入口参数：reg  要写的寄存器地址            */
/*           *pBuf 值的存放数组               */
/*           len   数组字节长度               */
/**********************************************/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,i;
	SPI_RF_CS_LOW() ;
	

	status=SPI_RF_SendByte(reg);
	for(i=0; i<len; i++)SPI_RF_SendByte(*pBuf++); //写入数据
	
//  HAL_SPI_TransmitReceive(&hspi1,&reg,&status,1,500);
//	HAL_SPI_Transmit(&hspi1,pBuf,len,500);
//	HAL_SPIEx_FlushRxFifo(&hspi1);

	SPI_RF_CS_HIGH(); 
  return status;          //返回读到的状态值
}

/********************************************/
/* 函数功能：检测24L01是否存在              */
/* 返回值；  0  存在                        */
/*           1  不存在                      */
/********************************************/	
uint8_t NRF24L01_Check(void)
{
	uint8_t check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uint8_t check_out_buf[5]={0x00};

	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR, check_in_buf, 5);

	NRF24L01_Read_Buf(NRF_READ_REG+TX_ADDR, check_out_buf, 5);

	if((check_out_buf[0] == 0x11)&&\
	   (check_out_buf[1] == 0x22)&&\
	   (check_out_buf[2] == 0x33)&&\
	   (check_out_buf[3] == 0x44)&&\
	   (check_out_buf[4] == 0x55))return 0;
	else return 1;
}

/*********************************************/
/* 函数功能：设置24L01为接收模式             */
/*********************************************/
void NRF24L01_RX_Mode(void)
{
	nrfsta=0;
	SPI_RF_CE_LOW() ;	//CE拉低，使能24L01配置
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);//写RX接收地址
	  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //开启通道0自动应答    
  	//NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x00);    //关闭自动应答    
		NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//通道0接收允许  	 
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //设置RF工作通道频率 		  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
		NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启 	     
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
	SPI_RF_CE_HIGH();	//CE置高，使能接收
	Delay_us(130); 
}

/*********************************************/
/* 函数功能：设置24L01为发送模式             */
/*********************************************/
void NRF24L01_TX_Mode(void)
{
		nrfsta=1;
		SPI_RF_CE_LOW() ;	//CE拉低，使能24L01配置	    
  	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //关闭所有自动应答    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //关闭通道0的接收地址  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x01);//no设置自动重发间隔时间:250us + 86us;最大自动重发次数:1次
  	
		NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	  NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		SPI_RF_CE_HIGH();	//CE置高，使能发送
		Delay_us(10);
}

/*********************************************/
/* 函数功能：24L01接收数据                   */
/* 入口参数：rxbuf 接收数据数组              */
/* 返回值： 0   成功收到数据                 */
/*          1   没有收到数据                 */
/*********************************************/
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;

	state=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //清除标志
	if(state&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}

/**********************************************/
/* 函数功能：设置24L01为发送模式              */
/* 入口参数：txbuf  发送数据数组              */
/* 返回值； 0x10    达到最大重发次数，发送失败*/
/*          0x20    成功发送完成              */
/*          0xff    发送失败                  */
/**********************************************/
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
   
	SPI_RF_CE_LOW() ;	//CE拉低，使能24L01配置
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	SPI_RF_CE_HIGH();	//CE置高，使能发送
//	while (RF_READINTPIN()!=0);//等待发送完成 (note: this can be changed)
	
	
	while(TransCop!=1);
	TransCop=0;	
	state=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
	if(state&MAX_TX)//达到最大重发次数
	{
		
		return MAX_TX; 
	}
	if(state&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//发送失败
}	


uint8_t SPI_RF_SendByte(uint8_t temp)
{
	while(((SPI1->SR)&SPI_FLAG_TXE)!=SPI_FLAG_TXE);
	
	*((__IO uint8_t *)&SPI1->DR) =temp;
	
	while(((SPI1->SR)&SPI_FLAG_RXNE)!=SPI_FLAG_RXNE);
	
	return (uint8_t)SPI1->DR;
	
}



/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/



