#include "Testing.h"
uint32 time_record;
uint32 time_stack[10];
uint16 timestack_cnt=0;
void NRF_Test(uint8_t cmd)
{
	uint8_t state;
	switch (cmd)
	{
		case  1:
		printf("%d\r\n",HAL_GPIO_ReadPin(NRF_INT_GPIO_Port,NRF_INT_Pin));
		break;
		
		case  2:
		__HAL_GPIO_EXTI_GENERATE_SWIT(NRF_INT_Pin) ;
		break;
		case	3:			
			printf("0x%x\r\n",NRF24L01_Read_Reg(STATUS) );break;
		
		case  4:NRF24L01_TX_Mode();	break;
		case 	5:NRF24L01_RX_Mode();	break;
		case  6:
				state=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
				NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
				NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
			break;
		
		
		default:break;
		}			
		
				
				
}
void unlockflash(unsigned int passwd)
{
	
	#ifdef FLASHPROTECT
	static uint8 i=5;
		#ifndef MAXRDPLEVEL
		if(i>0)
		{
			if(passwd==26172617)
			{
				HAL_FLASH_Unlock();
				HAL_FLASH_OB_Unlock();
				FLASH_OB_RDP_LevelConfig(OB_RDP_Level_0);
				HAL_FLASH_OB_Lock();
				HAL_FLASH_Lock();
				printf("Chip will unlock and flash will be erased after reset. \r\n");
			}
			else
			{
				i--;
				printf("Error password! %d times left!\r\n",i);
			}
		}
		else
		{
			printf("CHIP LOCKED!!!");
		}
		#else
		printf("The chip has been protected forever!");
		#endif
	#else
	printf("Chip will explode in 3 secs!!!");
	#endif
	
}	

void going(void)
{
	triggle=1;
}
void Start_dwrx(void)
{
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
void Read_status(void)
{
	uint32 status_reg;
	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
	printf("0x%lx\r\n",status_reg);
}

void SET_Tpoint(void)
{
	time_record=dwt_readsystimestamphi32();
}
void GET_Time2Tpoint(void)
{
	uint32 timetmp;
	timetmp=dwt_readsystimestamphi32();
	time_stack[timestack_cnt++]=timetmp-time_record;//低九位为常0，寄存器40位分辨率15.65ps，高32位字节分辨率4.006ns
	
}
void ShowTimeStack(void)
{
	int i=timestack_cnt;
	while(i)
	{
		double tmp=(double)time_stack[i-1]*4/1000;
		printf("%d:%f us \r\n",i--,tmp);
	}
}


void testI2C(uint8 resaddr)
{
	#define INT_PIN_CFGREG 55
	#define I2C_STA_REG 54
	#define I2C_SLV0_ADDR 37
	#define I2C_SLV0_REG 38
	#define I2C_SLV0_CTRL 39
	#define USER_CTRL 106
	#define PM1_REG 107
	#define MAG_CRTL1 0x0a
	#define MAG_CRTL2 0x0b
	#define DATA_REG 0x49
//	int ret;
	uint8 val;
//	uint16 i=0;
//	unsigned char data[6]={0};
//	short accel_bias[3];
//	short accel[3]={0};
//	float accel_value[3];
//	unsigned char accel_st_data[3]={0};
//	
//	I2C_ReadRegister_9250(0x68,119,2,&data[0]);
//	I2C_ReadRegister_9250(0x68,122,2,&data[2]);
//	I2C_ReadRegister_9250(0x68,125,2,&data[4]);
//	accel_bias[0]=((short)data[0]<<8)|data[1];
//	accel_bias[1]=((short)data[2]<<8)|data[3];
//	accel_bias[2]=((short)data[4]<<8)|data[5];

//	I2C_ReadRegister_9250(0x68,13,3,accel_st_data);
//	for(i=0;i<100;i++)
//	{
//		I2C_ReadRegister_9250(0x68,59,6,data);
//		accel[0]=((short)data[0]<<8)|data[1];
//		accel[1]=((short)data[2]<<8)|data[3];
//		accel[2]=((short)data[4]<<8)|data[5];
//		accel_value[0]=(float)accel[0]/16384;
//		accel_value[1]=(float)accel[1]/16384;
//		accel_value[2]=(float)accel[2]/16384;
//		printf("%f %f %f\r\n",accel_value[0],accel_value[1],accel_value[2]);
//		delay_ms(100);
//	}


//	uint8 addr=0x0c;
//	val=0x80;
//	I2C_WriteRegister_9250(0x68,PM1_REG,1,&val);//soft reset mpu
//	delay_ms(100);
//	
	val=0x02;
	I2C_WriteRegister_9250(0x68,INT_PIN_CFGREG,1,&val); //enable the bypass mode
	delay_us(50);
	printf("bypass seted\r\n");
	val=0x00; //disable I2C MASTER mode
	I2C_WriteRegister_9250(0x68,USER_CTRL,1,&val);//mpu cntl 
	printf("master disabled\r\n");
	delay_us(100);
	I2C_ReadRegister_9250(0x0c,0x00,1,&val);//try to read
	printf("%x\r\n",val);
	val=0x1f;
	I2C_WriteRegister_9250(0x0c,0x0a,1,&val);
	delay_us(100);
	I2C_ReadRegister_9250(0x0c,0x10,1,&val);
	printf("asax=0x%X",val);
	I2C_ReadRegister_9250(0x0c,0x11,1,&val);
	printf("asay=0x%X",val);
	I2C_ReadRegister_9250(0x0c,0x12,1,&val);
	printf("asaz=0x%X",val);
	val=0x00;
	I2C_WriteRegister_9250(0x0c,0x0a,1,&val);
//	
//	val=0x00;
//	I2C_WriteRegister_9250(0x68,INT_PIN_CFGREG,1,&val);//disable the bypass mode 
//	delay_us(50);
//	val=0x20; //enable I2C MASTER mode
//	I2C_WriteRegister_9250(0x68,USER_CTRL,1,&val);//mpu cntl 
//	delay_us(50);
//	val=0x01;
//	I2C_WriteRegister_9250(0x68,MAG_CRTL2,1,&val);//reset mag
//	delay_ms(30);
//	val=0x00;//disable slv0 
//	I2C_WriteRegister_9250(0x68,I2C_SLV0_CTRL,1,&val);//
//	delay_us(50);
//	val=addr|0x80;//Read commed
//	I2C_WriteRegister_9250(0x68,I2C_SLV0_ADDR,1,&val);//set comp addr
//	delay_us(50);	
//	val=0x00;
//	I2C_WriteRegister_9250(0x68,I2C_SLV0_REG,1,&val);//reg addr
//	delay_us(50);
//	val=0x81;//enable slv0 and 1 byte
//	I2C_WriteRegister_9250(0x68,I2C_SLV0_CTRL,1,&val);//
//	Delay_ms(20);
//	val=0x00;//disable slv0 
//	I2C_WriteRegister_9250(0x68,I2C_SLV0_CTRL,1,&val);//
//	val=0;
//	I2C_ReadRegister_9250(0x68,DATA_REG,1,&val);
//	printf("id= %x\r\n",val);
//	

	
}
