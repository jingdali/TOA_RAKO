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
