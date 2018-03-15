
#include "test_fun.h"
uint32 time_record;
uint32 time_stack[10];
uint16 timestack_cnt=0;
uint8 triggle=0;
void System_GetClocks(void)
{
  RCC_ClocksTypeDef rcc_clocks;
   
  RCC_GetClocksFreq(&rcc_clocks);
   
  printf("SYSCLK    = %dMHz\r\n", rcc_clocks.SYSCLK_Frequency / 1000000);
  printf("HCLK(AHB) = %dMHz\r\n", rcc_clocks.HCLK_Frequency / 1000000);
  printf("HCLK(AHB) = %dMHz\r\n", rcc_clocks.HCLK_Frequency / 1000000);
  printf("PCLK(APB) = %dMHz\r\n", rcc_clocks.PCLK_Frequency / 1000000);
}



void getSYSstatus(void)
{
	uint32 status_reg;
	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
	printf("0x%lx\r\n",status_reg);
	
}
void getIDs(unsigned char index)
{
	uint8 headbuff[1]={0x00};
	uint8 headlength=1;
	uint8 bodylength=4;
	uint8 bodybuff[4];
	readfromspi(headlength,headbuff,bodylength,bodybuff);
	printf("%d\r\n",bodybuff[index]);
}	

void read_test(unsigned char add)
{
	uint32 id;
	id=dwt_readdevid();
	printf("%lx\r\n",id);
	
}

void Prt_anchnum(void)
{
	static int i=ANCHOR_NUM;
	printf("anchor num=%d\r\n",i);
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
				FLASH_Unlock();
				FLASH_OB_Unlock();
				FLASH_OB_RDPConfig(OB_RDP_Level_0);
				FLASH_OB_Lock();
				FLASH_Lock();
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
void testfun1(void)
{
	uint8 txframe[19]={0x61,0x88,0x00,0xca,0xde,0x01,0x00,0x02,0x00,0xa1,0x02,0x00,0xac,0x2e,0x9d,0x7d,0x3d,0,0};
	dwt_forcetrxoff();
	dwt_writetxdata((19), txframe, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl((19), 0, 1); /* Zero offset in TX buffer, ranging. */		
	printf("testfun1 executed!\r\n");
	dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
}
void testfun2(void)
{
	uint8 txframe[19]={0x41,0x88,0x00,0xca,0xde,0xFF,0xFF,0x01,0x00,0xa0,0,0,0xac,0x2e,0x9d,0x7d,0x3d,0,0};
	dwt_forcetrxoff();
	dwt_writetxdata((12), txframe, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl((12), 0, 1); /* Zero offset in TX buffer, ranging. */		
	printf("testfun2 executed!\r\n");
	dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
}
void getloctime(void)
{
	printf("time=%d",msec);
}

void DMA_test(void)
{
	DMA_transing=1;
	DMA1_Channel2->CNDTR=32;
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	while(DMA_transing);
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
void going(void)
{
	triggle=1;
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
