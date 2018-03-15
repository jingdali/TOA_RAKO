#include "delay.h"


static uint8_t  fac_us=0;//us延时倍乘数
static uint16_t fac_ms=0;//ms延时倍乘数
// __IO uint16_t msec=0;
// __IO uint16_t usec=0;


//初始化延迟函数
//当使用ucos的时候,此函数会初始化ucos的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()	 
{

//	  if (SysTick_Config(SystemCoreClock / 8000))//The function initializes the System Timer and its interrupt, and starts the System Tick Timer. Counter is in free running mode to generate periodic interrupts.
//  { 
//    /* Capture error */ 
//    while (1);
//  }
	SysTick->CTRL&=~(SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);	//选择时钟  HCLK
	fac_us=SystemCoreClock/8000000;
	fac_ms=SystemCoreClock/8000;
	 

}								    

//void Delay_ms(__IO uint32_t nTime)
//{ 
//  __IO uint16_t temp1=0;
//	uint16_t flag=0;
//	temp1=msec;
//	while(flag<(nTime))
//	{
//		if(msec>=temp1)
//		flag=(msec-temp1);
//		else
//		flag=(65535-temp1+msec);
//	};
//}
//void Delay_us(__IO uint32_t nTime)
//{ 
//   __IO uint16_t temp1;
//	temp1=usec;
//	while((usec>temp1)?(usec-temp1):(temp1-usec)<nTime);
//}


//延时nus
//nus为要延时的us数.		    								   
void Delay_us(volatile uint32_t nus)
{		
	volatile uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void Delay_ms(volatile uint16_t nms)
{	 		  	  
	volatile uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 

void get_msticks(unsigned long *timestamp)
{
	*timestamp=NULL;
}

void HAL_Delay(__IO uint32_t Delay)
{
	Delay_ms(Delay);
}
