#include "delay.h"
#include "sys.h"
#include "core_cm0.h"

//static uint8_t  fac_us=0;//us延时倍乘数
//static uint16_t fac_ms=0;//ms延时倍乘数
 __IO uint32_t msec=0;

// __IO uint16_t usec=0;


//初始化延迟函数
//当使用ucos的时候,此函数会初始化ucos的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()	 
{

	  if (SysTick_Config(SystemCoreClock / 8000))
  { 
    /* Capture error */ 
    while (1);
  }
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择时钟  HCLK/8

	 

}								    

void Delay_ms(__IO uint32_t nTime)
{ 
  __IO uint16_t temp1=0;
	uint16_t flag=0;
	temp1=msec;
	while(flag<(nTime))
	{
		if(msec>=temp1)
		flag=(msec-temp1);
		else
		flag=(0xffffffff-temp1+msec);
	};
}
//void Delay_us(__IO uint32_t nTime)
//{ 
//   __IO uint16_t temp1;
//	temp1=usec;
//	while((usec>temp1)?(usec-temp1):(temp1-usec)<nTime);
//}


////延时nus
////nus为要延时的us数.		    								   
//void delay_us(uint32_t nus)
//{		
//	uint32_t temp;	    	 
//	SysTick->LOAD=nus*fac_us; //时间加载	  		 
//	SysTick->VAL=0x00;        //清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数	 
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//	SysTick->VAL =0X00;       //清空计数器	 
//}
////延时nms
////注意nms的范围
////SysTick->LOAD为24位寄存器,所以,最大延时为:
////nms<=0xffffff*8*1000/SYSCLK
////SYSCLK单位为Hz,nms单位为ms
////对72M条件下,nms<=1864 
//void delay_ms(uint16_t nms)
//{	 		  	  
//	uint32_t temp;		   
//	SysTick->LOAD=(uint32_t)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
//	SysTick->VAL =0x00;           //清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
//	do
//	{
//		temp=SysTick->CTRL;
//	}
//	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//	SysTick->VAL =0X00;       //清空计数器	  	    
//} 

