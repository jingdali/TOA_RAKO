/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "mpu9250.h"
#include "I2C.h"
#include "dw1000.h"
#include "deca_callback.h"
#include "testing.h"
#include "MadgwickAHRS.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart1;
IWDG_HandleTypeDef IwdgHandle;
RTC_HandleTypeDef hrtc;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//static dwt_config_t config = {
//    2,               /* Channel number. */
//    DWT_PRF_64M,     /* Pulse repetition frequency. */
//    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
//    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
//    9,               /* TX preamble code. Used in TX only. */
//    9,               /* RX preamble code. Used in RX only. */
//    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
//    DWT_BR_110K,     /* Data rate. */
//    DWT_PHRMODE_STD, /* PHY header mode. */
//    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//};
static dwt_config_t config = {
    1,               /* Channel number. */
    DWT_PRF_16M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,   /* Preamble length. Used in TX only. */
    DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
    2,               /* TX preamble code. Used in TX only. */
    2,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
uint16 Acaddrtable[QUANTITY_ANCHOR]={0x01,0x02,0x03,0x04,0x05};
sys_config_t sys_config = {
	.rangingtype=0,	/* 0 --> TOA , 1 --> TDOA */
	.timebase=1,	/* 1 --> use timebase */
	.mpu_use=0,		/* 1 --> use mpu */
	.pmpudata=NULL,
	.mpudatacnt=200,//200*float
	.mpufreq=10,//ms
	.uwbfreq=1000,//ms
	.TBfreq=1000,//ms
	.id=TAG_ID,
	.anchorcnt=QUANTITY_ANCHOR,
	.panchordis=NULL,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void POLL_TimeWindow(void);
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM14_Init(void);
static void IWDG_init(uint32_t LsiFreq);
static void IWDG_Feed(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void EXTI2_3_IRQHandler_Config(void);
static void EXTI4_15_IRQHandler_Config(void);
void SYSCLKConfig_STOP(void);
void dw_setARER(int enable);
void dw_closeack(void);
int TWRinit(uint16 base_addr,float *dis);
int TOAsend2MainAnch(float *data,int len);
int mpudata_send2MA(void);
static void read_mpudata(float *accel,float *gyro,float *mag);
static void Butterworth_filter(float *data);
static int dw1000_init(void);
static void sysconfig_init(void);
static void get_wdaccel(void);
static void TBprocess(void);
static void TDOAprocess(void);
static void TOAprocess(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static uint16 pan_id = 0xDECA;
static uint8 eui[] = {'A', 'C', 'K', 'D', 'A', 'T', 'R', 'X'};


uint8_t aTxBuffer=0xAA;
uint8_t aRxBuffer;
uint8_t SenLsta=0;
uint8_t SenRsta=0;
uint8_t USART_tmp=0;
uint8_t Tx_Buffer[33] ; // 无线传输发送数据
uint8_t Rx_Buffer[33] ; // 无线传输接收数据
uint8_t nrf_Tx_Buffer[33] ; // nrf无线传输发送数据
uint8_t nrf_Rx_Buffer[33] ; // nrf无线传输接收数据
uint8_t MPUdatabuff[5][33];
unsigned int localtime=0;//本地时间
uint8_t usart_rx_buff[64];//串口buf
uint8 tx_poll_msg[TDOAMSGLEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0x80, 0, 0};

volatile uint8 dw_txseq_num=1;
usart_bitfield USART_STA={
	0,
	0,
	0
};//串口接受狀態

uint32 ACtime=0;
volatile unsigned char newdata=0;
uint8 dwiswake=0;
uint16 TBsynctime=0;
volatile uint8 tim14_int=0;
uint16 newdatacnt=0;
//CALIB DATA
short *pmag_sens_adj;//point to the fuseROM calib data in function setup_compass()
float K_matrix[3][3]={{1.04654758,0, -0.009884},{0.05359849,  1.02752393,  0.03621744},{0, 0, 1.04087807}};
float B_matrix[3]={-17.28529639,66.32626503,-50.01458553};
float accelbias[3]={-755.40186823, -359.90878696, -874.79820164};

//
//FOR TESTING
uint8 triggle=0;
uint8 tmp[5];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM14_Init();
	MX_RTC_Init();
  /* USER CODE BEGIN 2 */
#ifdef	FLASHPROTECT
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();
		#ifdef MAXRDPLEVEL
		FLASH_OB_RDP_LevelConfig(OB_RDP_Level_2);
		#else
		FLASH_OB_RDP_LevelConfig(0xBB);
		#endif
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
#endif
	//disable systick interrupt , I will use the systick in IIC. The hal_delay is strong defined in delay.c
	delay_init();
	usmart_init(48);//in this case, parameter is useless
	HAL_UART_Receive_IT(&huart1, usart_rx_buff, 64);
	EXTI4_15_IRQHandler_Config();
	EXTI2_3_IRQHandler_Config();
	EXTI->PR = 0x7bffff;//clear pending bits
	HAL_TIM_Base_Start_IT(&htim14);//tim14開始計時
	sysconfig_init();
	/* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
//	IWDG_init(40000);
//	IWDG_Feed();
	tim14_int=0;
	

	while(1)
	{
//		IWDG_Feed();
		if(sys_config.timebase==1)
		{
			TBprocess(); 
		}
		else
		{
			if(sys_config.rangingtype==0)
			{
				TOAprocess();
			}
			else
			{
				TDOAprocess();
			}		
		}

//			POLL_TimeWindow();
//			IWDG_Feed();
//			Delay_ms(1000-ACtime%1000);
//			Delay_ms((sys_config.id-0x8000u)*100-3);//每个标签拥有ms的间隔。第一個ms給同步時間信號
//			IWDG_Feed();
//			HAL_TIM_Base_Stop_IT(&htim14);
//			TIM14->CNT=0;
//			HAL_TIM_Base_Start_IT(&htim14);//tim14開始計時	
//			tim14_int=0;
//		HAL_PWR_DisableSleepOnExit();//sleep now
//		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}
}
static void TDOAprocess(void)
{
	uint32 status;
	if(sys_config.mpu_use==0)
	{
		do
		{
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_SET);//wake up
			Delay_us(700);
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_RESET);
			Delay_ms(5);
			status=0x02&dwt_read32bitreg(SYS_STATUS_ID);
		}while(!status);
		dwt_setrxantennadelay(RX_ANT_DLY);
		dwt_settxantennadelay(TX_ANT_DLY);		
		
		tx_poll_msg[FRAME_IDX]=dw_txseq_num++;	
		tx_poll_msg[WLIDX]=0;
		tx_poll_msg[WRIDX]=0;
		tx_poll_msg[UWBFREQ1]=(uint8)sys_config.uwbfreq;
		tx_poll_msg[UWBFREQ2]=(uint8)(sys_config.uwbfreq>>8);
		tx_poll_msg[FUNCODE_IDX]=0x80;
		dwt_writetxdata(TDOAMSGSIZE, tx_poll_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(TDOAMSGSIZE, 0, 1); /* Zero offset in TX buffer, ranging. */
		while(dwt_starttx(DWT_START_TX_IMMEDIATE)!=DWT_SUCCESS)
		{}
		WAIT_SENT(2000)
		isframe_sent=0;
		dwt_entersleep();
		while(!tim14_int);
		tim14_int=0;//tim14 发生中断		
	}
	else
	{
		get_wdaccel();
		do
		{
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_SET);//wake up
			Delay_us(700);
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_RESET);
			Delay_ms(5);
			status=0x02&dwt_read32bitreg(SYS_STATUS_ID);
		}while(!status);
		dwt_setrxantennadelay(RX_ANT_DLY);
		dwt_settxantennadelay(TX_ANT_DLY);	
		tx_poll_msg[FRAME_IDX]=dw_txseq_num++;	
		tx_poll_msg[WLIDX]=0;
		tx_poll_msg[WRIDX]=0;
		tx_poll_msg[UWBFREQ1]=(uint8)sys_config.uwbfreq;
		tx_poll_msg[UWBFREQ2]=(uint8)(sys_config.uwbfreq>>8);
		tx_poll_msg[FUNCODE_IDX]=0x80|0x40;
		tx_poll_msg[MPUFREQ1]=(uint8)sys_config.mpufreq;
		tx_poll_msg[MPUFREQ2]=(uint8)(sys_config.mpufreq>>8);
		tx_poll_msg[MPUCNT1]=(uint8)sys_config.mpudatacnt;
		tx_poll_msg[MPUCNT2]=(uint8)(sys_config.mpudatacnt>>8);
		dwt_writetxdata(TDOAMPUMSGSIZE, tx_poll_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(TDOAMPUMSGSIZE, 0, 1); /* Zero offset in TX buffer, ranging. */
		while(dwt_starttx(DWT_START_TX_IMMEDIATE)!=DWT_SUCCESS)
		{}
		WAIT_SENT(2000)
		isframe_sent=0;
		mpudata_send2MA();
		dwt_entersleep();
	}		
}
static void TOAprocess(void)
{
	uint8 failtime;
	uint8 i=0;
	uint32 status;
	if(sys_config.mpu_use==0)
	{
		do
		{
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_SET);//wake up
			Delay_us(700);
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_RESET);
			Delay_ms(5);
			status=0x02&dwt_read32bitreg(SYS_STATUS_ID);
		}while(!status);
		dwt_setrxantennadelay(RX_ANT_DLY);
		dwt_settxantennadelay(TX_ANT_DLY);				
		
		
		for(i=0;i<sys_config.anchorcnt;i++)
		{
			failtime=0;
			while(failtime!=3)
			{
				if(TWRinit(Acaddrtable[i],sys_config.panchordis+i)!=0)
				{
					failtime++;
				}
				else
				{
					break;
				}
			}
		
		}
//		printf("TWR done, ac1 %f\r\n",*sys_config.panchordis);
		TOAsend2MainAnch(sys_config.panchordis,sys_config.anchorcnt);
		dwt_entersleep();
		while(!tim14_int);
		tim14_int=0;//tim14 发生中断
	}
	else
	{
		get_wdaccel();
		do
		{
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_SET);//wake up
			Delay_us(700);
			HAL_GPIO_WritePin(DWWAKE_GPIO_Port, DWWAKE_Pin, GPIO_PIN_RESET);
			Delay_ms(5);
			status=0x02&dwt_read32bitreg(SYS_STATUS_ID);
		}while(!status);
		dwt_setrxantennadelay(RX_ANT_DLY);
		dwt_settxantennadelay(TX_ANT_DLY);	
		
		failtime=0;
		for(i=0;i<sys_config.anchorcnt;i++)
		{
			while(failtime!=10)
			{
				if(TWRinit(Acaddrtable[i],sys_config.panchordis+i)!=0)
				{
					failtime++;
				}
				else
				{
					break;
				}
			}
		
		}
		TOAsend2MainAnch(sys_config.panchordis,sys_config.anchorcnt);		
		mpudata_send2MA();
		dwt_entersleep();
	}
}
	
static void TBprocess(void)
{
	uint32 addtime;
	uint32 txtime;
	addtime=(uint32)((float)sys_config.TBfreq/0.0000040064);
	
	WAIT_SENT(200*3000)
	isframe_sent=0;
	tx_timestamp=get_tx_timestamp_u64();
	txtime=(uint32)(tx_timestamp>>8)+addtime; 
	dwt_setdelayedtrxtime(txtime);
	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
	dwt_starttx(DWT_START_TX_DELAYED);	
}

static void get_wdaccel(void)
{
	float accel[3];
	float gyro[3];
	float mag[3];
	float wdaccel[3];
	uint8 i=0;
	uint8 mpucnt=sys_config.uwbfreq/sys_config.mpufreq;	
	
	while(i!=mpucnt)
	{
		while(newdatacnt)
		{
			i++;
			newdatacnt--;
			read_mpudata(accel,gyro,mag);
			Butterworth_filter(accel);
			Horizonaccel(gyro,accel,mag,wdaccel);
			sys_config.pmpudata[2*i-2]=wdaccel[0];
			sys_config.pmpudata[2*i-1]=wdaccel[1];

			if(i==mpucnt)
			{
				break;
			}
		}
	}

	
}
static void sysconfig_init(void)
{
	dw1000_init();
	sys_config.panchordis=(float*)malloc(sys_config.anchorcnt*sizeof(float));
	if(sys_config.mpu_use==1)
	{
		sys_config.pmpudata=(float*)malloc(sys_config.mpudatacnt*sizeof(float));
		if(!sys_config.pmpudata)
		{
			printf("not enough memeory!!!\r\n");
			while(1)
			{
			}
		}
		
	}
/*
	MESSAGE
*/if(!sys_config.timebase)
	{
		printf("TAG ID: %d\r\n",sys_config.id&0x7fff);
		if(!sys_config.rangingtype)
		{
			printf("RANGING TYPE: TOA\r\n");
		}
		else
		{
			printf("RANGING TYPE: TDOA\r\n");
		}
		printf("UWB FREQUENCY: %d ms\r\n",sys_config.uwbfreq);
		printf("ANCHOR NUM: %d\r\n",sys_config.anchorcnt);
		
		if(sys_config.mpu_use)
		{
			printf("MPU enabled.\r\n");
			printf("MPU FREQUENCY: %d ms\r\n",sys_config.mpufreq);
//			HAULT_POINT
			if(MPU9250_Init())
			{
				printf("MPU initialized unsuccessful.\r\n");
			}
		}
		else
		{
			printf("MPU disabled.\r\n");
		}
			
	}	
	else
	{
		tx_poll_msg[FRAME_IDX]=dw_txseq_num++;	
		tx_poll_msg[WLIDX]=0;
		tx_poll_msg[WRIDX]=0;
		tx_poll_msg[UWBFREQ1]=(uint8)sys_config.TBfreq;
		tx_poll_msg[UWBFREQ2]=(uint8)(sys_config.TBfreq>>8);		
		dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
		dwt_starttx(DWT_START_TX_IMMEDIATE);	
	}

}
static void EXTI4_15_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  __HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
  /* Enable and set EXTI line 0_1 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
static void EXTI2_3_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Pin = DW_IRQ_Pin;
  HAL_GPIO_Init(DW_IRQ_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set EXTI line 2_3 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}
//================

//================
int mpudata_send2MA(void)
{
	uint8 tx_TOAdata[112]={0x41,0x88,0,0xCA, 0xDE,0x01, 0x00, 0x00, 0x00,0x20};//TOA数据
	uint16 TimeOutCNT=0;
	uint8 dataidx;
	unsigned char i;
	uint8 tanscnt=sys_config.mpudatacnt*sizeof(float)/100;
	tx_TOAdata[SOURADD]=(uint8)sys_config.id;
	tx_TOAdata[SOURADD+1]=(uint8)(sys_config.id>>8);
	dwt_setrxtimeout(3000);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);	
	while(1)
	{
		WAIT_REC_TO(9000)
		if(isreceive_To==1)
		{
			isreceive_To=0;
			TimeOutCNT++;
			if(TimeOutCNT==4)
			{
				TimeOutCNT=0;
				break;
			}
			else
			{
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
		}
		else
		{
			isframe_rec=0;
			dataidx=rx_buffer[FRAME_IDX];
			tx_TOAdata[FRAME_IDX]=dataidx;
			memcpy(tx_TOAdata+10,(uint8*)sys_config.pmpudata+dataidx*100,100);
			dwt_writetxdata(112, tx_TOAdata, 0);
			dwt_writetxfctrl(112, 0, 0);
			dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
			WAIT_SENT(2000)
			isframe_sent=0;
			TimeOutCNT=0;
		}

	}	
	
	dwt_setrxtimeout(0);
	return 0;
}



void dw_setARER(int enable)
{
	uint32 syscfg;
	syscfg=dwt_read32bitreg(SYS_CFG_ID);
	if(enable)
	{
		syscfg |= SYS_CFG_RXAUTR;
	}
	else
	{
		syscfg &=~(SYS_CFG_RXAUTR);
	}
	dwt_write32bitreg(SYS_CFG_ID,syscfg);
}

void dw_closeack(void)
{
	uint32 tmp;
	tmp=dwt_read32bitreg(SYS_CFG_ID);
	tmp &= ~SYS_CFG_AUTOACK;
  dwt_write32bitreg(SYS_CFG_ID,tmp) ;
}
void POLL_TimeWindow(void)
{
	
	uint8 tx_poll_time[12]={0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, (uint8)sys_config.id, (uint8)(sys_config.id>>8), 0x2B, 0, 0};//用来查询时间戳的
	dwt_setrxtimeout(800);//设置接受超时
	dwt_writetxdata(sizeof(tx_poll_time), tx_poll_time, 0);
	dwt_writetxfctrl(sizeof(tx_poll_time), 0, 0);
	ACtime=0;
	while(!ACtime)
	{
		do
		{
			isreceive_To=0;
			dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
			WAIT_SENT(2000)
			isframe_sent=0;	
			WAIT_REC_TO(2300)
			if(isreceive_To==1)
			{
				isreceive_To=0;
				//printf("Time out\r\n");
				Delay_ms(200);
			}
			
		}while(isframe_rec!=1);
		isframe_rec=0;		
		if(rx_buffer[FUNCODE_IDX]==0x2C)
		{
				ACtime=rx_buffer[10];
				ACtime+=(uint32)((rx_buffer[11])<<8);
				ACtime+=(uint32)((rx_buffer[12])<<16);
				ACtime+=(uint32)((rx_buffer[13])<<24);
				//TBsynctime=(uint16)(rx_buffer[14]);
				//TBsynctime+=(uint16)(rx_buffer[15]<<8);
				//localtime=ACtime;
		}
	}
	//printf("ACtime=%lx\r\n",ACtime);
	dwt_setrxtimeout(0);
	
}
static int dw1000_init(void)
{
	decaIrqStatus_t  stat ;
	reset_DW1000();
	tx_poll_msg[SOURADD]=(uint8)sys_config.id;
	tx_poll_msg[SOURADD+1]=(uint8)(sys_config.id>>8);
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
			printf("INIT FAILED\r\n");
			return -1;
	}
	else
	{
		printf("UWB Device initialised\r\n");
	}

	stat = decamutexon();// care should be taken that the spi should never use interrupt cause the interrupts are disabled.
	dwt_configure(&config);
	dwt_setpanid(pan_id);
  dwt_seteui(eui);
  dwt_setaddress16(sys_config.id);
	dwt_setleds(DWT_LEDS_ENABLE);//set the led

	port_set_deca_isr(dwt_isr);		
	dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
	dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
	decamutexoff(stat) ;
	//dw_setARER(1);//使能接收机自动重启
	dwt_enableautoack(5);//使能自动应答
	dwt_enableframefilter(DWT_FF_DATA_EN|DWT_FF_ACK_EN);//使能帧过滤
	
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS|SYS_STATUS_RXFCG|SYS_STATUS_SLP2INIT);//清除标志位
	dwt_setinterrupt(0xffff,0);//关闭中断
	dwt_setinterrupt(DWT_INT_ALLERR|DWT_INT_TFRS|DWT_INT_RFCG|DWT_INT_RFTO,1);//开启中断
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	//dwt_setpreambledetecttimeout(8);//需要查驗，不確定效果
//	lp_osc_freq = (XTAL_FREQ_HZ / 2) / dwt_calibratesleepcnt();
//	sleep_cnt = ((SLEEP_TIME_MS * lp_osc_freq) / 1000) >> 12;	
//dwt_configuresleepcnt(sleep_cnt);
	dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG |DWT_LLDLOAD|DWT_LLD0, DWT_WAKE_WK | DWT_SLP_EN);
	dwt_setlnapamode(1,1);
	dwt_write16bitoffsetreg(PMSC_ID,PMSC_RES3_OFFSET+2,0);
	return 0;
}
/*

*/
int TWRinit(uint16 base_addr,float *distance)//單次測距函數
{
	//虽然这些数据很多，但是默认栈有1kb，所以不会有什么问题
	uint8 TimeOutCNT=0;
	uint32 delayed_resp_time;
	uint64 poll_rx_ts, resp_tx_ts, final_rx_ts;
	uint64 poll_tx_ts, resp_rx_ts, final_tx_ts;
	double Ra,Rb,Da,Db;
	int ret;
	double tof_dtu;
	double tof;
	uint8 tx_TOAbuff[12]={0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0, 0, 0x10};//TOA定位所使用的buff
	tx_TOAbuff[SOURADD]=(uint8)sys_config.id;
	tx_TOAbuff[SOURADD+1]=(uint8)(sys_config.id>>8);
	tx_TOAbuff[DESTADD]=(uint8)base_addr;
	tx_TOAbuff[DESTADD+1]=(uint8)(base_addr>>8);
	tx_TOAbuff[FRAME_IDX]=dw_txseq_num++;
	tx_TOAbuff[0]=0x41;//
	dwt_writetxdata(12, tx_TOAbuff, 0);//发起定位请求
	dwt_writetxfctrl(12, 0, 0);
//	dwt_setrxtimeout(800);//设置接受超时
//	do
//	{

//		ret=dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
//		if(ret == DWT_ERROR)
//		{
//			*distance=0;
//			return -2;	
//		}
//		while(!isframe_sent);
//		isframe_sent=0;
//		while(!isreceive_To&&!istxframe_acked);
//		if(isreceive_To==1)
//		{
//			printf("W4ACk TO\r\n");
//			isreceive_To=0;
//			TimeOutCNT++;
//		}
//		if(TimeOutCNT==3)
//		{
//			*distance=0;
//			goto error1;			
//		}
//	}while(istxframe_acked!=1);
//	istxframe_acked=0;
	TimeOutCNT=0;
	dwt_setrxtimeout(2000);//设置接受超时
	do
	{
		ret=dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
		WAIT_SENT(2000)
		if(ret == DWT_ERROR)
		{
				*distance=0;
				goto error2;
		}
		WAIT_REC_TO(6000)
		if(isreceive_To==1)
		{
			isreceive_To=0;
			TimeOutCNT++;
		}
		if(TimeOutCNT==1)
		{
			TimeOutCNT=0;
			*distance=0;
			Delay_ms(3);//保证基站已经退出定位子程序
			goto error1;			
		}
	}while(!isframe_rec);
	isframe_rec=0;//接受到第一次数据
	TimeOutCNT=0;	
	poll_rx_ts = get_rx_timestamp_u64();
	delayed_resp_time = (poll_rx_ts + (RESP_TX_DELAYED_UUS * UUS_TO_DWT_TIME)) >> 8;
  dwt_setdelayedtrxtime(delayed_resp_time);
	tx_TOAbuff[0]=0x41;//不需要應答
	tx_TOAbuff[FUNCODE_IDX]=0x12;
//	dwt_setrxaftertxdelay(1);//here some value can be set to reduce power consumption
	dwt_setrxtimeout(2200);	
	dwt_writetxdata(12, tx_TOAbuff, 0);//
	dwt_writetxfctrl(12, 0, 1);	
	
	ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
	if(ret == DWT_ERROR)
	{
			*distance=0;
			goto error2;
	}
	WAIT_SENT(2000)
	isframe_sent=0;	
	WAIT_REC_TO(6000)
	if(isreceive_To==1)
	{
			//printf("WF_I_2_TO\r\n");
			isreceive_To=0;
			*distance=0;
			goto error1;
	}
	isframe_rec=0;
	dwt_setrxaftertxdelay(0);
	resp_tx_ts = get_tx_timestamp_u64();
	final_rx_ts = get_rx_timestamp_u64();
	
	final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
	final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
	final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
	
	Ra = (double)(resp_rx_ts - poll_tx_ts);
	Rb = (double)(final_rx_ts - resp_tx_ts);
	Da = (double)(final_tx_ts - resp_rx_ts);
	Db = (double)(resp_tx_ts - poll_rx_ts);
	tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);
	tof = tof_dtu * DWT_TIME_UNITS;
  *distance = RANGE_BIAS+(float)tof * SPEED_OF_LIGHT;
	return 0;
error1:
	//printf("error1\r\n");
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	return -1;
error2:
	//printf("error2\r\n");
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	return -2;
}


int TOAsend2MainAnch(float *data,int len)//發送數據給主機站
{
	uint8 tx_TOAdata[TOA_MSG_LEN]={0x61,0x88,0,0xCA, 0xDE,0x01, 0x00, 0x00, 0x00,0x1a};//发送TOA数据
	uint16 TimeOutCNT=0;
	tx_TOAdata[SOURADD]=(uint8)sys_config.id;
	tx_TOAdata[SOURADD+1]=(uint8)(sys_config.id>>8);
	tx_TOAdata[FRAME_IDX]=dw_txseq_num++;
	tx_TOAdata[WLIDX]=0;//佩戴信息
	tx_TOAdata[WRIDX]=0;
	tx_TOAdata[UWBFREQ1]=(uint8)sys_config.uwbfreq;
	tx_TOAdata[UWBFREQ2]=(uint8)(sys_config.uwbfreq>>8);		
	if(sys_config.mpu_use)
	{
		tx_TOAdata[FUNCODE_IDX]=0x1a|0x40;
		tx_TOAdata[MPUFREQ1]=(uint8)sys_config.mpufreq;
		tx_TOAdata[MPUFREQ2]=(uint8)(sys_config.mpufreq>>8);
		tx_TOAdata[MPUCNT1]=(uint8)sys_config.mpudatacnt;
		tx_TOAdata[MPUCNT2]=(uint8)(sys_config.mpudatacnt>>8);
		memcpy(tx_TOAdata+TOAMPU_DATA_IDX,data,len*sizeof(float));			
		dwt_writetxdata(TOAMPUMSGSIZE, tx_TOAdata, 0);
		dwt_writetxfctrl(TOAMPUMSGSIZE, 0, 0);
	}
	else
	{
		memcpy(tx_TOAdata+TOA_DATA_IDX,data,len*sizeof(float));
		dwt_writetxdata(TOAMSGSIZE, tx_TOAdata, 0);
		dwt_writetxfctrl(TOAMSGSIZE, 0, 0);		
	}

	dwt_setrxtimeout(2000);//设置接受超时
	TimeOutCNT=0;
	do
	{
		dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
		WAIT_SENT(2000)
		isframe_sent=0;
		WAIT_REC_ACK(5600)
		if(isreceive_To==1)
		{
			//printf("wait 4 ack Time out\r\n");
			isreceive_To=0;
			TimeOutCNT++;
		}
		if(TimeOutCNT==3)
		{
			return -1;			
		}
	}while(istxframe_acked!=1);
	istxframe_acked=0;	
	dwt_setrxtimeout(0);//设置接受超时
	return 0;
}

static void read_mpudata(float *accel,float *gyro,float *mag)
{
	short	gyro_short[3];
	short	accel_short[3];
	short	mag_short[3];
	unsigned char fifodata[20];
	unsigned char index=0;
	unsigned char i;
	float mag_t[3];
//	mpu_read_fifo_stream(20, fifodata, &more);
	I2C_ReadRegister_9250(0x68, 0x74, 20, fifodata);
	accel_short[0] = (fifodata[index+0] << 8) | fifodata[index+1];
	accel_short[1] = (fifodata[index+2] << 8) | fifodata[index+3];
	accel_short[2] = (fifodata[index+4] << 8) | fifodata[index+5];
	index+=6;
	gyro_short[0] = (fifodata[index+0] << 8) | fifodata[index+1];
	gyro_short[1] = (fifodata[index+2] << 8) | fifodata[index+3];
	gyro_short[2] = (fifodata[index+4] << 8) | fifodata[index+5];
	index+=6;
	mag_short[0] = (fifodata[index+2] << 8) | fifodata[index+1];
	mag_short[1] = (fifodata[index+4] << 8) | fifodata[index+3];
	mag_short[2] = (fifodata[index+6] << 8) | fifodata[index+5];
	mag_short[0] = ((long)mag_short[0] * pmag_sens_adj[0]) >> 8;
	mag_short[1] = ((long)mag_short[1] * pmag_sens_adj[1]) >> 8;
	mag_short[2] = ((long)mag_short[2] * pmag_sens_adj[2]) >> 8;
	for(i=0;i<3;i++)
	{
		mag_t[i]=(float)mag_short[i]-B_matrix[i];
	}
	mag[1]=mag_t[0]*K_matrix[0][0]+mag_t[1]*K_matrix[0][1]+mag_t[2]*K_matrix[0][2];
	mag[0]=mag_t[0]*K_matrix[1][0]+mag_t[1]*K_matrix[1][1]+mag_t[2]*K_matrix[1][2];
	mag[2]=-(mag_t[0]*K_matrix[2][0]+mag_t[1]*K_matrix[2][1]+mag_t[2]*K_matrix[2][2]);
	for(i=0;i<3;i++)
	{
		accel[i]=((float)accel_short[i]+accelbias[i])/16384.0f*9.8f;
		mag[i]*=0.15f;
		gyro[i]=(float)gyro_short[i]/32.768f/57.296f;
	}
	
}
static void Butterworth_filter(float *data)//sample rate 100hz, cut frequency 10hz, order 2
{
	//使用static就不用每次調用的時候分配棧了
	static float a[3]={1.0, -1.1429805,  0.4128016};
	static float b[3]={0.06745527,  0.13491055,  0.06745527};
	static float x[3][3]={{0,0,0},{0,0,0},{0,0,0}};
	static float y[3][3]={{0,0,0},{0,0,0},{0,0,0}};
	uint8 i;
	for(i=0;i<3;i++)
	{
		x[i][0]=x[i][1];
		x[i][1]=x[i][2];
		x[i][2]=(float)data[i];
		y[i][0]=y[i][1];
		y[i][1]=y[i][2];
		y[i][2]=b[0]*x[i][2] + b[1]*x[i][1]+b[0]*x[i][2]-a[1]*y[i][1]-a[2]*y[i][0];
		data[i]=y[i][2];
	}
}
void IWDG_init(uint32_t LsiFreq)
{
	/*##-3- Configure the IWDG peripheral ######################################*/
  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     IWDG counter clock Frequency = LsiFreq / 32
     Counter Reload Value = 250ms / IWDG counter clock period
                          = 0.25s / (32/LsiFreq)
                          = LsiFreq / (32 * 4)
                          = LsiFreq / 128 */
  IwdgHandle.Instance = IWDG;

  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;
  IwdgHandle.Init.Reload    = LsiFreq / 12;
  IwdgHandle.Init.Window    = IWDG_WINDOW_DISABLE;

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	/*##-4- Start the IWDG #####################################################*/
  if (HAL_IWDG_Start(&IwdgHandle) != HAL_OK)
  {
    Error_Handler();
  }
}


void IWDG_Feed(void)
{
	
	/* Refresh IWDG: reload counter */
    if (HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
    {
      /* Refresh Error */
      Error_Handler();
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

//  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void SYSCLKConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  uint32_t pFLatency = 0;

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
	
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
	
}
/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the WakeUp 
    */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 16383, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4799;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = sys_config.uwbfreq*10;//1000ms 中断一次
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = SWICH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWICH_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = MPU_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MPU_CE_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LED_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pin : DW_IRQ_Pin */
  GPIO_InitStruct.Pin = DW_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DW_IRQ_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pin : DWWAKE_Pin */
  GPIO_InitStruct.Pin = DWWAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DWWAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DWRESET_Pin */
  GPIO_InitStruct.Pin = DWRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DWRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENL_INT_Pin */
  GPIO_InitStruct.Pin = SENL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SENL_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENR_INT_Pin */
  GPIO_InitStruct.Pin = SENR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SENR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU_SCL_Pin MPU_SDA_Pin */
  GPIO_InitStruct.Pin = MPU_SCL_Pin|MPU_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin Output Level */

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DWWAKE_Pin|MPU_SCL_Pin|MPU_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
	printf("error\r\n");
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
