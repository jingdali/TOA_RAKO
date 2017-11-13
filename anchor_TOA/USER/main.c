#include "sys.h"
#include "spi_v1.h"
#include "usart_v1.h"
#include "delay.h"
#include "LED.h"
#include "usmart.h"
#include "EXTI_v1.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "string.h"
#include "deca_callback.h"
#include "rf24l01.h"
#include "Deca_user_api.h"
#include "test_fun.h"
//#define FLASHPROTECT
//#define MAXRDPLEVEL
#define TGDATA_BUFFLEN 10

/* Declaration of static functions. */
static int WirelessSyncDataProcess_MA(void);
static int WirelessSyncDataProcess_SA(void);
static int AssignTimeWindow(void);
static int TOAdata_process(void);
static int DS_TwoWayRanging(void);
static void Send2PC(void);
void TIM7_init(void);
void SWITCH_DB(void);
void cacu_crc(void);

void start_DMA(uint8 data_cnt);
void DMA_init(void);
void CRC_init(void);

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

uint8_t nrf_Tx_Buffer[33] ; // nrf无线传输发送数据
uint8_t nrf_Rx_Buffer[33] ; // nrf无线传输接收数据
/* Frames used in the ranging process. See NOTE 2 below. */
/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
static uint16 pan_id = 0xDECA;
static uint8 eui[] = {'A', 'C', 'K', 'D', 'A', 'T', 'R', 'X'};
static uint16 Achor_addr = ANCHOR_NUM; /* "RX" */

uint8 rx_buffer[RX_BUF_LEN];
volatile uint8 DMA_transing=0;
uint8 frame_seq_nb=0;
//uint8 ACKframe[12]={0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0xAC, 0, 0};
uint8 ACKframe[5]={ACK_FC_0,ACK_FC_1,0,0,0};
uint8 send_pc[22]={0xFB, 0xFB, 0x11, 0, 0, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 dw_payloadbuff[127];
uint16 TBsyctime=0xff;

int idxreco=-1;
uint16 TAG_datacnt=0;
uint8 crc=0;
uint8 QuantityAC=QUANTITY_ANCHOR;
#ifdef TIMEBASE

uint8 TBPOLL[14] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0x80, 0x80, 0, 0};

uint32 sync_preiod=0;
uint32 preiodtable[]={
											0x7EDFC54UL,//533ms 0
											0x7704D3FUL,//500ms 1
											0x6712014UL,//433ms 2
											0x5F370FCUL,//400ms 3
											0x4F443D4UL,//333ms 4
											0x47694BFUL,//300ms 5
											0x3776794UL,//233ms 6
											0x2F9B87FUL,//200ms 7
											0x1FA8B54UL,//133ms 8
											0x17CDC3FUL,//100ms 9
											};
uint16 preiodtable_ms[]={
											533,//533ms 0
											500,//500ms 1
											433,//433ms 2
											400,//400ms 3
											333,//333ms 4
											300,//300ms 5
											233,//233ms 6
											200,//200ms 7
											133,//133ms 8
											100,//100ms 9
											};
#endif


struct HEAD_str
{
	uint16 STX;
	uint16 LEN;
	uint16 ADDR;
	uint16 CMD;
	//
	uint16 tagcnt;
	uint16 anchcnt;
}	HEAD;
struct ANCHOR_TBDATA_str
{
	uint16 DATATYPE;
	uint16 anchnum;
	uint16 tagnum;
	uint16 idx;
	uint64 timestamp;
}AC_DATA[ANCHORCNT];//16*anchorcnt bytes
struct TG_DATA_str
{

	uint16 DATATYPE;
	uint8 wearIDCA;//佩戴标记 高4位为左 低4位为右
	uint8 validcnt;
	uint16 tagnum;
	uint16 idx;
	uint64 anchTS[ANCHORCNT];
	#ifdef MPUUSING
	uint8 MPUdata[4][28];
	#endif
}TG_DATA[TGDATA_BUFFLEN];//*10 bytes



int main(void)
{

	decaIrqStatus_t  stat ;
	uint32 status;
	uint8 MPUdataidx;
#ifdef	FLASHPROTECT
	FLASH_Unlock();
	FLASH_OB_Unlock();
		#ifdef MAXRDPLEVEL
		FLASH_OB_RDPConfig(OB_RDP_Level_2);
		#else
		FLASH_OB_RDPConfig(OB_RDP_Level_1);
		#endif
	FLASH_OB_Lock();
	FLASH_Lock();
#endif
	CRC_init();
	LED_Init();
	delay_init();
  USART_Config();
	usmart_dev.init(48);
	spi_init();
	reset_DW1000();
	SPI_RF_Init();
	nrf24l01_init();
	if(NRF24L01_Check())
	{
		printf("24L01 not exist\r\n");
	}
	else
	{
		printf("24L01 check ok\r\n");
	}

	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)	//dw1000 init
		{
				printf("INIT FAILED\r\n");
				while (1)
				{ };
		}
	
		dw1000IRQ_init();
		stat = decamutexon() ;
		set_spi_high();
		dwt_configure(&config);	
		dwt_setleds(DWT_LEDS_ENABLE);
		port_set_deca_isr(dwt_isr);
		decamutexoff(stat) ;
//--------------------------------
		/* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
		 /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG|SYS_STATUS_SLP2INIT);
    dwt_setinterrupt(DWT_INT_ALLERR|DWT_INT_TFRS|DWT_INT_RFCG|DWT_INT_RFTO,1);
		//dw_setARER(1);
		dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
		
		dwt_setpanid(pan_id);
    dwt_seteui(eui);
    dwt_setaddress16(Achor_addr);

    /* Configure frame filtering. Only data frames are enabled in this example. Frame filtering must be enabled for Auto ACK to work. */
    //dwt_enableframefilter(DWT_FF_DATA_EN);
		dwt_enableframefilter(DWT_FF_DATA_EN|DWT_FF_ACK_EN);

    /* Activate auto-acknowledgement. Time is set to 0 so that the ACK is sent as soon as possible after reception of a frame. */
    dwt_enableautoack(8);
		dwt_setrxtimeout(0);
		
#ifdef 		MAIN_ANCHOR
//		dwt_setdblrxbuffmode(1);
		DMA_init();
//		NRF24L01_RX_Mode();
//		TIM7_init();
#endif

#ifdef TIMEBASE	
	sync_preiod=preiodtable[SYNCIDX];
	TBPOLL[2]=frame_seq_nb++;	
	dwt_writetxdata(sizeof(TBPOLL), TBPOLL, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(TBPOLL), 0, 1); /* Zero offset in TX buffer, ranging. */
	while(dwt_starttx(DWT_START_TX_IMMEDIATE)!=DWT_SUCCESS);
#else
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

#endif
	 while(1) 
	 {
		
	#ifdef MAIN_ANCHOR

		 while(Qcnt)
			{
				switch(Que[front].buff[FUNCODE_IDX])
					{
						case 0x80:WirelessSyncDataProcess_MA();break;
						case 0x81:WirelessSyncDataProcess_MA();break;
						case 0x2B:AssignTimeWindow();break;
						//case 0x1A:TOAdata_process();break;
						case 0x1A:
							Send2PC();
							break;
						case 0x10:
							DS_TwoWayRanging();
							break;
						default:;//printf("UNknow cmd\r\n");
					}
							
				front=(front+1)%Que_Length;
				Qcnt--;
				
			}



		#elif defined TIMEBASE
			//时钟基站的代码
		while(!isframe_sent);
		isframe_sent=0;
		tx_timestamp=get_tx_timestamp_u64();
		delayed_txtime=(uint32)(tx_timestamp>>8)+sync_preiod;//300ms
		dwt_setdelayedtrxtime(delayed_txtime);
		TBPOLL[2]=frame_seq_nb++;
		TBPOLL[10]=(uint8)preiodtable_ms[SYNCIDX];
		TBPOLL[11]=(uint8)(preiodtable_ms[SYNCIDX]>>8);
		dwt_writetxdata(sizeof(TBPOLL), TBPOLL, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(TBPOLL), 0, 1); /* Zero offset in TX buffer, ranging. */
		while(dwt_starttx(DWT_START_TX_DELAYED)!=DWT_SUCCESS);
		#else
			//从基站的程序代码
		while(Qcnt)
		{
			switch(Que[front].buff[FUNCODE_IDX])
			{
				case 0x80:WirelessSyncDataProcess_SA();break;
				case 0x10:DS_TwoWayRanging();break;
				default:;//printf("unkonwn cmd\r\n");
			}
			front=(front+1)%Que_Length;
			Qcnt--;
		}


		#endif

    }			
				
	}
/*========================================
	分支功能代码实现部分
	========================================*/
int WirelessSyncDataProcess_MA(void)
{
	uint16 anchnumtmp;
	uint16 tagnumtmp;
	uint8 idx=0,i=0,wearing=0;
	if(Que[front].buff[FUNCODE_IDX]==0x80)//處理得到相關數據，0x80说明是来自标签的
		{
			tagnumtmp=Que[front].buff[SOURADD];
			tagnumtmp+=(Que[front].buff[SOURADD+1]-128)<<8;
			anchnumtmp=ANCHOR_NUM;
			rx_timestamp=Que[front].rx_timestamp;
			idx=Que[front].buff[FRAME_SN_IDX];
			if(tagnumtmp!=0)//判断是否来自同步时基
			{
				wearing=0;
				wearing|=(Que[front].buff[WLIDX])<<4;
				wearing|=(Que[front].buff[WRIDX]);
			}
			else
			{
				TBsyctime=Que[front].buff[WLIDX];
				TBsyctime+=Que[front].buff[WRIDX]<<8;
			}
		}
		else if(Que[front].buff[FUNCODE_IDX]==0x81)
		{
			final_msg_get_ts(&Que[front].buff[RXBUFFTS_IDX], &rx_timestamp);
			anchnumtmp=Que[front].buff[SOURADD];
			anchnumtmp+=Que[front].buff[SOURADD+1]<<8;
			tagnumtmp=Que[front].buff[TIMSTAMPS_OWNERID];
			tagnumtmp+=(Que[front].buff[TIMSTAMPS_OWNERID+1]-128)<<8;
			idx=Que[front].buff[FRAME_SN_IDX];
			if(tagnumtmp!=0)
			{
				wearing=0;
				wearing=(Que[front].buff[RXBUFF_WLIDX])<<4;
				wearing=(Que[front].buff[RXBUFF_WRIDX]);
			}				
			else
			{
				TBsyctime=Que[front].buff[RXBUFF_WLIDX];
				TBsyctime+=Que[front].buff[RXBUFF_WRIDX]<<8;
			}
		}

		
		if(tagnumtmp==0)
		{
			//时钟同步数据
			if(idxreco==-1)
			{
				idxreco=idx;
			}
			if(idx==idxreco)
			{
				AC_DATA[anchnumtmp-1].anchnum=anchnumtmp;
				AC_DATA[anchnumtmp-1].tagnum=tagnumtmp;
				AC_DATA[anchnumtmp-1].idx=idx;
				AC_DATA[anchnumtmp-1].timestamp=rx_timestamp;	
				AC_DATA[anchnumtmp-1].DATATYPE=0xaa;
			}
			else
			{
				while(DMA_transing);
				HEAD.anchcnt=4;
				HEAD.ADDR=1;
				HEAD.CMD=0x02;
				HEAD.LEN=sizeof(AC_DATA)+TAG_datacnt*sizeof(struct TG_DATA_str)+3;
				HEAD.STX=0xfbfb;
				HEAD.tagcnt=TAG_datacnt;
				cacu_crc();
				//usart_trans((uint8*)&HEAD,sizeof(HEAD));//如果地址不連續，就必須分開傳輸
				start_DMA(TAG_datacnt);//發送數據
				while(DMA_transing);
				fputc((int)crc,NULL);
				crc=0;
				memset(AC_DATA,0,sizeof(struct ANCHOR_TBDATA_str)*ANCHORCNT);
				memset(TG_DATA,0,sizeof(struct TG_DATA_str)*TAG_datacnt);
				
				TAG_datacnt=0;
				idxreco=idx;
				AC_DATA[anchnumtmp-1].anchnum=anchnumtmp;
				AC_DATA[anchnumtmp-1].tagnum=tagnumtmp;
				AC_DATA[anchnumtmp-1].idx=idx;
				AC_DATA[anchnumtmp-1].timestamp=rx_timestamp;
				AC_DATA[anchnumtmp-1].DATATYPE=0xaa;
				
			}

		}
		else//标签数据处理
		{
			for(i=0;i<TGDATA_BUFFLEN;i++)
			{
				if((TG_DATA[i].tagnum==tagnumtmp)&&(TG_DATA[i].idx==idx))
				{//说明是存在这个标签的数据的
					TG_DATA[i].validcnt|=1<<(anchnumtmp-1);
					TG_DATA[i].anchTS[anchnumtmp-1]=rx_timestamp;
					TG_DATA[i].wearIDCA|=wearing;
					break;
				}
				else if(TG_DATA[i].tagnum==0)
				{//说明没遍历到相关数据
					TG_DATA[i].tagnum=tagnumtmp;
					TG_DATA[i].idx=idx;
					TG_DATA[i].validcnt|=1<<(anchnumtmp-1);
					TG_DATA[i].anchTS[anchnumtmp-1]=rx_timestamp;
					TG_DATA[i].DATATYPE=0xbb;
					TG_DATA[i].wearIDCA|=wearing;
					#ifdef MPUUSING
					if(anchnumtmp==1)
					{
						memcpy(TG_DATA[i].MPUdata,Que[front].buff+12,112);
					}
					#endif
					TAG_datacnt++;
					
					break;
				}							
				
			}
			
			
		}
		return 0;
}
int WirelessSyncDataProcess_SA(void)
{
	uint16 tagnumtmp;
	tagnumtmp=Que[front].buff[SOURADD];
	tagnumtmp+=(Que[front].buff[SOURADD+1]-128)<<8;
	if(tagnumtmp==0)
	{
		//来自时钟基站的数据
		final_msg_set_ts(&dw_payloadbuff[PAYLOADTS_IDX], Que[front].rx_timestamp);
		dw_payloadbuff[0]=Que[front].buff[SOURADD];
		dw_payloadbuff[1]=Que[front].buff[SOURADD+1];
		dw_payloadbuff[PAYLOOAD_WLIDX]=Que[front].buff[WLIDX];//在時鐘基站的數據裏面，這個是時鐘同步週期的值而不是佩戴信息
		dw_payloadbuff[PAYLOOAD_WRIDX]=Que[front].buff[WRIDX];					
		dw_txframe[FRAME_SN_IDX]=Que[front].buff[FRAME_SN_IDX];
		while(Que[front].arrivetime+(ANCHOR_NUM-1)*2>msec);
		dwt_forcetrxoff();
		SentFrame_ack(dw_payloadbuff, 9, 1, 0x81);	
		
	}
	else
	{
		//来自标签的数据
		final_msg_set_ts(&dw_payloadbuff[PAYLOADTS_IDX], Que[front].rx_timestamp);
		dw_payloadbuff[0]=Que[front].buff[SOURADD];
		dw_payloadbuff[1]=Que[front].buff[SOURADD+1];
		dw_payloadbuff[PAYLOOAD_WLIDX]=Que[front].buff[WLIDX];
		dw_payloadbuff[PAYLOOAD_WRIDX]=Que[front].buff[WRIDX];	
		dw_txframe[FRAME_SN_IDX]=Que[front].buff[FRAME_SN_IDX];
		while(Que[front].arrivetime+(ANCHOR_NUM-1)*2>msec);
		dwt_forcetrxoff();
		SentFrame_ack(dw_payloadbuff, 9, 1, 0x81);	
		
	}
	return 0;
}
int AssignTimeWindow(void)
{
	uint8 tx_resp_time[18]={0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x01, 0x00, 0x2C, 0, 0, 0, 0};
//SET_Tpoint();
	tx_resp_time[DESTADD]=Que[front].buff[SOURADD];
	tx_resp_time[DESTADD+1]=Que[front].buff[SOURADD+1];
//	*(uint32*)(tx_resp_time+10)=msec;
	tx_resp_time[10]=(uint8)(msec);
	tx_resp_time[11]=(uint8)(msec>>8);
	tx_resp_time[12]=(uint8)(msec>>16);
	tx_resp_time[13]=(uint8)(msec>>24);
	tx_resp_time[14]=(uint8)(TBsyctime);
	tx_resp_time[15]=(uint8)(TBsyctime>>8);
	dwt_forcetrxoff();
	dwt_writetxdata(sizeof(tx_resp_time), tx_resp_time, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_resp_time), 0, 0); /* Zero offset in TX buffer, ranging. */
//GET_Time2Tpoint();
	dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
	while(!isframe_sent);
	isframe_sent=0;
//	while(timestack_cnt)
//	{
//		double tmp=(double)time_stack[timestack_cnt-1]*4/1000;
//		printf("%d:%f us \r\n",timestack_cnt--,tmp);
//	}
	return 0;
}

int TOAdata_process(void)
{
	uint16 tagnumtmp;
	uint8 idx=0;
	tagnumtmp=Que[front].buff[SOURADD];
	tagnumtmp+=(Que[front].buff[SOURADD+1]-128)<<8;
	idx=Que[front].buff[FRAME_SN_IDX];
	printf("TAG:%d No:%d\r\n",tagnumtmp,idx);
	return 0;
}
void Send2PC(void)
{
	float buff_dis[QUANTITY_ANCHOR];
	int buff_dis2[QUANTITY_ANCHOR];
	uint8_t i;
	uint8_t len=17;
	uint8_t crc=0;
	uint8_t *pointerP;
	
	memcpy(buff_dis,Que[front].buff+10,sizeof(float));
	memcpy(buff_dis+1,Que[front].buff+10+4,sizeof(float));
	memcpy(buff_dis+2,Que[front].buff+10+8,sizeof(float));
	//
	send_pc[3]=0x38;
	send_pc[4]=ANCHOR_NUM;
	//tag_id
	send_pc[6]=Que[front].buff[SOURADD];
	send_pc[7]=Que[front].buff[SOURADD+1]-128;
	
	//wearing status
	send_pc[8]=0x0E;
	for(i=0;i<QuantityAC;i++)
	{
		buff_dis2[i]=(int)(buff_dis[i]*1000);
	}
	send_pc[9]=(uint8)((buff_dis2[0]>>8)&0xff);
	send_pc[10]=(uint8)(buff_dis2[0]&0xff);

	send_pc[11]=(uint8)((buff_dis2[1]>>8)&0xff);
	send_pc[12]=(uint8)(buff_dis2[1]&0xff);

	send_pc[13]=(uint8)((buff_dis2[2]>>8)&0xff);
	send_pc[14]=(uint8)(buff_dis2[2]&0xff);
	
	pointerP=&send_pc[2];
	CRC_ResetDR();
	while(len--)
	{
		CRC_CalcCRC8bits(*pointerP++);
	}
	crc=(uint8)CRC_GetCRC();
	send_pc[19]=crc;
	send_pc[20]='\r';
	send_pc[21]='\n';
	while(DMA_transing);
	DMA_Cmd(DMA1_Channel2, DISABLE);	
	DMA1_Channel2->CMAR=(uint32_t)send_pc;
	DMA1_Channel2->CNDTR=22;
	DMA_transing=1;
	DMA_Cmd(DMA1_Channel2, ENABLE);	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

}

int DS_TwoWayRanging(void)
{
	int ret;
	uint32 delayed_txtime=0;
	uint64 poll_tx_ts;
	uint64 resp_rx_ts;
	uint64 final_tx_ts;
	uint8 tx_TOAbuff[27]={0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x10};//TOA定位所使用的buff
	uint32 timetmp;
	uint8 TimeOutCNT=0;
	dwt_forcetrxoff();
	dwt_rxreset();
	TOARanging=1;
	tx_TOAbuff[DESTADD]=Que[front].buff[SOURADD];
	tx_TOAbuff[DESTADD+1]=Que[front].buff[SOURADD+1];
	tx_TOAbuff[SOURADD]=(uint8)Achor_addr;
	tx_TOAbuff[SOURADD+1]=(uint8)(Achor_addr>>8);
	tx_TOAbuff[FUNCODE_IDX]=0x11;
	dwt_setrxtimeout(2200);//设置接受超时
	TimeOutCNT=0;
	dwt_writetxdata(12, tx_TOAbuff, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(12, 0, 1); /* Zero offset in TX buffer, ranging. */
	dwt_setrxaftertxdelay(1);//unit is ms
	ret=dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);//init first trans
	if(ret == DWT_ERROR)
	{
			goto error2;	
	}
	while(!isframe_sent);
	isframe_sent=0;
	TimeOutCNT=0;	
	do
	{
		while(!isreceive_To&&!isframe_rec);
		if(isreceive_To==1)
		{
			//printf("I_1_TO\r\n");
			isreceive_To=0;
			TimeOutCNT++;
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		if(TimeOutCNT==1)
		{
			goto error1;			
		}
		
	}while(!isframe_rec);
	isframe_rec=0;//接受到第一次resp包
	TimeOutCNT=0;
	 /* Retrieve poll transmission and response reception timestamp. */
	poll_tx_ts = get_tx_timestamp_u64();
	resp_rx_ts = get_rx_timestamp_u64();
	
//	time_stack[timestack_cnt++]=(uint32)(resp_rx_ts-poll_tx_ts);//SET POINT

	delayed_txtime = (resp_rx_ts + (INIT_TX_DELAYED_TIME_UUS * UUS_TO_DWT_TIME)) >> 8;
  dwt_setdelayedtrxtime(delayed_txtime);
	final_tx_ts = (((uint64)(delayed_txtime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
		
	final_msg_set_ts(&tx_TOAbuff[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);//發送最後一個數據包
	final_msg_set_ts(&tx_TOAbuff[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
	final_msg_set_ts(&tx_TOAbuff[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
	tx_TOAbuff[FUNCODE_IDX]=0x13;
	dwt_writetxdata(27, tx_TOAbuff, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(27, 0, 1); /* Zero offset in TX buffer, ranging. */
	ret = dwt_starttx(DWT_START_TX_DELAYED);
	if(ret == DWT_ERROR)
	{
			goto error2;	
	}
	while(!isframe_sent);
	isframe_sent=0;
	//printf("final sent!\r\n");
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
//	while(timestack_cnt)
//	{
//		double tmp=(double)time_stack[timestack_cnt-1]*4/1000;
//		printf("%d:%f us \r\n",timestack_cnt--,tmp);
//	}

	TOARanging=0;
	return 0;
error1:
	TOARanging=0;
	//printf("error1\r\n");
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	return -1;
error2:
	TOARanging=0;
	//printf("error2\r\n");
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	return -2;
}
/*========================================
	
	========================================*/
void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	Delay_ms(1);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

  Delay_ms(5);
}

//使能自動重啓接收器
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



void TIM7_init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 5000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值    计数到5000为500ms
  TIM_TimeBaseStructure.TIM_Prescaler =4800-1; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM7, TIM_IT_Update , ENABLE);
     
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
  NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	TIM_Cmd(TIM7, ENABLE);
}

void TIM7_IRQHandler(void)
{

	unsigned char led_sta=0;		
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET)
	{
		LED0(led_sta);
		led_sta=!led_sta;		 
		dwt_forcetrxoff();
		dw_txframe[FRAME_SN_IDX] = frame_seq_nb++;
		
		dw_txframe[0]=0x41;//data
		dw_txframe[1]=0x88;
		dw_txframe[FUNCODE_IDX]=0xa0;
		dw_txframe[DESTADD]=(uint8)0xffff;
		dw_txframe[DESTADD+1]=(uint8)(0xffff>>8);
		dwt_writetxdata((12), dw_txframe, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl((12), 0, 1); /* Zero offset in TX buffer, ranging. */	

		dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);

	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);

}

void CRC_init(void)
{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	CRC_DeInit();
	CRC_SetInitRegister(0);
	CRC_PolynomialSizeSelect(CRC_PolSize_8);
	CRC_SetPolynomial(0x07);//CRC-8 x8+x2+x+1 8位校验
	
}
void DMA_init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_InitStructure.DMA_BufferSize =16 ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AC_DATA;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR;//TDR ADDRESS
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	DMA_RemapConfig(DMA1, DMA1_CH2_USART1_TX);
		
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
  NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	//DMA_Cmd(DMA1_Channel2, ENABLE);
	//while(!DMA_GetFlagStatus(DMA1_FLAG_TC2));
	//DMA_ClearFlag(DMA1_FLAG_TC2);
	//DMA_Cmd(DMA1_Channel2, DISABLE);
}

void DMA1_Channel2_3_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC2))
	{
		DMA_ClearFlag(DMA1_FLAG_TC2);
		DMA_ClearFlag(DMA1_FLAG_GL2);
		DMA_transing=0;
		
	}

}
void cacu_crc(void)
{
	uint8 *pointerP;
	uint16 len;
	pointerP=(uint8*)&HEAD.ADDR;
	len=HEAD.LEN+1;	

	CRC_ResetDR();

	while(len--)
	{
		CRC_CalcCRC8bits(*pointerP++);
	}
	crc=(uint8)CRC_GetCRC();

}
void start_DMA(uint8 data_cnt)
{
	
	//DMA1_Channel2->CNDTR=sizeof(AC_DATA)+data_cnt*sizeof(struct TG_DATA_str);
	DMA1_Channel2->CMAR=(uint32_t)&HEAD;
	DMA1_Channel2->CNDTR=sizeof(AC_DATA)+data_cnt*sizeof(struct TG_DATA_str)+sizeof(struct HEAD_str);
	DMA_transing=1;
	DMA_Cmd(DMA1_Channel2, ENABLE);	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

}


void SWITCH_DB(void)
{
  uint8 tmp;
	uint32 statetmp;  
	tmp = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3); // Read 1 byte at offset 3 to get the 4th byte out of 5

	if((tmp & (SYS_STATUS_ICRBP >> 24)) ==     // IC side Receive Buffer Pointer
		 ((tmp & (SYS_STATUS_HSRBP>>24)) << 1) ) // Host Side Receive Buffer Pointer
	{
		statetmp=dwt_read32bitreg(SYS_MASK_ID);
		statetmp&=~(SYS_STATUS_RXFCG|SYS_STATUS_LDEDONE|SYS_STATUS_RXDFR|SYS_STATUS_RXFCE);
		dwt_write32bitreg(SYS_MASK_ID, statetmp);//disable interrupt
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG|SYS_STATUS_LDEDONE|SYS_STATUS_RXDFR|SYS_STATUS_RXFCE);//clear status bits
		statetmp|=(SYS_STATUS_RXFCG|SYS_STATUS_LDEDONE|SYS_STATUS_RXDFR|SYS_STATUS_RXFCE);
		dwt_write32bitreg(SYS_MASK_ID, statetmp);//enable interrupt
		dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01) ; // We need to swap RX buffer status reg (write one to toggle internally)
	}
	else
	{
		dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01) ; // We need to swap RX buffer status reg (write one to toggle internally)
	}
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the EVB1000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 */
 

