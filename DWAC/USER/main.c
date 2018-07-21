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
#include "wdg.h"
#include "stdio.h"

//#define FLASHPROTECT
//#define MAXRDPLEVEL
#define TGDATA_BUFFLEN 10

/* Declaration of static functions. */
static int AssignTimeWindow(void);
static int TOAdata_process(void);
static int TOAdata_mpu_process(void);
static int TDOAprocess_TAG(void);
static int TDOAprocess_mpu_TAG(void);
static int TDOAprocess_AC(void);
static int TWRresp(void);
static void dw1000_init(void);
static void system_init(void);
static void DMAtrans(void * addr ,uint16_t cnt);
static TAGlist_t* updatetag(uint8 mpu_use,uint8 RangingType);
static void UpLoad(TAGlist_t *ptag);
void TIM7_init(void);
void SWITCH_DB(void);
static uint8 cacu_crc(void *addr, uint16 cnt);
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
sys_config_t sys_config={
	.timebase=0,
	.ACtype=0,
	.id=ANCHOR_NUM,
	.TBfreq=1000,
	.acnum=ANCHORCNT,
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
uint8 send_pc[20]={0xFB, 0xFB, 0x11, 0, 0, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 dw_payloadbuff[127];
uint16 TBsyctime=0xff;
float *pglobalmpudata;
int idxreco=-1;
uint16 TAG_datacnt=0;
uint8 crc=0;
uint8 QuantityAC=QUANTITY_ANCHOR;
uint8 TBPOLL[TDOAMSGLEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, 0x80, 0, 0};

LIST_HEAD(TAG_list);

typedef struct 
{
	uint16 Mark;
	uint16 Tagid;
	uint8 Idx;
	uint8 Accnt;
	uint8 Rangingtype;
	uint8 Mpu;
	uint16 datalen;
}	Headdata_t;

int main(void)
{
	uint32 addtime;
	uint32 txtime;
	addtime=(uint32)((float)sys_config.TBfreq/0.0000040064);
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
	system_init();
//	IWDG_Init(6,625);
	if(!sys_config.timebase)
	{	
		printf("System initialized successfully......\r\n");
		printf("Anchor Number: %d	Anchor Type: %d\r\n",sys_config.id, sys_config.ACtype);
	}
	else
	{
		//running as TB
	}
	while(1) 
	{
//		IWDG_Feed();
		if(sys_config.timebase)
		{
//			WAIT_SENT(3000*200)
			while(!isframe_sent);
			isframe_sent=0;
			tx_timestamp=get_tx_timestamp_u64();
			txtime=(uint32)(tx_timestamp>>8)+addtime; 
			dwt_setdelayedtrxtime(txtime);
			TBPOLL[FRAME_SN_IDX]=frame_seq_nb++;	
			TBPOLL[WLIDX]=0;
			TBPOLL[WRIDX]=0;
			TBPOLL[UWBFREQ1]=(uint8)sys_config.TBfreq;
			TBPOLL[UWBFREQ2]=(uint8)(sys_config.TBfreq>>8);
			dwt_writetxdata(sizeof(TBPOLL), TBPOLL, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(TBPOLL), 0, 1); /* Zero offset in TX buffer, ranging. */
			dwt_starttx(DWT_START_TX_DELAYED);	

		}
		else
		{
			while(!isframe_rec);
			isframe_rec=0;
			switch(uwbrevbuff[FUNCODE_IDX])
			{
				case 0x80:TDOAprocess_TAG();break;
				case 0x80|0x40:TDOAprocess_mpu_TAG();break;
				case 0x40:TDOAprocess_AC();break;				
				case 0x2B:AssignTimeWindow();break;
				case 0x1A:TOAdata_process();break;
				case 0x1A|0x40:TOAdata_mpu_process();break;
				case 0x10:TWRresp();break;
				default:dwt_rxenable(DWT_START_RX_IMMEDIATE);//printf("UNknow cmd\r\n");				
					
			}
			uwbrevbuff[FUNCODE_IDX]=0;
		}


    }			
				
	}
/*========================================
	分支功能代码实现部分
	========================================*/

int AssignTimeWindow(void)
{
	
	uint8 tx_resp_time[18]={0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x01, 0x00, 0x2C, 0, 0, 0, 0};
	tx_resp_time[DESTADD]=uwbrevbuff[SOURADD];
	tx_resp_time[DESTADD+1]=uwbrevbuff[SOURADD+1];
	tx_resp_time[10]=(uint8)(msec);
	tx_resp_time[11]=(uint8)(msec>>8);
	tx_resp_time[12]=(uint8)(msec>>16);
	tx_resp_time[13]=(uint8)(msec>>24);
	tx_resp_time[14]=(uint8)(TBsyctime);
	tx_resp_time[15]=(uint8)(TBsyctime>>8);
	dwt_forcetrxoff();
	dwt_writetxdata(sizeof(tx_resp_time), tx_resp_time, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_resp_time), 0, 0); /* Zero offset in TX buffer, ranging. */
	dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
	WAIT_SENT(2000)
	isframe_sent=0;
	return 0;
}

static int TDOAprocess_TAG(void)
{

	uint8 i,j;
	TAGlist_t *ptag;
	uint8 Poll4TS[12+2+1] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x01, 0x00, 0x40};
	uint8 TOcnt=0;
	static uint16 totalcnt=0;
	static uint16 lostcntac[ANCHORCNT]={0};
	static uint16 lostcnttag[ANCHORCNT]={0};
	totalcnt++;
	if(!sys_config.ACtype)//main anchor
	{
		ptag=updatetag(0,1);
		Poll4TS[10]=(uint8)ptag->tagid;
		Poll4TS[11]=(uint8)(ptag->tagid>>8);
		Poll4TS[12]=ptag->seqid;
		for(j=2;j<=sys_config.acnum;j++)//get timestamps from slave anchor
		{
			Poll4TS[DESTADD]=(uint8)j;
			Poll4TS[DESTADD+1]=(uint8)(j>>8);
			dwt_writetxdata(15, Poll4TS, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(15, 0, 1); /* Zero offset in TX buffer, ranging. */
			dwt_setrxtimeout(3000);
			do
			{
				isframe_rec=0;
				dwt_starttx(DWT_START_TX_IMMEDIATE);
				WAIT_SENT(2000)
				isframe_sent=0;
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				WAIT_REC_TO(9000)
				if(isreceive_To==1)
				{
					isreceive_To=0;
					TOcnt++;
					if(TOcnt==3)
					{
						*((uint64*)ptag->puwbdata+j-1)=0;
						TOcnt=0;
						lostcntac[j-1]++;
						break;
					}
				}
				else 
				{
					isframe_rec=0;
					if(uwbrevbuff[10]==Poll4TS[10]&&uwbrevbuff[11]==Poll4TS[11]&&uwbrevbuff[12]==Poll4TS[12])
					{
						TOcnt=0;
						final_msg_get_ts(uwbrevbuff+13,(uint64*)ptag->puwbdata+j-1);
						break;
					}
					else
					{
						TOcnt++;
						lostcnttag[j-1]++;
						break;
					}
				}					
			}while(1);
		}
		
		dwt_setrxtimeout(0);
		
#ifdef EASY_READ		
		printf("TAGID: %d Tol:%d\r\n",ptag->tagid,totalcnt);
		for(i=0;i<sys_config.acnum;i++)
		{
			printf("ANCHOR: %d TS: %llx Lostac:%.3f\r\n",i+1,*((uint64*)ptag->puwbdata+i),(float)lostcntac[i]/(float)totalcnt);
		}
#else
		UpLoad(ptag);
#endif	
		dwt_rxenable(DWT_START_RX_IMMEDIATE);	
		return 1;
	}
	else//slave anchor
	{
		updatetag(0,1);
		dwt_setrxtimeout(0);
		dwt_rxenable(DWT_START_RX_IMMEDIATE);	
		return 1;
	}
	
}
static int TDOAprocess_mpu_TAG(void)
{
	uint8 j;
	TAGlist_t *ptag;
	uint8 Poll4TS[12+2+1] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x01, 0x00, 0x40};
	uint8 TOcnt=0;
	uint16 mpuframecnt;
	uint8 i;
	uint8 POLL4MPU[12]={0x41,0x88,0,0xCA, 0xDE,0x01, 0x00, 0x01, 0x00,0x20,0x00};

	if(!sys_config.ACtype)//main anchor
	{
		ptag=updatetag(1,1);
		dwt_setrxtimeout(4000);
		mpuframecnt=ptag->mpudatacnt*sizeof(float)/100;
		uwbrevbuff[FRAME_SN_IDX]=0;
		POLL4MPU[DESTADD]=(uint8)ptag->tagid;
		POLL4MPU[DESTADD+1]=(uint8)(ptag->tagid>>8)|0x80;
		ptag->mpudata_fault=0;	
		for(i=0;i<mpuframecnt;i++)
		{
			POLL4MPU[FRAME_SN_IDX]=i;
			dwt_writetxdata(12, POLL4MPU, 0);
			dwt_writetxfctrl(12, 0, 0);				
			while(1)
			{
				dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
				WAIT_SENT(2000)
				isframe_sent=0;
				WAIT_REC_TO(12000)
				if(isreceive_To==1)
				{
					isreceive_To=0;
					TOcnt++;
					if(TOcnt==3)
					{
						TOcnt=0;
						ptag->mpudata_fault++;
						memset((uint8*)ptag->pmpudata+i*100,0,100);
						break;
						
					}
				}
				else			
				{
					isframe_rec=0;
					if(uwbrevbuff[FUNCODE_IDX]==0x20&&uwbrevbuff[FRAME_SN_IDX]==i)
					{
						memcpy((uint8*)ptag->pmpudata+i*100,uwbrevbuff+10,100);
					}
					break;
				}
			}
		}
					  
		Poll4TS[10]=(uint8)ptag->tagid;
		Poll4TS[11]=(uint8)(ptag->tagid>>8);
		Poll4TS[12]=ptag->seqid;
		for(j=2;j<sys_config.acnum;j++)//get timestamps from slave anchor
		{
			Poll4TS[DESTADD]=(uint8)j;
			Poll4TS[DESTADD+1]=(uint8)(j>>8);
			dwt_writetxdata(15, Poll4TS, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(15, 0, 1); /* Zero offset in TX buffer, ranging. */
			dwt_setrxtimeout(3000);
			
			do
			{
				isframe_rec=0;
				dwt_starttx(DWT_START_TX_IMMEDIATE);
				WAIT_SENT(2000)
				isframe_sent=0;
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				WAIT_REC_TO(9000)
				if(isreceive_To==1)
				{
					isreceive_To=0;
					TOcnt++;
					if(TOcnt==3)
					{
						TOcnt=0;
						*((uint64*)ptag->puwbdata+j-1)=0;
						break;
					}
				}
				else 
				{
					isframe_rec=0;
					if(uwbrevbuff[10]==Poll4TS[10]&&uwbrevbuff[11]==Poll4TS[11]&&uwbrevbuff[12]==Poll4TS[12])
					{
						TOcnt=0;
						final_msg_get_ts(Poll4TS+12,(uint64*)ptag->puwbdata+j-1);
						break;
					}
					else
					{
						TOcnt++;
						break;
					}
				}					
			}while(1);
		}
		dwt_setrxtimeout(0);
		dwt_rxenable(DWT_START_RX_IMMEDIATE);	
#ifdef EASY_READ		
		printf("TAGID: %d\r\n",ptag->tagid);
		for(i=0;i<sys_config.acnum;i++)
		{
			printf("ANCHOR: %d TS: %llx\r\n",i+1,*((uint64*)ptag->puwbdata+i));
		}
		for(i=0;i<ptag->mpudatacnt/2;i++)
		{
			printf("%.2f	%.2f\r\n",*(ptag->pmpudata+2*i),*(ptag->pmpudata+2*i+1));
		}
#else
		UpLoad(ptag);
#endif	
		return 1;
	}
	else//slave anchor
	{
		updatetag(1,1);
		dwt_setrxtimeout(0);
		dwt_rxenable(DWT_START_RX_IMMEDIATE);	
		return 1;
	}
}
static int TDOAprocess_AC(void)
{
	uint16 tagid;
	uint8 Resp4TS[12+2+1+5] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x01, 0x00, 0, 0, 0x41};
	uint8 isexist=0;
	uint8 idx;
	struct list_head *i;
	TAGlist_t *ptag;
	
	Resp4TS[SOURADD]=(uint8)sys_config.id;
	Resp4TS[SOURADD+1]=(uint8)(sys_config.id>>8);
	Resp4TS[10]=uwbrevbuff[10];
	Resp4TS[11]=uwbrevbuff[11];
	Resp4TS[12]=uwbrevbuff[12];
	
	if(sys_config.ACtype)//slave anchor
	{
		tagid=uwbrevbuff[10];
		tagid+=uwbrevbuff[11]<<8;
		idx=uwbrevbuff[12];

		list_for_each(i, &TAG_list)
		{
			ptag=list_entry2(i, TAGlist_t, taglist);
			if(ptag->tagid==tagid&&ptag->seqid==idx)
			{
				isexist=1;
				break;
			}		
		}
		if(isexist)
		{
			final_msg_set_ts(Resp4TS+13,*(uint64*)list_entry2(i, TAGlist_t, taglist)->puwbdata);

		}
		else
		{
			final_msg_set_ts(Resp4TS+13,0);
		}
		dwt_writetxdata(20, Resp4TS, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(20, 0, 1); /* Zero offset in TX buffer, ranging. */
		dwt_starttx(DWT_START_TX_IMMEDIATE);
		WAIT_SENT(3000)
		isframe_sent=0;
		dwt_rxenable(DWT_START_RX_IMMEDIATE);	
		return 1;
	}
	else
	{
		return -1;//main anchor should not receive this
	}
}
static int TOAdata_process(void)
{
	TAGlist_t *ptag;
//	static uint16 lost[ANCHORCNT]={0};
	if(!sys_config.ACtype)
	{
		int i;
		ptag=updatetag(0,0);
		dwt_setrxtimeout(0);
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
#ifdef EASY_READ		
		for(i=0;i<sys_config.acnum;i++)
		{
			printf("ANCHOR: %d DIS: %.2f\r\n",i+1,*((float*)ptag->puwbdata+i));
		}
#else
		UpLoad(ptag);
#endif		
		return 0;
			
	}
	else
	{
		return -1;//slave anchoe should not receive this message
	}
}
static int TOAdata_mpu_process(void)
{
	uint8 mpuframecnt;
	uint8 TOcnt=0;
	TAGlist_t* ptag;
	uint8 i;
	uint8 POLL4MPU[12]={0x41,0x88,0,0xCA, 0xDE,0x01, 0x00, 0x01, 0x00,0x20,0x00};
	if(!sys_config.ACtype)
	{
		ptag=updatetag(1,0);
		dwt_setrxtimeout(4000);
		mpuframecnt=ptag->mpudatacnt*sizeof(float)/100;
		uwbrevbuff[FRAME_SN_IDX]=0;
		POLL4MPU[DESTADD]=(uint8)ptag->tagid;
		POLL4MPU[DESTADD+1]=(uint8)(ptag->tagid>>8)|0x80;
		ptag->mpudata_fault=0;
		for(i=0;i<mpuframecnt;i++)
		{
			POLL4MPU[FRAME_SN_IDX]=i;
			dwt_writetxdata(12, POLL4MPU, 0);
			dwt_writetxfctrl(12, 0, 0);				
			while(1)
			{
				dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
				WAIT_SENT(2000)
				isframe_sent=0;
				WAIT_REC_TO(12000)
				if(isreceive_To==1)
				{
					isreceive_To=0;
					TOcnt++;
					if(TOcnt==3)
					{
						TOcnt=0;
						ptag->mpudata_fault++;
						memset((uint8*)ptag->pmpudata+i*100,0,100);
						break;
						
					}
				}
				else			
				{
					isframe_rec=0;
					if(uwbrevbuff[FUNCODE_IDX]==0x20&&uwbrevbuff[FRAME_SN_IDX]==i)
					{
						memcpy((uint8*)ptag->pmpudata+i*100,uwbrevbuff+10,100);
					}
					break;
				}
			}
		}
		
		dwt_setrxtimeout(0);
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		
#ifdef EASY_READ		
		for(i=0;i<sys_config.acnum;i++)
		{
			printf("ANCHOR: %d DIS: %.2f\r\n",i+1,*((float*)ptag->puwbdata+i));
		}
		for(i=0;i<ptag->mpudatacnt/2;i++)
		{
			printf("%.2f	%.2f\r\n",*(ptag->pmpudata+2*i),*(ptag->pmpudata+2*i+1));
		}
//		printf("fail time %d\r\n",ptag->mpudata_fault);
#else
		UpLoad(ptag);
#endif		
		return 0;
	}
	else
	{
		return -2;//slave anchoe should not receive this message
	}
}
static TAGlist_t* updatetag(uint8 mpu_use,uint8 RangingType)
{
	uint16 tagid;
	uint8 idx=0;
	uint8 isexist=0;
	TAGlist_t *ptag;
	uint16 uwbfreq;
	uint8 wearleft;
	uint8 wearright;
	uint16 mpudatacnt;
	uint16 mpufreq;
	struct list_head *i;
	
	wearleft=uwbrevbuff[WLIDX];
	wearright=uwbrevbuff[WRIDX];
	tagid=uwbrevbuff[SOURADD];
	tagid+=(uwbrevbuff[SOURADD+1]-128)<<8;
	idx=uwbrevbuff[FRAME_SN_IDX];
	uwbfreq=uwbrevbuff[UWBFREQ1];
	uwbfreq+=(uwbrevbuff[UWBFREQ1+1])<<8;	
	if(mpu_use)
	{
		mpudatacnt=uwbrevbuff[MPUCNT1];
		mpudatacnt+=(uwbrevbuff[MPUCNT2])<<8;	
		mpufreq=uwbrevbuff[MPUFREQ1];
		mpufreq+=(uwbrevbuff[MPUFREQ2])<<8;	
	}


	list_for_each(i, &TAG_list)
	{
		ptag=list_entry2(i, TAGlist_t, taglist);
		if(ptag->tagid==tagid )
		{
			isexist=1;
			break;
		}		
	}
	if(!isexist)
	{
		ptag=(TAGlist_t*)malloc(sizeof(TAGlist_t));
		list_add(&ptag->taglist,&TAG_list);		
		if(!sys_config.ACtype)		
		{
			ptag->puwbdata=malloc(sys_config.acnum*sizeof(uint64));
		}
		else
		{
			ptag->puwbdata=malloc(sizeof(uint64));
		}
	}
	else
	{
		ptag=list_entry2(i, TAGlist_t, taglist);
	}
	
	if(!sys_config.ACtype)// main anchor
	{
		if(!RangingType)//TOA
		{
			if(mpu_use)
			{
				memcpy(ptag->puwbdata,uwbrevbuff+TOAMPU_DATA_IDX,sys_config.acnum*sizeof(float));
			}
			else
			{
				memcpy(ptag->puwbdata,uwbrevbuff+TOA_DATA_IDX,sys_config.acnum*sizeof(float));
			}
			
		}
		else//TDOA
		{
			*(uint64 *)ptag->puwbdata=rx_timestamp;
		}
	}
	else// slave anchor
	{
		if(!RangingType)//TOA
		{
			goto error1;//slave should not go here
		}
		else//TDOA
		{
			*(uint64 *)ptag->puwbdata=rx_timestamp;
		}
	}
	ptag->datatype=RangingType;
	ptag->mpudata_fault=0;
	ptag->acnum=sys_config.acnum;
	ptag->seqid=idx;
	ptag->tagid=tagid;
	ptag->uwbfreq=uwbfreq;
	if(mpu_use)
	{
		ptag->mpu_use=1;
		ptag->mpudatacnt=mpudatacnt;
		ptag->mpufreq=mpufreq;
		ptag->pmpudata=pglobalmpudata;
	}
	return ptag;
error1:
		return 0;
	
}
static void UpLoad(TAGlist_t *ptag)
{
	Headdata_t headdata={
		.Mark=0xfefe,
		.Tagid=ptag->tagid,
		.Idx=ptag->seqid,
		.Accnt=ptag->acnum,
		.Rangingtype=ptag->datatype,
		.Mpu=ptag->mpu_use,
		.datalen=0,
	};
	if(ptag->datatype)
	{
		headdata.datalen+=sizeof(uint64)*ptag->acnum;
	}
	else
	{
		headdata.datalen+=sizeof(float)*ptag->acnum;
	}
	if(ptag->mpu_use)
	{
		headdata.datalen+=ptag->mpudatacnt*sizeof(float);
	}
	while(DMA_transing);
	DMAtrans(&headdata ,sizeof(headdata));
	while(DMA_transing);
	if(ptag->datatype)
	{
		DMAtrans(ptag->puwbdata ,ptag->acnum*sizeof(uint64));
	}
	else
	{
		DMAtrans(ptag->puwbdata ,ptag->acnum*sizeof(float));
	}
	if(ptag->mpu_use)
	{
		while(DMA_transing);
		DMAtrans(ptag->pmpudata ,ptag->mpudatacnt*sizeof(float));
	}

}

int TWRresp(void)
{
	int ret;
	uint32 delayed_txtime=0;
	uint64 poll_tx_ts;
	uint64 resp_rx_ts;
	uint64 final_tx_ts;
	uint8 tx_TOAbuff[27]={0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x10};//TOA定位所使用的buff
	uint8 TimeOutCNT=0;
	
	
	dwt_forcetrxoff();
	dwt_rxreset();
	tx_TOAbuff[DESTADD]=uwbrevbuff[SOURADD];
	tx_TOAbuff[DESTADD+1]=uwbrevbuff[SOURADD+1];
	tx_TOAbuff[SOURADD]=(uint8)Achor_addr;
	tx_TOAbuff[SOURADD+1]=(uint8)(Achor_addr>>8);
	tx_TOAbuff[FUNCODE_IDX]=0x11;
	tx_TOAbuff[FRAME_SN_IDX]=uwbrevbuff[FRAME_SN_IDX];
	dwt_setrxtimeout(2200);//设置接受超时
	TimeOutCNT=0;
	dwt_writetxdata(12, tx_TOAbuff, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(12, 0, 1); /* Zero offset in TX buffer, ranging. */
//	dwt_setrxaftertxdelay(1);//unit is ms
	ret=dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);//init first trans
	if(ret == DWT_ERROR)
	{
			goto error2;	
	}
	WAIT_SENT(2000)
	isframe_sent=0;
	TimeOutCNT=0;	
	do
	{
		WAIT_REC_TO(6500)
		if(isreceive_To==1)
		{
			//printf("I_1_TO\r\n");
			isreceive_To=0;
			TimeOutCNT++;
		}
		if(TimeOutCNT==1)
		{
			goto error1;			
		}
		
	}while(!isframe_rec);
	isframe_rec=0;//接受到第一次resp包
	TimeOutCNT=0;
	dwt_forcetrxoff();
	 /* Retrieve poll transmission and response reception timestamp. */
	poll_tx_ts = get_tx_timestamp_u64();
	resp_rx_ts = get_rx_timestamp_u64();

	delayed_txtime = (resp_rx_ts + (INIT_TX_DELAYED_TIME_UUS * UUS_TO_DWT_TIME)) >> 8;
	final_tx_ts = (((uint64)(delayed_txtime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
		
	final_msg_set_ts(&tx_TOAbuff[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);//發送最後一個數據包
	final_msg_set_ts(&tx_TOAbuff[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
	final_msg_set_ts(&tx_TOAbuff[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
	tx_TOAbuff[FUNCODE_IDX]=0x13;
	dwt_writetxdata(27, tx_TOAbuff, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(27, 0, 1); /* Zero offset in TX buffer, ranging. */
	dwt_setdelayedtrxtime(delayed_txtime);
	ret = dwt_starttx(DWT_START_TX_DELAYED);
	if(ret == DWT_ERROR)
	{
			goto error2;	
	}
	WAIT_SENT(2000)
	isframe_sent=0;
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	return 0;
error1:
	//printf("error1\r\n");
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	return -1;
error2:
	//printf("error2\r\n");
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	return -2;
}
/*========================================
	
	========================================*/
static void dw1000_init(void)
{
	decaIrqStatus_t  stat ;
	reset_DW1000();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)	//dw1000 init
	{
			printf("INIT FAILED\r\n");
			while (1)
			{ };
	}
	else
	{
			printf("UWB Device initialised\r\n");
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
		dwt_enableframefilter(DWT_FF_DATA_EN|DWT_FF_ACK_EN);

    /* Activate auto-acknowledgement. Time is set to 0 so that the ACK is sent as soon as possible after reception of a frame. */
    dwt_enableautoack(8);
		dwt_setrxtimeout(0);
		dwt_setlnapamode(1,1);
		dwt_write16bitoffsetreg(PMSC_ID,PMSC_RES3_OFFSET+2,0);
}

static void system_init(void)
{
	dw1000_init();
	if(sys_config.id==1)
	{
		sys_config.ACtype=0;
	}
	else
	{
		sys_config.ACtype=1;
	}
	if(!sys_config.ACtype)
	{
		DMA_init();
	}
	if(!sys_config.timebase)
	{
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
	else
	{
		sys_config.id=0;
		TBPOLL[FRAME_SN_IDX]=frame_seq_nb++;	
		TBPOLL[WLIDX]=0;
		TBPOLL[WRIDX]=0;
		TBPOLL[UWBFREQ1]=(uint8)sys_config.TBfreq;
		TBPOLL[UWBFREQ2]=(uint8)(sys_config.TBfreq>>8);
		dwt_writetxdata(sizeof(TBPOLL), TBPOLL, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(TBPOLL), 0, 1); /* Zero offset in TX buffer, ranging. */
		dwt_starttx(DWT_START_TX_IMMEDIATE);

	}
	pglobalmpudata=(float*)malloc(MAX_MPUDATA_CNT);


}
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
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
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
static uint8 cacu_crc(void *addr, uint16 cnt)
{
	uint8 *pointerP;
	uint16 len=cnt;
	pointerP=(uint8*)addr;
	CRC_ResetDR();

	while(len--)
	{
		CRC_CalcCRC8bits(*pointerP++);
	}
	return (uint8)CRC_GetCRC();

}
static void DMAtrans(void * addr ,uint16_t cnt)
{
	DMA1_Channel2->CMAR=(uint32_t)addr;
	DMA1_Channel2->CNDTR=cnt;
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

