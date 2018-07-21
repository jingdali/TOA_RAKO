
#ifndef __SYS_H
#define __SYS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stdlib.h"
#include <stdio.h>
#include "deca_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "list.h"
typedef unsigned long long uint64;
/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16495
#define RX_ANT_DLY 16495
/* Index to access some of the fields in the frames involved in the process. */
#define ACK_FC_0 0x02
#define ACK_FC_1 0x00
#define ANCHTYPE 1
#define TAGTYPE	0
#define FRAME_SN_IDX 2
#define DESTADD 5
#define SOURADD 7
#define FUNCODE_IDX 9
#define TIMSTAMPS_OWNERID 10
#define RXBUFFTS_IDX 12
#define PAYLOADTS_IDX 2

#define TDOAMSGLEN 16
#define TDOAMSGSIZE 16
#define TDOAMPUMSGSIZE 16+4
#define UWBFREQ1 12
#define UWBFREQ2 13
#define WLIDX 10
#define WRIDX 11

#define ANCHORCNT 5
#define ANCHOR_NUM 1
#define TOA_MSG_LEN 10+2+2+4+4*ANCHORCNT+2
#define TOAMSGSIZE 10+2+2+4*ANCHORCNT+2
#define TOAMPUMSGSIZE 10+2+2+4+4*ANCHORCNT+2
#define TOA_DATA_IDX 14
#define TOAMPU_DATA_IDX 18
#define MPUFREQ1 14
#define MPUFREQ2 15
#define MPUCNT1 16
#define MPUCNT2 17


#define PAYLOOAD_WLIDX 7
#define PAYLOOAD_WRIDX 8
#define RXBUFF_WLIDX 17
#define RXBUFF_WRIDX 18
#define FCTRL_ACK_REQ_MASK 0x20
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 15
#define FINAL_MSG_FINAL_TX_TS_IDX 20
#define FINAL_MSG_TS_LEN 5
#define DWT_TIME_UNITS     (1.0/499.2e6/128.0)
#define INIT_TX_DELAYED_TIME_UUS 2000
/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

#define RX_BUF_LEN 127
#define USAMRTCMD 63//==/?
#define TXDELAYTIME_US (ANCHOR_NUM-1)*1500UL
//#define MPUUSING
//#define TIMEBASE
#ifndef TIMEBASE
	#if ANCHOR_NUM==1
		#define MAIN_ANCHOR
	#else
		#define SLAVE_ANCHOR
	#endif
#else
	#define SYNCIDX 6
#endif
#define NRFQUELen 50

#define QUANTITY_ANCHOR 3
#define EASY_READ
#define MAX_MPUDATA_CNT 800

#define WAIT_REC_ACK(t)	{uint16 __t=t; \
while(!isreceive_To&&!istxframe_acked) \
{__t?__t--:isreceive_To++;}}

#define WAIT_REC_TO(t)	{uint16 __t=t; \
while(!isreceive_To&&!isframe_rec) \
{__t?__t--:isreceive_To++;}}

#define WAIT_SENT(t)	{uint32 __t=t; \
while(!isframe_sent&&__t) \
{__t--;}}

extern  __IO uint32_t msec;

extern uint8_t nrf_Tx_Buffer[33] ; // 无线传输发送数据
extern uint8_t nrf_Rx_Buffer[33] ; // 无线传输接收数据
extern uint8 rx_buffer[RX_BUF_LEN];
extern uint8 frame_seq_nb;
extern uint8 dw_payloadbuff[];
void reset_DW1000(void);
void dw_setARER(int enable);
extern volatile uint8 DMA_transing;
extern uint8 ACKframe[];

typedef struct
{
	uint8 timebase;
	uint8 ACtype;
	uint16 id;
	uint16 TBfreq;
	uint8 acnum;
}sys_config_t;

typedef struct tag_str
{
	uint8 datatype;
	uint8 tagid;
	uint8 seqid;
	uint8 acnum;
	uint16 uwbfreq;
	void *puwbdata;
	uint8 mpu_use;
	uint16 mpufreq;
	uint8 mpudata_fault;
	uint16 mpudatacnt;
	float *pmpudata;
	struct list_head taglist;
}TAGlist_t;

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
