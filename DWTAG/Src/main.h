#ifndef __MAIN_H
#define __MAIN_H


#include "deca_types.h"  
#include "usmart.h"
#include "stdint.h"
#include "rf24l01.h"
#include "delay.h"
#include "math.h"

//#inlcude "USART.h"
#define ACK_FC_0 0x02
#define ACK_FC_1 0x00
#define RX_BUF_LEN 35
#define FRAME_IDX 2
#define DESTADD 5
#define SOURADD 7
#define FUNCODE_IDX 9
#define XTAL_FREQ_HZ 38400000
#define SLEEP_TIME_MS 500
#define q30 1073741824.0f
#define TX_ANT_DLY 16495
#define RX_ANT_DLY 16495
#define TAG_ID 1u|0x8000u


#define TDOAMSGLEN 16+4
#define TDOAMSGSIZE 16
#define TDOAMPUMSGSIZE 16+4
#define UWBFREQ1 12
#define UWBFREQ2 13
#define WLIDX 10
#define WRIDX 11

#define QUANTITY_ANCHOR 5
#define TOA_MSG_LEN 10+2+2+4+4*QUANTITY_ANCHOR+2
#define TOAMSGSIZE 10+2+2+4*QUANTITY_ANCHOR+2
#define TOAMPUMSGSIZE 10+2+2+4+4*QUANTITY_ANCHOR+2
#define TOA_DATA_IDX 14
#define TOAMPU_DATA_IDX 18

#define MPUFREQ1 14
#define MPUFREQ2 15
#define MPUCNT1 16
#define MPUCNT2 17
/*
	一些用於TOA定位的宏
*/
#define RESP_TX_DELAYED_UUS 1500
#define RESP_TX_DELAYED_MS 30
#define UUS_TO_DWT_TIME 65536
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 15
#define FINAL_MSG_FINAL_TX_TS_IDX 20
#define SPEED_OF_LIGHT 299702547
#define RANGE_BIAS 0.57

#define WAIT_REC_TO(t)	{uint16 __t=t; \
while(!isreceive_To&&!isframe_rec) \
{__t?__t--:isreceive_To++;}}
#define WAIT_REC_ACK(t)	{uint16 __t=t; \
while(!isreceive_To&&!istxframe_acked) \
{__t?__t--:isreceive_To++;}}
#define WAIT_SENT(t)	{uint32 __t=t; \
while(!isframe_sent&&__t) \
{__t--;}}

//#define FLASHPROTECT
//#define MAXRDPLEVEL
typedef unsigned long long uint64;
typedef struct 
{
	uint16_t RC :1;
	uint16_t RD :1;
	uint16_t count :14;//16383 bytes
	
}usart_bitfield;
typedef struct
{
	uint8 rangingtype;//0 ==> TOA, 1 ==> TDOA,
	uint8 timebase;
	uint8 mpu_use;
	float *pmpudata;
	uint16 mpudatacnt;
	uint16 mpufreq;
	uint16 uwbfreq;
	uint16 TBfreq;
	uint16 id;
	uint8 anchorcnt;
	float *panchordis;
}sys_config_t;

extern uint8_t nrf_Tx_Buffer[33] ; // 无线传输发送数据
extern uint8_t nrf_Rx_Buffer[33] ; // 无线传输接收数据
extern uint8_t USART_tmp;
extern uint8_t usart_rx_buff[64];
extern usart_bitfield USART_STA;
extern unsigned int localtime;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart1;
extern volatile unsigned char newdata;
extern volatile uint8 dw_txseq_num;
extern uint8 dwiswake;

extern volatile uint8 isframe_sent;
extern volatile uint8 istxframe_acked;
extern volatile uint8 isreceive_To;
extern volatile uint8 isframe_rec;
extern volatile uint8 isack_sent;
extern uint8 rx_buffer[];
extern volatile uint8 tim14_int;

#endif
