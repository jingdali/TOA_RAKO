#include "Deca_user_api.h"
uint8 dw_txframe[128]={0x61, 0x88, 
					0, 
					0xCA, 0xDE, //PANID
					0, 0, //DESTID
					ANCHOR_NUM, 0, //SOURID
					0};	//FUNCODE

int SentFrame_ack(uint8 *buff, uint16 bufflen, uint16 targetID, uint8 Funcode)
{
	uint8 rtxtimes=0;
	 __IO uint32_t time=0;
	isframe_sent=0;
	istxframe_acked=0;
	isreceive_To=0;
	isframe_rec=0;
	isack_sent=0;
	
//	if(Funcode==0xa0)
//	{
//		dw_txframe[0]=0x63;//
//	}
//	else
//	{
//		dw_txframe[0]=0x61;//data
//	}
	dw_txframe[0]=0x61;//data
	dw_txframe[1]=0x88;
//	dw_txframe[FRAME_SN_IDX] = frame_seq_nb++;
	
	dw_txframe[FUNCODE_IDX]=Funcode;
	dw_txframe[DESTADD]=(uint8)targetID;
	dw_txframe[DESTADD+1]=(uint8)(targetID>>8);
	if(bufflen!=0)memcpy(dw_txframe+10,buff,bufflen);
	dwt_writetxdata((bufflen+12), dw_txframe, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl((bufflen+12), 0, 1); /* Zero offset in TX buffer, ranging. */
	
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(300);
	
	//若是doublebuff模式下开启发送前把该关的关了
//	statetmp=dwt_read32bitreg(SYS_MASK_ID);
//	statetmp&=~(SYS_STATUS_RXFCG|SYS_STATUS_LDEDONE|SYS_STATUS_RXDFR|SYS_STATUS_RXFCE);
//	dwt_write32bitreg(SYS_MASK_ID, statetmp);//disable interrupt
//	dwt_forcetrxoff();
//	dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01) ;
//	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG|SYS_STATUS_LDEDONE|SYS_STATUS_RXDFR|SYS_STATUS_RXFCE);
//	dwt_syncrxbufptrs();
//	statetmp|=(SYS_STATUS_RXFCG|SYS_STATUS_LDEDONE|SYS_STATUS_RXDFR|SYS_STATUS_RXFCE);
//	dwt_write32bitreg(SYS_MASK_ID, statetmp);//enable interrupt
	//dwt_readsystime(systim);
	
//	if(Funcode&0x01)
//	{
//		dwt_starttx(DWT_START_TX_DELAYED|DWT_RESPONSE_EXPECTED);	
//	}
//	else
//	{
//		dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
//	}
	dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
	time=msec;
	while(rtxtimes!=3)
	{
		
		while(istxframe_acked!=1&&isreceive_To!=1&&isframe_rec!=1&&(4+time>msec));
		if(isreceive_To)
		{
			isreceive_To=0;
			rtxtimes++;
			dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
		}
		else if(isframe_rec)
		{
			dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
			//dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01) ;
		}
		else 
		{
			break;
		}
		
		
	}
	dw_txframe[FRAME_SN_IDX]--;
	dwt_setrxtimeout(0);
	if(istxframe_acked)
	{
		istxframe_acked=0;
		isframe_sent=0;
		isreceive_To=0;
		isframe_rec=0;
		//dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01) ;//triggle HRBT,这里rxbuff用完了，可以用来接受新的数据了
		return 1;
	}
	else
	{
		
		istxframe_acked=0;
		isframe_sent=0;
		isreceive_To=0;
		isframe_rec=0;
		//dwt_rxenable(DWT_START_RX_IMMEDIATE);
		return 0;
	}
	
	
}
int SentFrame_noack(uint8 *buff, uint16 bufflen, uint16 targetID, uint8 Funcode)
{
	__IO uint32_t time;
	isframe_sent=0;
	isreceive_To=0;
	isframe_rec=0;
	isack_sent=0;
//	if(Funcode==0xa0)
//	{
//		dw_txframe[0]=0x43;//
//	}
//	else
//	{
//		dw_txframe[0]=0x41;//data
//	}
	dw_txframe[0]=0x41;//data
	dw_txframe[1]=0x88;
	dw_txframe[FUNCODE_IDX]=Funcode;
	dw_txframe[DESTADD]=(uint8)targetID;
	dw_txframe[DESTADD+1]=(uint8)(targetID>>8);
	if(bufflen!=0)memcpy(dw_txframe+10,buff,bufflen);
	dwt_writetxdata((bufflen+12), dw_txframe, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl((bufflen+12), 0, 1); /* Zero offset in TX buffer, ranging. */
	
//	dwt_forcetrxoff();
//	dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , 0x01) ;
	

	if(Funcode&0x01)
	{
		dwt_starttx(DWT_START_TX_DELAYED|DWT_RESPONSE_EXPECTED);	
	}
	else
	{
		dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
	}
	
	time=msec;
	while(isframe_sent!=1)
	{
		
	}
	dw_txframe[FRAME_SN_IDX]--;
	if(isframe_sent)
	{
		isframe_sent=0;
		isreceive_To=0;
		return 1;
	}
	else
	{
		isframe_sent=0;
		isreceive_To=0;
		return 0;
	}
	
}




