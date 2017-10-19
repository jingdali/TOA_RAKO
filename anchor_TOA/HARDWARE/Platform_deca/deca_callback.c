#include "deca_callback.h"
/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */


uint64 tx_timestamp=0;
uint64 rx_timestamp=0;

Qnode_def Que[Que_Length];
uint8 rear=0;
uint8 front=0;
uint8 Qcnt=0;


volatile uint8 isframe_sent=0;
volatile uint8 istxframe_acked=0;
volatile uint8 isreceive_To=0;
volatile uint8 isframe_rec=0;
volatile uint8 isack_sent=0;
volatile uint8 TOARanging=0;

void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

    /* TESTING BREAKPOINT LOCATION #4 */
//		if(cb_data->status & SYS_STATUS_AAT)
//		{
//				isack_sent=1;
//		}
//		else
//		{
//				isframe_sent=1;
//		}

	isframe_sent=1; //since AAT is processed in rx_ok_cb, there is no need to handle in this cb.
}
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	uint16 frame_len;
	uint8 rtxtimes=0;
	frame_len=cb_data->datalength;
	#ifdef MAIN_ANCHOR
	if (frame_len <= RX_BUF_LEN)
	{
		if ((cb_data->fctrl[0] == ACK_FC_0) && (cb_data->fctrl[1] == ACK_FC_1))
		{
			istxframe_acked=1;
		}
		else
		{
						
			if(cb_data->status&SYS_STATUS_AAT)
			{
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS ));
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
				dwt_forcetrxoff(); 
        dwt_rxreset(); 
				
			}
			isframe_rec=1;
			
			if(TOARanging)//处于toa定位流程中的时候不适合使用队列
			{
				dwt_readrxdata(Que[front].buff, frame_len, 0);
//				Que[front].rx_timestamp=get_rx_timestamp_u64();
			}
			else
			{
				if(Qcnt<=Que_Length)
				{
					dwt_readrxdata(Que[rear].buff, frame_len, 0);
					if(Que[rear].buff[FUNCODE_IDX]==0x80)
					{
						Que[rear].rx_timestamp=get_rx_timestamp_u64();
					}
					rear=(rear+1)%Que_Length;
					Qcnt++;
				}
				isframe_rec=0;
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			
		}

		
	}

	

	#else
	//从基站回调处理
	if (frame_len <= RX_BUF_LEN)
	{
		if ((cb_data->fctrl[0] == ACK_FC_0) && (cb_data->fctrl[1] == ACK_FC_1))
		{
			istxframe_acked=1;
		}
		else
		{
			
			if(cb_data->status&SYS_STATUS_AAT)
			{
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS ));
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
				dwt_forcetrxoff(); 
        dwt_rxreset(); 
			}
			isframe_rec=1;
			if(TOARanging)//处于toa定位流程中的时候不适合使用队列
			{
				dwt_readrxdata(Que[front].buff, frame_len, 0);
				//Que[front].rx_timestamp=get_rx_timestamp_u64();
			}
			else
			{
				if(Qcnt<=Que_Length)
				{
					Que[rear].arrivetime=msec;
					dwt_readrxdata(Que[rear].buff, frame_len, 0);
					if(Que[rear].buff[FUNCODE_IDX]==0x80)
					{
					Que[rear].rx_timestamp=get_rx_timestamp_u64();
					}
					rear=(rear+1)%Que_Length;
					Qcnt++;
				}
				isframe_rec=0;
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			
			
		}

		
	}
	

	#endif	

}


void rx_err_cb(const dwt_cb_data_t *cb_data)//接收器自动重启参考user manual p72 RXAUTR
{
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	
}

void rx_to_cb(const dwt_cb_data_t *cb_data)
{
	isreceive_To=1;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
//void final_msg_get_ts(const uint8 *ts_field, uint64 *ts)
//{
//    int i;
//    *ts = 0;
//    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
//    {
//        *ts += ts_field[i] << (i * 8);
//    }
//}
void final_msg_get_ts(const uint8 *ts_field, uint64 *ts)
{
    int i;
		uint8 *p;
		*ts = 0;
		p=(uint8 *)ts;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *p= ts_field[i];
				p++;
    }
		
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
*        message, the least significant byte is at the lower address.(roger: it seems that the highest byte is discarded instead of the lowest byte.)
*				(ps:stm32 uses the little end.) 
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
//void final_msg_set_ts(uint8 *ts_field, uint64 ts)
//{
//    int i;
//    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
//    {
//        ts_field[i] = (uint8) ts;
//        ts >>= 8;
//    }
//}

void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
