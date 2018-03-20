#include "deca_callback.h"
uint64 tx_timestamp=0;
uint64 rx_timestamp=0;
__align(4) uint8 rx_buffer[35];

volatile uint8 isframe_sent=0;
volatile uint8 istxframe_acked=0;
volatile uint8 isreceive_To=0;
volatile uint8 isframe_rec=0;
volatile uint8 isack_sent=0;


void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

    /* TESTING BREAKPOINT LOCATION #4 */
	isframe_sent=1;
	//dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	
}
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	uint16 frame_len;
	static uint8 lastmesg[10]={0};
	frame_len=cb_data->datalength;
	uint16 i;
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
				
				while(HAL_GPIO_ReadPin(DW_IRQ_GPIO_Port,DW_IRQ_Pin) != 1)
				{
					i++;
					if(i==200)break;//in test its always 0xa1
				}
				i=0;
				dwt_forcetrxoff(); 
				dwt_rxreset();				
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
			}

			dwt_readrxdata(rx_buffer, frame_len, 0);
			if(!memcmp(lastmesg,rx_buffer,10))
			{
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				return;
			}
			memcpy(lastmesg,rx_buffer,10);
			isframe_rec=1;
		}
	}

}
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
	isreceive_To=1;
}

void rx_err_cb(const dwt_cb_data_t *cb_data)//接收器自动重启参考user manual p72 RXAUTR
{

	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

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
uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
//		uint8 *p=(uint8 *)&ts;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
//				*(p+i)=ts_tab[i];
    }
    return ts;
}
void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < 5; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
void final_msg_get_ts(const uint8 *ts_field, uint64 *ts)
{
    int i;
		uint8 *p;
		*ts = 0;
		p=(uint8 *)ts;
    for (i = 0; i < 5; i++)
    {
        *p= ts_field[i];
				p++;
    }
		
}
