#ifndef __DECACALLBACK_H
#define __DECACALLBACK_H
#include "deca_device_api.h"
#include "deca_regs.h"
#include "sys.h"
#include "usart_v1.h"
#include "string.h"
#define Que_Length 10
typedef unsigned long long uint64;


void tx_conf_cb(const dwt_cb_data_t *cb_data);
void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);
uint64 get_rx_timestamp_u64(void);
uint64 get_tx_timestamp_u64(void);
void final_msg_set_ts(uint8 *ts_field, uint64 ts);
void final_msg_get_ts(const uint8 *ts_field, uint64 *ts);


extern volatile uint8 isframe_sent;
extern volatile uint8 istxframe_acked;
extern volatile uint8 isreceive_To;
extern volatile uint8 isframe_rec;
extern volatile uint8 isack_sent;
extern volatile uint8 TOARanging;
extern uint64 tx_timestamp;
extern uint64 rx_timestamp;
extern uint8 dw_txframe[];
extern uint8 uwbrevbuff[];
#endif
