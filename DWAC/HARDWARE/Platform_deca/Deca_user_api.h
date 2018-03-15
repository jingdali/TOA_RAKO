#ifndef __DECA_USR_API
#define __DECA_USR_API
#include "deca_types.h"
#include "deca_device_api.h"
#include "sys.h"
#include "string.h"
#include "deca_callback.h"
extern uint8 dw_rxframe[];
int SentFrame_ack(uint8 *buff, uint16 bufflen, uint16 targetID, uint8 Funcode);
int SentFrame_noack(uint8 *buff, uint16 bufflen, uint16 targetID, uint8 Funcode);
#endif
