#ifndef __TESTING_H
#define __TESTING_H

#include "main.h"
#include "dw1000.h"
#include "I2C.h"
#include "inv_mpu.h"
void NRF_Test(uint8_t stat);
void unlockflash(unsigned int passwd);
void going(void);
void Read_status(void);
void Start_dwrx(void);
void SET_Tpoint(void);
void GET_Time2Tpoint(void);
void ShowTimeStack(void);
void testI2C(uint8 resaddr);
extern uint32 time_stack[];
extern uint16 timestack_cnt;
extern uint32 time_record;
extern uint8 triggle;
#define HAULT_POINT {while(!triggle);triggle=0;}
#endif
