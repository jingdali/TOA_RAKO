#ifndef _TEST_FUN
#define _TEST_FUN
#include "sys.h"
#include "deca_regs.h"
#include "deca_device_api.h"
void System_GetClocks(void);
void getSYSstatus(unsigned char index);
void getIDs(unsigned char index);
void read_test(unsigned char add);
void Unset_start_AR(void);
void Set_start_AR(void);
void Prt_anchnum(void);
void unlockflash(unsigned int passwd);
void testfun1(void);
void testfun2(void);
void getloctime(void);
void DMA_test(void);
void SET_Tpoint(void);
void GET_Time2Tpoint(void);
void ShowTimeStack(void);
	
extern uint32 time_record;
extern uint32 time_stack[];
extern uint16 timestack_cnt;
void going(void);
extern uint8 triggle;
#define HAULT_POINT {while(!triggle);triggle=0;}
#endif
