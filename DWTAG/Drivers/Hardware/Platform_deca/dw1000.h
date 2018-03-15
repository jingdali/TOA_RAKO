#ifndef __DW1000_H
#define __DW1000_H
#include "stm32f0xx_hal.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "main.h"


typedef void (*port_deca_isr_t)(void);
/* DW1000 IRQ handler declaration. */
extern port_deca_isr_t port_deca_isr;

void reset_DW1000(void);
void port_set_deca_isr(port_deca_isr_t deca_isr);
void getSYSstatus(void);
void getIDs(void);

#endif
