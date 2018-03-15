#include "dw1000.h"

/* DW1000 IRQ handler definition. */
port_deca_isr_t port_deca_isr = 0;



void port_set_deca_isr(port_deca_isr_t deca_isr)
{

    /* If needed, deactivate DW1000 IRQ during the installation of the new handler. */

    port_deca_isr = deca_isr;

}

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = DWRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DWRESET_GPIO_Port, &GPIO_InitStruct);
	HAL_Delay(1);
	GPIO_InitStruct.Pin = DWRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DWRESET_GPIO_Port, &GPIO_InitStruct);
	HAL_Delay(5);
	
}


void getIDs(void)
{
	int i=0;
//	uint8 headbuff[1]={0x00};
//	uint8 headlength=1;
	uint8 bodylength=4;
	uint8 bodybuff[4];
	dwt_readfromdevice(0,0,4,bodybuff);
	//readfromspi(headlength,headbuff,bodylength,bodybuff);
	for(i=bodylength-1;i>=0;i--)
	{	
	printf("0x%x ",bodybuff[i]);
	}
	printf("\r\n");
}	
	

void getSYSstatus(void)
{
	int i=0;
//	uint8 headbuff[1]={0x0f};
//	uint8 headlength=1;
	uint8 bodylength=2;
	uint8 bodybuff[2];
	dwt_readfromdevice(0x18,0,2,bodybuff);
//	readfromspi(headlength,headbuff,bodylength,bodybuff);
	for(i=bodylength-1;i>=0;i--)
	{	
	printf("0x%x ",bodybuff[i]);
	}
	printf("\r\n");
}




