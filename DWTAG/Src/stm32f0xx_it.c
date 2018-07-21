/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "dw1000.h"
extern UART_HandleTypeDef huart1;
extern uint8_t usart_rx_buff[64];
extern uint8_t USART_tmp;
extern unsigned int localtime;
extern usart_bitfield USART_STA;
extern uint16 newdatacnt;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim14;
extern RTC_HandleTypeDef hrtc;
extern void SystemClock_Config(void);
extern void SYSCLKConfig_STOP(void);
/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
	HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM14 global interrupt.
*/
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
	

	}
		
  /* USER CODE END TIM14_IRQn 1 */

//callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	tim14_int=1;

		if(usart_rx_buff[USART_STA.count-1]==0x0a)//'/n'received
		{
		if(usart_rx_buff[USART_STA.count-2]==0x0d)//'/r'received
			{
				USART_STA.RC=1;
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);

				/* Disable the UART Parity Error Interrupt */
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);

				/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);

				/* Rx process is completed, restore huart->RxState to Ready */
				huart1.RxState = HAL_UART_STATE_READY;
			
				usmart_dev.scan();
				TIM14->ARR=10000;
				TIM14->CNT=0;
				USART_STA.count=0;
				USART_STA.RC=0;
				USART_STA.RD=0;
				HAL_UART_Receive_IT(&huart1, usart_rx_buff, 64);
			}
			else
			{
				printf("usart receive error/r/n");
				USART_STA.count=0;
				USART_STA.RC=0;
				USART_STA.RD=0;
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);

				/* Disable the UART Parity Error Interrupt */
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);

				/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);

				/* Rx process is completed, restore huart->RxState to Ready */
				huart1.RxState = HAL_UART_STATE_READY;
				//重新開始接受
				HAL_UART_Receive_IT(&huart1, usart_rx_buff, 64);
				
			}
		//stub;
		}
	

	TIM14->SR&=~1;//clear update flag
	
}


/* USER CODE BEGIN 1 */
//deca int
void EXTI2_3_IRQHandler(void)
{
	 uint32_t status;
		do
    {
			status = dwt_read32bitreg(SYS_STATUS_ID);// Read status register low 32bits

			if(status & SYS_STATUS_SLP2INIT)			//handle the wakeup event (added by roger 2016.9.30)
			{
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS|SYS_STATUS_SLP2INIT);
				dwiswake=1;
			}
			else
			{
				port_deca_isr();
				
			}


		
    } while (HAL_GPIO_ReadPin(DW_IRQ_GPIO_Port,DW_IRQ_Pin) != RESET);
    /* Clear EXTI Line  Pending Bit */
    __HAL_GPIO_EXTI_CLEAR_IT(DW_IRQ_Pin);
	
	
	
}
//NRF24l01 中断服务 MPU int
void EXTI4_15_IRQHandler(void)
{
	//STUB
	if(__HAL_GPIO_EXTI_GET_FLAG(MPU_INT_Pin))
	{	
		
		newdata=1;
		newdatacnt++;
		__HAL_GPIO_EXTI_CLEAR_FLAG(MPU_INT_Pin);//clear flag

	}
	if(__HAL_GPIO_EXTI_GET_FLAG(SWICH_Pin))
	{	
		
		__HAL_GPIO_EXTI_CLEAR_FLAG(SWICH_Pin);//clear flag

	}
		
}
//串口中断服务
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
	USART_STA.count++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	printf("usart buff overflow/r/n");
	USART_STA.count=0;
	USART_STA.RC=0;
	USART_STA.RD=0;
	HAL_UART_Receive_IT(&huart1, usart_rx_buff, 64);
	
}
void RTC_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_IRQn 0 */

  /* USER CODE END RTC_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_IRQn 1 */

  /* USER CODE END RTC_IRQn 1 */
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
