/**
  ******************************************************************************
  * @file FullHalfStepMode/src/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main Interrupt Service Routines.This file provides template for all 
  *         exceptions handler and peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "StepperMotor.h"


/** @addtogroup FullHalfStepMode
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint16_t SRC_Buffer_INC[20];
extern uint16_t SRC_Buffer_DEC[20];
FlagStatus acceleration = RESET;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
	 /* Change the DMA_MemoryBaseAddr to the decrement buffer */
  DMA1_Channel2->CMAR = (uint32_t)SRC_Buffer_DEC;
  
  /* Define the DMA_BufferSize */  
  DMA1_Channel2->CNDTR = BufferSize;
  
  /* Enable DMA1 Channel2 */
  DMA1_Channel2->CCR |= ((uint32_t)0x00000001);

  /* Enable TIM2 DMA request */ 
  TIM2->DIER |= TIM_DMA_Update;
  
  /* Disable SysTick Counter */ 
  SysTick->CTRL &= 0xFFFFFFFE;
  
  /* Clear SysTick Counter */
  SysTick->VAL = ((uint32_t)0x00000000);
  
  acceleration = SET;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (MC), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles MC interrupt request.
  * @param  None
  * @retval : None
  */


void DMA1_Channel2_IRQHandler(void)
{
  /* Disable TIM2 DMA */  
  TIM2->DIER &= ~TIM_DMA_Update;
    
  /* Disable DMA Channel2 request */  
  DMA1_Channel2->CCR &= ((uint32_t)0xFFFFFFFE);
  
  if(acceleration == SET)
  {       
    /* Change the DMA_MemoryBaseAddr to the decrement buffer */
    DMA1_Channel2->CMAR =(uint32_t)SRC_Buffer_INC;
  
    /* Define the DMA_BufferSize */  
    DMA1_Channel2->CNDTR = BufferSize;
  
    /* Enable DMA Channel2 */
    DMA1_Channel2->CCR |= ((uint32_t)0x00000001);

    /* Enable TIM2 DMA request */ 
    TIM2->DIER |= TIM_DMA_Update;
    
    acceleration = RESET; 
  }
  else
  {    
  
    /* SysTick end of count event each 8ms  */
    /* Setup SysTick Timer for 1 msec interrupts */
if (SysTick_Config(SystemFrequency / 8000)) /* SystemFrequency is defined in
“system_stm32f10x.h” and equal to HCLK frequency */
{
/* Capture error */
while (1);
 }     
    acceleration = SET;
  }
  /* Clear DMA1 Channel2 Transfer Complete pending bit */ 
  DMA1->IFCR = DMA1_FLAG_TC2; 




}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
