/**
  ******************************************************************************
  * @file DMA_Timeout/src/stm32f10x_it.c
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date  12/15/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler  and 
  *         peripherals interrupt service routine.
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

/** @addtogroup DMA_Timeout
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (Project_D), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles DMA1_Channel4 interrupt request.
  * @param  None
  * @retval : None
  */
void DMA1_Channel4_IRQHandler(void)
{
    /* Disable DMA1_Channel4 transfer*/
    DMA_Cmd(DMA1_Channel4, DISABLE);
    /*  Clear DMA1_Channel4 Transfer Complete Flag*/
    DMA_ClearFlag(DMA1_FLAG_TC4);
 
}

/**
  * @brief  This function handles DMA1_Channel2 interrupt request.
  * @param  None
  * @retval : None
  */
void DMA1_Channel2_IRQHandler(void)
{
    /* Disable DMA1_Channel2 transfer*/
    DMA_Cmd(DMA1_Channel2, DISABLE);
    /*  Clear DMA1_Channel2 Transfer Complete Flag*/
    DMA_ClearFlag(DMA1_FLAG_TC2);
 
}

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM2_IRQHandler(void)
{
  
   if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  { 
    /* Signal the DMA timeout by toggling Pin PC.07 */
    GPIO_SetBits(GPIOC, GPIO_Pin_7);
    GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    
    /* Clear TIM2 Capture Compare interrupt pending bit*/
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    
    /* Disable TIM2*/
    TIM_Cmd(TIM2, DISABLE);
  }
   
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
