/**
  ******************************************************************************
  * @file    ADC/ADC1_Freeze/stm32l1xx_it.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "Oversampling.h"

/** @addtogroup TriangularDitherMethod
  * @{
  */

/** @addtogroup ADC1_Freeze
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef Thermal_Noise_Measure
/* This parameter to compute the ADC thermal noise */
extern uint16_t ADC_ConvertedValue[Total_Samples_Number];
extern uint16_t ADC_Codes[Relevant_Samples_Number];
extern uint16_t ADC_Codes_Number[Relevant_Samples_Number];
#else
extern uint16_t ADC_ConvertedValue[ADC_Oversampling_Factor];
extern uint16_t TimerPulseWidth[BUFFER_PW_WIDTH];
#endif

#ifdef Oversampling_Test
extern uint16_t ADC_Oversampled_Buffer[Oversampling_Test_Samples];
#endif

__IO uint16_t ADC_Oversampled_Data;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
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
  * @retval None
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
  * @retval None
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
  * @retval None
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
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32L1xx Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles DMA1_Channel1 interrupt request.
  * @param  None
  * @retval : None
  */
void DMA1_Channel1_IRQHandler(void)
{
  #ifdef Oversampling_Test
    static uint32_t index = 0;
#endif

   /* Clear the DMA Global interrupt bit DMA_IT_GL1*/
   DMA1->IFCR = DMA1_IT_GL1;

 #ifdef Thermal_Noise_Measure
   Oversampling_TestADCThermalNoise( ADC_Codes, ADC_Codes_Number);
 #else

   /* Compute the oversampled result */
   ADC_Oversampled_Data = Oversampling_GetConversion();

   /* Disable DMA channel1 */
   DMA1_Channel1->CCR &= 0xFFFFFFFE;

   /* Update the Number of DMA transfer */
   DMA1_Channel1->CNDTR = ADC_Oversampling_Factor;

   /* Update the Destination Memory of the DMA pointer */
   DMA1_Channel1->CMAR = (uint32_t)ADC_ConvertedValue;

    /* Disable DMA channel3 */
   DMA1_Channel3->CCR &= 0xFFFFFFFE;

   /* Update the Number of DMA transfer */
   DMA1_Channel3->CNDTR = BUFFER_PW_WIDTH;

   /* Update the source memory of the DMA pointer */
   DMA1_Channel3->CMAR = (uint32_t)TimerPulseWidth;

   /* stop ADC DMA transfer */
   ADC_DMACmd(ADC1, DISABLE);

#ifdef Oversampling_Test
   ADC_Oversampled_Buffer[index] = ADC_Oversampled_Data;
   index++;
   if ( index == Oversampling_Test_Samples )
   {
     /* Disable the TIM2 Timer */
     TIM_Cmd(TIM2, DISABLE);

     /* copy the ADC_Oversampled_Buffer into a file */
     Oversampling_CopyConvValuesToFile(  ADC_Oversampled_Buffer , Oversampling_Test_Samples );
   }
#endif
#endif
}

/**
  * @}
  */

/**
  * @}
  */


/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval : None
  */
void TIM2_IRQHandler(void)
{
  /* Clear the interrupt Update pending bit */
  TIM_ClearITPendingBit( TIM2, TIM_IT_Update);

  /* Enable DMA channel 3 transfer */
  DMA1_Channel3->CCR |= 0x00000001;

  /* Enable DMA channel1 */
  DMA1_Channel1->CCR |= 0x00000001;

  /* enable ADC DMA transfer - to continue */
  ADC_DMACmd(ADC1, ENABLE);
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
