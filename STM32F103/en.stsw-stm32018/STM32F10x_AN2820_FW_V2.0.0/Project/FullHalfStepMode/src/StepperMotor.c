/**
  ******************************************************************************
  * @file FullHalfStepMode/src/StepperMotor.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains the stepper motor functions body.
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

/* Private define ------------------------------------------------------------*/



/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "StepperMotor.h"


/* Private typedef -------------------------------------------------------------*/

GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
DMA_InitTypeDef DMA_InitStructure;

/* Private define --------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------*/ 

uint16_t SRC_Buffer_DEC[20] ={15000,15000,20000,20000,25000,25000,30000,30000,35000,35000,
                         40000,40000,45000,45000,50000,50000,55000,55000,60000,60000};

uint16_t SRC_Buffer_INC[20] ={60000,60000,55000,55000,50000,50000,45000,45000,40000,40000,
                         35000,35000,30000,30000,25000,25000,20000,20000,15000,15000};
                         
                                          
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Configures the driver control pin 
  * @param  None
  * @retval : None
  */

void Stepper_PinControlConfig(void)
{
   /* Configure PC.04, PC.05, PC.06, PC.07, PC.08, PC.09 as output push pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7 
                                | GPIO_Pin_8 | GPIO_Pin_9;
                                
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
}


/**
  * @brief  Starts or Stops the stepper motor
  * @param NewState: This parameter can be ENABLE or DISABLE.
  * @retval : None
  */
void Stepper_Start(FunctionalState NewState)
{   
  if(NewState == ENABLE)
  {
     /* Set the C.09 pin */
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
  }
  else
  {   
    /* Reset the C.09 pin */
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  }
}



/**
  * @brief  Disables the initialization of the driver to its default reset value
  * @param  None
  * @retval : None
  */
void Stepper_ResetDisable(void)
{   
  /* Set the C.05pin */
  GPIO_SetBits(GPIOC, GPIO_Pin_5);
}



/**
  * @brief  Selects the direction of the rotation 
  * @param Stepper_RotationDirection: Specifies the rotation direction
  *   This parameter can be one of the following values:
  * @arg Stepper_RotationDirection_CW : sets clockwise direction
  * @arg Stepper_RotationDirection_CCW: sets counterclockwise direction
  *   
  * @retval : None
  */
void Stepper_SetRotationDirection(uint16_t Stepper_RotationDirection)
{
    if(Stepper_RotationDirection == Stepper_RotationDirection_CW )
  {
    /* Set the C.06 pin */
    GPIO_SetBits(GPIOC, GPIO_Pin_6);
  }
  else
  {   
    /* Reset the C.06 pin */
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
  }
}



/**
  * @brief  Selects the step mode  
  * @param Stepper_Mode: Specifies the Step Mode
  *   This parameter can be one of the following values:
  * @param Stepper_Full : Sets FULL STEP Mode
  * @param Stepper_Half : Sets HALF STEP Mode
  *   
  * @retval : None
  */
void Stepper_SelectMode(uint16_t Stepper_Mode)
{
  if(Stepper_Mode ==  Stepper_Full)
  {
     /* Reset the C.07 pin */
   GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  }
  else
  {   
    /* Set the C.07 pin */
   GPIO_SetBits(GPIOC, GPIO_Pin_7);
  }
}



/**
  * @brief  Selects the decay mode  
  * @param StepperControlMode: Specifies the Decay Mode
  *   This parameter can be one of the following values:
  * @param Stepper_ControlFast : Sets FAST DECAY Mode
  * @param Stepper_ControlSlow : Sets SLOW DECAY Mode
  *   
  * @retval : None
  */
void Stepper_SetControlMode(uint16_t Stepper_ControlMode)
{
   if(Stepper_ControlMode ==  Stepper_ControlFast)
  {
    /* Reset the C.08 pin */
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  }
  else
  {   
   /* Set the C.08 pin */
   GPIO_SetBits(GPIOC, GPIO_Pin_8);
  }

}


/**
  * @brief  Activates or Desactivates the driver
  * @param NewState: This parameter can be ENABLE or DISABLE.
  * @retval : None
  */
void Stepper_Cmd(FunctionalState NewState)
{   
    if(NewState == ENABLE)
  {
    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
     /* GPIOA clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* GPIOC clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
  }
  else
  {   
    /* TIM2 clock disable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
     /* GPIOA clock disable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
    /* GPIOC clock disable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
    /* Disable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
   
  }
}



/**
  * @brief  Configures the peripherals used to control the stepper motor 
  * @param  None
  * @retval : None
  */
void Stepper_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration:TIM2 Channel1 in Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* ---------------------------------------------------------------
  TIM2 Configuration: Output Compare Toggle Mode:
  --------------------------------------------------------------- */
   /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period =60000;
  TIM_TimeBaseStructure.TIM_Prescaler = 2;
  TIM_OCInitStructure.TIM_Pulse = 0;  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Output Compare Toggle Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_Toggle;  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
 
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  
  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);
  
  /* -------------------------------------------------------------------
  DMA1 configuration 
  ---------------------------------------------------------------------- */
  
  /* DMA1 channel2 configuration ----------------------------------*/
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM2_ARR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SRC_Buffer_INC;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
  
  /* Enable DMA1 Channel2 Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
  
  /* Enable DMA1 Channel2 */
  DMA_Cmd(DMA1_Channel2, ENABLE);
  
  /* Enable TIM2 DMA update request */
  TIM_DMACmd(TIM2,TIM_DMA_Update, ENABLE);
    
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

