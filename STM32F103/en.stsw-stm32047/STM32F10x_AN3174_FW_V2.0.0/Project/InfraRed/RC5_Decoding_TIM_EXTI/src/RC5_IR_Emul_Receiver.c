/**
  ******************************************************************************
  * @file    InfraRed/RC5_Decoding_TIM_EXTI/src/RC5_IR_Emul_Receiver.c 
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   This file provides all the RC5 firmware functions. 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "RC5_IR_Emul_Receiver.h"

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup RC5_Decoding_TIM_EXTI
  * @brief RC5 Decoding using Timer and EXTI driver modules
  * @{
  */ 

/** @addtogroup RC5_IR_EMUL_RECEIVER
  * @brief RC5 Decoding using Timer and EXTI driver modules
  * @{
  */ 
  
/* Private typedef -----------------------------------------------------------*/
typedef enum { NOTOK = 0, OK = !NOTOK} BitTimigStatus;

/* Private define ------------------------------------------------------------*/
#define   NOMINAL_HALF_BIT_TIME_US   889  /* Nominal half bit time in 탎 */
#define   MIN_HALF_BIT_TIME_US       640  /* Minimum half bit time in 탎 */
#define   MAX_HALF_BIT_TIME_US       1140 /* Maximum half bit time in 탎 */

#define   NOMINAL_FULL_BIT_TIME_US   1778 /* Nominal Full bit time in 탎 */
#define   MIN_FULL_BIT_TIME_US       1340 /* Minimum Full bit time in 탎 */
#define   MAX_FULL_BIT_TIME_US       2220 /* Maximum Full bit time in 탎 */

#define   RC5_TIM_PRESCALER          2

/* Private macro -------------------------------------------------------------*/
/* Private EdgesNumberiables ---------------------------------------------------------*/
__IO uint8_t EdgesNumber = 0; /* To count the first edges of the frame (max 3 edges) */
StatusYesOrNo Bit4_HasBeen_Sampled = NO; /* To know if the next sampling will be after 3/4 bit time or one bit time */
StatusYesOrNo RC5_FrameReceived = NO; /* Will be used by the user to check if an RC5 has been received or not */
BitTimigStatus BitTimeStatus = OK; /* Variable to store the timing status of the first low duration */
__IO uint8_t FieldBit = 0; /* Bit field value (command between 0-63 or 64-127 */
__IO uint16_t RC5_data[13]; /* Table that contains the value of the GPIOx_IDR register each sampling */
uint8_t RC5_Indexdata = 0; /* Variable that increments each time a RC5 bit is sampled */
__IO uint16_t LowDuration = 0; /* Contains the first low duration in in TIM count */
__IO uint32_t RC5_TIM_CLKValuekHz = 0; /* Contains the frequency input of RC5_TIM in Khz */
uint16_t HalfBitDurationCount_Min = 0; /* Minimum Half bit duration in TIM count */
uint16_t HalfBitDurationCount_Max = 0; /* Maximum Half bit duration in TIM count*/
uint16_t NominalHalfBitDurationCount  = 0; /* Nominal Half bit duration in TIM count */
uint16_t FullBitDurationCount_Min = 0; /* Minimum Full bit duration in TIM count */
uint16_t FullBitDurationCount_Max = 0; /* Maximum Full bit duration in TIM count */
uint16_t NominalBitDurationCount  = 0; /* Nominal Full bit duration in TIM count */
uint16_t NominalBitDurationCount_3div4 = 0; /* 3/4 of nominal bit time in TIM count */
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

/* Private function prototypes -----------------------------------------------*/
static void RC5_RCC_Configuration(void);
static void RC5_GPIO_Configuration(void);
static void RC5_NVIC_Configuration(void);
static uint32_t RC5_TIM_GetCounterCLKValue(void);
static void RC5_WaitForNextFallingEdge(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize the RC5 reception.
  * @param  None
  * @retval None
  */
void RC5_Receiver_Init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* System Clocks Configuration for RC5 reception */
  RC5_RCC_Configuration();

  /* NVIC configuration for RC5 reception */
  RC5_NVIC_Configuration();

  /* Configure the GPIO port for RC5 reception */
  RC5_GPIO_Configuration();

  /* Get frequency input of RC5_TIM in Khz */
  RC5_TIM_CLKValuekHz = RC5_TIM_GetCounterCLKValue()/1000;
  
  /* Compute the different timing tolerences in TIM count */
  NominalBitDurationCount =  (RC5_TIM_CLKValuekHz * NOMINAL_FULL_BIT_TIME_US)/1000;
  NominalHalfBitDurationCount = (RC5_TIM_CLKValuekHz * NOMINAL_HALF_BIT_TIME_US)/1000;
  HalfBitDurationCount_Min = (RC5_TIM_CLKValuekHz * MIN_HALF_BIT_TIME_US)/1000;
  HalfBitDurationCount_Max = (RC5_TIM_CLKValuekHz * MAX_HALF_BIT_TIME_US)/1000;
  FullBitDurationCount_Min = (RC5_TIM_CLKValuekHz * MIN_FULL_BIT_TIME_US)/1000;
  FullBitDurationCount_Max = (RC5_TIM_CLKValuekHz * MAX_FULL_BIT_TIME_US)/1000;
  
  /* Compute the 3/4 bit time duration in TIM count */
  NominalBitDurationCount_3div4 = (NominalBitDurationCount * 3)/4;
  
  /* Enable EXTIx to detect the start bit of the RC5 frame */
  EXTI_ClearITPendingBit(RC5_EXTI_LINE);
  EXTI_InitStructure.EXTI_Line = RC5_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Wait for next falling edge of RC5 frame */
  RC5_WaitForNextFallingEdge();
}

/**
  * @brief  Configure the system to receive the next RC5 frame.
  * @param  None
  * @retval None
  */
static void RC5_WaitForNextFallingEdge(void)
{
  /* Enable EXTI line x */
  EXTI->IMR |= RC5_EXTI_LINE;
  
  /* RC5_TIM Configuration ------------------------------------------------------*/
  TIM_DeInit(RC5_TIM); 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = RC5_TIM_PRESCALER;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(RC5_TIM, &TIM_TimeBaseStructure);
  
  /* Clear RC5_TIM Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(RC5_TIM, TIM_IT_Update);	
}

/**
  * @brief  Sample the RC5 data bits.
  * @param  None
  * @retval None
  */
void RC5_Sample_Data(void)
{
  if (TIM_GetITStatus(RC5_TIM, TIM_IT_Update) != RESET) /* Update event occured */
  { 
    /* Sample RC5 GPIO input */
    RC5_data[RC5_Indexdata++] = RC5_GPIO_PORT->IDR & RC5_GPIO_PIN;
    
    if(Bit4_HasBeen_Sampled == NO) /* RC5 bit 4 has been sampled? */
    {                              /*(to know if the next sampling will be after 3/4 or 1 bit time) */

      /* If NO: Set the RC5_TIM auto-reload register to allow the next sampling at 1bit time */ 
      RC5_TIM->ARR = NominalBitDurationCount; 
      
      /* Set the variable to yes */
      Bit4_HasBeen_Sampled = YES;
    }
    
    /* If the number of data reaches 13 (without start bit and field bit and extra bit is sampled: (14-2) + 1) */
    if(RC5_Indexdata == 13) 
    {
      /* Initialize the RC5 data index */
      RC5_Indexdata = 0;
      
      /* Set this flag software to inform the user that a RC5 frame is ready to be read */
      RC5_FrameReceived = YES;
      
      /* Initialize  Bit4_HasBeen_Sampled variable for next reception */
      Bit4_HasBeen_Sampled = NO;

      /* Disable RC5_TIM Update event Interrupt Request */
      TIM_ITConfig(RC5_TIM, TIM_IT_Update, DISABLE);
      
      /* Disable RC5_TIM counter */
      TIM_Cmd(RC5_TIM, DISABLE); 
    }

    /* Clear RC5_TIM Update event interrupt pending bit */
    TIM_ClearITPendingBit(RC5_TIM, TIM_IT_Update);
  }
}  

/**
  * @brief  Measure the first low duration of the RC5 frame.
  * @param  None
  * @retval None
  */
void RC5_MeasureFirstLowDuration(void)
{
  /* If an edge is detected on RC5 input pin */
  if(EXTI_GetITStatus(RC5_EXTI_LINE) != RESET) 
  {
    /* Increment the edges number */
    EdgesNumber++;

    /* If it was the first edge */
    if(EdgesNumber == 1)
    {
      /* Reset the RC5_TIM counter */
      RC5_TIM->CNT = 0;
      
      /* Enable RC5_TIM counter */
      TIM_Cmd(RC5_TIM, ENABLE); 
    }
    /* If it was the 2nd edge */
    else if(EdgesNumber == 2)
    {
      /* Disable RC5_TIM counter */
      TIM_Cmd(RC5_TIM, DISABLE);
      
      /* Read RC5_TIM counter to get the first low duration */
      LowDuration = RC5_TIM->CNT;
      
      /* Reset RC5_TIM counter */
      RC5_TIM->CNT = 0;
    
      /* If low duration is between 640탎*TimClk and 1140탎*TimClk (min and max half bit time) */
      if ((LowDuration >= HalfBitDurationCount_Min) && (LowDuration <= HalfBitDurationCount_Max)) 
      {
        /* Validate the frame, by setting this variable to OK */
        BitTimeStatus = OK; 
      }
      /* If low duration is between 1340탎*TimClk and 2220탎*TimClk (min and max full bit time)*/
      else if ((LowDuration >= FullBitDurationCount_Min) && (LowDuration <= FullBitDurationCount_Max)) 
      {
        /* Disable EXTI line x to avoid jumping to this interrupt while receiving
        the RC5 data bits (it will be reenabled in the next RC5 data reception */
        EXTI->IMR &= ((uint32_t)~RC5_EXTI_LINE);

        /* Enable RC5_TIM counter */
        TIM_Cmd(RC5_TIM, ENABLE);
       
        /* Enable RC5_TIM Update Event Interrupt Request */
        TIM_ITConfig(RC5_TIM, TIM_IT_Update, ENABLE);
       
        /* Validate the frame, by setting this variable to OK */
        BitTimeStatus = OK;

        /* Set the RC5_TIM auto-reload register to allow the next sampling at 3/4 bit time */
        RC5_TIM->ARR = NominalBitDurationCount_3div4; 
       
        /* The bit field value is equal to 0 */
        FieldBit = 0;
       
        /* Initialize edges detected number */
        EdgesNumber = 0;

      }
      else /* If the first low duration is not in the right timing range */
      {  
        /* Set the Bit timing to NOTOK */
        BitTimeStatus = NOTOK;

        /* Reset RC5_TIM counter */
        RC5_TIM->CNT = 0;
         
        /* Initialize the number of glitches detected */
        EdgesNumber = 0;
      }
    }
    else if(EdgesNumber == 3) /* If the number of edges is equal to 3 */
    {
      /* Disable EXTI line x to avoid jumping to this interrupt while receiving
      the RC5 data bits (it will be reenabled in the next RC5 data reception */
      EXTI->IMR &= ((uint32_t)~RC5_EXTI_LINE);
       
      /* Enable RC5_TIM counter */
      TIM_Cmd(RC5_TIM, ENABLE);
      
      /* Enable RC5_TIM Update Event Interrupt Request */
      TIM_ITConfig(RC5_TIM, TIM_IT_Update, ENABLE);
       
      /* Validate the frame, by setting this variable to OK */
      BitTimeStatus = OK;
       
      /* Set the RC5_TIM auto-reload register to allow the next sampling at 3/4 bit time */
      RC5_TIM->ARR = NominalBitDurationCount_3div4;  
       
      /* The bit field value is equal to 1 */
      FieldBit = 1;

      /* Initialize the number of glitches detected */
      EdgesNumber = 0;
    }
   
    /* Clear the RC5 EXTI line pending bit */
    EXTI_ClearITPendingBit(RC5_EXTI_LINE);
  }
}

/**
  * @brief  Decode the RC5 frame.
  * @param  None
  * @retval RC5 structure
  */
RC5Frame_TypeDef RC5_Decode(void)
{
   RC5Frame_TypeDef RC5_frame_struct;
   uint32_t RC5_dataIndex = 0;
   
   /* Initialize the different fields of the RC5 structure */
   RC5_frame_struct.ToggleBit = 0;
   RC5_frame_struct.Address = 0; 
   RC5_frame_struct.Command = 0;
  
   /* Get the Toggle bit value */
   if(RC5_data[0] & RC5_GPIO_PIN)
   {
     RC5_frame_struct.ToggleBit = 1;
   }
   else
   {
     RC5_frame_struct.ToggleBit = 0;
   }
  
   /* Get RC5 Address value */
   for(RC5_dataIndex=1; RC5_dataIndex<6; RC5_dataIndex++)
   {
     RC5_frame_struct.Address <<= 1 ;
     if(RC5_data[RC5_dataIndex] & RC5_GPIO_PIN)
     {
       RC5_frame_struct.Address |= 1;
     }
   }
     
  /* Get RC5 Command value */
  for(RC5_dataIndex=6; RC5_dataIndex<12; RC5_dataIndex++)
   {
     RC5_frame_struct.Command <<= 1 ;
     if(RC5_data[RC5_dataIndex] & RC5_GPIO_PIN)
     {
       RC5_frame_struct.Command |= 1;
     }
   }
  
   /* Set the 6th bit of the command regarding of the filed bit value */
   if(FieldBit == 0) /* logic 0 = command from 64 to 127 */
   {
     RC5_frame_struct.Command |= 0x40;
   }
   
   /* Initialize RC5_FrameReceived for next RC5 reception */
   RC5_FrameReceived = NO;  
  
   /* Wait for next falling edge of RC5 frame*/
   RC5_WaitForNextFallingEdge();
  
   /* Return the RC5 struct */ 
   return RC5_frame_struct;
}
  
/**
  * @brief  Configures the different system clocks for RC5 reception.
  * @param  None
  * @retval None
  */
static void RC5_RCC_Configuration(void)
{
 
 if((RC5_TIM == TIM1) || (RC5_TIM == TIM8) || (RC5_TIM == TIM15) ||
   (RC5_TIM == TIM16) || (RC5_TIM == TIM17)
    /* || (RC5_TIM == TIM9) || (RC5_TIM == TIM10) || (RC5_TIM == TIM11)*/)
  {    
    /* RC5_TIM clock enable (APB2) */
    RCC_APB2PeriphClockCmd(RC5_TIM_CLK, ENABLE);
  }
  else if((RC5_TIM == TIM2) || (RC5_TIM == TIM3) || (RC5_TIM == TIM4) ||
          (RC5_TIM == TIM5) || (RC5_TIM == TIM6) || (RC5_TIM == TIM7) 
      /* || (RC5_TIM == TIM12) || (RC5_TIM == TIM13)|| (RC5_TIM == TIM14)*/ )
  {
    /* RC5_TIM clock enable (APB1) */
    RCC_APB1PeriphClockCmd(RC5_TIM_CLK, ENABLE);
  }


  /* GPIOx clock enable for RC5 input pin */
  RCC_APB2PeriphClockCmd(RC5_GPIO_CLK | RCC_APB2Periph_AFIO , ENABLE);
}

/**
  * @brief  Configure the RC5 input pin.
  * @param  None
  * @retval None
  */
static void RC5_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* RC5 pin configuration: input floating */
  GPIO_InitStructure.GPIO_Pin = RC5_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RC5_GPIO_PORT, &GPIO_InitStructure);
  
  /* Connect EXTI Line x to RC5 pin */
  GPIO_EXTILineConfig(RC5_EXTI_PORT_SOURCE, RC5_EXTI_PIN_SOURCE);
}

/**
  * @brief  Configure the nested vectored interrupt controller for RC5 reception.
  * @param  None
  * @retval None
  */
static void RC5_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the EXTIx global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RC5_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  /* Enable the RC5_TIM global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RC5_TIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);  
}

/**
  * @brief  Compute the RC5_TIM frequency input in Hz.
  * @param  None
  * @retval RC5_TIM Frequency value in Hz
  */
static uint32_t RC5_TIM_GetCounterCLKValue(void)
{
  uint32_t apbprescaler = 0, apbfrequency = 0;
  
  RCC_ClocksTypeDef RCC_ClockFreq;
  
  /* This function fills the RCC_ClockFreq structure with the current
     frequencies of different on chip clocks */
  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  if((RC5_TIM == TIM1) || (RC5_TIM == TIM8) || (RC5_TIM == TIM15) ||
     (RC5_TIM == TIM16) || (RC5_TIM == TIM17)
      /* || (RC5_TIM == TIM9) || (RC5_TIM == TIM10) || (RC5_TIM == TIM11)*/)
  {    
    /* Get the clock prescaler of APB2 */
    apbprescaler = ((RCC->CFGR >> 11) & 0x7);
    apbfrequency = RCC_ClockFreq.PCLK2_Frequency;
  }
  else if((RC5_TIM == TIM2) || (RC5_TIM == TIM3) || (RC5_TIM == TIM4) ||
          (RC5_TIM == TIM5) || (RC5_TIM == TIM6) || (RC5_TIM == TIM7) 
      /* || (RC5_TIM == TIM12) || (RC5_TIM == TIM13)|| (RC5_TIM == TIM14)*/ )
  {
    /* Get the clock prescaler of APB1 */
    apbprescaler = ((RCC->CFGR >> 8) & 0x7);
    apbfrequency = RCC_ClockFreq.PCLK1_Frequency;  
  }
  
  /* If APBx clock div >= 4 */
  if (apbprescaler >= 4)
  {
    return ((apbfrequency * 2)/(RC5_TIM_PRESCALER + 1));
  }
  else
  {
    return (apbfrequency/(RC5_TIM_PRESCALER + 1));
  }
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
