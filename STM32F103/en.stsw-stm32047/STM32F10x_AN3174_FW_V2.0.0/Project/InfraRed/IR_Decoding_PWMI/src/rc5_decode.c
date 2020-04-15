/**
  ******************************************************************************
  * @file    InfraRed/IR_Decoding_PWMI/src/rc5_decode.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   This file provides all the RC5 firmware functions.
  *   
  * 1. How to use this driver
  * -------------------------  
  *      - Call the function IR_RC5_Init() to configure the Timer and GPIO hardware
  *        resources needed for RC5 decoding.
  *        
  *      - TIM3 Capture Compare and Update interrupts are used to decode the RC5
  *        frame, if a frame is received correctly a global variable RC5FrameReceived 
  *        will be set to inform the application.
  *          
  *      - Then the application should call the function IR_RC5_Decode() to retrieve 
  *        the received RC5 frame.              
  *                   
  * 2. Important to know
  * --------------------  
  *      - TIM3_IRQHandler  ISRs are coded within
  *        this driver.
  *        If you are using one or both Interrupts in your application you have 
  *        to proceed carefully:   
  *           - either add your application code in these ISRs
  *           - or copy the contents of these ISRs in you application code                    
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
#include "rc5_decode.h"
#include "stm32100e_eval_lcd.h"
#include "ir_commands.c"

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup IR_Decoding_PWMI
  * @{
  */
  
/** @addtogroup RC5
  * @brief RC5 driver modules
  * @{
  */

/** @defgroup RC5_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup RC5_Private_Defines
  * @{
  */
/**
  * @}  
  */


/** @defgroup RC5_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup RC5_Private_Variables
  * @{
  */
/* Logic table for rising edge: every line has values corresponding to previous bit.
   In columns are actual bit values for given bit time. */
const tRC5_lastBitType RC5_logicTableRisingEdge[2][2] =
{
  {RC5_ZER ,RC5_INV}, /* lastbit = ZERO */
  {RC5_NAN ,RC5_ZER}, /* lastbit = ONE  */
};

/* Logic table for falling edge: every line has values corresponding to previous bit. 
   In columns are actual bit values for given bit time. */
const tRC5_lastBitType RC5_logicTableFallingEdge[2][2] =
{
  {RC5_NAN ,RC5_ONE},  /* lastbit = ZERO */
  {RC5_ONE ,RC5_INV},  /* lastbit = ONE  */
};


__IO StatusYesOrNo RC5FrameReceived = NO; /*!< RC5 Frame state */ 
__IO tRC5_packet   RC5TmpPacket;          /*!< First empty packet */

/* RC5  bits time definitions */
uint16_t  RC5MinT = 0;
uint16_t  RC5MaxT = 0;
uint16_t  RC5Min2T = 0;
uint16_t  RC5Max2T = 0;
__IO uint32_t TIMCLKValueKHz; /*!< Timer clock */
uint16_t RC5TimeOut = 0;
/**
  * @}
  */

/** @defgroup RC5_Private_FunctionPrototypes
  * @{
  */
static void IR_RC5_DataSampling(uint16_t rawPulseLength, uint8_t edge);
static uint8_t IR_RC5_GetPulseLength (uint16_t pulseLength);
static void IR_RC5_modifyLastBit(tRC5_lastBitType bit);
static void IR_RC5_WriteBit(uint8_t bitVal);
static uint32_t TIM_GetCounterCLKValue(void);

/**
  * @}
  */

/** @defgroup RC5_Private_Functions
  * @{
  */
  
/**
  * @brief  De-initializes the peripherals (RCC,GPIO, TIM)       
  * @param  None
  * @retval None
  */
void IR_RC5_DeInit(void)
{ 
  TIM_DeInit(IR_TIM);
  GPIO_DeInit(IR_GPIO_PORT);
}

/**
  * @brief  Initialize the RC5 decoder module ( Time range)
  * @param  None
  * @retval None
  */
void IR_RC5_Init(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  
  /*  Clock Configuration for TIMER */
  RCC_APB1PeriphClockCmd(IR_TIM_CLK , ENABLE);

  /* Enable Button GPIO clock */
  RCC_APB2PeriphClockCmd(IR_GPIO_PORT_CLK | RCC_APB2Periph_AFIO, ENABLE);
 
  /* Pin configuration: input floating */
  GPIO_InitStructure.GPIO_Pin = IR_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(IR_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the TIM global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = IR_TIM_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIMER frequency input */
  TIM_PrescalerConfig(IR_TIM, TIM_PRESCALER, TIM_PSCReloadMode_Immediate);

  /* TIM configuration */
  TIM_ICInitStructure.TIM_Channel = IR_TIM_Channel;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_PWMIConfig(IR_TIM, &TIM_ICInitStructure); 

  /* Timer Clock */
  TIMCLKValueKHz = TIM_GetCounterCLKValue()/1000; 

  /* Select the TIM3 Input Trigger: TI1FP1 */
  TIM_SelectInputTrigger(IR_TIM, TIM_TS_TI1FP1);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(IR_TIM, TIM_SlaveMode_Reset);

  /* Enable the Master/Slave Mode */
  TIM_SelectMasterSlaveMode(IR_TIM, TIM_MasterSlaveMode_Enable);

  /* Configures the TIM Update Request Interrupt source: counter overflow */
  TIM_UpdateRequestConfig(IR_TIM,  TIM_UpdateSource_Regular);
   
  RC5TimeOut = TIMCLKValueKHz * RC5_TIME_OUT_US/1000;

  /* Set the TIM auto-reload register for each IR protocol */
  IR_TIM->ARR = RC5TimeOut;
  
  /* Clear update flag */
  TIM_ClearFlag(IR_TIM, TIM_FLAG_Update);

  /* Enable TIM3 Update Event Interrupt Request */
  TIM_ITConfig(IR_TIM, TIM_IT_Update, ENABLE);

  /* Enable the CC2/CC1 Interrupt Request */
  TIM_ITConfig(IR_TIM, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(IR_TIM, TIM_IT_CC1, ENABLE);

  /* Enable the timer */
  TIM_Cmd(IR_TIM, ENABLE); 
#ifdef USE_LCD   
  /* Set the LCD Back Color */
  LCD_SetBackColor(LCD_COLOR_RED);
  
  /* Set the LCD Text Color */
  LCD_SetTextColor(LCD_COLOR_GREEN);    
  LCD_DisplayStringLine(LCD_LINE_0, "   STM32100E-EVAL   ");
  LCD_DisplayStringLine(LCD_LINE_1, " RC5 InfraRed Demo  ");
  LCD_SetBackColor(LCD_COLOR_BLUE);
  
  /* Set the LCD Text Color */
  LCD_SetTextColor(LCD_COLOR_WHITE);  
#endif  
  /* Bit time range */
  RC5MinT = (RC5_T_US - RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  RC5MaxT = (RC5_T_US + RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  RC5Min2T = (2 * RC5_T_US - RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  RC5Max2T = (2 * RC5_T_US + RC5_T_TOLERANCE_US) * TIMCLKValueKHz / 1000;
  
  /* Default state */
  IR_RC5_ResetPacket();
}

/**
  * @brief  Decode the IR frame (ADDRESS, COMMAND) when all the frame is 
  *         received, the IRFrameReceived will equal to YES.
  *         The IRFrameReceived is set to YES inside the  IR_DataSampling() 
  *         function when the whole IR frame is received without any error.
  *         If any received bit timing is out of the defined range, the IR packet
  *         is reset and the IRFrameReceived is set to NO.
  *         After the IR received display, the IRFrameReceived is set NO.
  *         User can check the IRFrameReceived variable status to verify if there
  *         is a new IR frame already received.          
  * @param  rc5_frame: pointer to IR_Frame_TypeDef structure that contains the 
  *         the IR protocol fields (Address, Command,...).
  */
void IR_RC5_Decode(IR_Frame_TypeDef *rc5_frame)
{ 
  /* If frame received */
  if(RC5FrameReceived != NO)  
  {
    /* RC5 frame field decoding */
    rc5_frame->FieldBit = (RC5TmpPacket.data >> 12) & 0x1;
    rc5_frame->ToggleBit = (RC5TmpPacket.data >> 11) & 0x1;
    rc5_frame->Address = (RC5TmpPacket.data >> 6) & 0x1F;
    rc5_frame->Command = (RC5TmpPacket.data) & 0x3F; 
    
    /* Default state */
    RC5FrameReceived = NO;
    IR_RC5_ResetPacket();
    
#ifdef USE_LCD 
    /* Display RC5 message */
    LCD_DisplayStringLine(LCD_LINE_5, rc5_Commands[rc5_frame->Command]);
    LCD_DisplayStringLine(LCD_LINE_6, rc5_devices[rc5_frame->Address]);
#endif
  }
}

/**
  * @brief  Set the incoming packet structure to default state.
  * @param  None
  * @retval None
  */
void IR_RC5_ResetPacket(void)
{
  RC5TmpPacket.data = 0;
  RC5TmpPacket.bitCount = RC5_PACKET_BIT_COUNT - 1;
  RC5TmpPacket.lastBit = RC5_ONE;
  RC5TmpPacket.status = RC5_PACKET_STATUS_EMPTY;
}

/**
  * @brief  This function handles TIM interrupt Handler.
  *         RC5 Infrared decoder Implementation
  *         ===================================  
  *         Capture Compare interrupt:
  *         --------------------------
  *        - Timer Falling Edge Event:
  *         The Timer interrupt is used to measure the period between two 
  *         successive falling edges (The whole pulse duration).
  *
  *         - Timer Rising Edge Event:  
  *         It is also used to measure the duration between falling and rising 
  *         edges (The low pulse duration).
  *         The two durations are useful to determine the bit value. Each bit is 
  *         determined according to the last bit. 
  *
  *         Update event:InfraRed decoders time out event.
  *         ---------------------------------------------
  *         It resets the InfraRed decoders packet.
  *         - The Timer Overflow is set to 3.6 ms .
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler (void)
{
  static uint32_t ICValue1;
  static uint32_t ICValue2;
  
  /* IC1 Interrupt*/
  if((TIM_GetFlagStatus(IR_TIM, TIM_FLAG_CC1) != RESET))
  {
    TIM_ClearFlag(IR_TIM, TIM_FLAG_CC1);
    /* Get the Input Capture value */
    ICValue2 = TIM_GetCapture1(IR_TIM);
    /* RC5 */
    IR_RC5_DataSampling(ICValue2 - ICValue1, 0);
  }  /* IC2 Interrupt */   
  else  if((TIM_GetFlagStatus(IR_TIM, TIM_FLAG_CC2) != RESET))
  {
    TIM_ClearFlag(IR_TIM, TIM_FLAG_CC2);
    /* Get the Input Capture value */
    ICValue1 = TIM_GetCapture2(IR_TIM);
    IR_RC5_DataSampling(ICValue1 , 1);
  } 
  /* Checks whether the IR_TIM flag is set or not.*/
  else if ((TIM_GetFlagStatus(IR_TIM, TIM_FLAG_Update) != RESET))
  { 
    /* Clears the IR_TIM's pending flags*/
    TIM_ClearFlag(IR_TIM, TIM_FLAG_Update);
    
    IR_RC5_ResetPacket(); 
  }
}

/**
  * @brief  Identify the RC5 data bits.
  * @param  rawPulseLength: low/high pulse duration
  * @param  edge: '1' for Rising  or '0' for falling edge
  * @retval None
  */
static void IR_RC5_DataSampling(uint16_t rawPulseLength, uint8_t edge)
{ 
  uint8_t pulse;
  tRC5_lastBitType tmpLastBit;
  
  /* Decode the pulse length in protocol units */
  pulse = IR_RC5_GetPulseLength(rawPulseLength);

  /* On Rising Edge */
  if (edge == 1)
  { 
    if (pulse <= RC5_2T_TIME) 
    { 
      /* Bit determination by the rising edge */
      tmpLastBit = RC5_logicTableRisingEdge[RC5TmpPacket.lastBit][pulse];
      IR_RC5_modifyLastBit (tmpLastBit);
    }
    else
    {
      IR_RC5_ResetPacket();
    }
  } 
  else     /* On Falling Edge */
  {
    /* If this is the first falling edge - don't compute anything */
    if (RC5TmpPacket.status & RC5_PACKET_STATUS_EMPTY)
    { 
      RC5TmpPacket.status &= (uint8_t)~RC5_PACKET_STATUS_EMPTY;
    }
    else	
    {
      if (pulse <= RC5_2T_TIME) 
      { 
        /* Bit determination by the falling edge */
        tmpLastBit = RC5_logicTableFallingEdge[RC5TmpPacket.lastBit][pulse];
        IR_RC5_modifyLastBit (tmpLastBit);
      }
      else
      {
        IR_RC5_ResetPacket();
      }
    }
  }
}  

/**
  * @brief  Convert raw pulse length expressed in timer ticks to protocol bit times.
  * @param  pulseLength:pulse duration
  * @retval bit time value
  */
static uint8_t IR_RC5_GetPulseLength (uint16_t pulseLength)
{ 
  /* Valid bit time */
  if ((pulseLength > RC5MinT) && (pulseLength < RC5MaxT))
  { 
    /* We've found the length */
    return (RC5_1T_TIME);	/* Return the correct value */
  }
  else if ((pulseLength > RC5Min2T) && (pulseLength < RC5Max2T))
  {
    /* We've found the length */
    return (RC5_2T_TIME);/* Return the correct value */
  }
  return RC5_WRONG_TIME;/* Error */
}

/**
  * @brief  perform checks if the last bit was not incorrect. 
  * @param  bit: where bit can be  RC5_NAN or RC5_INV or RC5_ZER or RC5_ONE
  * @retval None
  */
static void IR_RC5_modifyLastBit(tRC5_lastBitType bit)
{
  if (bit != RC5_NAN)
  {
    if (RC5TmpPacket.lastBit != RC5_INV)
    { 
      /* Restore the last bit */
      RC5TmpPacket.lastBit = bit;

      /* Insert one bit into the RC5 Packet */
      IR_RC5_WriteBit(RC5TmpPacket.lastBit);
    }
    else 
    {
      IR_RC5_ResetPacket();
    }
  }
}

/**
  * @brief  Insert one bit into the final data word. 
  * @param  bitVal: bit value 'RC5_ONE' or 'RC5_ZER'
  * @retval None
  */
static void IR_RC5_WriteBit(uint8_t bitVal)
{
  /* First convert RC5 symbols to ones and zeros */
  if (bitVal == RC5_ONE)
  { 
    bitVal = 1;
  }
  else if (bitVal == RC5_ZER)
  {
    bitVal = 0;
  }
  else
  {
    IR_RC5_ResetPacket();
    return;
  } 

  /* Write this particular bit to data field */
  RC5TmpPacket.data |=  bitVal; 
  
  /* Test the bit number determined */ 
  if (RC5TmpPacket.bitCount != 0)  /* If this is not the last bit */
  {
    /* Shift the data field */
    RC5TmpPacket.data = RC5TmpPacket.data << 1;
    /* And decrement the bitCount */
    RC5TmpPacket.bitCount--;
  } 
  else
  {
   RC5FrameReceived = YES;
  }
}

/**
  * @brief  Identify TIM clock
  * @param  None
  * @retval Timer clock
  */
static uint32_t TIM_GetCounterCLKValue(void)
{
  uint32_t apbprescaler = 0, apbfrequency = 0;
  uint32_t timprescaler = 0;
  __IO RCC_ClocksTypeDef RCC_ClockFreq;   
  
  /* This function fills the RCC_ClockFreq structure with the current
  frequencies of different on chip clocks */
  RCC_GetClocksFreq((RCC_ClocksTypeDef*)&RCC_ClockFreq);
  
  if((IR_TIM == TIM1) || (IR_TIM == TIM8) || (IR_TIM == TIM15) ||
     (IR_TIM == TIM16) || (IR_TIM == TIM17))
  {    
    /* Get the clock prescaler of APB2 */
    apbprescaler = ((RCC->CFGR >> 11) & 0x7);
    apbfrequency = RCC_ClockFreq.PCLK2_Frequency;
    timprescaler = TIM_PRESCALER;
  }
  else if((IR_TIM == TIM2) || (IR_TIM == TIM3) || (IR_TIM == TIM4) ||
          (IR_TIM == TIM5) || (IR_TIM == TIM6) || (IR_TIM == TIM7) )
  {
    /* Get the clock prescaler of APB1 */
    apbprescaler = ((RCC->CFGR >> 8) & 0x7);
    apbfrequency = RCC_ClockFreq.PCLK1_Frequency; 
    timprescaler = TIM_PRESCALER;
  }
  
  /* If APBx clock div >= 4 */
  if (apbprescaler >= 4)
  {
    return ((apbfrequency * 2)/(timprescaler + 1));
  }
  else
  {
    return (apbfrequency/(timprescaler+ 1));
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

/**
  * @}
  */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
