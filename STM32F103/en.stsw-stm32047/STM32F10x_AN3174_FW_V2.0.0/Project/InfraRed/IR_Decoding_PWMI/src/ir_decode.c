/**
  ******************************************************************************
  * @file    InfraRed/IR_Decoding_PWMI/src/ir_decode.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   This file provides all the InfraRed Decode firmware functions.
  *    
  * 1. How to use this driver
  * -------------------------         
  *      - TIM3 Capture Compare and Update interrupts are used to decode the IR
  *        frame, if a frame is received correctly a global variable IRFrameReceived 
  *        will be set to inform the application.
  *          
  *      - Then the application should call the function IR_Decode() to retrieve 
  *        the received IR frame. 
  *        
  *      - You can easily tailor this driver to any other InfraRed protocol, 
  *        by just adapting the defines from sirc_decode.h to the InfraRed 
  *        protocol specification(Bit Duration, Header Duration, Marge Tolerance, 
  *        Number of bits...) and the command and device tables from ir_commands.c.         
  *                   
  * 2. Important to know
  * --------------------  
  *      - TIM3_IRQHandler ISRs are coded within this driver.
  *        If you are using one or both Interrupts in your application you have 
  *        to proceed carefully:   
  *           - either add your application code in these ISRs
  *           - or copy the contents of these ISRs in you application code
  *                              
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
#include "ir_decode.h"
#include "stm32100e_eval_lcd.h"
#include "ir_commands.c"

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup IR_Decoding_PWMI
  * @{
  */

/** @addtogroup IR_DECODE
  * @brief IR_DECODE driver modules
  * @{
  */

/** @defgroup IR_DECODE_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup IR_DECODE_Private_Defines
  * @{
  */
/**
  * @}  
  */


/** @defgroup IR_DECODE_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup IR_DECODE_Private_Variables
  * @{
  */
__IO StatusYesOrNo IRFrameReceived = NO;   /*!< IR frame status */
  
/* IR bit definitions*/
uint16_t IROnTimeMin = 0; 
uint16_t IROnTimeMax = 0; 
uint16_t IRValueStep = 0;
uint16_t IRValueMargin = 0;
uint16_t IRValue00 = 0;

/* Header time definitions*/
uint16_t IRHeaderLowMin = 0;
uint16_t IRHeaderLowMax = 0;
uint16_t IRHeaderWholeMin = 0;
uint16_t IRHeaderWholeMax = 0;
tIR_packet IRTmpPacket; /*!< IR packet*/

uint16_t IRTimeOut = 0;
__IO uint32_t TIMCLKValueKHz = 0;/*!< Timer clock */

/**
  * @}
  */

/** @defgroup IR_DECODE_Private_FunctionPrototypes
  * @{
  */
 
static void IR_DataSampling(uint32_t lowPulseLength, uint32_t wholePulseLength);
static uint8_t IR_DecodeHeader(uint32_t lowPulseLength, uint32_t wholePulseLength);
static uint8_t IR_DecodeBit(uint32_t lowPulseLength, uint32_t wholePulseLength);
static uint32_t TIM_GetCounterCLKValue(void);
 
/**
  * @}
  */

/** @defgroup IR_DECODE_Private_Functions
  * @{
  */

/**
  * @brief  De-initializes the peripherals (RCC,GPIO, TIM)       
  * @param  None
  * @retval None
  */
void IR_DeInit(void)
{ 
  TIM_DeInit(IR_TIM);
  GPIO_DeInit(IR_GPIO_PORT);
}

/**
  * @brief  Initialize the IR bit time range.
  * @param  None
  * @retval None
  */
void IR_Init(void)
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
   
  IRTimeOut = TIMCLKValueKHz * IR_TIME_OUT_US/1000;

  /* Set the TIM auto-reload register for each IR protocol */
  IR_TIM->ARR = IRTimeOut;
  
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
  LCD_DisplayStringLine(LCD_LINE_1, IR_MESSAGE);
  
  LCD_SetBackColor(LCD_COLOR_BLUE);
  /* Set the LCD Text Color */
  LCD_SetTextColor(LCD_COLOR_WHITE);
#endif
  /* Header */	
  IRHeaderLowMin = TIMCLKValueKHz * IR_HEADER_LOW_MIN_US/1000;
  IRHeaderLowMax = TIMCLKValueKHz * IR_HEADER_LOW_MAX_US/1000;
  IRHeaderWholeMin = TIMCLKValueKHz * IR_HEADER_WHOLE_MIN_US/1000;
  IRHeaderWholeMax = TIMCLKValueKHz * IR_HEADER_WHOLE_MAX_US/1000;

  /* Bit time range*/
  IROnTimeMax = TIMCLKValueKHz * IR_ONTIME_MAX_US /1000;
  IROnTimeMin = TIMCLKValueKHz * IR_ONTIME_MIN_US/1000; 
  IRValueStep = TIMCLKValueKHz * IR_VALUE_STEP_US/1000;
  IRValueMargin = TIMCLKValueKHz * IR_VALUE_MARGIN_US/1000;
  IRValue00 = TIMCLKValueKHz * IR_VALUE_00_US/1000;

  /* Default state */
  IR_ResetPacket();
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
  * @param  ir_frame: pointer to IR_Frame_TypeDef structure that contains the 
  *         the IR protocol fields (Address, Command,...).
  * @retval None
  */
void IR_Decode(IR_Frame_TypeDef *ir_frame)
{  
  if(IRFrameReceived != NO) 
  {
    IRTmpPacket.data = __RBIT(IRTmpPacket.data);

#ifdef IR_SIRC_PROTOCOL   
    ir_frame->Command = (IRTmpPacket.data >> 20)& 0x7F;
    ir_frame->Address = (IRTmpPacket.data >> 27) & 0x1F;
#endif
    
    /* Default state */
    IRFrameReceived = NO; 
    IR_ResetPacket();
    
#ifdef USE_LCD 
    /* Display command and address */
    LCD_DisplayStringLine(LCD_LINE_5, IR_Commands[ir_frame->Command]);
    LCD_DisplayStringLine(LCD_LINE_6, IR_devices[ir_frame->Address]);
#endif
  }
}

/**
  * @brief  Put the packet to default state, ready to be filled in by incoming 
  *         data.
  * @param  None
  * @retval None
  */
void IR_ResetPacket(void)
{
  IRTmpPacket.count = 0;
  IRTmpPacket.status = INITIAL_STATUS;
  IRTmpPacket.data = 0;
}

/**
  * @brief  This function handles TIM interrupt Handler.
  *         SIRC Infrared decoder Implementation
  *         ===================================  
  *         Capture Compare Interrupt:Timer Falling Edge Event:
  *         ---------------------------------------------------
  *         The Timer interrupt is used to measure the different period between 
  *         two successive falling edges in order to identify the frame bits.
  *
  *         We measure the low pulse duration:
  *           - If the period measured is equal to T = 1200 µs and the low pulse 
  *             duration is equal to T/2 = 600µs => the bit is logic '0'. 
  *           - If the period measured is equal to 3T/2 = 1800µs and the low pulse 
  *             duration is equal to T = 1200µs => the bit is logic '1'.
  *           - If the whole period measured is equal to 3000µs and the low pulse 
  *             duration is equal to 2400µs => the bit is ‘start bit’.
  *
  *         Update event:InfraRed decoders time out event
  *         ----------------------------------------------
  *         It resets the InfraRed decoders packet.                 
  *         - The Timer Overflow is set to 4 ms. 
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler (void)
{
  static uint32_t ICValue1;
  static uint32_t ICValue2;
  
  /* IC1 Interrupt */
  if((TIM_GetFlagStatus(IR_TIM, TIM_FLAG_CC1) != RESET))
  {
    TIM_ClearFlag(IR_TIM, TIM_FLAG_CC1);
    /* Get the Input Capture value */
    ICValue2 = TIM_GetCapture1(IR_TIM);
    IR_DataSampling(ICValue1, ICValue2); 
  }  /* IC2 Interrupt*/   
  else  if((TIM_GetFlagStatus(IR_TIM, TIM_FLAG_CC2) != RESET))
  {
    TIM_ClearFlag(IR_TIM, TIM_FLAG_CC2);
    /* Get the Input Capture value */
    ICValue1 = TIM_GetCapture2(IR_TIM);
  } 
  /* Checks whether the IR_TIM flag is set or not. */
  else if ((TIM_GetFlagStatus(IR_TIM, TIM_FLAG_Update) != RESET))
  { 
    /* Clears the IR_TIM's pending flags*/
    TIM_ClearFlag(IR_TIM, TIM_FLAG_Update);
    
    IR_ResetPacket();
  }
}

/**
  * @brief  Identify the IR data.
  * @param  lowPulseLength: low pulse duration. 
  * @param  wholePulseLength: whole pulse duration.
  * @retval None
  */
static void IR_DataSampling(uint32_t lowPulseLength, uint32_t wholePulseLength)
{
  uint8_t  tmpBit = 0;
  
  /* If the pulse timing is correct */
  if ((IRTmpPacket.status == IR_STATUS_RX)) 
  {
    /* Convert raw pulse timing to data value */
    tmpBit = IR_DecodeBit(lowPulseLength, wholePulseLength); 
    if (tmpBit != IR_BIT_ERROR) /* If the pulse timing is correct */
    {
      /* This operation fills in the incoming bit to the correct position in
      32 bit word*/
      IRTmpPacket.data |= tmpBit;
      IRTmpPacket.data <<= 1;
      /* Increment the bit count  */
      IRTmpPacket.count++;
    }
    /* If all bits identified */
    if (IRTmpPacket.count == IR_TOTAL_BITS_COUNT)
    {
      /* Frame received*/
      IRFrameReceived = YES;
    }
    /* Bit 15:the idle time between IR message and the inverted one */
    else if (IRTmpPacket.count == IR_BITS_COUNT)
    {
      IRTmpPacket.status = IR_STATUS_HEADER; 
    }
  }
  else if ((IRTmpPacket.status == IR_STATUS_HEADER))
  {
    /* Convert raw pulse timing to data value */
    tmpBit = IR_DecodeHeader(lowPulseLength, wholePulseLength);
    
    /* If the Header timing is correct */
    if ( tmpBit!= IR_HEADER_ERROR)
    { 
      /* Reception Status for the inverted message */
      IRTmpPacket.status = IR_STATUS_RX; 
    }
    else
    {
      /* Wrong header time */
      IR_ResetPacket();  
    }
  }
}

/**
  * @brief  Check the header pulse if it has correct pulse time.
  * @param  lowPulseLength: low pulse header duration.
  * @param  wholePulseLength: whole pulse header duration.
  * @retval IR_HEADER_OK or IR_HEADER_ERROR
  */
static uint8_t IR_DecodeHeader(uint32_t lowPulseLength, uint32_t wholePulseLength)
{
  /* First check if low pulse time is according to the specs */
  if ((lowPulseLength >= IRHeaderLowMin) && (lowPulseLength <= IRHeaderLowMax)) 
  {
    /* Check if the high pulse is OK */
    if ((wholePulseLength >= IRHeaderWholeMin) && (wholePulseLength <= IRHeaderWholeMax))
    {
      return IR_HEADER_OK; /* Valid Header */
    }
  }	
  return IR_HEADER_ERROR;  /* Wrong Header */
}

/**
  * @brief  Convert raw time to data value.
  * @param  lowPulseLength: low pulse bit duration.
  * @param  wholePulseLength: whole pulse bit duration.
  * @retval BitValue( data0 or data1) or IR_BIT_ERROR
  */
static uint8_t IR_DecodeBit(uint32_t lowPulseLength , uint32_t wholePulseLength)
{
  uint8_t i = 0;
  
  /* First check if low pulse time is according to the specs */
  if ((lowPulseLength >= IROnTimeMin) && (lowPulseLength <= IROnTimeMax))
  {
    for (i = 0; i < 2; i++) /* There are data0 to data1 */
    {
      /* Check if the length is in given range */
      if ((wholePulseLength >= IRValue00 + (IRValueStep * i) - IRValueMargin) 
          && (wholePulseLength <= IRValue00 + (IRValueStep * i) + IRValueMargin))
        return i; /* Return bit value */
    }
  }
  return IR_BIT_ERROR; /* No correct pulse length was found */
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
