/**
  ******************************************************************************
  * @file WhiteNoiseMethod/src/main.c
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main program body.
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
#include "stm32l1xx.h"
#include "oversampling.h"


/** @addtogroup WhiteNoiseMethod
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x40012458)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef Thermal_Noise_Measure
/* This parameter to compute the ADC thermal noise */
extern uint16_t ADC_ConvertedValue[Total_Samples_Number];
#else
extern uint16_t ADC_ConvertedValue[ADC_Oversampling_Factor];
#endif
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void TIM2_Configuration ( uint32_t Sampling_Period );
void DMA_ADC_Configuration ( void );
#if defined (Thermal_Noise_Measure) || defined(Oversampling_Test)
void USART_Configuration (void);
#endif


/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int main(void)
{


  /* Setup STM32 system (clock, PLL and Flash configuration) -----------------*/
  SystemInit();

  /* NVIC configuration ------------------------------------------------------*/
  NVIC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

  /* USART2 Configuration ----------------------------------------------------*/
  #if defined ( Thermal_Noise_Measure ) || defined (Oversampling_Test)
  USART_Configuration();
  #endif

  /* ADC / DMA configuration -------------------------------------------------*/
  DMA_ADC_Configuration();

   /* OverSampling Buffers Initialization -------------------------------------*/
  Oversampling_InitData();

  /* TIM2 configuration with the sampling frequency --------------------------*/
  TIM2_Configuration( Input_Signal_Sampling_Period );

  /* Start ADC conversion ----------------------------------------------------*/
  ADC_SoftwareStartConv(ADC1);

#ifndef  Thermal_Noise_Measure
  /* TIM2 enable counter -----------------------------------------------------*/
  TIM_Cmd(TIM2, ENABLE);

#endif

  while(1)
  {
  }

}



/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* enable clock for PA and PD ports */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOD, ENABLE);

  /* Set USART2 Rx Tx Pins remapped */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  
  /* Configure PA5 (ADC Channel5) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  #if defined (Thermal_Noise_Measure) || defined(Oversampling_Test)
  /* Configure USART2 Tx (PD5) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure USART2 Rx (PD6) as alternate function input*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  #endif
}



/**
  * @brief  Configures Vector Table base location.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

  /* Enable DMA channel1 IRQ Channel -----------------------------------------*/
   NVIC_SetPriorityGrouping(7); /* 0 bits for pre-emption priority 4 bits for
subpriority*/
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0x00); /* 0x00 = 0x0 << 3 | (0x0 & 0x7)*/
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);


  /* Enable TIM2 IRQ Channel -------------------------------------------------*/
  NVIC_SetPriorityGrouping(7); /* 0 bits for pre-emption priority 4 bits for
subpriority*/
  NVIC_SetPriority(TIM2_IRQn, 0x02); /* 0x00 = 0x0 << 3 | (0x02 & 0x7) */
  NVIC_EnableIRQ(TIM2_IRQn);

}



/**
  * @brief  Configures the TIM2 to generate an interrupt after each
  *   sampling period
  * @param e required oversampling period in useconds
  * @retval : None
  */
void TIM2_Configuration ( uint32_t Sampling_Period )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* enable clock for TIM2 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* -----------------------------------------------------------------------
  TIM2CLK = 32 MHz, Prescaler = 32, TIM2 counter clock = 1 MHz
  Period is configured with the Sampling_Period - 1
  ----------------------------------------------------------------------- */
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = Sampling_Period - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = 31;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM2, ENABLE);

  /* Clear the interrupt Update pending bit */
  TIM_ClearITPendingBit( TIM2, TIM_IT_Update);

  /* Enable Timer 2 Update Event Interrupt */
  TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE);
}



/**
  * @brief  Configures the ADC and corresponding DMA channel
  * @param  None
  * @retval : None
  */
void DMA_ADC_Configuration ( void )
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  /* DMA channel1 configuration ----------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;

#ifdef Thermal_Noise_Measure
  DMA_InitStructure.DMA_BufferSize = Total_Samples_Number;
#else
  DMA_InitStructure.DMA_BufferSize = ADC_Oversampling_Factor;
#endif
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable the End of Transfer Interrupt after each oversampling factor samples */
  DMA_ITConfig( DMA1_Channel1, DMA_IT_TC, ENABLE );

  /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;

  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel5 configuration */
  ADC_RegularChannelConfig(ADC1,ADC_Channel_5, 1, ADC_SampleTime_4Cycles);

  /* Enable ADC1 DMA */
  ADC_DMARequestAfterLastTransferCmd(ADC1, DISABLE);
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
  {
  }
}

#if defined ( Thermal_Noise_Measure ) || defined (Oversampling_Test)


/**
  * @brief  Configures the USART2.
  * @param  None
  * @retval : None
  */
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStructure;

/* USART2 configuration ------------------------------------------------------*/
  /* USART2 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
                         the SCLK pin
  */
  /* enable clock for USART2 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
  USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

  USART_Init(USART2, &USART_InitStructure);

  USART_ClockInit(USART2, &USART_ClockInitStructure);

  /* Enable USART2 */
  USART_Cmd(USART2, ENABLE);
}
#endif

#ifdef  USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
