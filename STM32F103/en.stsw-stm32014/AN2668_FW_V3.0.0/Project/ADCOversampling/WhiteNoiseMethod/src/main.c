/**
  ******************************************************************************
  * @file WhiteNoiseMethod/src/main.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date   04/27/2009
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
#include "stm32f10x.h"
#include "oversampling.h"



/** @addtogroup WhiteNoiseMethod
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

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
void RCC_Configuration(void);
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
  
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* NVIC configuration ------------------------------------------------------*/
  NVIC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();
  
  /* USART1 Configuration ----------------------------------------------------*/
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
  ADC_SoftwareStartConvCmd( ADC1, ENABLE);
   
#ifndef  Thermal_Noise_Measure  
  /* TIM2 enable counter -----------------------------------------------------*/
  TIM_Cmd(TIM2, ENABLE);
  
#endif  
  
  while(1)
  { 
  }
  
}



/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
   
    /* PLLCLK = 8MHz * 7 = 56 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Enable Peripheral on ABP1 : Timet TIM2 Clock Enable*/
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE);
  
  /* Enable USART1 clock if Thermal Noise or Oversampling test is undertaken */
  #if defined ( Thermal_Noise_Measure ) || defined (Oversampling_Test)
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );
  #endif
}



/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure PC.00 (ADC Channel10) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
   #if defined (Thermal_Noise_Measure) || defined(Oversampling_Test)
  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
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
  * @param e required oversampling period in µseconds
  * @retval : None
  */
void TIM2_Configuration ( uint32_t Sampling_Period )
{  
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  /* -----------------------------------------------------------------------
  TIM2CLK = 56 MHz, Prescaler = 0x37, TIM2 counter clock = 1 MHz 
  Period is configured with the Sampling_Period - 1
  ----------------------------------------------------------------------- */  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = Sampling_Period - 1;           
  TIM_TimeBaseStructure.TIM_Prescaler = 0x37;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;  
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  
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
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel10 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_1Cycles5);
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));     
}


#if defined ( Thermal_Noise_Measure ) || defined (Oversampling_Test)


/**
  * @brief  Configures the USART1.
  * @param  None
  * @retval : None
  */
void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStructure;

/* USART1 configuration ------------------------------------------------------*/
  /* USART1 configured as follow:
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

  USART_Init(USART1, &USART_InitStructure);
  USART_ClockInit(USART1, &USART_ClockInitStructure);

  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
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
