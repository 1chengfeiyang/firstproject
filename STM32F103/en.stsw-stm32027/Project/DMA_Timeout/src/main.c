/**
  ******************************************************************************
  * @file DMA_Timeout/src/main.c 
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date  12/15/2009
  * @brief  Main program body
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
#include "platform_config.h"

/** @addtogroup DMA_Timeout
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804
#define BufferSize             5
#define BufferSize1            300

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
__IO uint16_t CCR1_Val = 8000;
uint8_t TxBuffer1[BufferSize];
uint8_t RxBuffer2[BufferSize1]; 

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM_Configuration(void);
void DMA_Configuration(void);
void NVIC_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int main(void)
{
   uint16_t i;
   
  /* System Clocks Configuration */
  RCC_Configuration();
  
  /* NVIC configuration */
  NVIC_Configuration();

  /* Configure the GPIO ports */
  GPIO_Configuration();

/* Configure the Timer in slave reset mode  */
  TIM_Configuration();
  
  /* Fill TxBuffer1 buffer */
  for(i=0; i<BufferSize; i++)
  {
    TxBuffer1[i] = i;
  }
  
  /* Configure the DMA */
  DMA_Configuration();

/* USART_Tx and USART_Rx configuration ------------------------------------------------------*/
  /* USART1 and USART2 configured as follow:
        - BaudRate = 460800 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_DeInit(USART_Tx);
  USART_DeInit(USART_Rx);
  USART_InitStructure.USART_BaudRate = 460800;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure USART_Tx */
  USART_Init(USART_Tx, &USART_InitStructure);
  /* Configure USART_Rx */
  USART_Init(USART_Rx, &USART_InitStructure);

  /* Enable USART_Tx DMA Tansmit request */
  USART_DMACmd(USART_Tx,  USART_DMAReq_Tx, ENABLE);
  
  /* Enable USART_Rx DMA Receive request */
  USART_DMACmd(USART_Rx, USART_DMAReq_Rx , ENABLE);
  
  /* Enable DMA1 Channel_Tx */
  DMA_Cmd(DMA1_Channel_Tx, ENABLE);
  /* Enable DMA1 Channel6 */
  DMA_Cmd(DMA1_Channel6, ENABLE);  

  /* Enable the USART_Tx */
  USART_Cmd(USART_Tx, ENABLE);

  /* Enable the USART_Rx */
  USART_Cmd(USART_Rx, ENABLE);
  
    /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);  
  
  while (1)
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
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
   
  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable GPIO and AFIO clocks */
  RCC_APB2PeriphClockCmd(USART_Tx_GPIO_CLK | USART_Rx_GPIO_CLK |RCC_APB2Periph_GPIOC|
                         RCC_APB2Periph_AFIO, ENABLE);
     
#ifdef USE_STM3210C_EVAL
  /* Enable USART_Tx Clock */
  RCC_APB1PeriphClockCmd(USART_Tx_CLK, ENABLE); 
#else
  /* Enable USART_Tx Clock */
  RCC_APB2PeriphClockCmd(USART_Tx_CLK, ENABLE); 
#endif
  /* Enable USART_Rx Clock */
  RCC_APB1PeriphClockCmd(USART_Rx_CLK, ENABLE);  
  
    /* Enable TIM2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
  
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USE_STM3210C_EVAL
  /* Enable the USART3 Pins Software Remapping */
  GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
 
  /* Enable the USART2 Pins Software Remapping */
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);  
#elif defined USE_STM3210B_EVAL
  /* Enable the USART2 Pins Software Remapping */
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#endif
  
   /* Configure USART_Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART_Rx_GPIO, &GPIO_InitStructure);
  
  /* Configure USART_Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART_Tx_GPIO, &GPIO_InitStructure);
  
    /* GPIOA.1 Configuration: TIM2 Channel2 as input floatinng */
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* GPIOC Configuration: Pin 7 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/**
  * @brief  Configures TIM2 in slave reset mode
  * @param  None
  * @retval : None
  */
void TIM_Configuration(void)
{
  
  /* TIM2 configuration ------------------------------------------------------*/ 
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* Output Compare  Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  
   /* TIM2 Channel 2 Input Capture Configuration */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* TIM2 Input trigger configuration: External Trigger connected to TI2 */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
  
  /* TIM2 configuration in slave reset mode  where the timer counter is 
     re-initialied in response to rising edges on an input capture (TI2) */
  TIM_SelectSlaveMode(TIM2,  TIM_SlaveMode_Reset);
  
  /* TIM2 IT CC1 enable */
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  
}  
  
/**
  * @brief  Configures the DMA.
  * @param  None
  * @retval : None
  */
void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* DMA1 Channel (triggered by USART_Tx event) Config */
  DMA_DeInit(DMA1_Channel_Tx);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART_Tx_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer1;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel_Tx, &DMA_InitStructure);

   
  /* DMA1 Channel (triggered by USART2 Rx event) Config */
  DMA_DeInit(DMA1_Channel6);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxBuffer2;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize1;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);
  
  /* Enable DMA1_Channel Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel_Tx, DMA_IT_TC, ENABLE);  

}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
   /* 1 bit for pre-emption priority, 3 bits for subpriority */
    NVIC_SetPriorityGrouping(6); 

    /* Configure DMA1_Channel_Tx interrupt */
    NVIC_SetPriority(DMA1_Channel_Tx_IRQn, 0x01); 
    NVIC_EnableIRQ(DMA1_Channel_Tx_IRQn);
   
      /* Configure TIM2 interrupt */
    NVIC_SetPriority(TIM2_IRQn, 0x02); 
    NVIC_EnableIRQ(TIM2_IRQn);
    
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
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
