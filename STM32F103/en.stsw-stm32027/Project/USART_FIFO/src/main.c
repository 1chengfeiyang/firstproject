/**
  ******************************************************************************
  * @file USART_FIFO/src/main.c 
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


/** @addtogroup USART_FIFO
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804
#define BufferSize             250
#define FIFO_SIZE              200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
uint8_t TxBuffer1[BufferSize];
uint8_t RxBuffer2[FIFO_SIZE]; /* Buffer emulating the FIFO */
uint8_t RxBuffer2_SW[BufferSize]; /* Buffer used for final data store */

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
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
  
  /* Enable USART_Rx Receive interrupt */
  USART_ITConfig(USART_Rx, USART_IT_RXNE, ENABLE);
  
  /* Enable DMA1 Channel_Tx */
  DMA_Cmd(DMA1_Channel_Tx, ENABLE);
  /* Enable DMA1 Channel6 */
  DMA_Cmd(DMA1_Channel6, ENABLE);  

  /* Enable the USART_Tx */
  USART_Cmd(USART_Tx, ENABLE);

  /* Enable the USART_Rx */
  USART_Cmd(USART_Rx, ENABLE);
  
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
  RCC_APB2PeriphClockCmd(USART_Tx_GPIO_CLK | USART_Rx_GPIO_CLK |
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
  DMA_InitStructure.DMA_BufferSize = FIFO_SIZE;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
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
   
    /* Configure USART2 interrupt */
    NVIC_SetPriority(USART2_IRQn, 0x00); 
    NVIC_EnableIRQ(USART2_IRQn);
    
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
