/**
  ******************************************************************************
  * @file USART FIFO/inc/platform_config.h
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date  12/15/2009
  * @brief  Evaluation board specific configuration file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run the example */
#if !defined (USE_STM3210B_EVAL) &&  !defined (USE_STM3210E_EVAL) &&  !defined (USE_STM3210C_EVAL)
 #define USE_STM3210B_EVAL
 //#define USE_STM3210E_EVAL
 //#define USE_STM3210C_EVAL
#endif

/* Define the STM32F10x hardware depending on the used evaluation board */
#ifdef USE_STM3210B_EVAL 

#define  USART_Rx                 USART2
#define  USART_Rx_GPIO            GPIOD
#define  USART_Rx_CLK             RCC_APB1Periph_USART2 
#define  USART_Rx_GPIO_CLK        RCC_APB2Periph_GPIOD 
#define  GPIO_RxPin               GPIO_Pin_6

#define  USART_Tx                 USART1
#define  USART_Tx_GPIO            GPIOA
#define  USART_Tx_CLK             RCC_APB2Periph_USART1
#define  USART_Tx_GPIO_CLK        RCC_APB2Periph_GPIOA 
#define  GPIO_TxPin               GPIO_Pin_9
#define  USART_Tx_DR_Base         USART1_DR_Base
#define  DMA1_Channel_Tx          DMA1_Channel4
#define  DMA1_Channel_Tx_IRQn     DMA1_Channel4_IRQn


#elif defined USE_STM3210E_EVAL 

#define  USART_Rx                 USART2
#define  USART_Rx_GPIO            GPIOA
#define  USART_Rx_CLK             RCC_APB1Periph_USART2 
#define  USART_Rx_GPIO_CLK        RCC_APB2Periph_GPIOA
#define  GPIO_RxPin               GPIO_Pin_3

#define  USART_Tx                 USART1
#define  USART_Tx_GPIO            GPIOA
#define  USART_Tx_CLK             RCC_APB2Periph_USART1
#define  USART_Tx_GPIO_CLK        RCC_APB2Periph_GPIOA
#define  GPIO_TxPin               GPIO_Pin_9
#define  USART_Tx_DR_Base         USART1_DR_Base
#define  DMA1_Channel_Tx          DMA1_Channel4
#define  DMA1_Channel_Tx_IRQn     DMA1_Channel4_IRQn


#elif defined USE_STM3210C_EVAL

#define  USART_Rx                 USART2
#define  USART_Rx_GPIO            GPIOD
#define  USART_Rx_CLK             RCC_APB1Periph_USART2 
#define  USART_Rx_GPIO_CLK        RCC_APB2Periph_GPIOD 
#define  GPIO_RxPin               GPIO_Pin_6

#define  USART_Tx                 USART3
#define  USART_Tx_GPIO            GPIOC
#define  USART_Tx_CLK             RCC_APB1Periph_USART3
#define  USART_Tx_GPIO_CLK        RCC_APB2Periph_GPIOC 
#define  GPIO_TxPin               GPIO_Pin_10
#define  USART_Tx_DR_Base         USART3_DR_Base
#define  DMA1_Channel_Tx          DMA1_Channel2
#define  DMA1_Channel_Tx_IRQn     DMA1_Channel2_IRQn


#endif /* USE_STM3210B_EVAL */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
