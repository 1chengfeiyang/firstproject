/**
  ******************************************************************************
  * @file    hal_uart_driver_definition.h
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Header for hal_uart.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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

/**
 * This file is expected to be used only by the UART driver implemented in the file hal_uart.c.
 * It provided the mapping of the user selection to the UART driver
 * The user is not expected to modify this file
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_UART_DRIVER_DEFINITION_H
#define __HAL_UART_DRIVER_DEFINITION_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal_uart_driver_configuration.h"

/* Exported types ------------------------------------------------------------*/
/* Exported defines --------------------------------------------------------*/

 /**
  *	UART mapping definition
  *
  *	The list of definition below is used to map the selected UART, the IO dedicated
  *	and the different resources used in the STM32 (Clock, DMA, etc...)
  */
 #if (USART_SELECTED == BLE_ON_USART1)

 #define	USART_BLE					USART1
 #define RCC_APBPeriph_USART_BLE		RCC_APB2Periph_USART1
 #define USART_NVIC_VECTOR			USART1_IRQn

 /**
  * DMA UART TX CHANNEL
  */
 #define DMA_USART_TX_CHANNEL   		DMA1_Channel4
 /**
  * DMA UART RX CHANNEL
  */
 #define DMA_USART_RX_CHANNEL		DMA1_Channel5
 #define DMA_USART_RX_CHANNEL_FLAG	DMA1_IT_TC5
 #define DMA_USART_RX_NVIC_VECTOR	DMA1_Channel5_IRQn

 #if	(USART_TX_PIN_SELECTED == PA9)

 #define USART_TX_GPIO_AHB_CLK		RCC_AHBPeriph_GPIOA
 #define USART_TX_GPIO_PORT          GPIOA
 #define USART_TX_PIN                GPIO_Pin_9
 #define USART_TX_PIN_SOURCE         GPIO_PinSource9
 #define USART_TX_AF                 GPIO_AF_USART1

 #elif (USART_TX_PIN_SELECTED == PB6)

 #define USART_TX_GPIO_AHB_CLK		RCC_AHBPeriph_GPIOB
 #define USART_TX_GPIO_PORT          GPIOB
 #define USART_TX_PIN                GPIO_Pin_6
 #define USART_TX_PIN_SOURCE         GPIO_PinSource6
 #define USART_TX_AF                 GPIO_AF_USART1

 #else
 #error USART_TX_PIN NOT SUPPORTED
 #endif

 #if	(USART_RX_PIN_SELECTED == PA10)

 #define USART_RX_GPIO_AHB_CLK       RCC_AHBPeriph_GPIOA
 #define USART_RX_GPIO_PORT          GPIOA
 #define USART_RX_PIN                GPIO_Pin_10
 #define USART_RX_PIN_SOURCE         GPIO_PinSource10
 #define USART_RX_AF                 GPIO_AF_USART1

 #elif (USART_RX_PIN_SELECTED == PB7)

 #define USART_RX_GPIO_AHB_CLK       RCC_AHBPeriph_GPIOB
 #define USART_RX_GPIO_PORT          GPIOB
 #define USART_RX_PIN                GPIO_Pin_7
 #define USART_RX_PIN_SOURCE         GPIO_PinSource7
 #define USART_RX_AF                 GPIO_AF_USART1

 #else
 #error USART_RX_PIN NOT SUPPORTED
 #endif

 #if	(USART_RTS_PIN_SELECTED == PA12)

 #define USART_WRTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOA
 #define USART_WRTS_GPIO_PORT        GPIOA
 #define USART_WRTS_PIN              GPIO_Pin_12
 #define USART_WRTS_PIN_SOURCE       GPIO_PinSource12
 #define USART_WRTS_AF               GPIO_AF_USART1

 #else
 #error USART_RTS_PIN NOT SUPPORTED
 #endif

 #if	(USART_CTS_PIN_SELECTED == PA11)

 #define USART_WCTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOA
 #define USART_WCTS_GPIO_PORT        GPIOA
 #define USART_WCTS_PIN              GPIO_Pin_11
 #define USART_WCTS_PIN_SOURCE       GPIO_PinSource11
 #define USART_WCTS_AF               GPIO_AF_USART1
 #define USART_WCTS_EXTI_PORT		EXTI_PortSourceGPIOA
 #define USART_WCTS_EXTI_PIN			EXTI_PinSource11
 #define USART_WCTS_NVIC_VECTOR		EXTI15_10_IRQn
 #define USART_WCTS_EXTI_LINE		EXTI_Line11

 #else
 #error USART_CTS_PIN NOT SUPPORTED
 #endif


 #elif (USART_SELECTED == BLE_ON_USART2)

 #define	USART_BLE					USART2
 #define RCC_APBPeriph_USART_BLE		RCC_APB1Periph_USART2
 #define USART_NVIC_VECTOR			USART2_IRQn

 /**
  * DMA UART TX CHANNEL
  */
 #define DMA_USART_TX_CHANNEL   		DMA1_Channel7
 /**
  * DMA UART RX CHANNEL
  */
 #define DMA_USART_RX_CHANNEL		DMA1_Channel6
 #define DMA_USART_RX_CHANNEL_FLAG	DMA1_IT_TC6
 #define DMA_USART_RX_NVIC_VECTOR	DMA1_Channel6_IRQn

 #if	(USART_TX_PIN_SELECTED == PA2)

 #define USART_TX_GPIO_AHB_CLK		RCC_AHBPeriph_GPIOA
 #define USART_TX_GPIO_PORT          GPIOA
 #define USART_TX_PIN                GPIO_Pin_2
 #define USART_TX_PIN_SOURCE         GPIO_PinSource2
 #define USART_TX_AF                 GPIO_AF_USART2

 #elif (USART_TX_PIN_SELECTED == PD5)

 #define USART_TX_GPIO_AHB_CLK		RCC_AHBPeriph_GPIOD
 #define USART_TX_GPIO_PORT          GPIOD
 #define USART_TX_PIN                GPIO_Pin_5
 #define USART_TX_PIN_SOURCE         GPIO_PinSource5
 #define USART_TX_AF                 GPIO_AF_USART2

 #else
 #error USART_TX_PIN NOT SUPPORTED
 #endif

 #if	(USART_RX_PIN_SELECTED == PA3)

 #define USART_RX_GPIO_AHB_CLK       RCC_AHBPeriph_GPIOA
 #define USART_RX_GPIO_PORT          GPIOA
 #define USART_RX_PIN                GPIO_Pin_3
 #define USART_RX_PIN_SOURCE         GPIO_PinSource3
 #define USART_RX_AF                 GPIO_AF_USART2

 #elif (USART_RX_PIN_SELECTED == PD6)

 #define USART_RX_GPIO_AHB_CLK       RCC_AHBPeriph_GPIOD
 #define USART_RX_GPIO_PORT          GPIOD
 #define USART_RX_PIN                GPIO_Pin_6
 #define USART_RX_PIN_SOURCE         GPIO_PinSource6
 #define USART_RX_AF                 GPIO_AF_USART2

 #else
 #error USART_RX_PIN NOT SUPPORTED
 #endif

 #if	(USART_RTS_PIN_SELECTED == PA1)

 #define USART_WRTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOA
 #define USART_WRTS_GPIO_PORT        GPIOA
 #define USART_WRTS_PIN              GPIO_Pin_1
 #define USART_WRTS_PIN_SOURCE       GPIO_PinSource1
 #define USART_WRTS_AF               GPIO_AF_USART2

 #elif (USART_RTS_PIN_SELECTED == PD4)

 #define USART_WRTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOD
 #define USART_WRTS_GPIO_PORT        GPIOD
 #define USART_WRTS_PIN              GPIO_Pin_4
 #define USART_WRTS_PIN_SOURCE       GPIO_PinSource4
 #define USART_WRTS_AF               GPIO_AF_USART2

 #else
 #error USART_RTS_PIN NOT SUPPORTED
 #endif

 #if	(USART_CTS_PIN_SELECTED == PA0)

 #define USART_WCTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOA
 #define USART_WCTS_GPIO_PORT        GPIOA
 #define USART_WCTS_PIN              GPIO_Pin_0
 #define USART_WCTS_PIN_SOURCE       GPIO_PinSource0
 #define USART_WCTS_AF               GPIO_AF_USART2
 #define USART_WCTS_EXTI_PORT		EXTI_PortSourceGPIOA
 #define USART_WCTS_EXTI_PIN			EXTI_PinSource0
 #define USART_WCTS_NVIC_VECTOR		EXTI0_IRQn
 #define USART_WCTS_EXTI_LINE		EXTI_Line0

 #elif (USART_CTS_PIN_SELECTED == PD3)

 #define USART_WCTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOD
 #define USART_WCTS_GPIO_PORT        GPIOD
 #define USART_WCTS_PIN              GPIO_Pin_3
 #define USART_WCTS_PIN_SOURCE       GPIO_PinSource3
 #define USART_WCTS_AF               GPIO_AF_USART2
 #define USART_WCTS_EXTI_PORT		EXTI_PortSourceGPIOD
 #define USART_WCTS_EXTI_PIN			EXTI_PinSource3
 #define USART_WCTS_NVIC_VECTOR		EXTI3_IRQn
 #define USART_WCTS_EXTI_LINE		EXTI_Line3

 #else
 #error USART_CTS_PIN NOT SUPPORTED
 #endif

 #elif (USART_SELECTED == BLE_ON_USART3)

 #define	USART_BLE					USART3
 #define RCC_APBPeriph_USART_BLE		RCC_APB1Periph_USART3
 #define USART_NVIC_VECTOR			USART3_IRQn

 /**
  * DMA UART TX CHANNEL
  */
 #define DMA_USART_TX_CHANNEL   		DMA1_Channel2
 /**
  * DMA UART RX CHANNEL
  */
 #define DMA_USART_RX_CHANNEL		DMA1_Channel3
 #define DMA_USART_RX_CHANNEL_FLAG	DMA1_IT_TC3
 #define DMA_USART_RX_NVIC_VECTOR	DMA1_Channel3_IRQn

 #if	(USART_TX_PIN_SELECTED == PB10)

 #define USART_TX_GPIO_AHB_CLK		RCC_AHBPeriph_GPIOB
 #define USART_TX_GPIO_PORT          GPIOB
 #define USART_TX_PIN                GPIO_Pin_10
 #define USART_TX_PIN_SOURCE         GPIO_PinSource10
 #define USART_TX_AF                 GPIO_AF_USART3

 #elif (USART_TX_PIN_SELECTED == PC10)

 #define USART_TX_GPIO_AHB_CLK		RCC_AHBPeriph_GPIOC
 #define USART_TX_GPIO_PORT          GPIOC
 #define USART_TX_PIN                GPIO_Pin_10
 #define USART_TX_PIN_SOURCE         GPIO_PinSource10
 #define USART_TX_AF                 GPIO_AF_USART3

 #elif (USART_TX_PIN_SELECTED == PD8)

 #define USART_TX_GPIO_AHB_CLK		RCC_AHBPeriph_GPIOD
 #define USART_TX_GPIO_PORT          GPIOD
 #define USART_TX_PIN                GPIO_Pin_8
 #define USART_TX_PIN_SOURCE         GPIO_PinSource8
 #define USART_TX_AF                 GPIO_AF_USART3

 #else
 #error USART_TX_PIN NOT SUPPORTED
 #endif

 #if	(USART_RX_PIN_SELECTED == PB11)

 #define USART_RX_GPIO_AHB_CLK       RCC_AHBPeriph_GPIOB
 #define USART_RX_GPIO_PORT          GPIOB
 #define USART_RX_PIN                GPIO_Pin_11
 #define USART_RX_PIN_SOURCE         GPIO_PinSource11
 #define USART_RX_AF                 GPIO_AF_USART3

 #elif (USART_RX_PIN_SELECTED == PC11)

 #define USART_RX_GPIO_AHB_CLK       RCC_AHBPeriph_GPIOC
 #define USART_RX_GPIO_PORT          GPIOC
 #define USART_RX_PIN                GPIO_Pin_11
 #define USART_RX_PIN_SOURCE         GPIO_PinSource11
 #define USART_RX_AF                 GPIO_AF_USART3

 #elif (USART_RX_PIN_SELECTED == PD9)

 #define USART_RX_GPIO_AHB_CLK       RCC_AHBPeriph_GPIOD
 #define USART_RX_GPIO_PORT          GPIOD
 #define USART_RX_PIN                GPIO_Pin_9
 #define USART_RX_PIN_SOURCE         GPIO_PinSource9
 #define USART_RX_AF                 GPIO_AF_USART3

 #else
 #error USART_RX_PIN NOT SUPPORTED
 #endif

 #if	(USART_RTS_PIN_SELECTED == PB14)

 #define USART_WRTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOB
 #define USART_WRTS_GPIO_PORT        GPIOB
 #define USART_WRTS_PIN              GPIO_Pin_14
 #define USART_WRTS_PIN_SOURCE       GPIO_PinSource14
 #define USART_WRTS_AF               GPIO_AF_USART3

 #elif (USART_RTS_PIN_SELECTED == PD12)

 #define USART_WRTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOD
 #define USART_WRTS_GPIO_PORT        GPIOD
 #define USART_WRTS_PIN              GPIO_Pin_12
 #define USART_WRTS_PIN_SOURCE       GPIO_PinSource12
 #define USART_WRTS_AF               GPIO_AF_USART3

 #else
 #error USART_RTS_PIN NOT SUPPORTED
 #endif

 #if	(USART_CTS_PIN_SELECTED == PB13)

 #define USART_WCTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOB
 #define USART_WCTS_GPIO_PORT        GPIOB
 #define USART_WCTS_PIN              GPIO_Pin_13
 #define USART_WCTS_PIN_SOURCE       GPIO_PinSource13
 #define USART_WCTS_AF               GPIO_AF_USART3
 #define USART_WCTS_EXTI_PORT		EXTI_PortSourceGPIOB
 #define USART_WCTS_EXTI_PIN			EXTI_PinSource13
 #define USART_WCTS_NVIC_VECTOR		EXTI15_10_IRQn
 #define USART_WCTS_EXTI_LINE		EXTI_Line13

 #elif (USART_CTS_PIN_SELECTED == PD11)

 #define USART_WCTS_GPIO_AHB_CLK     RCC_AHBPeriph_GPIOD
 #define USART_WCTS_GPIO_PORT        GPIOD
 #define USART_WCTS_PIN              GPIO_Pin_11
 #define USART_WCTS_PIN_SOURCE       GPIO_PinSource11
 #define USART_WCTS_AF               GPIO_AF_USART3
 #define USART_WCTS_EXTI_PORT		EXTI_PortSourceGPIOD
 #define USART_WCTS_EXTI_PIN			EXTI_PinSource11
 #define USART_WCTS_NVIC_VECTOR		EXTI15_10_IRQn
 #define USART_WCTS_EXTI_LINE		EXTI_Line11

 #else
 #error USART_CTS_PIN NOT SUPPORTED
 #endif

 #else
 #error USART NOT SUPPORTED
 #endif

 #define USART_GPIO_AHB_CLK 			(USART_TX_GPIO_AHB_CLK | USART_RX_GPIO_AHB_CLK | USART_WRTS_GPIO_AHB_CLK | USART_WCTS_GPIO_AHB_CLK)
 #define USART_DMA_AHB_CLK 			RCC_AHBPeriph_DMA1

/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

#ifdef __cplusplus
}
#endif

#endif /*__HAL_UART_DRIVER_DEFINITION_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
