/**
  ******************************************************************************
  * @file    hal_uart_driver_configuration.h
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
  *	This file contains the full configuration of the UART available to the user.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_UART_DRIVER_CONFIGURATION_H
#define __HAL_UART_DRIVER_CONFIGURATION_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal_uart_build_definition.h"

/* Exported types ------------------------------------------------------------*/
/* Exported defines --------------------------------------------------------*/

 /**
  *  USART baudrate selection
  */
#define	USART_BAUDRATE	((uint32_t)1000000)

 /**
  *  USART selection
  *  The selection shall be either:
  *  BLE_ON_USART1
  *  BLE_ON_USART2
  *  BLE_ON_USART3
  */
#define USART_SELECTED 	BLE_ON_USART2

 /**
  *  USART TX pin selection
  *  The selection shall be either:
  *  For USART1: PA9, PB6
  *  For USART2: PA2, PD5
  *  For USART3: PB10, PC10, PD8
  */
#define USART_TX_PIN_SELECTED	PA2

 /**
  *  USART RX pin selection
  *  The selection shall be either:
  *  For USART1: PA10, PB7
  *  For USART2: PA3, PD6
  *  For USART3: PB11, PC11, PD9
  */
#define USART_RX_PIN_SELECTED	PA3

 /**
  *  USART RTS pin selection
  *  The selection shall be either:
  *  For USART1: PA12
  *  For USART2: PA1, PD4
  *  For USART3: PB14, PD12
  */
#define USART_RTS_PIN_SELECTED	PA1

 /**
  *  USART CTS pin selection
  *  The selection shall be either:
  *  For USART1: PA11
  *  For USART2: PA0, PD3
  *  For USART3: PB13, PD11
  */
#define USART_CTS_PIN_SELECTED	PA0

 /**
  *  USART Wakeup pin selection
  *  It shall be selected which wakeup pin is used to exit from Standby mode
  *  in the PWR_CSR register
  */
#define USART_WAKEUP_PIN_SELECTED	PWR_WakeUpPin_1

 /**
  *  Define a critical section in the Low Power UART driver
  *  The default implementations is masking all interrupts using the PRIMASK bit
  *  When the application is implementing low latency interrupts that would not impact the system (only FW processing),
  *  the critical section may use the basepri register to mask out only interrupt that would impact the system
  *  This would keep the low latency interrupt with higher priority enable
  */
#define USART_ENTER_CRITICAL_SECTION	__disable_irq()	/**< Enter the critical section */
#define USART_EXIT_CRITICAL_SECTION		__enable_irq()	/**< Exit the critical section */


 /**
  * These are the interrupt priority settings of the 3 hardware resources used when transfering data over UART
  * The priority and the order may be changed by the user
  * To achieve best performance, the order should be as follow (from highest priority to lowest)
  * 	1) DMA
  * 	2) UART
  * 	3) CTS
  */
#define	NVIC_UART_DMA_IT_PRIORITY		0
#define	NVIC_UART_UART_IT_PRIORITY		1
#define	NVIC_UART_WCTS_IT_PRIORITY		2

/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

#ifdef __cplusplus
}
#endif

#endif /*__HAL_UART_DRIVER_CONFIGURATION_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
