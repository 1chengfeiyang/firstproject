/**
  ******************************************************************************
  * @file    hal_uart_interfaces.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_UART_INTERFACES_H
#define __HAL_UART_INTERFACES_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

 /**
   * The UART driver manages the Rx and Tx path separately
   * These are the identifier the UART driver uses to request or release the HS clock
   * of the related path
   */
 typedef enum{
	 eUART_Tx,							/**<  The UART transmitter */
	 eUART_Rx							/**<  The UART receiver */
}eHAL_UART_HSclkRequester_t;

/**
  * These are the requests from the UART driver to the low power manager regarding the availability of the HS clock
  */
typedef enum{
	 eUART_HSclkDisable,				/**<  The HS clock may be disabled */
	 eUART_HSclkEnable					/**<  The HS clock shall be kept enabled */
}eHAL_UART_HSclkMode_t;

/**
  *	These are the notification regarding transfer with the BLE connectivity device
  *	The source is the STM32 and the destination is the BLE connectivity device
  */
  typedef enum{
	 eUART_TxDone,						/**<  The transmission has been completed */
	 eUART_RxDone						/**<  A packet has been received */
 }eHAL_UART_PhyDriverNotification_t;

 /**
   * Informations to be reported to the BLE module in the UART interrupt handler
   */
 typedef struct{
	 eHAL_UART_PhyDriverNotification_t	ePhyDriverNotification;	/**<  Event to be reported to the BLE module */
 	 uint32_t							uwParam1;				/**<  Reserved for future use */
 	 uint32_t							uwParam2;				/**<  Reserved for future use */
 }sHAL_UART_PhyDriverEvent_t;

 /**
   * Type of the UART interrupt handler
   */
 typedef void (*pf_HAL_UART_PhyDriverEvent_Handler_t)(sHAL_UART_PhyDriverEvent_t pHAL_UART_PhyDriverEvent);

/* Exported defines --------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
 void HAL_UART_uart_open(pf_HAL_UART_PhyDriverEvent_Handler_t pf_PhyDriverEvent_Handler);
 void HAL_UART_send_data(uint8_t * pbuffer, uint16_t uhlength);
 void HAL_UART_EnterStandByMode(void);
 void HAL_UART_ExitStandByMode(void);
 void HAL_UART_receive_data(uint8_t * pbuffer);
 void HAL_UART_HSclkRequest(eHAL_UART_HSclkRequester_t eHSclkRequester, eHAL_UART_HSclkMode_t eHSclkMode);
 /**
   * @brief	This is the CTS interrupt handler required for the Low Power Protocol implemented over UART
   * @note  It shall be called in the EXTI15_10_IRQHandler by the application ONLY when either USART1 or USART3 is selected
   * 		When USART2 is selected, the UART driver is taking care of it.
   *
   * @param  None
   * @note   None
   * @retval None
   */
 void HAL_UART_wcts_handler(void);

 /**
   * @brief	This is the handler called by the driver when a full message is received from the nRF51 device
   * @note  The purpose of this function is to handle the received message out of the UART interrupt context.
   * 		The application may either:
   * 			a) send a signal to a process in case an OS is implemented
   * 			b) set a flag to be polled in the idle context
   * 			c) keep running in the UART context
   * 		The event handler registered by the BLE module shall be called in the new context.
   *
   * @param1 The event handler registered by the BLE module to process the message received from the nRF51
   * @note   None
   * @param2 The parameter to be passed to the event handler registered by the BLE module (param1)
   * @note   None
   * @retval None
   */
 void HAL_UART_Msg_Handler(pf_HAL_UART_PhyDriverEvent_Handler_t pf_PhyDriverEvent_Handler, sHAL_UART_PhyDriverEvent_t pPhyDriverEvent);


#ifdef __cplusplus
}
#endif

#endif /*__HAL_UART_INTERFACES_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
