/**
  ******************************************************************************
  * @file    ble_uart.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Wrapper between BLE UART interface and low power potocol UART driver
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
  * @note	This file contains the mapping of the BLE module UART interface to the low power protocol UART driver
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "nrf_error.h"
#include "app_uart_stream.h"
#include "hal_uart_interfaces.h"
#include "hal_timer.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

uint32_t app_uart_stream_open(const app_uart_stream_comm_params_t * p_comm_params)
 {
	 return NRF_SUCCESS;
 }

uint32_t app_uart_stream_evt_handler_reg(app_uart_stream_event_handler_t event_handler)
{
	 HAL_UART_uart_open((pf_HAL_UART_PhyDriverEvent_Handler_t)event_handler);

	 return NRF_SUCCESS;
}

 uint32_t app_uart_stream_close(void)
 {
	 return NRF_SUCCESS;
 }

 uint32_t app_uart_stream_write(const uint8_t * p_buffer, uint16_t length)
 {
	 HAL_UART_send_data((uint8_t*)p_buffer, length);

	 return NRF_SUCCESS;
 }

 uint32_t app_uart_stream_rx_buffer_set(uint8_t * p_buffer, uint16_t num_of_bytes, bool header)
 {
	 HAL_UART_receive_data(p_buffer);

	 return NRF_SUCCESS;
 }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
