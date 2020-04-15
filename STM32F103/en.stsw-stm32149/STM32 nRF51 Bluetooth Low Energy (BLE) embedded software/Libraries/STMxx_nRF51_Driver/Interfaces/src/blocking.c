/**
  ******************************************************************************
  * @file    blocking.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Pausing BLE between Command and Response
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

/* Includes ------------------------------------------------------------------*/
#include "blocking.h"

#include <stdint.h>
#include <stdbool.h>

#include "nrf_error.h"
#include "ble.h"
#include "ble_encode_access.h"
#include "app_error.h"
#include "stm32l1xx_conf.h"
#include "main.h"

#if (APP_HRS == 1)
#include "ble_app_main.h"
#endif

/* Private variables ---------------------------------------------------------*/
static volatile bool        m_cmd_rsp_event_rcvd = false;
static uint32_t             m_cmd_result_code;

/* Private function prototypes -----------------------------------------------*/
static void internal_response_handler(uint32_t result_code);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Handler to notifying a response packet has been received
  *
  * @note	This API is called when a response packet has been received
  *
  * @param  result_code: Error code while decoding the response packet
  *
  * @retval None
  */
static void internal_response_handler(uint32_t result_code)
{
    m_cmd_result_code = result_code;
    m_cmd_rsp_event_rcvd = true;

    /**
     * This is added to solve race condition when this event occurs between the check of the variable m_cmd_rsp_event_rcvd
     * and the time we enter low power mode
     * This will prevent entering low power mode and will force the re-evaluation of the variable m_cmd_rsp_event_rcvd
     */
    TaskExecutionRequest(eMAIN_Main_SD_Command_Resp);

    return;
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialization of the blocking mechanism to prevent sending two SD command back to back
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
uint32_t blocking_init(void)
{
    uint32_t err_code;
    err_code = ble_encode_cmd_resp_handler_reg(internal_response_handler);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}

/**
  * @brief  Interface to pause the BLE module until a response packet is received
  *
  * @note	This API is called after each SD command sent to the nRF device.
  *
  * @param  None
  *
  * @retval None
  */
uint32_t blocking_resp_wait(void)
{
    while (!m_cmd_rsp_event_rcvd)
    {
    	Background(SD_COMMAND_NOT_ALLOWED);
    }
    
    m_cmd_rsp_event_rcvd = false;
    TaskExecuted(eMAIN_Main_SD_Command_Resp);
    
    return m_cmd_result_code;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
