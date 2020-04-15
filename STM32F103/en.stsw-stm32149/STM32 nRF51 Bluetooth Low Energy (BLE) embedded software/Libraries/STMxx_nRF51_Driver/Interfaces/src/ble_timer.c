/**
  ******************************************************************************
  * @file    ble_timer.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Wrapper between the BLE timer interface and the Timer server
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
  * @note	This file contains the mapping of the BLE module TIMER interface to the TIMER server
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_timer.h"
#include "hal_timer.h"
#include "nrf_error.h"

/* Public functions ----------------------------------------------------------*/

uint32_t ble_timer_create(ble_timer_id_t *            p_timer_id,
                          ble_timer_mode_t            mode,
                          ble_timer_timeout_handler_t timeout_handler)
{
    HAL_TIMER_Create(	eTimerModuleID_BLE, (uint8_t *)p_timer_id, (eHAL_TIMER_TimerMode_t)mode, (pf_HAL_TIMER_TimerCallBack_t)timeout_handler);
    return NRF_SUCCESS;
}

uint32_t ble_timer_start(ble_timer_id_t timer_id, uint32_t timeout_ms, void * p_context)
{
    HAL_TIMER_Start(timer_id, timeout_ms);		
    return NRF_SUCCESS;               
}

uint32_t ble_timer_stop(ble_timer_id_t timer_id)
{
    HAL_TIMER_Delete(timer_id);
    return NRF_SUCCESS;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
