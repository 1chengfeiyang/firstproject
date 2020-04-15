/**
  ******************************************************************************
  * @file    ble_app_main.h
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Header file for ble_app_main.c module
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

#ifndef BLE_HRS_MAIN_H__
#define BLE_HRS_MAIN_H__

void ble_app_main_init_Reset(void);

void advertising_start(void);
uint32_t heart_rate_meas_timeout_handler(void);

uint32_t battery_level_update(void);

uint32_t ble_button_process(uint8_t button_id);

uint32_t ble_hrs_evt_process(void);

void user_nvm_bond_delete(void);

void temperature_measurement_send(void);

uint16_t GetConnectionHandle(void);


#endif /**< BLE_HRS_MAIN_H__ */
