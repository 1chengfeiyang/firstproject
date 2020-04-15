/**
  ******************************************************************************
  * @file    ble_nvm.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Wrapper between the BLE NVM interface and the NVM driver
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
  * @note	This file contains the mapping of the BLE module NVM interface to the NVM driver
  */

/* Includes ------------------------------------------------------------------*/
#include "pstorage.h"
#include "app_error.h"
#include "hal_nvm.h"

/* Private defines -----------------------------------------------------------*/

/**
 * Base address to store the BLE parameters
 * Bank2 is selected as the code is running in bank1
 * This allows RWW (Read While Write) feature to improve performance of the system
 */
#define BLE_NVM_BASE_ADDRESS		0x08082000

/* Private variables ---------------------------------------------------------*/
static uint16_t aBlockSizeList[3];				/**< Only 3 BLE modules Id supported */
static uint8_t	ModuleId;
static uint8_t *pCurrentPointerToNVMLocation;

/* Public functions ----------------------------------------------------------*/

uint32_t pstorage_init(void)
{
	ModuleId = 0;
	pCurrentPointerToNVMLocation = (uint8_t *)BLE_NVM_BASE_ADDRESS;
	HAL_NVM_Init();

    return NRF_SUCCESS;
}

uint32_t pstorage_register(pstorage_module_param_t * p_module_param,
                           pstorage_handle_t       * p_block_id)
{
	p_block_id->block_id = (uint32_t)pCurrentPointerToNVMLocation;
	(p_block_id->module_id) = ModuleId;
	 /*
	  * store the size of the allocated block to be used in the API pstorage_block_identifier_get()
	  */
	aBlockSizeList[ModuleId] = p_module_param->block_size;

	pCurrentPointerToNVMLocation = pCurrentPointerToNVMLocation + (p_module_param->block_size)*(p_module_param->block_count);
	ModuleId++;			/**< increment the module Id for the next registration */

    return NRF_SUCCESS;
}

uint32_t pstorage_block_identifier_get(pstorage_handle_t * p_base_id,
                                       pstorage_size_t     block_num,
                                       pstorage_handle_t * p_block_id)
{
    
	p_block_id->block_id = (p_base_id->block_id) + (block_num * aBlockSizeList[p_base_id->module_id]);

    return NRF_SUCCESS;
}

uint32_t pstorage_store(pstorage_handle_t * p_dest,
                        uint8_t           * p_src,
                        pstorage_size_t     size,
                        pstorage_size_t     offset)
{
	HAL_NVM_Operation(eNVM_Write, p_src, (uint8_t *)(p_dest->block_id) + offset, size);

    return NRF_SUCCESS;
}

uint32_t pstorage_load(uint8_t *           p_dest,
                       pstorage_handle_t * p_src,
                       pstorage_size_t     size,
                       pstorage_size_t     offset)
{
	HAL_NVM_Read(p_dest, (uint8_t *)(p_src->block_id) + offset, size);

    return NRF_SUCCESS;
}

uint32_t pstorage_clear(pstorage_handle_t * p_dest, pstorage_size_t size)
{
	HAL_NVM_Operation(eNVM_Clear, 0, (uint8_t *)(p_dest->block_id), aBlockSizeList[p_dest->module_id] * size);

    return NRF_SUCCESS;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
