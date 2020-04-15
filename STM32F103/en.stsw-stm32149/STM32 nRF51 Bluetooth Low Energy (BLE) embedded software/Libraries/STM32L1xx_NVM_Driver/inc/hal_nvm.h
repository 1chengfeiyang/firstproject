/**
  ******************************************************************************
  * @file    hal_nvm.h
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Header for hal_timer.c module
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
#ifndef __HAL_NVM_H
#define __HAL_NVM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
 typedef enum
 {
 	eNVM_Write,
 	eNVM_Clear
 } eNVM_Operation_t;

 typedef enum{
 	 eNVM_HSclkDisable,
 	 eNVM_HSclkEnable
 }eHAL_NVM_HSclkMode_t;


/* Exported constants --------------------------------------------------------*/
#define	NVM_OPERATION_QUEUE_SIZE		30							/* Queue of Write command */
#define NVM_BASE_ADDRESS				0x08082460					/* Bank2 - After the memory (1120 bytes) reserved for BLE */
#define NVM_IT_PRIORITY					3

 /**
  *  Define a critical section in the NVM driver
  *  The NVM Driver does not support the API to be nested
  *  The  Application shall either:
  *  	a) Ensure this will never happen (When using an OS, there could be one NVM process)
  *  	b) Define the critical section
  *  The default implementations is masking all interrupts using the PRIMASK bit
  *  When the application is implementing low latency interrupts that would not support to be masked out,
  *  the critical section may use the basepri register to mask out only interrupt that have lower priority than those interrupts
  *  In that case, the application shall not call the NVM API interface within these interrupt handlers.
  */
#define NVM_ENTER_CRITICAL_SECTION	__disable_irq()	/**< Enter the critical section */
#define NVM_EXIT_CRITICAL_SECTION	__enable_irq()	/**< Exit the critical section */


/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
 void HAL_NVM_Init(void);
 void HAL_NVM_Register(uint32_t *NVMAddressBlock, uint16_t BlockSize);
 void HAL_NVM_Read(uint8_t *UserAddress, uint8_t *NVMAddress, uint16_t Size);
 void HAL_NVM_Operation(eNVM_Operation_t eOperation, uint8_t *UserAddress, uint8_t *NVMAddress, uint16_t Size);
 void HAL_NVM_HSclkRequest(eHAL_NVM_HSclkMode_t eHSclkMode);

#ifdef __cplusplus
}
#endif

#endif /*__HAL_NVM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
