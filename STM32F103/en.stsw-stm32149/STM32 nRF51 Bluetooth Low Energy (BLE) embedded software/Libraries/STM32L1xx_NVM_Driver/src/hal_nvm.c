/**
  ******************************************************************************
  * @file    hal_nvm.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   NVM driver
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
#include "stm32l1xx.h"
#include "hal_nvm.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	eNVM_Operation_t	eOperation;
	uint8_t				*SourceAddress;
	uint8_t				*DestAddress;
	uint16_t			Size;
}sNVM_OperationContext_t;

/* Private macros ------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile sNVM_OperationContext_t	aNVM_Operation_Queue[NVM_OPERATION_QUEUE_SIZE];
volatile uint8_t		IndexOfNextFreeSlotInQueue;
volatile uint8_t		IndexOfPendingCommandToExecute;
volatile uint8_t		NumberOfPendingCommand;
uint8_t		*pCurrentPointerToFreeShadowBufferLocation;


/* Private function prototypes -----------------------------------------------*/
static void nvm_operation(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Execute the operation required
  *
  * @note	This API execute the operation (Erase/Write) that is in the queue
  * 		As it always write a 4bytes data, the write operation is done in following steps
  * 		1) Set the read point to the correct 4bytes boundary address
  * 		2) Update the read value with the data to write
  * 		3/ Write the 4 bytes data in the EEPROM
  *
  * @param  None
  *
  * @retval None
  */
static void nvm_operation(void)
{
	uint32_t index;
	uint32_t loop;
	uint8_t data[8];
	uint32_t *pAddressAligned;
	uint8_t	localIndexOfPendingCommandToExecute;

	localIndexOfPendingCommandToExecute = IndexOfPendingCommandToExecute;

	pAddressAligned = (uint32_t*)((uint32_t)(aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].DestAddress) & (uint32_t)0xFFFFFFFC);
	index = ((uint32_t)aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].DestAddress & 0x03);
	*(uint32_t*)&data[0] = *pAddressAligned;

	if(aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].Size >= 4)
	{
		if(aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].eOperation == eNVM_Write)
		{
			*(uint32_t*)&data[index] = *(uint32_t *)aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].SourceAddress;
		}
		else
		{
			*(uint32_t*)&data[index] = 0x00;
		}

		aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].Size = aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].Size - 4 + index;
		aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].DestAddress = aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].DestAddress + 4 - index;
		aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].SourceAddress = aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].SourceAddress + 4 - index;

		*pAddressAligned = *(uint32_t*)&data[0];
	}
	else
	{
		for(loop = 0; loop < aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].Size; loop++)
		{
			if(aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].eOperation == eNVM_Write)
			{
				data[loop] = aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].SourceAddress[loop];
			}
			else
			{
				data[loop] = 0x00;
			}
		}

		aNVM_Operation_Queue[localIndexOfPendingCommandToExecute].Size = 0;

		*pAddressAligned = *(uint32_t*)&data[0];
	}

	return;
}


/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize the NVM driver
  *
  * @note	This API initializes the NVM pointer used in the driver and enables the NVIC interrupt to schedule consecutive write
  *
  * @param  None
  *
  * @retval None
  */
void HAL_NVM_Init(void)
{
	IndexOfNextFreeSlotInQueue = 0;
	IndexOfPendingCommandToExecute = 0;
	NumberOfPendingCommand = 0;

	pCurrentPointerToFreeShadowBufferLocation = (uint8_t *)NVM_BASE_ADDRESS;

	/* Set NVIC priority */
	NVIC_SetPriority(FLASH_IRQn, NVM_IT_PRIORITY);

	/* Enable NVIC interrupt */
	NVIC_EnableIRQ(FLASH_IRQn);

	DATA_EEPROM_Unlock();
	FLASH_ITConfig(FLASH_IT_EOP, ENABLE);
	DATA_EEPROM_Lock();

	return;
}

/**
  * @brief  Interface to the user to reserve a block in the EEPROM
  *
  * @note
  *
  * @param  NVMAddressBlock: Address reserved to the user
  *
  * @param  BlockSize: Size of block reserved
  *
  * @retval None
  */
void HAL_NVM_Register(uint32_t *NVMAddressBlock, uint16_t BlockSize)
{
	NVM_ENTER_CRITICAL_SECTION;
	*NVMAddressBlock = (uint32_t)pCurrentPointerToFreeShadowBufferLocation;
	pCurrentPointerToFreeShadowBufferLocation += BlockSize;
	NVM_EXIT_CRITICAL_SECTION;

	return;
}

/**
  * @brief  Interface to the user to request a Write or Erase operation in the NVM
  *
  * @note	When a write operation is required, the source data shall be kept available as long as the
  * 		NVM driver is running operation (Write or Erase) with the NVM
  *
  * @param  eOperation: Write or Erase operation
  *
  * @param  UserAddress: Source of data to be written (Not used when Erase operation is requested)
  *
  * @param  NVMAddress: NVM destination address
  *
  * @param  Size: Size of data to write/erase
  *
  * @retval None
  */
void HAL_NVM_Operation(eNVM_Operation_t eOperation, uint8_t *UserAddress, uint8_t *NVMAddress, uint16_t Size)
{
	NVM_ENTER_CRITICAL_SECTION;

	aNVM_Operation_Queue[IndexOfNextFreeSlotInQueue].eOperation = eOperation;
	aNVM_Operation_Queue[IndexOfNextFreeSlotInQueue].SourceAddress = UserAddress;
	aNVM_Operation_Queue[IndexOfNextFreeSlotInQueue].DestAddress = NVMAddress;
	aNVM_Operation_Queue[IndexOfNextFreeSlotInQueue].Size = Size;

	IndexOfNextFreeSlotInQueue++;
	IndexOfNextFreeSlotInQueue = (IndexOfNextFreeSlotInQueue % NVM_OPERATION_QUEUE_SIZE);
	NumberOfPendingCommand++;

	if(NumberOfPendingCommand == 1)
	{
		NVM_EXIT_CRITICAL_SECTION;

		HAL_NVM_HSclkRequest(eNVM_HSclkEnable);

		DATA_EEPROM_Unlock();

		nvm_operation();
	}
	else
	{
		NVM_EXIT_CRITICAL_SECTION;
	}

	return;
}

/**
  * @brief  Interface to the user to read data from the NVM
  *
  * @note  The data read is done asynchronously to the write/erase operation. There is no check of operation in the queue
  * 	   The user shall make sure to not read a data that may be requested to be written in the queue
  *
  * @param  UserAddress: Destination address where to store the data read
  *
  * @param  NVMAddress: Source address in NVM to read
  *
  * @param  Size: Size of data to read
  *
  * @retval None
  */
void HAL_NVM_Read(uint8_t *UserAddress, uint8_t *NVMAddress, uint16_t Size)
{
	uint32_t index;

	for(index = 0 ; index < (uint32_t)Size ; index++)
	{
		*(UserAddress + index) = *(NVMAddress + index);
	}

	return;
}

/**
  * @brief  Flash interrupt handler
  *
  * @note  The driver is writing pending data from the queue in the NVM when it is notified the previous operation is completed
  * 	   In the meantime, the application may execute code when the NVM is in a different bank than the user code.
  *
  * @param  None
  *
  * @retval None
  */
void FLASH_IRQHandler(void)
{
	FLASH_ClearFlag(FLASH_FLAG_EOP);

	if(aNVM_Operation_Queue[IndexOfPendingCommandToExecute].Size != 0)
	{
		nvm_operation();
	}
	else
	{
		NVM_ENTER_CRITICAL_SECTION;

		NumberOfPendingCommand--;

		IndexOfPendingCommandToExecute++;
		IndexOfPendingCommandToExecute = (IndexOfPendingCommandToExecute % NVM_OPERATION_QUEUE_SIZE);

		if(NumberOfPendingCommand != 0)
		{
			NVM_EXIT_CRITICAL_SECTION;

			nvm_operation();
		}
		else
		{
			DATA_EEPROM_Lock();

			NVM_EXIT_CRITICAL_SECTION;

			HAL_NVM_HSclkRequest(eNVM_HSclkDisable);
		}
}

	return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
