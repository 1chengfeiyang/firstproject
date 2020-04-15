/**
  ******************************************************************************
  * @file USBI2CBridge/src/i2c_if.c
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief  specific media access Layer for I2C
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */


/* Includes ------------------------------------------------------------------*/
#include "i2c_flash.h"
#include "i2c_if.h"
#include "dfu_mal.h"
#include "i2c_flash.h"



/** @addtogroup USBI2CBridge
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the Media on the STM32
  * @param  None
  * @retval : None
  */
uint16_t I2C_If_Init(void)
{
    I2C_FLASH_Init();
    return MAL_OK;
}



/**
  * @brief  No operation (just to keep compatibility with the original DFU)
  * @param  None
  * @retval : None
  */
uint16_t I2C_If_Erase(uint32_t SectorAddress)
{
    return MAL_OK;
}



/**
  * @brief  Write sectors
  * @param  None
  * @retval : None
  */
uint16_t I2C_If_Write(uint32_t SectorAddress, uint32_t DataLength)
{
    uint32_t idx;

   if  (DataLength < 0x400) 
    {
        for ( idx = DataLength; idx < 0x400 ; idx++)
        {
            MAL_Buffer[idx] = 0xFF;
        }

    }

    I2C_FLASH_PageWrite(&MAL_Buffer[0], SectorAddress, 0x400);

    return MAL_OK;
}



/**
  * @brief  Read sectors
  * @param  None
  * @retval : buffer address pointer
  */
uint8_t *I2C_If_Read(uint32_t SectorAddress, uint32_t DataLength)
{
    I2C_FLASH_BufferRead(MAL_Buffer, SectorAddress, (uint16_t)DataLength);

    return MAL_Buffer;
}


/**
  * @}
  */


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
