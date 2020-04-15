/**
  ******************************************************************************
  * @file USBI2CBridge/src/dfu_mal.c
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date   09/15/2010
  * @brief  Generic media access Layer
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
#include "dfu_mal.h"
#include "i2c_if.h"
#include "usb_lib.h"
#include "usb_type.h"
#include "usb_desc.h"
#include "platform_config.h"

/** @addtogroup USBI2CBridge
  * @{
  */




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t (*pMAL_Init) (void);
uint16_t (*pMAL_Erase) (uint32_t SectorAddress);
uint16_t (*pMAL_Write) (uint32_t SectorAddress, uint32_t DataLength);
uint8_t  *(*pMAL_Read)  (uint32_t SectorAddress, uint32_t DataLength);
uint8_t  MAL_Buffer[wTransferSize]; /* RAM Buffer for Downloaded Data */
extern ONE_DESCRIPTOR DFU_String_Descriptor[];


static const uint16_t  TimingTable[5][2] =
    {

        { 100  , 130 }, /* I2C Flash */
        { 1000 ,  25 }, /* NOR Flash M29W128F */
        {100   ,  130 }, /* SPI Flash */
        {1000 ,  25 }, /* NOR Flash M29W128G */
        { 1000 ,  45 }  /* NOR Flash S29GL128 */
    };


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Initializes the Media on the STM32
  * @param  None
  * @retval : None
  */
uint16_t MAL_Init(void)
{
   
    I2C_If_Init();

    return MAL_OK;
}



/**
  * @brief  Erase sector
  * @param  None
  * @retval : None
  */
uint16_t MAL_Erase(uint32_t SectorAddress)
{

    switch (SectorAddress & MAL_MASK)
    {
    case I2C_FLASH_BASE:
        pMAL_Erase = I2C_If_Erase;
        break;


    default:
        return MAL_FAIL;
    }
    return pMAL_Erase(SectorAddress);
}



/**
  * @brief  Write sectors
  * @param  None
  * @retval : None
  */
uint16_t MAL_Write (uint32_t SectorAddress, uint32_t DataLength)
{

    switch (SectorAddress & MAL_MASK)
    {
    case I2C_FLASH_BASE:
        pMAL_Write = I2C_If_Write;
        break;


    default:
        return MAL_FAIL;
    }
    return pMAL_Write(SectorAddress, DataLength);
}



/**
  * @brief  Read sectors
  * @param  None
  * @retval : Buffer pointer
  */
uint8_t *MAL_Read (uint32_t SectorAddress, uint32_t DataLength)
{

    switch (SectorAddress & MAL_MASK)
    {
    case I2C_FLASH_BASE:
        pMAL_Read = I2C_If_Read;
        break;


    default:
        return 0;
    }
    return pMAL_Read (SectorAddress, DataLength);
}



/**
  * @brief  Get status
  * @param  None
  * @retval : Buffer pointer
  */
uint16_t MAL_GetStatus(uint32_t SectorAddress , uint8_t Cmd, uint8_t *buffer)
{
    uint8_t x = (SectorAddress  >> 26) & 0x03 ; /* 0x000000000 --> 0 */

    uint8_t y = Cmd & 0x01;

    SET_POLLING_TIMING(TimingTable[x][y]);  /* x: Erase/Write Timing */
    /* y: Media              */
    return MAL_OK;
    
    
}

/**
  * @}
  */


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
