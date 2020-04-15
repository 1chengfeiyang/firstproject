/**
  ******************************************************************************
  * @file USBI2CBridge/inc/i2c_flash.h
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date  09/15/2010
  * @brief  Header for i2c_flash.c file.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_FLASH_H
#define __I2C_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void I2C_FLASH_Init(void);
void i2c_send_opcode(uint8_t opcode,uint8_t I2C1_Buffer_Tx[]);
void i2c_send_add(uint32_t ADD_FLASH,uint8_t I2C1_Buffer_Tx[]);
void i2c_send_byte_number(uint16_t byte_number,uint8_t I2C1_Buffer_Tx[]);
void I2C_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void I2C_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void i2c_delay (uint32_t delay);
#endif /* __I2C_FLASH_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
