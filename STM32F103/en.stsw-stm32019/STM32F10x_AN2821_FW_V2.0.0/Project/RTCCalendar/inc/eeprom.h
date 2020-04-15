/**
  ******************************************************************************
  * @file RTC/inc/eeprom.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains  the functions prototypes for the
  *         EEPROM emulation firmware for Time Stamping in Clock/Calendar
  *         application
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
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Page size */
#define PAGE_SIZE  (uint16_t)0x400  /* 1KByte */

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS  (uint32_t)0x0801F000

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS  (uint32_t)(EEPROM_START_ADDRESS + 0x000)
#define PAGE0_END_ADDRESS   (uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1))

/* Used Flash pages for EEPROM emulation */
#define PAGE0    (uint16_t)0x0000

/* No valid page define */
#define NO_VALID_PAGE    (uint16_t)0x00AB

/* Page full define */
#define PAGE_FULL    (uint8_t)0x80

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
FLASH_Status EE_Format(void);
uint16_t EE_WriteVariable(uint16_t);
uint16_t EE_ReadValue(void);

#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
