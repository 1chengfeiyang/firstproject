/**
  ******************************************************************************
  * @file IAPOverI2C/inc/commands.h
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief  Header file for commands.c
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
#ifndef __COMMANDS_H
#define __COMMANDS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "config.h"



/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t Read_Opcode(void);
uint32_t Read_Add(void);
uint16_t Read_Byte_Page_Number(void);
void Read_Memory_Command(void);
void Write_Memory_Command(void);
void Erase_Page(uint32_t Add_Flash,uint16_t page_number);
void Erase_Page_Command(void);
void User_Space_Memory_Erase_Command(void);

#endif /* __COMMANDS_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
