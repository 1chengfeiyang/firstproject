/**
  ******************************************************************************
  * @file IAPOverI2C/inc/main.h
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief  Header file for main.c
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "commands.h"
#include "config.h"





/* Exported types ------------------------------------------------------------*/
typedef enum {NOEVENT = 0, EVENT_OPCOD_NOTYET_READ = 1,EVENT_OPCOD_READ =2} EventStatus;
/* Exported constants --------------------------------------------------------*/
#define I2C_SLAVE_ADDRESS7     0x30
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void iap_i2c(void);
#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
