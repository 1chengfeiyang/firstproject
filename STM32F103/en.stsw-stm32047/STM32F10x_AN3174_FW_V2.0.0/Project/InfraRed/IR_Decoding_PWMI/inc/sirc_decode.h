/**
  ******************************************************************************
  * @file    InfraRed/IR_Decoding_PWMI/inc/sirc_decode.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   This file contains all the functions prototypes for the SIRC 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIRC_DECODE_H
#define __SIRC_DECODE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup IR_Decoding_PWMI
  * @{
  */

/** @addtogroup SIRC
  * @{
  */

/** @defgroup SIRC_Exported_Types
  * @{
  */
typedef enum { NO = 0, YES = !NO} StatusYesOrNo;
/** 
  * @brief  SIRC frame structure  
  */
typedef struct
{  
  __IO uint8_t Command;   /*!< Command field */
  __IO uint8_t Address ;  /*!< Address field */
} IR_Frame_TypeDef;
/** 
  * @brief  SIRC packet structure  
  */
typedef struct
{
  uint8_t count;  /*!< Bit count */
  uint8_t status; /*!< Status */
  uint32_t data;  /*!< Data */
} tIR_packet;
/**
  * @}
  */

/** @defgroup SIRC_Exported_Constants
  * @{
  */
#define USE_LCD
#define IR_MESSAGE                    " SIRC InfraRed Demo  "

#define IR_TIME_OUT_US                4000 

#define IR_STATUS_HEADER              1 << 1
#define IR_STATUS_RX                  1 << 0
#define INITIAL_STATUS                IR_STATUS_HEADER 

#define IR_BIT_ERROR                  0xFF
#define IR_HEADER_ERROR               0xFF
#define IR_HEADER_OK                  0x00

#define IR_BITS_COUNT                 11
#define IR_TOTAL_BITS_COUNT           11

#define IR_ONTIME_MIN_US              (600 - 50)
#define IR_ONTIME_MAX_US              (1200 + 50)

#define IR_HEADER_LOW_MIN_US          (2400 - 100)
#define IR_HEADER_LOW_MAX_US          (2400 + 100)
#define IR_HEADER_WHOLE_MIN_US        (2400 + 600 - 50)
#define IR_HEADER_WHOLE_MAX_US        (2400 + 600 + 50)

#define IR_VALUE_STEP_US              600
#define IR_VALUE_MARGIN_US            100
#define IR_VALUE_00_US                1200

/**
  * @}
  */
  
/** @defgroup SIRC_Exported_Macros
  * @{
  */
/**
  * @}
  */
/** @defgroup SIRC_Exported_Functions
  * @{
  */


#ifdef __cplusplus
}
#endif

#endif /* __SIRC_DECODE_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
