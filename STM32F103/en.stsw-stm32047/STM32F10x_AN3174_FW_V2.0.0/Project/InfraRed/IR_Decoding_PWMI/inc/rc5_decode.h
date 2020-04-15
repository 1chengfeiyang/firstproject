/**
  ******************************************************************************
  * @file    InfraRed/IR_Decoding_PWMI/inc/rc5_decode.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   This file contains all the functions prototypes for the RC5 
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
#ifndef __RC5_DECODE_H
#define __RC5_DECODE_H

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

/** @addtogroup RC5
  * @{
  */

/** @defgroup RC5_Exported_Types
  * @{
  */
   
typedef enum { NO = 0, YES = !NO} StatusYesOrNo;

/** 
  * @brief  RC5 frame structure  
  */
typedef struct
{   
  __IO uint8_t FieldBit;   /*!< Field bit */   
  __IO uint8_t ToggleBit;  /*!< Toggle bit field */
  __IO uint8_t Address;    /*!< Address field */ 
  __IO uint8_t Command;    /*!< Command field */ 
  
} IR_Frame_TypeDef;
/** 
  * @brief  RC5 packet structure
  */
typedef struct
{
 __IO uint16_t data;     /*!< RC5 data */ 
 __IO uint8_t  status;   /*!< RC5 status */
 __IO uint8_t  lastBit;  /*!< RC5 last bit */
 __IO uint8_t  bitCount; /*!< RC5 bit count */
} tRC5_packet;

enum RC5_lastBitType
{
  RC5_ZER,
  RC5_ONE,
  RC5_NAN,
  RC5_INV
};

typedef enum RC5_lastBitType tRC5_lastBitType;

/**
  * @}
  */

/** @defgroup RC5_Exported_Constants
  * @{
  */

#define USE_LCD

#define IR_TIM                               TIM3                     /*!< Timer used for IR decoding */
#define TIM_PRESCALER                        23                       /* !< TIM prescaler */
#define IR_TIM_CLK                           RCC_APB1Periph_TIM3      /*!< Clock of the used timer */
#define IR_TIM_IRQn                          TIM3_IRQn                /*!< IR TIM IRQ */
#define IR_TIM_Channel                       TIM_Channel_1            /*!< IR TIM Channel */

#define IR_GPIO_PORT                         GPIOC                    /*!< Port which IR output is connected */
#define IR_GPIO_PORT_CLK                     RCC_APB2Periph_GPIOC     /*!< IR pin GPIO Clock Port */
#define IR_GPIO_PIN                          GPIO_Pin_6               /*!< Pin which IR is connected */

#define RC5_1T_TIME                          0x00
#define RC5_2T_TIME                          0x01
#define RC5_WRONG_TIME                       0xFF
#define RC5_TIME_OUT_US                      3600
#define RC5_T_US                             900     /*!< Half bit period */
#define RC5_T_TOLERANCE_US                   300     /*!< Tolerance time */
#define RC5_NUMBER_OF_VALID_PULSE_LENGTH     2
#define RC5_PACKET_BIT_COUNT                 13      /*!< Total bits */

/* Packet struct for reception*/
#define RC5_PACKET_STATUS_EMPTY              1 << 0
/**
  * @}
  */

/** @defgroup RC5_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup RC5_Exported_Functions
  * @{
  */
void IR_RC5_DeInit(void);
void IR_RC5_Init(void);
void IR_RC5_Decode(IR_Frame_TypeDef *rc5_frame);
void IR_RC5_ResetPacket(void);

#ifdef __cplusplus
}
#endif

#endif /* __RC5_DECODE_H */
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
