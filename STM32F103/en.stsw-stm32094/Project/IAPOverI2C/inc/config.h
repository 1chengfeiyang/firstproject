/**
  ******************************************************************************
  * @file IAPOverI2C/inc/config.h
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief  Header file for config.c
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
#ifndef __CONFIG_H
#define __CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
typedef  void (*pFunction)(void);
/* Exported constants --------------------------------------------------------*/


/*User Start Adrress @ page 4 for Medium  density and 2 for  High density*/
#define USER_START_ADDRESS  ((uint32_t)0x08001000)

#define GPIO_PIN_SCL                       GPIO_Pin_6
#define GPIO_PIN_SDA                       GPIO_Pin_7

/* Define the STM32F10x hardware depending on the used evaluation board */
#if defined USE_STM3210B_EVAL || USE_STM32100B_EVAL
#define GPIO_KEY_BUTTON                   GPIOB
#define RCC_APB2Periph_GPIO_KEY_BUTTON    RCC_APB2Periph_GPIOB
#define GPIO_PIN_KEY_BUTTON               GPIO_Pin_9
#define PAGE_SIZE                         (0x400)
#define FLASH_SIZE                        (0x20000) /* 128 K */
#define TOTAL_PAGE_NUMBER    		    ((uint16_t)0x80)/* 128 PAGES */
#define TOTAL_USER_PAGE_NUMBER    	    ((uint16_t)0x7C)/* 124 PAGES */

#elif defined USE_STM3210C_EVAL 
#define GPIO_KEY_BUTTON                   GPIOB
#define RCC_APB2Periph_GPIO_KEY_BUTTON    RCC_APB2Periph_GPIOB
#define GPIO_PIN_KEY_BUTTON               GPIO_Pin_9
#define PAGE_SIZE                         (0x800)
#define FLASH_SIZE                        (0x40000) /* 256 K */
#define TOTAL_PAGE_NUMBER    		    ((uint16_t)0x80)/* 128 PAGES */
#define TOTAL_USER_PAGE_NUMBER    	    ((uint16_t)0x7E)/* 126 PAGES */

#elif defined USE_STM3210E_EVAL 
#if defined STM32F10X_HD
#define GPIO_KEY_BUTTON                   GPIOG
#define RCC_APB2Periph_GPIO_KEY_BUTTON    RCC_APB2Periph_GPIOG
#define GPIO_PIN_KEY_BUTTON               GPIO_Pin_8
#define PAGE_SIZE                         (0x800)
#define FLASH_SIZE                        (0x80000) /* 512 K */
#define TOTAL_PAGE_NUMBER    		    ((uint16_t)0x100)/* 256 PAGES */
#define TOTAL_USER_PAGE_NUMBER    	    ((uint16_t)0xFE) /* 254 PAGES */

#elif  defined STM32F10X_XL
#define GPIO_KEY_BUTTON                   GPIOG
#define RCC_APB2Periph_GPIO_KEY_BUTTON    RCC_APB2Periph_GPIOG
#define GPIO_PIN_KEY_BUTTON               GPIO_Pin_8
#define PAGE_SIZE                         (0x800)
#define FLASH_SIZE                        (0x1000000) /* 1M */
#define TOTAL_PAGE_NUMBER    		  ((uint16_t)0x200)/* 512 PAGES */
#define TOTAL_USER_PAGE_NUMBER    	  ((uint16_t)0x1FE) /* 510 PAGES */

#endif 
#endif


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
uint8_t Push_Button_Read (void);

#endif /* __CONFIG_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
