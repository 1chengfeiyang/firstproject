/**
  ******************************************************************************
  * @file CurrentMeasurements/inc/hw_config.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Hardware configuration header file.
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
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Define the entry to the low power mode */
#define Entry_WFE
//#define Entry_WFI

/* Define the clock settings */
#define HSE_PLL_ON
#define HSE_PLL_ON_72MHz
//#define HSE_PLL_ON_48MHz
//#define HSE_PLL_ON_36MHz
//#define HSE_PLL_ON_24MHz
//#define HSE_PLL_ON_16MHz

//#define HSE_PLL_OFF
//#define HSE_PLL_OFF_8MHz
//#define HSE_PLL_OFF_4MHz
//#define HSE_PLL_OFF_2MHz
//#define HSE_PLL_OFF_1MHz
//#define HSE_PLL_OFF_500KHz
//#define HSE_PLL_OFF_125KHz

//#define HSI_PLL_ON
//#define HSI_PLL_ON_64MHz
//#define HSI_PLL_ON_48MHz
//#define HSI_PLL_ON_36MHz
//#define HSI_PLL_ON_24MHz
//#define HSI_PLL_ON_16MHz

//#define HSI_PLL_OFF
//#define HSI_PLL_OFF_8MHz
//#define HSI_PLL_OFF_4MHz
//#define HSI_PLL_OFF_2MHz
//#define HSI_PLL_OFF_1MHz
//#define HSI_PLL_OFF_500KHz
//#define HSI_PLL_OFF_125KHz

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RCC_Configuration(void);
void All_PeriphClock_Enable(void);
void All_PeriphClock_Disable(void);
void GPIO_Config_ALL_AIN(void);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void RTC_Configuration(void);
#endif /* __HW_CONFIG_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
