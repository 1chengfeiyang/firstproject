/**
  ******************************************************************************
  * @file    stm32l1xx_conf.h
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Library configuration file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L1xx_CONF_H
#define __STM32L1xx_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_rtc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_syscfg.h"
#include "stm32l1xx_dma.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_dbgmcu.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported defines -----------------------------------------------------------*/

/* JTAG support for debugging */
#define	JTAG_SUPPORTED	0

/* Exported macros -----------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports
  *         the name of the source file and the source line number of the call
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
 #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

 /* Compiler specific definition ------------------------------------------------------- */
 /*
  * Make sure the function (or variable) marked is part of the object file
  */
#if defined ( __CC_ARM )
#define USED __attribute__((used))
#elif defined ( __ICCARM__ )
#define USED __root
#elif defined ( __GNUC__ )
#define USED __attribute__((used))
#endif


#ifdef __cplusplus
}
#endif

#endif /*__STM32L1xx_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
