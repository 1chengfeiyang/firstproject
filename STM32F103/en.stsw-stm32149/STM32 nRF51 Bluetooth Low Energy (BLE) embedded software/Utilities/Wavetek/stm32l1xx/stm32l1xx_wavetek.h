/**
  ******************************************************************************
  * @file    stm32l1xx_wavetek.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-August-2014
  * @brief   This file contains all the functions prototypes for the
  *          stm32l1xx_wavetek.c driver.
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
#ifndef __STM32L1XX_WAVETEK_H
#define __STM32L1XX_WAVETEK_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

 /* Exported types ------------------------------------------------------------*/
  typedef enum{
 	 Wavetek_LED1_Id = 0,
 	 Wavetek_LED2_Id = 1,
 	 Wavetek_LED3_Id = 2
 }eWavetek_Led_TypeDef_t;

 /* Exported functions ------------------------------------------------------- */
 void led_init(void);
 void button_init(void);
 void wavetek_led_on(eWavetek_Led_TypeDef_t Led_id);
 void wavetek_led_off(eWavetek_Led_TypeDef_t Led_id);
   

#ifdef __cplusplus
}
#endif

#endif /* __STM32L1XX_WAVETEK_H */

 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
