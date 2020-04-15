/**
  ******************************************************************************
  * @file    hal_timer.h
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Header for hal_timer.c module
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
#ifndef __HAL_TIMER_H
#define __HAL_TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
 typedef enum
 {
     eTimerMode_SingleShot,
     eTimerMode_Repeated
 } eHAL_TIMER_TimerMode_t;

 typedef enum
 {
     eTimerModuleID_BLE,
     eTimerModuleID_Interrupt,
     eTimerModuleID_HeartRate,
     eTimerModuleID_Battery
 } eHAL_TIMER_ModuleID_t;


 typedef void (*pf_HAL_TIMER_TimerCallBack_t)(void);

/* Exported constants --------------------------------------------------------*/
#define	MAX_NBR_CONCURRENT_TIMER	8
#define	NVIC_UART_RTC_WAKEUP_IT_PRIORITY	3

 /**
  *  Define a critical section in the Timer server
  *  The Timer server does not support the API to be nested
  *  The  Application shall either:
  *  	a) Ensure this will never happen (When using an OS, there could be one Timer process)
  *  	b) Define the critical section
  *  The default implementations is masking all interrupts using the PRIMASK bit
  *  When the application is implementing low latency interrupts that would not support to be masked out,
  *  the critical section may use the basepri register to mask out only interrupt that have lower priority than those interrupts
  *  In that case, the application shall not call the Timer interface within these interrupt handlers.
  */
#define TIMER_ENTER_CRITICAL_SECTION	__disable_irq()	/**< Enter the critical section */
#define TIMER_EXIT_CRITICAL_SECTION		__enable_irq()	/**< Exit the critical section */


/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
 void HAL_TIMER_Init(void);
 void HAL_TIMER_Create(eHAL_TIMER_ModuleID_t eTimerModuleID, uint8_t *pTimerId, eHAL_TIMER_TimerMode_t eTimerMode, pf_HAL_TIMER_TimerCallBack_t pfTimerCallBack);
 void HAL_TIMER_Stop(uint8_t ubTimerID);
 void HAL_TIMER_Start(uint8_t ubTimerID, uint16_t uhTimeoutTicks);
 void HAL_TIMER_Delete(uint8_t ubTimerID);
 void HAL_TIMER_Notification(eHAL_TIMER_ModuleID_t eTimerModuleID, pf_HAL_TIMER_TimerCallBack_t pfTimerCallBack);


#ifdef __cplusplus
}
#endif

#endif /*__HAL_TIMER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
