/**
  ******************************************************************************
  * @file    hal_timer.c
  * @author  MCD Application Team
  * @version V2.0
  * @date    17-September-2014
  * @brief   Timer server
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "hal_timer.h"
#include "stm32l1xx_it.h"
#include "stm32l1xx_pwr.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum{
	eTimerID_Free,
	eTimerID_Created,
	eTimerID_Running
}eTimerStatus_t;

typedef struct{
	pf_HAL_TIMER_TimerCallBack_t	pfTimerCallBack;
	uint16_t						uhCounterInit;
	uint16_t						uhCountLeft;
	eTimerStatus_t					eTimerStatus;
	eHAL_TIMER_TimerMode_t			eTimerMode;
	eHAL_TIMER_ModuleID_t			eTimerModuleID;
	uint8_t							ubPreviousID;
	uint8_t							ubNextID;
}sTimerContext_t;

/* Private macros ------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile sTimerContext_t aTimerContext[MAX_NBR_CONCURRENT_TIMER];
volatile uint8_t	ubCurrentRunningTimerID;



/* Private function prototypes -----------------------------------------------*/
static uint16_t GetCurrentCounter(void);
static void rtc_wakeup_handler(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Read the current wakeup counter
  *
  * @note	The API handles the corner case when the max value is read.
  * 		This could happen either because:
  * 		1) the timer did not yet start counting
  * 		2) The timer counted down to '0' and was reloaded automatically in which case the value to consider is '0'
  *
  * @param  None
  *
  * @retval Return the value read from the wakeup counter
  */
static uint16_t GetCurrentCounter(void)
{
	uint16_t uhCurrentCounter;

	uhCurrentCounter = 0;

	while((uhCurrentCounter != RTC_GetWakeUpCounter()) && (RTC_GetITStatus(RTC_IT_WUT) == RESET))
	{
		uhCurrentCounter = RTC_GetWakeUpCounter();
	}

	if(RTC_GetITStatus(RTC_IT_WUT) == SET)
	{
		uhCurrentCounter = 0;
	}

	return uhCurrentCounter;
}

/**
  * @brief  Schedule the timer list on the timer interrupt handler
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void rtc_wakeup_handler(void)
{
	pf_HAL_TIMER_TimerCallBack_t		pfTimerCallBack;
	eHAL_TIMER_ModuleID_t eTimerModuleID;
	uint16_t	localCountLeft;
	uint8_t	ubTimerIDLookUp;
	uint8_t	localCurrentRunningTimerID, localNextID;

    /**
     * Disable the Wakeup Timer
     */
    RTC->CR &= (uint32_t)~RTC_CR_WUTE;

	/**
	 * Clear flag in EXTI module
	 */
	EXTI_ClearITPendingBit(EXTI_Line20);

	localCurrentRunningTimerID = ubCurrentRunningTimerID;

	if(aTimerContext[localCurrentRunningTimerID].eTimerStatus == eTimerID_Running)
	{
		/**
		 * update count for all timers left
		 */
		ubTimerIDLookUp = localCurrentRunningTimerID;
		while((localNextID = aTimerContext[ubTimerIDLookUp].ubNextID) != MAX_NBR_CONCURRENT_TIMER)
		{
			localCountLeft = aTimerContext[localCurrentRunningTimerID].uhCountLeft;
			aTimerContext[localNextID].uhCountLeft -= localCountLeft;
			ubTimerIDLookUp = localNextID;
		}

		aTimerContext[localCurrentRunningTimerID].uhCountLeft = 0;

		pfTimerCallBack = aTimerContext[localCurrentRunningTimerID].pfTimerCallBack;
		eTimerModuleID = aTimerContext[localCurrentRunningTimerID].eTimerModuleID;

		if(aTimerContext[localCurrentRunningTimerID].eTimerMode == eTimerMode_Repeated)
		{
			HAL_TIMER_Start(localCurrentRunningTimerID, aTimerContext[localCurrentRunningTimerID].uhCounterInit);

			/**
			 * Clear flag in RTC module AFTER HAL_TIMER_Start() is called
			 * the RTC flag is checked when reading the wakeup timer to readout the correct value
			 * It shall not be cleared before the wakeup counter has been read
			 */
			RTC_ClearITPendingBit(RTC_IT_WUT);

			/**
			 * The timer may have been linked after the current timer running. The HAL_TIMER_Start() API is not aware the
			 * timer has been stopped in the handler so there should be a checked and a restart in case the timer is stopped			 *
			 */
			if( (RTC->CR & (uint32_t)RTC_CR_WUTE) == 0)
			{
				while((RTC->ISR & RTC_ISR_WUTWF) == 0);

				/**
				 * Write next count
				 */
				RTC->WUTR = aTimerContext[ubCurrentRunningTimerID].uhCountLeft;

				/**
				 * Enable the Wakeup Timer
				 */
				RTC->CR |= (uint32_t)RTC_CR_WUTE;
			}
		}
		else
		{
			HAL_TIMER_Stop(localCurrentRunningTimerID);

			/**
			 * Clear flag in RTC module AFTER HAL_TIMER_Stop(() is called
			 * the RTC flag is checked when reading the wakeup timer to readout the correct value
			 * It shall not be cleared before the wakeup counter has been read
			 */
			RTC_ClearITPendingBit(RTC_IT_WUT);
		}

		HAL_TIMER_Notification(eTimerModuleID, pfTimerCallBack);
	}
	else
	{
		RTC_ClearITPendingBit(RTC_IT_WUT);
	}

	return;
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize the timer server
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void HAL_TIMER_Init(void)
{
	uint8_t	ubLoop;
	EXTI_InitTypeDef RTC_EXTI_WakeupStruct;

	/**
	 * Reinit everything for the scope of this projet
	 */

    RTC->CR &= (uint32_t)~RTC_CR_WUTE;		/**<  Disable the Wakeup Timer */
	RTC_ClearITPendingBit(RTC_IT_WUT);		/**<  Clear flag in RTC module */
	EXTI_ClearITPendingBit(EXTI_Line20);	/**<  Clear flag in EXTI module  */
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);	/**<  Clear pending bit in NVIC  */
	RTC->CR |= RTC_CR_WUTIE;				/**<  Enable interrupt in RTC module  */

	/**
	 * Configure EXTI module
	 */
	RTC_EXTI_WakeupStruct.EXTI_Line = EXTI_Line20;
	RTC_EXTI_WakeupStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	RTC_EXTI_WakeupStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	RTC_EXTI_WakeupStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&RTC_EXTI_WakeupStruct);

	/**
	 * Initialize the timer server
	 */
	for(ubLoop = 0; ubLoop < MAX_NBR_CONCURRENT_TIMER; ubLoop++)
	{
		aTimerContext[ubLoop].eTimerStatus = eTimerID_Free;
	}

	ubCurrentRunningTimerID = MAX_NBR_CONCURRENT_TIMER;						/**<  Set ID to non valid value */
	NVIC_SetPriority(RTC_WKUP_IRQn, NVIC_UART_RTC_WAKEUP_IT_PRIORITY);		/**<  Set NVIC priority */

	return;
}

/**
  * @brief  Interface to create a virtual timer
  *
  * @note
  *
  * @param  eTimerModuleID:	This is an identifier provided by the user and returned in the callback to allow
  * 						identification of the requester
  *
  * @param  pTimerId:		Timer Id returned to the user to request operation (start, stop, delete)
  *
  * @param  eTimerMode:		Mode of the virtual timer (Single shot or repeated)
  *
  * @param  pftimeout_handler:	Callback when the virtual timer expires
  *
  * @retval None
  */
void HAL_TIMER_Create(eHAL_TIMER_ModuleID_t eTimerModuleID, uint8_t *pTimerId, eHAL_TIMER_TimerMode_t eTimerMode, pf_HAL_TIMER_TimerCallBack_t pftimeout_handler)
{
	uint8_t	ubLoop = 0;

	TIMER_ENTER_CRITICAL_SECTION;
	while(aTimerContext[ubLoop].eTimerStatus != eTimerID_Free)
	{
		ubLoop++;
	}

	aTimerContext[ubLoop].eTimerStatus = eTimerID_Created;
	TIMER_EXIT_CRITICAL_SECTION	;

	aTimerContext[ubLoop].eTimerModuleID = eTimerModuleID;
	aTimerContext[ubLoop].eTimerMode = eTimerMode;
	aTimerContext[ubLoop].pfTimerCallBack = pftimeout_handler;
	*pTimerId = ubLoop;

	return;
}

/**
  * @brief  Delete a virtual timer from the list
  *
  * @note	If the virtual timer is running, it is stopped and deleted.
  *
  * @param  ubTimerID:	Id of the timer to remove from the list
  *
  * @retval None
  */
void HAL_TIMER_Delete(uint8_t ubTimerID)
{
	HAL_TIMER_Stop(ubTimerID);

	aTimerContext[ubTimerID].eTimerStatus = eTimerID_Free;	/**<  release ID */

	return;
}

/**
  * @brief  Stop a virtual timer
  *
  * @note	A virtual timer that has been stopped may be restarted
  *
  * @param  ubTimerID:	Id of the timer to stop
  *
  * @retval None
  */
void HAL_TIMER_Stop(uint8_t ubTimerID)
{
	uint16_t	uhTimeElapsed;
	uint8_t		ubTimerIDLookUp;
	uint8_t		localID;

	TIMER_ENTER_CRITICAL_SECTION;

	NVIC_DisableIRQ(RTC_WKUP_IRQn);		/**<  Disable NVIC */

	if(aTimerContext[ubTimerID].eTimerStatus == eTimerID_Running)
	{
		/**
		 * Update current running timer ID if needed
		 */
		if(ubTimerID == ubCurrentRunningTimerID)
		{
			if(aTimerContext[ubTimerID].ubNextID != MAX_NBR_CONCURRENT_TIMER)
			{
				uhTimeElapsed = aTimerContext[ubTimerID].uhCountLeft - GetCurrentCounter();

				RTC->CR &= (uint32_t)~RTC_CR_WUTE;		/**<  Disable the Wakeup Timer */
				RTC_ClearITPendingBit(RTC_IT_WUT);		/**<  Clear flag in RTC module */
				EXTI_ClearITPendingBit(EXTI_Line20);	/**<  Clear flag in EXTI module */
				NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);	/**<  Clear pending bit in NVIC */

				/**
				 * update all counts
				 */
				ubTimerIDLookUp = ubTimerID;
				while((localID = aTimerContext[ubTimerIDLookUp].ubNextID) != MAX_NBR_CONCURRENT_TIMER)
				{
					aTimerContext[localID].uhCountLeft -= uhTimeElapsed;
					ubTimerIDLookUp = aTimerContext[ubTimerIDLookUp].ubNextID;
				}

				while( (RTC->ISR & RTC_ISR_WUTWF) == 0);

				/**
				 * Write next count
				 */
				RTC->WUTR = aTimerContext[aTimerContext[ubTimerID].ubNextID].uhCountLeft - uhTimeElapsed;

				RTC->CR |= (uint32_t)RTC_CR_WUTE;	/**<  Enable the Wakeup Timer */
			}
			else
			{
				RTC->CR &= (uint32_t)~RTC_CR_WUTE;		/**<  Disable the Wakeup Timer */
				RTC_ClearITPendingBit(RTC_IT_WUT);		/**<  Clear flag in RTC module */
				EXTI_ClearITPendingBit(EXTI_Line20);	/**<  Clear flag in EXTI module */
				NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);	/**<  Clear pending bit in NVIC */
			}

			ubCurrentRunningTimerID = aTimerContext[ubTimerID].ubNextID;
		}
		else
		{
			/**
			 * unlink timer
			 */
			localID = aTimerContext[ubTimerID].ubPreviousID;
			aTimerContext[localID].ubNextID = aTimerContext[ubTimerID].ubNextID;
			if((localID = aTimerContext[ubTimerID].ubNextID) != MAX_NBR_CONCURRENT_TIMER)
			{
				aTimerContext[localID].ubPreviousID = aTimerContext[ubTimerID].ubPreviousID;
			}
		}
	}

	/**
	 * Timer is out of the list
	 */
	aTimerContext[ubTimerID].eTimerStatus = eTimerID_Created;

	NVIC_EnableIRQ(RTC_WKUP_IRQn);	/**<  Enable NVIC */

	TIMER_EXIT_CRITICAL_SECTION;

	return;
}

/**
  * @brief  Start a virtual timer
  *
  * @note	The value of the ticks is depending on the user RTC clock input configuration
  *
  * @param  ubTimerID:	Id of the timer to star
  *
  * @param  uhTimeoutTicks:	Number of ticks of the virtual timer
  *
  * @retval None
  */
void HAL_TIMER_Start(uint8_t ubTimerID, uint16_t uhTimeoutTicks)
{
	uint16_t	uhTimeElapsed;
	uint8_t	ubTimerIDLookUp;
	uint8_t localNextID;

	TIMER_ENTER_CRITICAL_SECTION;

	NVIC_DisableIRQ(RTC_WKUP_IRQn);		/**<  Disable NVIC */

	aTimerContext[ubTimerID].eTimerStatus = eTimerID_Running;

	if(ubCurrentRunningTimerID == MAX_NBR_CONCURRENT_TIMER)
	{
		/**
		 * No timer in the list
		 */
		ubCurrentRunningTimerID = ubTimerID;
		aTimerContext[ubTimerID].ubNextID = MAX_NBR_CONCURRENT_TIMER;

		while( (RTC->ISR & RTC_ISR_WUTWF) == 0);

		RTC->WUTR = uhTimeoutTicks;			/**<  Write next count */
	    RTC->CR |= (uint32_t)RTC_CR_WUTE;	/**<  Enable the Wakeup Timer */
	}
	else
	{
		uhTimeElapsed = aTimerContext[ubCurrentRunningTimerID].uhCountLeft - GetCurrentCounter();

		ubTimerIDLookUp = ubCurrentRunningTimerID;
		/**
		 * Link the timer
		 */
		while( (aTimerContext[ubTimerIDLookUp].ubNextID != MAX_NBR_CONCURRENT_TIMER) && ((aTimerContext[ubTimerIDLookUp].uhCountLeft-uhTimeElapsed) < uhTimeoutTicks) )
		{
			ubTimerIDLookUp = aTimerContext[ubTimerIDLookUp].ubNextID;
		}

		/**
		 * ubTimerID is either juste BEFORE or just AFTER ubTimerIDLookUp
		 */

		if((aTimerContext[ubTimerIDLookUp].uhCountLeft-uhTimeElapsed) < uhTimeoutTicks)
		{
			/**
			 * Link timer AFTER ubTimerIDLookUp
			 */
			if( (ubTimerID == ubCurrentRunningTimerID) && (aTimerContext[ubCurrentRunningTimerID].ubNextID != MAX_NBR_CONCURRENT_TIMER) )
			{
				ubCurrentRunningTimerID = aTimerContext[ubCurrentRunningTimerID].ubNextID;
			}

			if(ubTimerIDLookUp != ubTimerID)
			{
				if(aTimerContext[ubTimerIDLookUp].ubNextID != MAX_NBR_CONCURRENT_TIMER)
				{
					aTimerContext[aTimerContext[ubTimerIDLookUp].ubNextID].ubPreviousID = ubTimerID;
				}

				aTimerContext[ubTimerID].ubNextID = aTimerContext[ubTimerIDLookUp].ubNextID;
				aTimerContext[ubTimerID].ubPreviousID = ubTimerIDLookUp ;
				aTimerContext[ubTimerIDLookUp].ubNextID = ubTimerID;
			}
		}
		else
		{
			/**
			 * Link timer BEFORE ubTimerIDLookUp
			 */

			if(ubTimerIDLookUp != ubCurrentRunningTimerID)
			{
				if( (aTimerContext[ubCurrentRunningTimerID].ubNextID != ubTimerIDLookUp) && (ubCurrentRunningTimerID == ubTimerID) )
				{
					ubCurrentRunningTimerID = aTimerContext[ubCurrentRunningTimerID].ubNextID;

					aTimerContext[aTimerContext[ubTimerIDLookUp].ubPreviousID].ubNextID = ubTimerID;
					aTimerContext[ubTimerID].ubPreviousID = aTimerContext[ubTimerIDLookUp].ubPreviousID ;

					aTimerContext[ubTimerID].ubNextID = ubTimerIDLookUp;
					aTimerContext[ubTimerIDLookUp].ubPreviousID = ubTimerID;

					while((RTC->ISR & RTC_ISR_WUTWF) == 0);

					RTC->WUTR = aTimerContext[ubCurrentRunningTimerID].uhCountLeft;	/**<  Write next count */
				    RTC->CR |= (uint32_t)RTC_CR_WUTE;								/**<  Enable the Wakeup Timer */
				}
				else
				{
					aTimerContext[aTimerContext[ubTimerIDLookUp].ubPreviousID].ubNextID = ubTimerID;
					aTimerContext[ubTimerID].ubPreviousID = aTimerContext[ubTimerIDLookUp].ubPreviousID ;

					aTimerContext[ubTimerID].ubNextID = ubTimerIDLookUp;
					aTimerContext[ubTimerIDLookUp].ubPreviousID = ubTimerID;
				}
			}
			else
			{
				if(ubTimerIDLookUp != ubTimerID)
				{
					aTimerContext[ubTimerID].ubNextID = ubTimerIDLookUp;
					aTimerContext[ubTimerIDLookUp].ubPreviousID = ubTimerID;
				}

			    RTC->CR &= (uint32_t)~RTC_CR_WUTE;		/**<  Disable the Wakeup Timer */
				RTC_ClearITPendingBit(RTC_IT_WUT);		/**<  Clear flag in RTC module */
				EXTI_ClearITPendingBit(EXTI_Line20);	/**<  Clear flag in EXTI module */
				NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);	/**<  Clear pending bit in NVIC */

				ubCurrentRunningTimerID = ubTimerID;

				while((RTC->ISR & RTC_ISR_WUTWF) == 0);

				RTC->WUTR = uhTimeoutTicks;				/**<  Write next count */
			    RTC->CR |= (uint32_t)RTC_CR_WUTE;		/**<  Enable the Wakeup Timer */

			    /**
			     * update all counts
			     */
				ubTimerIDLookUp = ubCurrentRunningTimerID;
				while((localNextID = aTimerContext[ubTimerIDLookUp].ubNextID) != MAX_NBR_CONCURRENT_TIMER)
				{
					aTimerContext[localNextID].uhCountLeft -= uhTimeElapsed;
					ubTimerIDLookUp = aTimerContext[ubTimerIDLookUp].ubNextID;
				}
			}
		}
	}

	aTimerContext[ubTimerID].uhCountLeft = uhTimeoutTicks;
	aTimerContext[ubTimerID].uhCounterInit = uhTimeoutTicks;

	NVIC_EnableIRQ(RTC_WKUP_IRQn);	/**<  Enable NVIC */

	TIMER_EXIT_CRITICAL_SECTION;

	return;
}

/**
  * @brief  RTC Wakeup handler
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void RTC_WKUP_IRQHandler(void)
{
	rtc_wakeup_handler();

	return;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
