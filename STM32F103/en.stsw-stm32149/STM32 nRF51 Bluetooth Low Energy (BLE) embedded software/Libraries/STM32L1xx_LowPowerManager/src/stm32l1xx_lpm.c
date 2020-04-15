/**
  ******************************************************************************
  * @file    stm32l1xx_lpm.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Low Power Manager
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
#include "stm32l1xx_lpm.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

/**
 * Masks to be used with the LPM_Mode_Request() API to read out the correct bit
 */
#define	LPM_DEEPSLEEP_MASK	0x04
#define	LPM_LPSDSR_MASK		0x02
#define	LPM_PDDS_MASK		0x01

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t uwDeepSleep_Req = 0;	/**< Default enabled - Control the CortexM3 DeepSleep bit */
static uint32_t uwLPSDSR_Req = 0;		/**< Default enabled - Control the LDO Eco Mode */
static uint32_t uwPDDS_Req = 0;			/**< Default enabled - Control the StandBy Mode */
static uint32_t uwRUN_Req = 0;			/**< Default disabled - RUN mode NOT requested */

/* Private function prototypes -----------------------------------------------*/
static void EnterStopMode(void);
static void EnterStandbyMode(void);

/* Public function prototypes -----------------------------------------------*/
WEAK void LPM_ExitStopMode(void);
WEAK void LPM_EnterStandbyMode(void);
WEAK void LPM_ExitStandbyMode(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Enter stop mode
  *
  * @note	The different configuration in the hardware has been done before calling that API
  *
  * @param  None
  *
  * @retval None
  */
static void EnterStopMode(void)
{
	PWR->CR |=  PWR_FLAG_WU << 2;	/**< clear the WUF flag */

	/**
	 * Wait for two cycles for the WUF clear to be done
	 */
	NOP;
	NOP;

	__WFI();

	/**
	 * NOT ALLOWED - Clear LPSDSR - LP_RUN and LP_SLEEP NOT supported in that Low Power Manager design
	 */
	PWR->CR &= ~((uint32_t)PWR_CR_LPSDSR);

	LPM_ExitStopMode();

	 return;
}

/**
  * @brief  Enter standby mode
  *
  * @note	The different configuration in the hardware has been done before calling that API
  *
  * @param  None
  *
  * @retval None
  */
static void EnterStandbyMode(void)
{
	LPM_EnterStandbyMode();

	PWR->CR |=  PWR_FLAG_WU << 2;	/**< clear the WUF flag */

	/**
	 * Wait for two cycles for the WUF clear to be done
	 */
	NOP;
	NOP;

	__WFI();

	LPM_ExitStandbyMode();

	return;
}


/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Interface to the user to request a specific low power mode
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void LPM_Mode_Request(eLPM_Id eId, eLPM_Mode eMode)
{
	uint32_t uwPRIMASK_Bit;

	uwPRIMASK_Bit = __get_PRIMASK();	/**< backup PRIMASK bit */
	__disable_irq();					/**< Disable all interrupts by setting PRIMASK bit on Cortex*/

	if(eMode == eLPM_Mode_RUN)			/**< RUN Mode request Management */
	{
		uwRUN_Req |= (uint32_t)(1<<eId);	/**< RUN mode enabled - set bit */
	}
	else
	{
		uwRUN_Req &= ~((uint32_t)(1<<eId));	/**< RUN mode Disable - clear bit */

		if(eMode&LPM_DEEPSLEEP_MASK)		/**< Register DeepSleep bit request */
		{
			uwDeepSleep_Req &= ~((uint32_t)(1<<eId));	/**< DeepSleep enabled - clear bit */
		}
		else
		{
			uwDeepSleep_Req |= (uint32_t)(1<<eId);
		}

		if(eMode&LPM_LPSDSR_MASK)	/**< Register LPSDSR bit request */
		{
			uwLPSDSR_Req &= ~((uint32_t)(1<<eId));	/**< LDO Eco Mode enabled - clear bit */
		}
		else
		{
			uwLPSDSR_Req |= (uint32_t)(1<<eId);
		}

		if(eMode&LPM_PDDS_MASK)	/**< Register PDDS bit request */
		{
			uwPDDS_Req &= ~((uint32_t)(1<<eId));	/**< StandBy Mode enabled - clear bit */
		}
		else
		{
			uwPDDS_Req |= (uint32_t)(1<<eId);
		}

		uwRUN_Req &= ~((uint32_t)(1<<eId));	/**< RUN mode disabled - clear bit */
	}

	__set_PRIMASK(uwPRIMASK_Bit);	/**< Restore PRIMASK bit*/

	return;
}
/**
  * @brief  Execute the low power mode configuration.
  *
  * @note	This function configures the MCU HW according to the lowest power mode requested by all FW modules.
  *			This function shall not be interrupted is executed in critical section using PRIMASK on the Cortex.
  *
  * @param  None
  *
  * @retval None
  */
void LPM_Enter_Mode(void)
{
	uint32_t uwPRIMASK_Bit;

	uwPRIMASK_Bit = __get_PRIMASK();	/**< backup PRIMASK bit */

	__disable_irq();	/**< Disable all interrupts by setting PRIMASK bit on Cortex*/

	if(uwRUN_Req == 0)
	{
		if(uwDeepSleep_Req)	/**< Setup the DeepSleep bit */
		{
			SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP);	/**< NOT ALLOWED - Clear SLEEPDEEP bit of Cortex System Control Register */
		}
		else
		{
			SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP;	/**< ALLOWED - Set SLEEPDEEP bit of Cortex System Control Register */
		}

		if(uwLPSDSR_Req)	/**< Setup the LPSDSR bit */
		{
			PWR->CR &= ~((uint32_t)PWR_CR_LPSDSR);	/**< NOT ALLOWED - Clear LPSDSR */
		}
		else
		{
			PWR->CR |= (uint32_t)PWR_CR_LPSDSR;	/**< ALLOWED - Set LPSDSR */
		}

		if(uwPDDS_Req)	/**< Setup the PDDS bit */
		{
			PWR->CR &= ~((uint32_t)PWR_CR_PDDS);	/**< NOT ALLOWED - Clear PDDS */
		}
		else
		{
			PWR->CR |= (uint32_t)PWR_CR_PDDS;		/**< ALLOWED - Set PDDS */
		}

		if(uwDeepSleep_Req)
		{
			__WFI();		/**< Enter Sleep Mode */
		}
		else if (uwPDDS_Req == 0)
		{
			EnterStandbyMode();	/**< Enter Standby Mode */
		}
		else
		{
			EnterStopMode();	/**< Enter Stop Mode */
		}
	}

	__set_PRIMASK(uwPRIMASK_Bit);	/**< Restore PRIMASK bit*/

    return;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
