/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   Main body - DTM implementation
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
#include "main.h"
#include "hal_uart_interfaces.h"
#include "app_uart_stream.h"
#include "ble_dtm_app.h"
#include "app_error.h"
#include "blocking.h"
#include "hal_timer.h"
#include "ble_encode_access.h"
#include "hal_uart_driver_definition.h"
#include "stm32l1xx_lpm.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define	DTM_RX_PIN_NUMBER		10
#define	DTM_TX_PIN_NUMBER		11
#define CONN_CHIP_RESET_TIME	125                                /**< The time to keep the reset line to the nRF51822 low ( 50 milliseconds). */
#define DEAD_BEEF               0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t	ubnRFresetTimerLock;

/* Private function prototypes -----------------------------------------------*/
int main(void);
static void pf_nRFResetTimerCallBack(void);
static void Init_RTC(void);
static void UART_BLE_Init(void);

/* Public functions ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Init_RTC(void)
{
	/* Initialize the HW - 37Khz LSI being used*/
	/* Enable the LSI clock */
	RCC_LSICmd(ENABLE);

	/* Enable power module clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Enable acces to the RTC registers */
	PWR_RTCAccessCmd(ENABLE);

	/* Select LSI as RTC Input */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	/* Enable RTC */
	RCC_RTCCLKCmd(ENABLE);

	/* Disable Write Protection */
	RTC_WriteProtectionCmd(DISABLE);

	/* Wait for LSI to be stable */
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == 0);

	return;
}

/**
  * @brief  UART Low power mode request
  * @note   None
  * @param  None
  * @retval None
  */
void HAL_UART_HSclkRequest(eHAL_UART_HSclkRequester_t eHSclkRequester, eHAL_UART_HSclkMode_t eHSclkMode)
{
	if(eHSclkMode == eUART_HSclkEnable)
	{
		LPM_Mode_Request((eLPM_Id) eHSclkRequester, eLPM_Mode_Sleep);
	}
	else
	{
		LPM_Mode_Request((eLPM_Id) eHSclkRequester, eLPM_Mode_StandBy);
	}

	return;
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    while(1);
}


void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    app_error_handler(DEAD_BEEF, line_num, file_name);
}

static void dtm_mode_init(void)
{
    uint32_t err_code;
    app_uart_stream_comm_params_t uart_params;

    uart_params.baud_rate  = UART_BAUD_RATE_57600;
    uart_params.rx_pin_no  = DTM_RX_PIN_NUMBER;
    uart_params.tx_pin_no  = DTM_TX_PIN_NUMBER;

    err_code = ble_dtm_init(&uart_params);
    APP_ERROR_CHECK(err_code);

    err_code = blocking_resp_wait();
    APP_ERROR_CHECK(err_code);

    return;
}

/**@brief Function for resetting the nRF51822.
 *
 * PC7
 */
static void connectivity_chip_reset(void)
{
	GPIO_InitTypeDef sGPIO_InitStruct;
	uint32_t uwTempRegister;
	uint8_t	ubnRFResetTimerID;

    /* Switch ON GPIO AHB clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* Drive low the GPIO PC7 to reset nRF51 */
	GPIOC->BSRRH = GPIO_Pin_7;

	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	sGPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC,&sGPIO_InitStruct);

    HAL_TIMER_Create(eTimerModuleID_Interrupt, &ubnRFResetTimerID, eTimerMode_SingleShot, pf_nRFResetTimerCallBack);
    HAL_TIMER_Start(ubnRFResetTimerID, CONN_CHIP_RESET_TIME);
    ubnRFresetTimerLock = 1;
    while(ubnRFresetTimerLock == 1);
    HAL_TIMER_Delete(ubnRFResetTimerID);

	/* Set IO pin in Analog mode */
	/* Set IO speed to 400Khz */
	/* The parameter from sGPIO_InitStruct.GPIO_Speed does not apply in Input mode */
	uwTempRegister = GPIOC->OSPEEDR;
	uwTempRegister &= ~(GPIO_OSPEEDER_OSPEEDR0 << ((uint16_t)GPIO_PinSource7 * 2));
	uwTempRegister |= (((uint32_t)GPIO_Speed_400KHz) << ((uint16_t)GPIO_PinSource7 * 2));
	GPIOC->OSPEEDR = uwTempRegister;

	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOC,&sGPIO_InitStruct);

	/* Switch OFF GPIO AHB clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);

	return;
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
	GPIO_InitTypeDef sGPIO_InitStruct;

	/* The APB power block clock is enable during the startup phase */

#if (JTAG_SUPPORTED == 1)
	/* Keep debugger enable while in any low power mode */
	DBGMCU_Config(DBGMCU_SLEEP, ENABLE);
	DBGMCU_Config(DBGMCU_STOP, ENABLE);
	DBGMCU_Config(DBGMCU_STANDBY, ENABLE);
#endif

	/* Enable Ultra low power mode */
	PWR_UltraLowPowerCmd(ENABLE);

	/* Enable Fast Wakeup  */
	PWR_FastWakeUpCmd(ENABLE);

/*******************************************************************************************/
/* Setting all IOs as Analog mode with no pull */
	/* Switch ON GPIO AHB clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOD|RCC_AHBPeriph_GPIOE|RCC_AHBPeriph_GPIOF|RCC_AHBPeriph_GPIOG|RCC_AHBPeriph_GPIOH, ENABLE);

	/* Setup all GPIO except JTAG (PA15/14/13 - PB4/3 in Analog NoPull) */
	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_400KHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	sGPIO_InitStruct.GPIO_Pin = GPIO_Pin_All;

	GPIO_Init(GPIOC,&sGPIO_InitStruct);
	GPIO_Init(GPIOD,&sGPIO_InitStruct);
	GPIO_Init(GPIOE,&sGPIO_InitStruct);
	GPIO_Init(GPIOF,&sGPIO_InitStruct);
	GPIO_Init(GPIOG,&sGPIO_InitStruct);
	GPIO_Init(GPIOH,&sGPIO_InitStruct);

#if (JTAG_SUPPORTED == 1)
	sGPIO_InitStruct.GPIO_Pin = GPIO_Pin_All ^ (GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13);
#endif
	GPIO_Init(GPIOA,&sGPIO_InitStruct);

#if (JTAG_SUPPORTED == 1)
	sGPIO_InitStruct.GPIO_Pin = GPIO_Pin_All ^ (GPIO_Pin_4|GPIO_Pin_3);
#endif
	GPIO_Init(GPIOB,&sGPIO_InitStruct);

	/* Switch OFF GPIO AHB clock except port C for user button*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOD|RCC_AHBPeriph_GPIOE|RCC_AHBPeriph_GPIOF|RCC_AHBPeriph_GPIOG|RCC_AHBPeriph_GPIOH, DISABLE);

/*******************************************************************************************/

	/* Define minimum low power mode allowed in the system */
	LPM_Mode_Request(eLPM_Application, eLPM_Mode_LP_Stop);

	Init_RTC();
    HAL_TIMER_Init();

    /* Request all resources for the UART communication with the nRF device */
    UART_BLE_Init();
	
    // Reset connectivity chip.
	connectivity_chip_reset();

    // Open encoder module.
    err_code = ble_encode_open();
    APP_ERROR_CHECK(err_code);
    
    err_code = blocking_init();
    APP_ERROR_CHECK(err_code);
    
	dtm_mode_init();

	while(1)
    {
        // Power management.
		Background(SD_COMMAND_ALLOWED);
    }
}

/**@brief Function for application main entry.
 */
void Background (uint32_t event_mask)
{
	LPM_Enter_Mode();

    return;
}


void pf_nRFResetTimerCallBack(void)
{
	ubnRFresetTimerLock = 0;

	return;
}

USED void LPM_ExitStopMode(void)
{
	/* Switch back to HSI */
	 RCC->CR |= ((uint32_t)RCC_CR_HSION);

	/* Wait till HSI is ready  */
	 while((RCC->CR & RCC_CR_HSIRDY) == 0);

    /* Select HSI as system clock source */
	 RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;

    /* Wait till HSI is used as system clock source */
	 while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_HSI);

	 return;
}

#if ( (USART_CTS_PIN_SELECTED == PA11) || (USART_CTS_PIN_SELECTED == PB13) || (USART_CTS_PIN_SELECTED == PD11))
void EXTI15_10_IRQHandler(void)
{
	HAL_UART_wcts_handler();

	return;
}
#endif

void UART_BLE_Init(void)
{
	__disable_irq();
	/* Switch ON USART GPIO AHB and DMA1 clock */
	RCC_AHBPeriphClockCmd(USART_GPIO_AHB_CLK | USART_DMA_AHB_CLK, ENABLE);
	/* Keep DMA clock in sleep mode */
	RCC_AHBPeriphClockLPModeCmd(USART_DMA_AHB_CLK, ENABLE);

	/* Switch ON SYSCFG APB Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

#if (USART_SELECTED == BLE_ON_USART1)
	/* Switch ON USART_BLE APB2 */
	RCC_APB2PeriphClockCmd(RCC_APBPeriph_USART_BLE, ENABLE);
	/* Keep UART clock is sleep mode to receive bytes */
	RCC_APB2PeriphClockLPModeCmd(RCC_APBPeriph_USART_BLE, ENABLE);
#elif ( (USART_SELECTED == BLE_ON_USART2) || (USART_SELECTED == BLE_ON_USART3) )
	/* Switch ON USART_BLE APB1 */
	RCC_APB1PeriphClockCmd(RCC_APBPeriph_USART_BLE, ENABLE);
	/* Keep UART clock is sleep mode to receive bytes */
	RCC_APB1PeriphClockLPModeCmd(RCC_APBPeriph_USART_BLE, ENABLE);
#endif

	/* Keep SRAM clock in sleep mode for DMA */
	RCC_AHBPeriphClockLPModeCmd(RCC_AHBPeriph_SRAM, ENABLE);

	/* Enable PWR APB1 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	__enable_irq();

	return;
}

void HAL_TIMER_Notification(eHAL_TIMER_ModuleID_t eTimerModuleID, pf_HAL_TIMER_TimerCallBack_t pfTimerCallBack)
{
	switch(eTimerModuleID)
	{
		default:
			if(pfTimerCallBack != 0)
			{
				pfTimerCallBack();
			}
			break;
	}

	return;
}

void HAL_UART_Msg_Handler(pf_HAL_UART_PhyDriverEvent_Handler_t pf_PhyDriverEvent_Handler, sHAL_UART_PhyDriverEvent_t pPhyDriverEvent)
{
	pf_PhyDriverEvent_Handler(pPhyDriverEvent);

	return;
}

/**
  * @brief  Request action to the scheduler in background
  *
  * @note	Not required in DTM project
  *
  * @param  eMAIN_Backround_Task_Id: Id of the request
  *
  * @retval None
  */
void TaskExecutionRequest(eMAIN_Backround_Task_Id_t eMAIN_Backround_Task_Id)
{
	return;
}

/**
  * @brief  Notify the action in background has been completed
  *
  * @note	Not required in DTM project
  *
  * @param  eMAIN_Backround_Task_Id: Id of the request
  *
  * @retval None
  */
void TaskExecuted(eMAIN_Backround_Task_Id_t eMAIN_Backround_Task_Id)
{
	return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
