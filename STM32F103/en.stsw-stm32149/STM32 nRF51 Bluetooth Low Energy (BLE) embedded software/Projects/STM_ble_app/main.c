/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V2.0
  * @date    29-August-2014
  * @brief   Main body
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
#include <stdlib.h>
#include <stdio.h>
#include "stm32l1xx.h"
#include "main.h"
#include "hal_uart_interfaces.h"
#include "app_uart_stream.h"
#include "hal_timer.h"
#include "hal_uart_driver_definition.h"
#include "ble_app_main.h"
#include "hal_nvm.h"
#include "stm32l1xx_lpm.h"
#include "ble_bondmngr.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_wavetek.h"
#include "ff.h"
#include "stm32_adafruit_spi_lcd.h"
#include "stm32_adafruit_spi_usd.h"
#include "fatfs_storage.h"

#if(APP_L2CAP_TX_TEST == 1)
#include "ble_l2cap.h"
#include "ble.h"
#include "blocking.h"
#endif

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
#if(APP_LCD == 1)
typedef enum{SHIELD_NOT_DETECTED = 0, SHIELD_DETECTED}ShieldStatus;
#endif

typedef struct
{
    uint16_t VREF;
    uint16_t TS_CAL_COLD;
    uint16_t reserved;
    uint16_t TS_CAL_HOT;
} CALIB_TypeDef;

/* Private defines -----------------------------------------------------------*/
#define CONN_CHIP_RESET_TIME		5	/**< The time to keep the reset line to the nRF51822 low ( 2 milliseconds). */
#define CONN_CHIP_STARTUP_TIME		19  /**< Wait for the nRF device to be ready before sending any command ( 7.5 milliseconds). */

#define FACTORY_CALIB_BASE        	((uint32_t)0x1FF800F8)    /**< Calibration Data Bytes base address */
#define FACTORY_CALIB_DATA        	((CALIB_TypeDef *) FACTORY_CALIB_BASE)
#define USER_CALIB_BASE           	((uint32_t)0x08080000)    /**< USER Calibration Data Bytes base address */
#define USER_CALIB_DATA           	((CALIB_TypeDef *) USER_CALIB_BASE)
#define TEST_CALIB_DIFF           	(int32_t) 50  /**< difference of hot-cold calib data to be considered as valid */
#define DEFAULT_HOT_VAL 			0x355
#define DEFAULT_COLD_VAL 			0x29B
#define TEMP_CORRECTION_ERROR 		22			/**< The calibrated value are not correct and an offfset shall be added to the measure
													 read from the temperature sensor */
#define MAX_TEMP_CHNL 				16

#if(APP_L2CAP_TX_TEST == 1)
#define L2CAP_HEADER_CID			0x0044		/**< L2CAP header CID */
#define L2CAP_PAYLOAD_LENGTH		23			/**< Length of the L2CAP payload */
#define L2CAP_PAYLOAD_PATTERN		0x55		/**< Pattern to be sent in the payload */
#endif

#if (APP_LEDBUTTON == 1)
#define BUTTON_1					0x01		/**< Value to send when Button 1 is pushed */
#define BUTTON_2					0x02    	/**< Value to send when Button 2 is pushed */
#define BUTTON_3					0x03    	/**< Value to send when Button 3 is pushed */

#define LED1_VALUE					0x01
#define LED2_VALUE					0x02
#define LED3_VALUE					0x04
#endif

#if(APP_LCD == 1)
#define SD_CARD_NOT_FOUND                        0
#define SD_CARD_NOT_FORMATTED                    1
#define SD_CARD_FILE_NOT_SUPPORTED               2
#define SD_CARD_OPEN_FAIL                        3
#define FATFS_NOT_MOUNTED                        4
#endif

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t TaskExecutionRequested = 0;
static volatile uint8_t	ubnRFresetTimerLock;
static pf_Main_UserButtonHandler_t pf_UserButtonHandler;
pf_HAL_TIMER_TimerCallBack_t pfTimerBLECallBack;
uint32_t tempAVG;
volatile int32_t temperature_C;
uint16_t ADC_ConvertedValueBuff[MAX_TEMP_CHNL];
CALIB_TypeDef calibdata;    /**< field storing temp sensor calibration data */
static __IO uint32_t TimingDelay;

#if(APP_L2CAP_TX_TEST == 1)
static uint8_t	FreeBufferCount;		/**< Number of buffer available for L2CAP transfer on nRF side */
uint8_t L2CAPPayloadBuffer[L2CAP_PAYLOAD_LENGTH];
ble_l2cap_header_t l2cap_header;
#endif

#if(APP_LCD == 1)
uint8_t str[20];
char* pDirectoryFiles[MAX_BMP_FILES];
FATFS microSDFatFs;
uint8_t bmpcounter = 0;
DIR directory;
FRESULT res;
#endif

/* Private function prototypes -----------------------------------------------*/
static void pf_nRFResetTimerCallBack(void);
static void Init_RTC(void);
static void UART_BLE_Init(void);
static void  powerDownADC_Temper(void);
static void  processTempData(void);
static void insertionSort(uint16_t *numbers, uint32_t array_size);
static uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples);
static void ProcessADCdata(void);
static void connectivity_chip_reset(void);
#if(APP_HTS == 1)
static void  configureADC_Temp(void);
static void  configureDMA(void);
static FunctionalState  testUserCalibData(void);
static FunctionalState  testFactoryCalibData(void);
#endif
#if(APP_LCD == 1)
static void LCD_Init(void);
static void SDCard_Config(void);
static ShieldStatus TFT_ShieldDetect(void);
static void Menu(void);
static void TFT_DisplayErrorMessage(uint8_t message);
static void Display_Images(void);
__IO uint8_t filesnumbers = 0;
#endif

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Switch ON resources needed for BLE operation
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void UART_BLE_Init(void)
{
	__disable_irq();

	RCC_AHBPeriphClockCmd(USART_GPIO_AHB_CLK | USART_DMA_AHB_CLK, ENABLE);	/**< Switch ON USART GPIO AHB and DMA1 clock */
	RCC_AHBPeriphClockLPModeCmd(USART_DMA_AHB_CLK, ENABLE);	/**< Keep DMA clock in sleep mode */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	/**< Switch ON SYSCFG APB Clock */

#if (USART_SELECTED == BLE_ON_USART1)
	RCC_APB2PeriphClockCmd(RCC_APBPeriph_USART_BLE, ENABLE);	/**< Switch ON USART_BLE APB2 */
	RCC_APB2PeriphClockLPModeCmd(RCC_APBPeriph_USART_BLE, ENABLE);	/**< Keep UART clock is sleep mode to receive bytes */
#elif ( (USART_SELECTED == BLE_ON_USART2) || (USART_SELECTED == BLE_ON_USART3) )
	RCC_APB1PeriphClockCmd(RCC_APBPeriph_USART_BLE, ENABLE);	/**< Switch ON USART_BLE APB1 */
	RCC_APB1PeriphClockLPModeCmd(RCC_APBPeriph_USART_BLE, ENABLE);	/**< Keep UART clock is sleep mode to receive bytes */
#endif
	RCC_AHBPeriphClockLPModeCmd(RCC_AHBPeriph_SRAM, ENABLE);	/**< Keep SRAM clock in sleep mode for DMA */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);			/**< Enable PWR APB1 clock */

	__enable_irq();

	return;
}

/**
  * @brief  This function processes the data received from ADC
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void ProcessADCdata(void)
{
  processTempData();
  LPM_Mode_Request(eLPM_ADC_Temp_Measurement, eLPM_Mode_StandBy);
  temperature_measurement_send();

  return;
}

/**
  * @brief  This function notify when then nRF51 nRESET may be released
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void pf_nRFResetTimerCallBack(void)
{
	ubnRFresetTimerLock = 0;

	return;
}

/**
  * @brief  Switch OFF resources used for ADC measurement
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void powerDownADC_Temper(void)
{
  ADC_Cmd(ADC1, DISABLE);								/**< Disable ADC1 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);	/**< Disable ADC1 clock */
  PWR_UltraLowPowerCmd(ENABLE);	/**< Enable Ultra low power mode */

  return;
}

#if(APP_HTS == 1)
/**
  * @brief  Configure the ADC for temperature measurement
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void configureADC_Temp(void)
{
  uint32_t ch_index;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_InitTypeDef ADC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB1Periph_PWR, ENABLE);	/**< Enable ADC clock & PWR */

  ADC_TempSensorVrefintCmd(ENABLE);	/**< Enable the internal connection of Temperature sensor and with the ADC channels*/

  /*
   *  Setup ADC common init struct
   */
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);


  /*
   * Initialise the ADC1 by using its init structure
   */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	/**< Set conversion resolution to 12bit */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;				/**< Enable Scan mode (single conversion for each channel of the group) */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		/**< Disable Continuous conversion */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None; /**< Disable external conversion trigger */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                  /**< Set conversion data alignement to right */
  ADC_InitStructure.ADC_NbrOfConversion = MAX_TEMP_CHNL;             /**< Set conversion data alignement to MAX_TEMP_CHNL */
  ADC_Init(ADC1, &ADC_InitStructure);

  /*
   * ADC1 regular Temperature sensor channel16 and internal reference channel17 configuration
   */
  for(ch_index = 1; ch_index <= MAX_TEMP_CHNL; ch_index++)
  {
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, ch_index, ADC_SampleTime_384Cycles);
  }
}

/**
  * @brief  Configure the DMA to read measurement from ADC
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void configureDMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	/**< Enable DMA1 clock */

  /*
   * DMA1 channel1 configuration
   */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	     	 	/**< Set DMA channel Peripheral base address to ADC Data register */
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValueBuff;  	/**< Set DMA channel Memeory base addr to ADC_ConvertedValueBuff address */
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                         	/**< Set DMA channel direction to peripheral to memory */
  DMA_InitStructure.DMA_BufferSize = MAX_TEMP_CHNL;                     	/**< Set DMA channel buffersize to peripheral to MAX_TEMP_CHNL */
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	         	/**< Disable DMA channel Peripheral address auto increment */
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    	/**< Enable Memeory increment (To be verified ....) */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	/**< set Peripheral data size to 8bit */
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	     	/**< set Memeory data size to 8bit */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                              	/**< Set DMA in Circular mode - auto reload */
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                     	/**< Set DMA channel priority to High */
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                               	/**< Disable memory to memory option */
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);								 	/**< Use Init structure to initialise channel1 (channel linked to ADC) */

  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);	/**< Enable Transmit Complete Interrup for DMA channel 1 */

  DMA_Cmd(DMA1_Channel1, ENABLE);  /**< Enable DMA channel1 */

  /*
   * Setup NVIC for DMA channel 1 interrupt request
   */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief  Check user calib data
  *
  * @note
  *
  * @param  None
  *
  * @retval FunctionalState: Status
  */
static FunctionalState  testUserCalibData(void)
{
  int32_t testdiff;
  FunctionalState retval = DISABLE;

  testdiff = USER_CALIB_DATA->TS_CAL_HOT - USER_CALIB_DATA->TS_CAL_COLD;

  if (testdiff > TEST_CALIB_DIFF)
	  {
	  	  retval = ENABLE;
	  }

  return retval;
}

/**
  * @brief  Check factory calib data
  *
  * @note
  *
  * @param  None
  *
  * @retval FunctionalState: Status
  */
static FunctionalState  testFactoryCalibData(void)
{
  int32_t testdiff;
  FunctionalState retval = DISABLE;

  testdiff = FACTORY_CALIB_DATA->TS_CAL_HOT - FACTORY_CALIB_DATA->TS_CAL_COLD;

  if (testdiff > TEST_CALIB_DIFF)
	  {
	  	  retval = ENABLE;
	  }

  return retval;
}
#endif
/**
  * @brief  Process the temperature acquired
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void processTempData(void)
{
  insertionSort(ADC_ConvertedValueBuff, MAX_TEMP_CHNL);	/**< sort received data in */

  tempAVG = interquartileMean(ADC_ConvertedValueBuff, MAX_TEMP_CHNL);	/**< Calculate the Interquartile mean */

  /*
   * Calculate temperature in °C from Interquartile mean
   */
  temperature_C = ( (int32_t) tempAVG - (int32_t) calibdata.TS_CAL_COLD ) ;
  temperature_C = temperature_C * (int32_t)(110 - 30);
  temperature_C = temperature_C /
                  (int32_t)(calibdata.TS_CAL_HOT - calibdata.TS_CAL_COLD);
  temperature_C = temperature_C + 30 + TEMP_CORRECTION_ERROR;
}

/**
  * @brief  Process data
  *
  * @note
  *
  * @param  numbers
  *
  * @param  array_size
  *
  * @retval None
  */
static void insertionSort(uint16_t *numbers, uint32_t array_size)
{
	uint32_t i, j;
	uint32_t index;

  for (i=1; i < array_size; i++)
  {
    index = numbers[i];
    j = i;
    while ((j > 0) && (numbers[j-1] > index))
    {
      numbers[j] = numbers[j-1];
      j = j - 1;
    }
    numbers[j] = index;
  }
}

/**
  * @brief  Process data
  *
  * @note
  *
  * @param  array
  *
  * @param  numOfSamples
  *
  * @retval None
  */
static uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples)
{
    uint32_t sum=0;
    uint32_t  index, maxindex;

    /*
     * discard  the lowest and the highest data samples
     */
	maxindex = 3 * numOfSamples / 4;
    for (index = (numOfSamples / 4); index < maxindex; index++)
    {
            sum += array[index];
    }

    return ( sum / (numOfSamples / 2) );	/**< return the mean value of the remaining samples value */
}

/**
  * @brief  Initialize RTC block
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void Init_RTC(void)
{
	/*
	 * Initialize the HW - 37Khz LSI being used
	 */

	RCC_LSICmd(ENABLE);									/**< Enable the LSI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);	/**< Enable power module clock */
	PWR_RTCAccessCmd(ENABLE);							/**< Enable acces to the RTC registers */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);				/**< Select LSI as RTC Input */
	RCC_RTCCLKCmd(ENABLE);								/**< Enable RTC */
	RTC_WriteProtectionCmd(DISABLE);					/**< Disable Write Protection */
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == 0);		/**< Wait for LSI to be stable */

	return;
}

/**
  * @brief  Function for resetting the nRF51 with IO PC7.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void connectivity_chip_reset(void)
{
	GPIO_InitTypeDef sGPIO_InitStruct;
	uint32_t uwTempRegister;
	uint8_t	ubnRFResetTimerID;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);	/**< Switch ON GPIO AHB clock */

	GPIOC->BSRRH = GPIO_Pin_7;	/**< Drive low the GPIO PC7 to reset nRF51 */

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

	/*
	 * Set IO pin in Analog mode
	 * Set IO speed to 400Khz
	 * The parameter from sGPIO_InitStruct.GPIO_Speed does not apply in Input mode
	 */
	uwTempRegister = GPIOC->OSPEEDR;
	uwTempRegister &= ~(GPIO_OSPEEDER_OSPEEDR0 << ((uint16_t)GPIO_PinSource7 * 2));
	uwTempRegister |= (((uint32_t)GPIO_Speed_400KHz) << ((uint16_t)GPIO_PinSource7 * 2));
	GPIOC->OSPEEDR = uwTempRegister;

	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOC,&sGPIO_InitStruct);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);	/**< Switch OFF GPIO AHB clock */

	/*
	 * Wait for the nRF device to be ready before sending any command
	 */
    HAL_TIMER_Start(ubnRFResetTimerID, CONN_CHIP_STARTUP_TIME);
    ubnRFresetTimerLock = 1;
    while(ubnRFresetTimerLock == 1);
    HAL_TIMER_Delete(ubnRFResetTimerID);

    return;
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  NVM Low power mode request
  *
  * @note
  *
  * @param  eHSclkMode: Mode requested
  *
  * @retval None
  */
void HAL_NVM_HSclkRequest(eHAL_NVM_HSclkMode_t eHSclkMode)
{
	if(eHSclkMode == eNVM_HSclkEnable)
	{
		LPM_Mode_Request(eLPM_NVM, eLPM_Mode_Sleep);
	}
	else
	{
		LPM_Mode_Request(eLPM_NVM, eLPM_Mode_StandBy);
	}

	return;
}

/**
  * @brief  UART Low power mode request
  *
  * @note
  *
  * @param  eHSclkRequester: Requester Id
  *
  * @param  eHSclkMode: Mode requested
  *
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

/**
  * @brief  Function for application main entry.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
int main(void)
{
	GPIO_InitTypeDef sGPIO_InitStruct;
#if(APP_L2CAP_TX_TEST == 1)
	uint8_t loop;
#endif
	/*
	 * The APB power block clock is enable during the startup phase
	 */

#if (JTAG_SUPPORTED == 1)
	/*
	 * Keep debugger enable while in any low power mode
	 */
	DBGMCU_Config(DBGMCU_SLEEP, ENABLE);
	DBGMCU_Config(DBGMCU_STOP, ENABLE);
	DBGMCU_Config(DBGMCU_STANDBY, ENABLE);
#endif

	PWR_ClearFlag(PWR_FLAG_SB);		/**< Clear Standby flag */
	RCC_ClearFlag();				/**< clear all reset flags */
	PWR_UltraLowPowerCmd(ENABLE);	/**< Enable Ultra low power mode */
	PWR_FastWakeUpCmd(ENABLE);		/**< Enable Fast Wakeup  */

/*******************************************************************************************/
/*
 * Setting all IOs as Analog mode with no pull
 */

	/**
	 * Switch ON GPIO AHB clock
	 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOD|RCC_AHBPeriph_GPIOE|RCC_AHBPeriph_GPIOF|RCC_AHBPeriph_GPIOG|RCC_AHBPeriph_GPIOH, ENABLE);

	/*
	 * Setup all GPIO except JTAG (PA15/14/13 - PB4/3 in Analog NoPull)
	 */
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

	/*
	 * Switch OFF GPIO AHB
	 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOD|RCC_AHBPeriph_GPIOE|RCC_AHBPeriph_GPIOF|RCC_AHBPeriph_GPIOG|RCC_AHBPeriph_GPIOH, DISABLE);
	
	/*
	 * Configure USER button
	 */
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	UserButtonHandlerRegister((pf_Main_UserButtonHandler_t)user_nvm_bond_delete);
	
#if (APP_LEDBUTTON == 1)
	led_init(); 		/**< initialize the LED */
	button_init();		/**< initialize the BUTTON */
#endif

	/*******************************************************************************************/

#if (APP_HTS == 1)
	/*
	 * Configure Temp Acquisition
	 */
	if(testUserCalibData() == ENABLE)
	{
		calibdata = *USER_CALIB_DATA;
	}
	else if(testFactoryCalibData() == ENABLE)
	{
		calibdata = *FACTORY_CALIB_DATA;
	}
	else
	{
		calibdata.TS_CAL_COLD = DEFAULT_COLD_VAL;
	    calibdata.TS_CAL_HOT = DEFAULT_HOT_VAL;
	    calibdata = *USER_CALIB_DATA;
	}

	configureDMA();			/**< Configure direct memory access for ADC usage */
	configureADC_Temp();	/**< Configure ADC for temperature sensor value conversion */
#endif
	/********************************************************************/





	LPM_Mode_Request(eLPM_Application, eLPM_Mode_LP_Stop);	/**< Define minimum low power mode allowed in the system */

	Init_RTC();

    HAL_TIMER_Init();

#if(APP_LCD == 1)	
    LCD_Init();
#endif	

    UART_BLE_Init();			/**< Request all resources for the UART communication with the nRF device */

	connectivity_chip_reset();	/**< Reset connectivity chip. */
	ble_app_main_init_Reset();	/**< Initialize connectivity chip. */

	LPM_Mode_Request(eLPM_Application, LOWEST_LOW_POWER_MODE);	/**< Define minimum low power mode allowed in the system */

#if(APP_L2CAP_TX_TEST == 1)
	/*
	 * Fill in payload buffer
	 */
	for(loop = 0; loop < L2CAP_PAYLOAD_LENGTH; loop++)
	{
		L2CAPPayloadBuffer[loop] = L2CAP_PAYLOAD_PATTERN;
	}

	/*
	 * Initialize L2CAP header
	 */
    l2cap_header.len = L2CAP_PAYLOAD_LENGTH;
	l2cap_header.cid = L2CAP_HEADER_CID;

	/*
	 * Get max buffer available for L2CAP data on nRF device
	 */
	 sd_ble_tx_buffer_count_get(&FreeBufferCount);
	 blocking_resp_wait();
#endif

	/*
	 * Enter main loop.
	 */
    for (;;)
    {
    	Background(SD_COMMAND_ALLOWED);
	}
}

/**
  * @brief  Request action to the scheduler in background
  *
  * @note
  *
  * @param  eMAIN_Backround_Task_Id: Id of the request
  *
  * @retval None
  */
void TaskExecutionRequest(eMAIN_Backround_Task_Id_t eMAIN_Backround_Task_Id)
{
	__disable_irq();
	TaskExecutionRequested |= (1 << eMAIN_Backround_Task_Id);
	__enable_irq();

	return;
}

/**
  * @brief  Notify the action in background has been completed
  *
  * @note
  *
  * @param  eMAIN_Backround_Task_Id: Id of the request
  *
  * @retval None
  */
void TaskExecuted(eMAIN_Backround_Task_Id_t eMAIN_Backround_Task_Id)
{
	__disable_irq();
	TaskExecutionRequested &= ~(1 << eMAIN_Backround_Task_Id);
	__enable_irq();

	return;
}

/**
  * @brief  Background task
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void Background (uint32_t event_mask)
{
	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_Hrs_Evt_Id))
	{
		TaskExecuted(eMAIN_Main_Hrs_Evt_Id);
		ble_hrs_evt_process();
	}

	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_HeartRate_Id))
	{
		TaskExecuted(eMAIN_Main_HeartRate_Id);
		heart_rate_meas_timeout_handler();
	}

	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_Battery_Id))
	{
		TaskExecuted(eMAIN_Main_Battery_Id);
		battery_level_update();
	}

	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_ConnUpdate_Id))
	{
		TaskExecuted(eMAIN_Main_ConnUpdate_Id);
		pfTimerBLECallBack();
	}

	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_ADC_Data_Processing_Id))
	{
		TaskExecuted(eMAIN_Main_ADC_Data_Processing_Id);
		ProcessADCdata();
	}
	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_GAP_Disc_Evt_Id))
	{
		TaskExecuted(eMAIN_Main_GAP_Disc_Evt_Id);
		ble_bondmngr_bonded_centrals_store();
    advertising_start();
	}
#if(APP_L2CAP_TX_TEST == 1)
	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_L2CAP_CID_Register_Id))
	{
		TaskExecuted(eMAIN_Main_L2CAP_CID_Register_Id);
		sd_ble_l2cap_cid_register(L2CAP_HEADER_CID);
		blocking_resp_wait();

        TaskExecutionRequest(eMAIN_Main_L2CAP_TX_Packet_Id);
	}

	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_L2CAP_TX_Packet_Id))
	{
		FreeBufferCount--;
		if(FreeBufferCount == 0)
		{
			TaskExecuted(eMAIN_Main_L2CAP_TX_Packet_Id);
		}

		sd_ble_l2cap_tx(GetConnectionHandle(), &l2cap_header , L2CAPPayloadBuffer);
		blocking_resp_wait();
	}
#endif
#if (APP_LEDBUTTON == 1)	
	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_Button1_Evt_Id))
	{
		TaskExecuted(eMAIN_Main_Button1_Evt_Id);
		ble_button_process((uint8_t)BUTTON_1);
	}
	
	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_Button2_Evt_Id))
	{
		TaskExecuted(eMAIN_Main_Button2_Evt_Id);
		ble_button_process((uint8_t)BUTTON_2);
	}

	if(TaskExecutionRequested & event_mask & (1<< eMAIN_Main_Button3_Evt_Id))
	{
		TaskExecuted(eMAIN_Main_Button3_Evt_Id);
		ble_button_process((uint8_t)BUTTON_3);
	}
#endif	
	/**
	 * Power management
	 */
	__disable_irq();
	if((TaskExecutionRequested & event_mask) == 0)
	{
		LPM_Enter_Mode();
	}
	__enable_irq();

        return;
}

/**
  * @brief  Actions to execute when going out of stop mode
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
USED void LPM_ExitStopMode(void)
{
	 RCC->CR |= ((uint32_t)RCC_CR_HSION);	/**< Switch back to HSI */
	 while((RCC->CR & RCC_CR_HSIRDY) == 0);	/**< Wait till HSI is ready  */

#if (AHB_32MHZ == 1)
	    RCC->CR |= RCC_CR_PLLON;				/**< Enable PLL */
	    while((RCC->CR & RCC_CR_PLLRDY) == 0);	/**< Wait till PLL is ready */
	    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;	/**< Select PLL as system clock source */
	    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);	/**< Wait till PLL is used as system clock source */
#else
	 RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;	/**< Select HSI as system clock source */
	 while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_HSI);	/**< Wait till HSI is used as system clock source */
#endif

	 return;
}

/**
  * @brief  Timer notification from the timer server
  *
  * @note	This is the location where the context may be changed to process the timer callback
  *
  * @param  eTimerModuleID: Identifier of the process owner of the callback
  *
  * @param  pfTimerCallBack: Callback to execute
  *
  * @retval None
  */
void HAL_TIMER_Notification(eHAL_TIMER_ModuleID_t eTimerModuleID, pf_HAL_TIMER_TimerCallBack_t pfTimerCallBack)
{
	switch(eTimerModuleID)
	{
		case eTimerModuleID_BLE:
			   pfTimerBLECallBack = pfTimerCallBack;
			   TaskExecutionRequest(eMAIN_Main_ConnUpdate_Id);
			   break;

		case eTimerModuleID_HeartRate:
			   TaskExecutionRequest(eMAIN_Main_HeartRate_Id);
			   break;

		case eTimerModuleID_Battery:
			   TaskExecutionRequest(eMAIN_Main_Battery_Id);
			   break;

		case eTimerModuleID_Interrupt:
			if(pfTimerCallBack != 0)
			{
				pfTimerCallBack();
			}
			break;

		default:
			if(pfTimerCallBack != 0)
			{
				pfTimerCallBack();
			}
			break;
	}

	return;
}
#if (APP_LEDBUTTON == 1)
/**
  * @brief  This function handles EXTI4_IRQHandler Handler.
  *
  * @note
  *
  *
  * @param  None
  *
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line4);	/**< Clear the EXTI line pending bit */
		TaskExecutionRequest(eMAIN_Main_Button3_Evt_Id);
	}


	return;
}

/**
  * @brief  This function handles EXTI9_5_IRQHandler Handler.
  *
  * @note
  *
  *
  * @param  None
  *
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line9) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);	/**< Clear the EXTI line pending bit */
		TaskExecutionRequest(eMAIN_Main_Button1_Evt_Id);
	}
	if(EXTI_GetITStatus(EXTI_Line8) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);	/**< Clear the EXTI line pending bit */
		TaskExecutionRequest(eMAIN_Main_Button2_Evt_Id);
	}

	return;
}
#endif
/**
  * @brief  This function handles EXTI4_15_IRQHandler Handler.
  *
  * @note
  *
  *
  * @param  None
  *
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line13) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line13);	/**< Clear the EXTI line pending bit */

		pf_UserButtonHandler();
	}
	else
	{
		#if ( (USART_CTS_PIN_SELECTED == PA11) || (USART_CTS_PIN_SELECTED == PB13) || (USART_CTS_PIN_SELECTED == PD11))
		HAL_UART_wcts_handler();
		#endif
	}

	return;
}

/**
  * @brief  This function handles all transfer over UART to the BLE module
  *
  * @note	This is the location where the context may be changed to process the message
  *
  * @param  pf_PhyDriverEvent_Handler: Callback
  *
  * @param  pPhyDriverEvent:	Type of notification
  *
  * @retval None
  */
void HAL_UART_Msg_Handler(pf_HAL_UART_PhyDriverEvent_Handler_t pf_PhyDriverEvent_Handler, sHAL_UART_PhyDriverEvent_t pPhyDriverEvent)
{
	pf_PhyDriverEvent_Handler(pPhyDriverEvent);

	return;
}

/**
  * @brief  This function handles the user button
  *
  * @note
  *
  * @param  pf_Main_UserButtonHandler: User button Callback
  *
  * @retval None
  */
void UserButtonHandlerRegister(pf_Main_UserButtonHandler_t pf_Main_UserButtonHandler)
{
	pf_UserButtonHandler = pf_Main_UserButtonHandler;

	return;
}

/**
  * @brief  This function handles DMA Transfer Complete interrupt request.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
  DMA_ClearFlag(DMA1_IT_TC1);
  powerDownADC_Temper();

  TaskExecutionRequest(eMAIN_Main_ADC_Data_Processing_Id);

  return;
}

/**
  * @brief  Return the temperature measured from the ADC
  *
  * @note
  *
  * @param  None
  *
  * @retval uint32_t: Temperature measured
  */
uint32_t getTemperatureValue(void)
{
	return temperature_C;
}

/**
  * @brief  Start acquisition from ADC
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void acquireTemperatureData(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);			/**< Enable ADC clock */

  PWR_UltraLowPowerCmd(DISABLE);	/**< Disable Ultra low power mode to keep VrefInt enable in low power mode */

  ADC_Cmd(ADC1, ENABLE);										/**< Enable ADC1 */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);		/**< Wait until the ADC1 is ready */

  ADC_DMACmd(ADC1, DISABLE);									/**< Disable DMA mode for ADC1 */
  ADC_DMACmd(ADC1, ENABLE);										/**< Enable DMA mode for ADC1 */

  /*
   *  Wait until ADC + Temp sensor start
   */
  while(PWR_GetFlagStatus(PWR_FLAG_VREFINTRDY) == RESET);

  ADC_SoftwareStartConv(ADC1);									/**< Start ADC conversion */

  LPM_Mode_Request(eLPM_ADC_Temp_Measurement, eLPM_Mode_Sleep);
}

#if (APP_LEDBUTTON == 1)
/**@brief Function  Led services.
 *
 * @param[in]   Led_signal_level  Requested ON/OFF led.
 */
void led_value(uint8_t led_value)
{
#if(APP_LCD == 1)
  LCD_SetTextColor(LCD_COLOR_BLUE);	/**< Set Text color */
  bmpcounter = 	led_value;
#endif

	if((led_value&LED1_VALUE)==LED1_VALUE )
	{
		wavetek_led_on(Wavetek_LED1_Id);
#if(APP_LCD == 1)
		LCD_DisplayStringLine(LCD_LINE_16, (uint8_t*)"   LED1 : ON   ");
#endif	
	}
	else
	{
		wavetek_led_off(Wavetek_LED1_Id);
#if(APP_LCD == 1)
		LCD_DisplayStringLine(LCD_LINE_16, (uint8_t*)"   LED1 : OFF    ");
#endif		
	}

	if((led_value&LED2_VALUE)==LED2_VALUE )
	{
		wavetek_led_on(Wavetek_LED2_Id);
#if(APP_LCD == 1)
		LCD_DisplayStringLine(LCD_LINE_17, (uint8_t*)"   LED2 : ON   ");
#endif		
	}
	else
	{
		wavetek_led_off(Wavetek_LED2_Id);
#if(APP_LCD == 1)
		LCD_DisplayStringLine(LCD_LINE_17, (uint8_t*)"   LED2 : OFF    ");
#endif		
	}

	if((led_value&LED3_VALUE)==LED3_VALUE )
	{
		wavetek_led_on(Wavetek_LED3_Id);
#if(APP_LCD == 1)
		LCD_DisplayStringLine(LCD_LINE_18, (uint8_t*)"   LED3 : ON   ");
#endif		
	}
	else
	{
		wavetek_led_off(Wavetek_LED3_Id);
#if(APP_LCD == 1)
		LCD_DisplayStringLine(LCD_LINE_18, (uint8_t*)"   LED3 : OFF   ");
#endif		
	}
}
#endif

#if(APP_L2CAP_TX_TEST == 1)
/**
  * @brief  Update buffer count
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void UpdateL2CAPBufferCount(uint8_t BufferCount)
{
	FreeBufferCount += BufferCount;

	return;
}
#endif

#if(APP_LCD == 1)

/**
  * @brief  Check the availability of adafruit 1.8" TFT shield on top of STM32NUCLEO
  *         board. This is done by reading the state of IO PB.00 pin (mapped to 
  *         JoyStick available on adafruit 1.8" TFT shield). If the state of PB.00 
  *         is high then the adafruit 1.8" TFT shield is available.
  * @note   This function should not be called after STM_ADC_Config(), otherwise 
  *         ADC conversion will not work.
  * @param  None
  * @retval SHIELD_DETECTED: 1.8" TFT shield is available
  *         SHIELD_NOT_DETECTED: 1.8" TFT shield is not available
  */
static ShieldStatus TFT_ShieldDetect(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);	/**< Enable GPIO clock */
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) != SHIELD_NOT_DETECTED)
  {
    return SHIELD_DETECTED;
  }
  else
  {
    return SHIELD_NOT_DETECTED;
  }
}

/**
  * @brief  Displays demonstration menu.
  * @param  None
  * @retval None
  */
static void Menu(void)
{
  LCD_SetTextColor(LCD_COLOR_BLUE);	/**< Set Text color */
  /*
   *  Display message
   */
  LCD_DisplayStringLine(LCD_LINE_15, (uint8_t*)" -------------- ");
  LCD_DisplayStringLine(LCD_LINE_16, (uint8_t*)" NUCLEO-L152RE  ");
  LCD_DisplayStringLine(LCD_LINE_17, (uint8_t*)"     DEMO       ");
  LCD_DisplayStringLine(LCD_LINE_18, (uint8_t*)" -------------- ");
}

/**
  * @brief  SD Card Configuration.
  * @param  None
  * @retval None
  */
static void SDCard_Config(void)
{
  uint32_t counter = 0;
  
  /*
   * Initialize the SD mounted on adafruit 1.8" TFT shield.
   * Before to call this function, the application code must call STM_SPI_Init()
   * to initialize the SPI Interface used to drive the SD.
   */
  SD_ADAFRUIT_Init();
  
  /*
   * Check the mounted device
   */
  if(f_mount(&microSDFatFs, (TCHAR const*)"/", 0) != FR_OK)
  {
    TFT_DisplayErrorMessage(FATFS_NOT_MOUNTED);
  }  
  else
  {
    /*
     * Initialize the Directory Files pointers (heap)
     */
    for (counter = 0; counter < MAX_BMP_FILES; counter++)
    {
      pDirectoryFiles[counter] = malloc(11); 
    }
  }
}

/**
  * @brief  Displays on TFT Images or error messages when error occurred.
  * @param  None
  * @retval None
  */
static void Display_Images(void)
{    
  uint32_t bmplen = 0x00;
  uint32_t checkstatus = 0x00;
  
  
  if(SD_GetStatus() != 0xFF)  
  {
    TFT_DisplayErrorMessage(SD_CARD_NOT_FOUND);	/**< Display message: SD card does not exist */
  }
  else
  {   
    /*
     * Open directory
     */
    res= f_opendir(&directory, "/");
    if((res != FR_OK))
    {
      if(res == FR_NO_FILESYSTEM)
      {
        TFT_DisplayErrorMessage(SD_CARD_NOT_FORMATTED);	/**< Display message: SD card not FAT formated */
      }
      else
      {
        TFT_DisplayErrorMessage(SD_CARD_OPEN_FAIL);	/**< Display message: Fail to open directory */
      }
    }
    
    filesnumbers = Storage_GetDirectoryBitmapFiles ("/", pDirectoryFiles);	/**< Get number of bitmap files */
    	
	sprintf((char*)str, "%-11.11s", pDirectoryFiles[bmpcounter]);
        
    checkstatus = Storage_CheckBitmapFile((const char*)str, &bmplen);
        
    if(checkstatus == 0)
    {
    	Storage_OpenReadFile(127, 159, (const char*)str);	/**< Format the string */
    }
    else if (checkstatus == 1)
    {
    	TFT_DisplayErrorMessage(SD_CARD_NOT_FOUND);	/**< Display message: SD card does not exist */
    }
  }
}
/**
  * @brief  Displays adequate message on TFT available on adafruit 1.8" TFT shield  
  * @param  message: Error message to be displayed on the LCD.
  *   This parameter can be one of following values:   
  *     @arg SD_CARD_NOT_FOUND: SD CARD is unplugged
  *     @arg SD_CARD_NOT_FORMATED: SD CARD is not FAT formatted
  *     @arg SD_CARD_FILE_NOT_SUPPORTED: File is not supported
  *     @arg SD_CARD_OPEN_FAIL: Failure to open directory
  *     @arg FATFS_NOT_MOUNTED: FatFs is not mounted
  * @retval None
  */
static void TFT_DisplayErrorMessage(uint8_t message)
{
  LCD_Clear(LCD_COLOR_WHITE);		/**< LCD Clear */
  LCD_SetBackColor(LCD_COLOR_GREY);	/**< Set Back color */
  LCD_SetTextColor(LCD_COLOR_RED);	/**< Set Text color */
  
  if(message == SD_CARD_NOT_FOUND)
  {
    /*
     * Display message
     */
    LCD_DisplayStringLine(LCD_LINE_8, (uint8_t*)"Please insert     ");
    LCD_DisplayStringLine(LCD_LINE_9, (uint8_t*)"                  "); 
    LCD_DisplayStringLine(LCD_LINE_10, (uint8_t*)"an SD card and   ");
    LCD_DisplayStringLine(LCD_LINE_11, (uint8_t*)"                 "); 
    LCD_DisplayStringLine(LCD_LINE_12, (uint8_t*)"reset the board  "); 
    while(1)
    {
    } 
  }
  if(message == SD_CARD_NOT_FORMATTED)
  {
    /*
     * Display message
     */
    LCD_DisplayStringLine(LCD_LINE_7, (uint8_t*)"SD Card is not     ");
    LCD_DisplayStringLine(LCD_LINE_8, (uint8_t*)"                   "); 
    LCD_DisplayStringLine(LCD_LINE_9, (uint8_t*)" FAT formatted     ");
    LCD_DisplayStringLine(LCD_LINE_10, (uint8_t*)"                  ");   
    LCD_DisplayStringLine(LCD_LINE_11, (uint8_t*)" Please Format    ");
    LCD_DisplayStringLine(LCD_LINE_12, (uint8_t*)"                  "); 
    LCD_DisplayStringLine(LCD_LINE_13, (uint8_t*)"   the card       ");
    while (1)
    {
    }    
  }
  if(message == SD_CARD_FILE_NOT_SUPPORTED)
  {
    /*
     * Display message
     */
    LCD_DisplayStringLine(LCD_LINE_8, (uint8_t*)"File type not     ");
    LCD_DisplayStringLine(LCD_LINE_9, (uint8_t*)"                  ");
    LCD_DisplayStringLine(LCD_LINE_10, (uint8_t*)"supported        ");
    while(1)
    {
    }    
  }
  if(message == SD_CARD_OPEN_FAIL)
  {
    /*
     * Display message
     */
    LCD_DisplayStringLine(LCD_LINE_8, (uint8_t*)"Open directory    ");
    LCD_DisplayStringLine(LCD_LINE_9, (uint8_t*)"                  "); 
    LCD_DisplayStringLine(LCD_LINE_10, (uint8_t*)"     fails       ");
    while(1)
    {
    }     
  }
  if(message == FATFS_NOT_MOUNTED)
  {
    /*
     * Display message
     */
    LCD_DisplayStringLine(LCD_LINE_8, (uint8_t*)"Cannot mount FatFs");
    LCD_DisplayStringLine(LCD_LINE_9, (uint8_t*)"                  "); 
    LCD_DisplayStringLine(LCD_LINE_10, (uint8_t*)" on Drive        "); 
    while (1)
    {
    }    
  }
}

static void LCD_Init(void)
{
	RCC_ClocksTypeDef RCC_Clocks;

	RCC_AHBPeriphClockCmd(SD_CS_GPIO_CLK, ENABLE);	/**< Enable GPIO clock used for CS */

	/*
     * SysTick end of count event each 1ms
     */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

    /*
     * Check the availability of adafruit 1.8" TFT shield on top of STM32NUCLEO
     * board. This is done by reading the state of IO PB.00 pin (mapped to JoyStick
     * available on adafruit 1.8" TFT shield). If the state of PB.00 is high then
     * the adafruit 1.8" TFT shield is available.
     */
    if(TFT_ShieldDetect() == SHIELD_DETECTED)
    {
    	/*
    	 *  Initialize SPI Interface to drive the LCD and uSD card available on
    	 * 	adafruit 1.8" TFT shield. Must be done before LCD and uSD initialization
    	 */
    	STM_SPI_Init();

    	LCD_ADAFRUIT_Init();	/**< Initialize LCD mounted on adafruit 1.8" TFT shield */
		Menu();
		SDCard_Config();		/**< Configure SD card */
		Display_Images();		/**< Display on TFT Images existing on SD card */
    }

	return;
}

#endif

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 1 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if(TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
