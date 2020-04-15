/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @version V2.0
  * @date    29-August-2014
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
	 
/* Exported constants --------------------------------------------------------*/
#define  MAX_BMP_FILES  25 

 /**
  * Number of event that would require sending SD command
  */
#define	EVENT_REQUIRING_SENDING_SD_COMMAND	11

 /**
  * Bit mapping of events
  * There definition below shall not be changed
  */
#define	SD_COMMAND_NOT_ALLOWED	(uint32_t)(~((1<<EVENT_REQUIRING_SENDING_SD_COMMAND)-1))
#define	SD_COMMAND_ALLOWED	(uint32_t)0xFFFFFFFF

/* Exported types ------------------------------------------------------------*/

 /**
  * The enum list shall be in line with the definition of EVENT_REQUIRING_SENDING_SD_COMMAND
  * The first enum which value is below EVENT_REQUIRING_SENDING_SD_COMMAND are allowed to send SD command
  * The enum above or equal to the value of EVENT_REQUIRING_SENDING_SD_COMMAND shall not sent SD command
  *
  * The current firmware implementation supports up to 32 events
  */
 typedef enum
 {
 	eMAIN_Main_HeartRate_Id,
 	eMAIN_Main_Battery_Id,
 	eMAIN_Main_ConnUpdate_Id,
 	eMAIN_Main_Hrs_Evt_Id,
 	eMAIN_Main_ADC_Data_Processing_Id,
 	eMAIN_Main_L2CAP_CID_Register_Id,
 	eMAIN_Main_L2CAP_TX_Packet_Id,
	eMAIN_Main_GAP_Disc_Evt_Id,
	eMAIN_Main_Button1_Evt_Id,
	eMAIN_Main_Button2_Evt_Id,
  eMAIN_Main_Button3_Evt_Id,	 
 	eMAIN_Main_SD_Command_Resp = EVENT_REQUIRING_SENDING_SD_COMMAND
 } eMAIN_Backround_Task_Id_t;

 typedef void (*pf_Main_UserButtonHandler_t)(void);

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
 
void TaskExecutionRequest(eMAIN_Backround_Task_Id_t eMAIN_Backround_Task_Id);
void TaskExecuted(eMAIN_Backround_Task_Id_t eMAIN_Backround_Task_Id);
void UserButtonHandlerRegister(pf_Main_UserButtonHandler_t pf_Main_UserButtonHandler);
void Background(uint32_t event_mask);
void acquireTemperatureData(void);
void led_value(uint8_t led_value);
uint32_t getTemperatureValue(void);

#if(APP_L2CAP_TX_TEST == 1)
void UpdateL2CAPBufferCount(uint8_t BufferCount);
#endif


 int main(void);


#ifdef __cplusplus
}
#endif

#endif /*__MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
