/**
  ******************************************************************************
  * @file RTC/inc/menu.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file consists of the function prtotypes for menu implementation
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
#ifndef __MENU_H
#define __MENU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/* Exported types ------------------------------------------------------------*/
extern __IO uint32_t TimingDelay;
extern uint8_t TimeDateDisplay;
extern uint8_t  DisplayTimeFlag;
extern uint32_t Dummy;
extern uint8_t DisplayDateFlag;
extern uint8_t  AlarmStatus;
extern uint8_t TotalMenuPointer;
extern uint8_t FlagKey;
extern uint8_t MenuLevelPointer;
extern uint8_t TamperEvent;
extern uint8_t TamperNumber;
extern float f32_Frequency;
extern uint8_t BatteryRemoved;
extern uint16_t SummerTimeCorrect;
extern uint8_t AlarmDate;
extern uint32_t TimerFrequency;
extern uint8_t ArrayTime[8];

/* Exported constants --------------------------------------------------------*/
#define Column0        320
#define Column1        304
#define Column2        288
#define Column3        272
#define Column4        256
#define Column5        240
#define Column6        224
#define Column7        208
#define Column8        192
#define Column9        176
#define Column10       160
#define Column11       144
#define Column12       128
#define Column13       112
#define Column14        96
#define Column15        80
#define Column16        64
#define Column17        48
#define Column18        32
#define Column19        16

#define NOKEY 0x01
#define LEFT 0x02
#define RIGHT 0x03
#define SEL 0x04
#define UP 0x05
#define DOWN 0x06
#define ASCII_COLON 58
#define ASCII_SLASH 47

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void UpFunction(void);
void DownFunction(void);
void LeftFunction(void);
void RightFunction(void);
void SelFunction(void);
void MenuInit(void);
void SelIntExtOnOffConfig(FunctionalState);
void UpDownIntOnOffConfig(FunctionalState NewState);
void RightLeftIntExtOnOffConfig(FunctionalState NewState);
void NVIC_JoyStickConfig(void);
void MenuManage(void);
void InitialiseTime(void);
void InitialiseDate(void);
uint8_t ReadKey(void);
void Delay(uint32_t nCount);

#endif   /*__MENU_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
