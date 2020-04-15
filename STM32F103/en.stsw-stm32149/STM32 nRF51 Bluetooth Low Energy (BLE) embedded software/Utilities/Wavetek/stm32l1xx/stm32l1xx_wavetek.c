/**
  ******************************************************************************
  * @file    stm32l1xx_wavetek.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   This file provides set of firmware functions to manage Leds and
  *          push-button available on the wavetek board
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
#include "stm32l1xx_wavetek.h"

/* Private defines -----------------------------------------------------------*/
#define LED_number		3
#define LED1_GPIO_PORT	GPIOB
#define LED2_GPIO_PORT	GPIOB
#define LED3_GPIO_PORT	GPIOA
#define LED1_PIN		GPIO_Pin_5
#define LED2_PIN		GPIO_Pin_3
#define LED3_PIN		GPIO_Pin_10

/* Private variables ---------------------------------------------------------*/
GPIO_TypeDef* WAVETEK_GPIO_PORT[LED_number] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT};
const uint16_t WAVETEK_GPIO_PIN[LED_number] = {LED1_PIN, LED2_PIN, LED3_PIN};


/* Public functions ---------------------------------------------------------*/
/**
  * @brief  Function for initializing the nRF51 LEDs 1 2 3 .
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void led_init(void)
{
	GPIO_InitTypeDef sGPIO_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	/**< Switch ON GPIO AHB clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/*
	 * switch OFF The LEDs
	 */
	wavetek_led_off(Wavetek_LED1_Id);
	wavetek_led_off(Wavetek_LED2_Id);
	wavetek_led_off(Wavetek_LED3_Id);

	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	sGPIO_InitStruct.GPIO_Pin = WAVETEK_GPIO_PIN[Wavetek_LED1_Id];
	GPIO_Init(WAVETEK_GPIO_PORT[Wavetek_LED1_Id],&sGPIO_InitStruct);

	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	sGPIO_InitStruct.GPIO_Pin = WAVETEK_GPIO_PIN[Wavetek_LED2_Id];
	GPIO_Init(WAVETEK_GPIO_PORT[Wavetek_LED2_Id],&sGPIO_InitStruct);

	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	sGPIO_InitStruct.GPIO_Pin = WAVETEK_GPIO_PIN[Wavetek_LED3_Id];
	GPIO_Init(WAVETEK_GPIO_PORT[Wavetek_LED3_Id],&sGPIO_InitStruct);

	return;
}

/**
  * @brief  Function for initializing the nRF51 buttons 1 2 3 .
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void button_init(void)
{
	GPIO_InitTypeDef sGPIO_InitStruct;
	EXTI_InitTypeDef sEXTI_InitStruct;

	/*******************************************************************************************/
	/*
	 * USER BUTTON WAVETEK SHIELD SW1 --> D8 -> PA9
	 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	/**< Switch ON SYSCFG APB Clock */

	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_400KHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	sGPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;

	GPIO_Init(GPIOA,&sGPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource9);
	sEXTI_InitStruct.EXTI_Line = EXTI_Line9;
	sEXTI_InitStruct.EXTI_LineCmd = ENABLE;
	sEXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	sEXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_Init(&sEXTI_InitStruct);

	/*******************************************************************************************/
	/*
	 * USER BUTTON WAVETEK SHIELD SW2 --> D7 -> PA8
	 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	/**< Switch ON SYSCFG APB Clock */

	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_400KHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	sGPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;

	GPIO_Init(GPIOA,&sGPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

	sEXTI_InitStruct.EXTI_Line = EXTI_Line8;
	sEXTI_InitStruct.EXTI_LineCmd = ENABLE;
	sEXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	sEXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_Init(&sEXTI_InitStruct);

	NVIC_SetPriority(EXTI9_5_IRQn, 4);	/**< Set NVIC priority */
	NVIC_EnableIRQ(EXTI9_5_IRQn);			/**< Enable NVIC interrupt */

	/*******************************************************************************************/
	/*
	 * USER BUTTON WAVETEK SHIELD SW3 --> D5 -> PB4
	 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	/**< Switch ON SYSCFG APB Clock */

	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_400KHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	sGPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;

	GPIO_Init(GPIOB,&sGPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);

	sEXTI_InitStruct.EXTI_Line = EXTI_Line4;
	sEXTI_InitStruct.EXTI_LineCmd = ENABLE;
	sEXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	sEXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_Init(&sEXTI_InitStruct);

	NVIC_SetPriority(EXTI4_IRQn, 4);	/**< Set NVIC priority */
	NVIC_EnableIRQ(EXTI4_IRQn);			/**< Enable NVIC interrupt */

	return;
}

void wavetek_led_off(eWavetek_Led_TypeDef_t Led_id)
{
	GPIO_SetBits(WAVETEK_GPIO_PORT[Led_id], WAVETEK_GPIO_PIN[Led_id]);

	return;
}

void wavetek_led_on(eWavetek_Led_TypeDef_t Led_id)
{
	GPIO_ResetBits(WAVETEK_GPIO_PORT[Led_id], WAVETEK_GPIO_PIN[Led_id]);

	return;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
