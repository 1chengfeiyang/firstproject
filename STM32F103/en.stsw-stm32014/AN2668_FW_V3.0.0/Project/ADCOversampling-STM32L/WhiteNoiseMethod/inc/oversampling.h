/**
  ******************************************************************************
  * @file WhiteNoiseMethod/inc/Oversampling.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains all the functions prototypes for the
*                      ADC oversampling of white noise method.
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
#ifndef __OVERSAMPLING_H
#define __OVERSAMPLING_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32l1xx.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
//#define Thermal_Noise_Measure
#define Oversampling_Test

#define ADC_Additional_Bits 2
#define Input_Signal_Sampling_Period 50 /* Input Signal sampling period in µs */ 

#define ADC_Oversampling_Factor  16 /*pow(4, ADC_Additional_Bits)*/

#ifdef Thermal_Noise_Measure
#define Total_Samples_Number 5000
#define Relevant_Samples_Number  100
#endif
#ifdef Oversampling_Test
#define Oversampling_Test_Samples 100
#endif 

#ifdef Thermal_Noise_Measure
void Oversampling_TestADCThermalNoise( __IO uint16_t *ADC_Code, __IO uint16_t *ADC_Code_Number );
#endif 
void Oversampling_InitData( void );
void Oversampling_CopyConvValuesToFile ( __IO uint16_t *Tab, uint32_t size );
uint16_t Oversampling_GetConversion( void );

#endif   /* __OVERSAMPLING_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
