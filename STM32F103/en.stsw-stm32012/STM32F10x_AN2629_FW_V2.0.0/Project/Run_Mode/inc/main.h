/**
  ******************************************************************************
  * @file Run_Mode/inc/main.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*------------------------------------------------------ */
/* 1st STEP -------------------------------------------- */
					 
/* APB1&APB2 frequency selection  ---------------------- */
/* 1st configuration */
#define ABP1_DIV4
#define ABP2_DIV2

/* 2nd configuration */
//#define ABP1_DIV8
//#define ABP2_DIV8
/*------------------------------------------------------ */

/*------------------------------------------------------ */
/* 2nd STEP -------------------------------------------- */
/* Clock gating : Peripherials selection --------------- */
#define ALL_PERIPHERIALS_ENABLE
//#define UART_ONLY

/*------------------------------------------------------ */

/*------------------------------------------------------ */
/* 3th STEP -------------------------------------------- */
/* Frequency selection --------------------------------- */
#define HCLK_FREQ_72MHz	/* Only with external oscillator */
//#define HCLK_FREQ_8MHz
/*------------------------------------------------------ */

/*------------------------------------------------------ */
/* 4th STEP -------------------------------------------- */
/* Using of PrefetchBuffer ----------------------------- */
#define PREFETCH_ON 
/* Using of Half cycle configuration ------------------- */
//#define HALF_CYCLE_ON
/*------------------------------------------------------ */

/* 5th STEP -------------------------------------------- */
/* Using of wait for interrupt ------------------------- */
//#define WFI_ON 
/*------------------------------------------------------ */

/*------------------------------------------------------ */
/* 6th STEP -------------------------------------------- */
/* External or Internal oscillator selection ----------- */
//#define HSI_ENABLE 	/* 	Use internal oscillator      */
#define HSE_ENABLE 		/*	Use external oscillator      */
/*------------------------------------------------------ */

/* Exported macro ------------------------------------------------------------*/

#ifdef HSI_ENABLE
	#ifdef HCLK_FREQ_72MHz
		#error HCLK max frequency in HSI mode is 64Mhz please change HCLK freq     		
	#endif
	#ifdef HSE_ENABLE
		#error HSE and HSI can't be enabled both
	#endif
#endif

#ifdef PREFETECH_ON
	#ifdef HALF_CYCLE_ON
		#error Prefetech buffer and half cycle mode can't  be enabled both
	#endif
#endif


/* Exported functions ------------------------------------------------------- */


#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/








