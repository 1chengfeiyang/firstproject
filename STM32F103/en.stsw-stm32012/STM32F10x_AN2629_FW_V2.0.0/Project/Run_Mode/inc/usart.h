/**
  ******************************************************************************
  * @file Run_Mode/inc/usart.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains all the functions prototypes for the
*           USART firmware library
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
#ifndef __USART_H
#define __USART_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Declare TestFunc as pointer to function returning TestStatus */
/* Declare string as character pointer */
/* Declare a structure which contains the test name and result */
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void GPIO_ConfigUSART(void);
void USART_Configuration(void);
//int fputc(int ch, FILE *f);
uint8_t USART_Scanf(uint32_t value);


#endif /* __USART_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
