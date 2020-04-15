/**
  ******************************************************************************
  * @file    InfraRed/IR_Decoding_PWMI/src/main.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ir_decode.h"
#include <stdio.h>
#include "stm32100e_eval_lcd.h"

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup IR_Decoding_PWMI
  * @{
  */
  
/** @addtogroup  MAIN
  * @brief driver modules
  * @{
  */

/** @defgroup MAIN_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup MAIN_Private_Defines
  * @{
  */
/**
  * @}  
  */

/** @defgroup MAIN_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup MAIN_Private_Variables
  * @{
  */
IR_Frame_TypeDef IR_FRAME;
/**
  * @}
  */

/** @defgroup MAIN_Private_FunctionPrototypes
  * @{
  */
/**
  * @}
  */

/** @defgroup MAIN_Private_Functions
  * @{
  */
      
/**
  * @brief  main program.
  * @param  None
  * @retval None
  */

int32_t main(void)
{ 
  /* Initialize the Color LCD mounted on STM32100E-EVAL */
  STM32100E_LCD_Init();
  
  /* Clear the LCD */ 
  LCD_Clear(LCD_COLOR_WHITE);

#if defined IR_RC5_PROTOCOL
  IR_RC5_Init();
#else
  IR_Init();
#endif 
  
  while(1)
  { 
#if defined (IR_RC5_PROTOCOL)
   /* RC5 */
   IR_RC5_Decode(&IR_FRAME);
#else
   IR_Decode(&IR_FRAME);
#endif
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
