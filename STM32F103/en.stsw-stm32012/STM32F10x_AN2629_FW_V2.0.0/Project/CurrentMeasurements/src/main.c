/**
  ******************************************************************************
  * @file CurrentMeasurements/src/main.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main program body.
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


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "hw_config.h"


/** @addtogroup CurrentMeasurements
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Define the Low power mode*/
#define SLEEP
//#define SLEEP_ALLPERIPH_ENABLE
#define SLEEP_ALLPERIPH_DISABLE

//#define STOP
//#define STOP_Regulator_ON
//#define STOP_Regulator_LowPower

//#define STANDBY
//#define RTC_ON

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
  /* Clock configuration */
  RCC_Configuration();

  /* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  /* NVIC configuration */
  NVIC_Configuration();
  
  GPIO_Config_ALL_AIN();
  
/********************************* SLEEP MODE *********************************/
#ifdef SLEEP 
  
  /* Config the EXTI to wake up from SLEEP Mode */
  EXTI_Configuration();
  
  #ifdef SLEEP_ALLPERIPH_ENABLE
    All_PeriphClock_Enable();
  #endif
    
  #ifdef SLEEP_ALLPERIPH_DISABLE
    All_PeriphClock_Disable();
  #endif
  
  #ifdef Entry_WFE
  /* Mode: SLEEP + Entry with WFE*/
  __WFE(); 
  #endif 
   
  #ifdef Entry_WFI
  /* Mode: SLEEP + Entry with WFI*/
  __WFI(); 
  #endif
  
#endif /* End of SLEEP test */
  
/********************************* STOP MODE **********************************/  
#ifdef STOP
  
  /* Config the GPIO on Analog Input mode */
  GPIO_Config_ALL_AIN();
  
  /* Config the EXTI to wake up from STOP Mode */
  EXTI_Configuration();
  
  /* Desable the SRAM and FLITF clock in Stop mode */ 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SRAM|RCC_AHBPeriph_FLITF, DISABLE);
    
  /* Request to enter STOP mode with regulator ON */
  #ifdef STOP_Regulator_ON
  
    #ifdef Entry_WFE
    /* Mode: STOP + Regulator in ON + Entry with WFE */
    PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE); 
    #endif 
   
    #ifdef Entry_WFI
    /* Mode: STOP + Regulator in ON + Entry with WFI */
    PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI); 
    #endif 
  
  #endif /* STOP Mode with regulator ON */
  
  /* Request to enter STOP mode with regulator in low power mode */
  #ifdef STOP_Regulator_LowPower
  
    #ifdef Entry_WFE
    /* Mode: STOP + Regulator in low power mode + Entry with WFE */ 
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);
    #endif
  
    #ifdef Entry_WFI
    /* Mode: STOP + Regulator in low power mode + Entry with WFI */ 
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    #endif  
  
   #endif /* Stop Mode with Regulator in low power mode */

#endif /* End of STOP test */   

    
/****************************** STANDBY MODE **********************************/    
#ifdef STANDBY
    
  /* Enable WKUP pin */
  PWR_WakeUpPinCmd(ENABLE);  
  
  /* GPIO configuration for the LED 1, 2, 3 and 4 */ 
  GPIO_Configuration();
  
  /* Turn ON the four LEDs */
  GPIO_Write(GPIOC, (uint16_t)(GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9));
  
  /* Insert a delay */
  Delay(0xFFFFFF);
 
  #ifdef RTC_ON
  /* Config the RTC */
  RTC_Configuration();
  #endif
  
  /* Request to enter STANDBY mode */
  PWR_EnterSTANDBYMode();
  
#endif  /* End of STANDBY test */
  
  while(1)
  {
    /* Infinite loop*/
  }

}


/**
  * @brief  Inserts a delay time.
  * @param nCount: specifies the delay time length.
  * @retval : None
  */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  *  where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
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
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
