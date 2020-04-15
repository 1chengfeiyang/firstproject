/**
  ******************************************************************************
  * @file Standby/src/main.c 
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


/** @addtogroup Standby
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WKUP_20ms
//#define WKUP_200ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef   ADC_InitStructure;
uint16_t RegularConvData;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void RTC_Configuration(void);
void ADC_Configuration(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{

  /* System Clocks Configuration */
  /* Keep the default configuration HSI 8MHz */

  /* GPIO configuration */
  GPIO_Configuration();

  /* Turn on led connected to PC.01 */
  GPIO_SetBits(GPIOC, GPIO_Pin_1); 

  /* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Configure RTC clock source and prescaler */
  RTC_Configuration();

  /* Wait till RTC Second event occurs */
  RTC_ClearFlag(RTC_FLAG_SEC);
 
  while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

    #ifdef WKUP_20ms
    /* Set the RTC Alarm after 20 ms ( = 327 * 61µs) */
    RTC_SetAlarm(RTC_GetCounter()+ 327);
    #else 
    #ifdef WKUP_200ms
      /* Set the RTC Alarm after 200 ms ( = 3276 * 61µs) */
      RTC_SetAlarm(RTC_GetCounter()+ 3276);
    #endif
    #endif


  /* Configure ADC conversion mode */
  ADC_Configuration();
  /* ADC1 regular Software Start Conv */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* Test EOC flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  /* Read regular converted data and clear EOC Flag */
  RegularConvData = ADC_GetConversionValue(ADC1); 

  ADC_Cmd(ADC1, DISABLE);

  /* Turn on led connected to PC.01 */
  GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
  PWR_EnterSTANDBYMode();

  while(1)
  {
  }
}



/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


  /* Enable GPIOC, GPIOB and AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /* Configure PC.00 as Output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}



/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval : None
  */
void RTC_Configuration(void)
{
  /* Check if the StandBy flag is set */
  if(PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
  {/* System resumed from STANDBY mode */

    /* Clear StandBy flag */
    PWR_ClearFlag(PWR_FLAG_SB);

    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();
    /* No need to configure the RTC as the RTC configuration(clock source, enable,
       prescaler,...) is kept after wake-up from STANDBY */
	
  }
  else
  {/* StandBy flag is not set */

    /* RTC clock source configuration ----------------------------------------*/
    /* Reset Backup Domain */
    BKP_DeInit();
  
    /* Enable LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* RTC configuration -----------------------------------------------------*/
    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();

    /* Set the RTC time base to ~61 us ( = 2/32768) */
    RTC_SetPrescaler(1);  
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
  }
}



/**
  * @brief  Configures ADC conversion mode.
  * @param  None
  * @retval : None
  */
void ADC_Configuration(void)
{
  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* Set the ADC Clock Divider */
  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
									
  /* Reset variables */
  RegularConvData=0;

  /* ADC1 Init */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 Regular Channel Config */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_1Cycles5);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* ADC1 calibaration start */
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1)); 
}

#ifdef  USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
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
