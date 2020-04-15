/**
  ******************************************************************************
  * @file WFE_Stop_Flash/src/main.c 
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


/** @addtogroup WFE_Stop_Flash
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define LOOP_20ms
#define LOOP_200ms

//#define WFE
#define STOP_WFE

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t TimingDelay;
ErrorStatus HSEStartUpStatus;
uint16_t RegularConvData;
ADC_InitTypeDef   ADC_InitStructure;
uint16_t RegularConvData;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void RTC_Configuration(void);
void ADC_Configuration(void);
void Delay(__IO uint32_t nTime);

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

  /* GPIO configuration */
  GPIO_Configuration();

  /* Configure EXTI Line9 to generate an interrupt on falling edge */
  EXTI_Configuration();

  /* Configure RTC clock source and prescaler */
  RTC_Configuration();

  /* Configure ADC conversion mode */
  ADC_Configuration();

  RTC_ClearFlag(RTC_FLAG_SEC);
  

  while(1)
  {
  
    while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

    #ifdef LOOP_20ms
    /* Set the RTC Alarm after 20 ms ( = 327 * 61µs) */
    RTC_SetAlarm(RTC_GetCounter()+ 327);
    #else 
    #ifdef LOOP_200ms
      /* Set the RTC Alarm after 200 ms ( = 3276 * 61µs) */
      RTC_SetAlarm(RTC_GetCounter()+ 3276);
    #endif
    #endif
    
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Turn off led connected to PC.01 */
    GPIO_ResetBits(GPIOC, GPIO_Pin_1);
   
     #ifdef WFE
     __WFE(); 
     #else 
     #ifdef STOP_WFE

     /* Request to enter STOP mode with regulator ON */
  //PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE);

    /* Request to enter STOP mode with regulator in Low Power */
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);
    #endif
    #endif

    /* Wait till RTC Second event occurs */
    RTC_ClearFlag(RTC_FLAG_SEC);

    /* At this stage the system has resumed from STOP mode -------------------*/
    /* Turn on led connected to PC.01 */
    GPIO_SetBits(GPIOC, GPIO_Pin_1);

    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* Test EOC flag */
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

  /* Read regular converted data and clear EOC Flag */
    RegularConvData = ADC_GetConversionValue(ADC1);
    ADC_Cmd(ADC1, DISABLE); 

  }
}



/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/   
  /* RCC system reset(for debug purpose) */
//    RCC_DeInit();

  /* Flash 0 wait state */
  FLASH_SetLatency(FLASH_Latency_0);

  /* Enable Prefetch Buffer */
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

  /* HCLK = SYSCLK */
  RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
  /* PCLK2 = HCLK */
  RCC_PCLK2Config(RCC_HCLK_Div1); 

  /* PCLK1 = HCLK/2 */
  RCC_PCLK1Config(RCC_HCLK_Div2);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOs Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* GPIOs Periph clock disable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, DISABLE);

  /* Enable GPIOC, GPIOB and AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |	RCC_APB2Periph_AFIO, ENABLE);

  /* Configure PC.00 as Output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}


/**
  * @brief  Configures EXTI Line17(RTC Alarm).
  * @param  None
  * @retval : None
  */
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd =ENABLE;
  EXTI_Init(&EXTI_InitStructure);    
}


/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval : None
  */
void RTC_Configuration(void)
{
  /* RTC clock source configuration ------------------------------------------*/
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset Backup Domain */
  BKP_DeInit();
  
  /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* RTC configuration -------------------------------------------------------*/
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Set the RTC time base to ~61 us ( = 2/32768) */
  RTC_SetPrescaler(1);
    
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Enable the RTC Alarm interrupt */
  RTC_ITConfig(RTC_IT_ALR, ENABLE);
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
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
  RegularConvData = 0;

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

  /* ADC1 regular Software Start Conv */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* Test EOC flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  /* Read regular converted data and clear EOC Flag */
  RegularConvData = ADC_GetConversionValue(ADC1); 

  ADC_Cmd(ADC1, DISABLE);
}

#ifdef USE_FULL_ASSERT


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
