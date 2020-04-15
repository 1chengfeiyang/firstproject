/**
  ******************************************************************************
  * @file CurrentMeasurements/src/hw_config.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Hardware configuration file.
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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();


#ifdef HSE_PLL_ON
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    /* Set the Flash wait states */
    #ifdef HSE_PLL_ON_72MHz /* 2 wait states */
    FLASH_SetLatency(FLASH_Latency_2);
    #endif
    
    #ifdef HSE_PLL_ON_48MHz /* 1 wait state */ 
    FLASH_SetLatency(FLASH_Latency_1);
    #endif
    
    #ifdef HSE_PLL_ON_36MHz /* 1 wait state */
    FLASH_SetLatency(FLASH_Latency_1);
    #endif
    
    #ifdef HSE_PLL_ON_24MHz /* 0 wait state */
    FLASH_SetLatency(FLASH_Latency_0);
    #endif
    
    #ifdef HSE_PLL_ON_16MHz /* 0 wait state */
    FLASH_SetLatency(FLASH_Latency_0);
    #endif
    
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK/2 */
    RCC_PCLK2Config(RCC_HCLK_Div2); 

    /* PCLK1 = HCLK/4 */
    RCC_PCLK1Config(RCC_HCLK_Div4);

    #ifdef HSE_PLL_ON_72MHz
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    #endif
    
    #ifdef HSE_PLL_ON_48MHz
    /* PLLCLK = 8MHz * 6 = 48 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
    #endif
    
    #ifdef HSE_PLL_ON_36MHz
    /* PLLCLK = 8MHz / 2 * 9 = 36 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
    #endif
    
    #ifdef HSE_PLL_ON_24MHz
    /* PLLCLK = 8MHz * 3 = 24 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_3);
    #endif
    
    #ifdef HSE_PLL_ON_16MHz
    /* PLLCLK = 8MHz * 2 = 16 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2);
    #endif
    
    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
#endif /* HSE_PLL_ON */
    
#ifdef HSE_PLL_OFF
  
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
 if(HSEStartUpStatus == SUCCESS)
  {
    /* Disable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Disable);
  
    /* Flash 0 wait state */
    FLASH_SetLatency(FLASH_Latency_0);
    
    #ifdef HSE_PLL_OFF_8MHz
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
    #endif 

    #ifdef HSE_PLL_OFF_4MHz
    /* HCLK = SYSCLK/2 = 4MHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div2); 
    #endif 
    
    #ifdef HSE_PLL_OFF_2MHz
    /* HCLK = SYSCLK/4 = 2MHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div4); 
    #endif 
    
    #ifdef HSE_PLL_OFF_1MHz
    /* HCLK = SYSCLK/8 = 1MHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div8); 
    #endif 
    
    #ifdef HSE_PLL_OFF_500KHz
    /* HCLK = SYSCLK/16 = 500KHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div16); 
    #endif 
    
    #ifdef HSE_PLL_OFF_125KHz
    /* HCLK = SYSCLK/64 = 125KHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div64); 
    #endif 
    
    /* PCLK2 = HCLK/2 */
    RCC_PCLK2Config(RCC_HCLK_Div2); 

    /* PCLK1 = HCLK/4 */
    RCC_PCLK1Config(RCC_HCLK_Div4);
  
    /* Select HSE as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

    /* Wait till HSE is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x04)
    {
    }  
  }
#endif /* HSE_PLL_OFF */ 
 
 
#ifdef HSI_PLL_ON
 
  /* Enable Prefetch Buffer */
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
 
  /* Set the Flash wait states */
  #ifdef HSI_PLL_ON_64MHz /* 2 wait states */
  FLASH_SetLatency(FLASH_Latency_2);
  #endif
    
  #ifdef HSI_PLL_ON_48MHz /* 1 wait state */ 
  FLASH_SetLatency(FLASH_Latency_1);
  #endif
    
  #ifdef HSI_PLL_ON_36MHz /* 1 wait state */
  FLASH_SetLatency(FLASH_Latency_1);
  #endif
    
  #ifdef HSI_PLL_ON_24MHz /* 0 wait state */
  FLASH_SetLatency(FLASH_Latency_0);
  #endif
    
  #ifdef HSI_PLL_ON_16MHz /* 0 wait state */
  FLASH_SetLatency(FLASH_Latency_0);
  #endif
    
  /* HCLK = SYSCLK */
  RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
  /* PCLK2 = HCLK/2 */
  RCC_PCLK2Config(RCC_HCLK_Div2); 

  /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div4);  
  
  #ifdef HSI_PLL_ON_64MHz
  /* PLLCLK = 8MHz/2 * 16 = 64 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
  #endif
    
  #ifdef HSI_PLL_ON_48MHz
  /* PLLCLK = 8MHz/2 * 12 = 48 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);
  #endif
    
  #ifdef HSI_PLL_ON_36MHz
  /* PLLCLK = 8MHz/2 * 9 = 36 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);
  #endif
    
  #ifdef HSI_PLL_ON_24MHz
  /* PLLCLK = 8MHz/2 * 6 = 24 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_6);
  #endif
    
  #ifdef HSI_PLL_ON_16MHz
  /* PLLCLK = 8MHz/2 * 4 = 16 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_4);
  #endif
    
  /* Enable PLL */ 
  RCC_PLLCmd(ENABLE);

  /* Wait till PLL is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {
  }

  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  /* Wait till PLL is used as system clock source */
  while(RCC_GetSYSCLKSource() != 0x08)
  {
  }
  
#endif /* HSI_PLL_ON */
 
#ifdef HSI_PLL_OFF
  
  /* Disable Prefetch Buffer */
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Disable);
  
  /* Flash 0 wait state */
  FLASH_SetLatency(FLASH_Latency_0);
  
  #ifdef HSI_PLL_OFF_8MHz
  /* HCLK = SYSCLK */
  RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  #endif 

  #ifdef HSI_PLL_OFF_4MHz
  /* HCLK = SYSCLK/2 = 4MHz */
  RCC_HCLKConfig(RCC_SYSCLK_Div2); 
  #endif 
    
  #ifdef HSI_PLL_OFF_2MHz
  /* HCLK = SYSCLK/4 = 2MHz */
  RCC_HCLKConfig(RCC_SYSCLK_Div4); 
  #endif 
    
  #ifdef HSI_PLL_OFF_1MHz
  /* HCLK = SYSCLK/8 = 1MHz */
  RCC_HCLKConfig(RCC_SYSCLK_Div8); 
  #endif 
   
  #ifdef HSI_PLL_OFF_500KHz
  /* HCLK = SYSCLK/16 = 500KHz */
  RCC_HCLKConfig(RCC_SYSCLK_Div16); 
  #endif 
    
  #ifdef HSI_PLL_OFF_125KHz
  /* HCLK = SYSCLK/64 = 125KHz */
  RCC_HCLKConfig(RCC_SYSCLK_Div64); 
  #endif 
    
  /* PCLK2 = HCLK/2 */
  RCC_PCLK2Config(RCC_HCLK_Div2); 

  /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div4);
  
  /* Select HSI as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

  /* Wait till HSI is used as system clock source */
   while(RCC_GetSYSCLKSource() != 0x00)
  {
  }    
#endif /* HSI_PLL_OFF */
  
}



/**
  * @brief  Enable the clock for all peripherals 
  * @param  None
  * @retval : None
  */
void All_PeriphClock_Enable(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL, ENABLE);
}



/**
  * @brief  Disable the clock for all peripherals 
  * @param  None
  * @retval : None
  */
void All_PeriphClock_Disable(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL, DISABLE);
}



/**
  * @brief  Configures the different GPIO ports as Analog Inputs.
  * @param  None
  * @retval : None
  */
void GPIO_Config_ALL_AIN(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOD and GPIOE clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
                         | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD 
                         | RCC_APB2Periph_GPIOE| RCC_APB2Periph_AFIO, ENABLE);
  
  
  /* Disable the Serial Wire Jtag Debug Port SWJ-DP */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); 
  
    /* PA  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
      /* PB  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
      /* PC  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
        /* PD  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}



/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOC, clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /* Configure PC.06, PC.07, PC.08 and PC.09 as Output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
}



/**
  * @brief  Configures EXTI Line0.
  * @param  None
  * @retval : None
  */
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure PA.0 as input floating (EXTI Line0) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA.0 */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  /* Configure EXTI Line0 to generate an event or an interrupt on falling edge */
  EXTI_ClearITPendingBit(EXTI_Line0);
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;

#ifdef Entry_WFE
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
#endif
  
#ifdef Entry_WFI
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#endif 
  
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
}



/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
#ifdef Entry_WFI
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 2 bits for Preemption Priority and 2 bits for Sub Priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif 
  
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
}



/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval : None
  */
void RTC_Configuration(void)
{ 
  /* RTC clock source configuration ----------------------------------------*/
  
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);
  
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

  /* Set the RTC time base to 1s */
  RTC_SetPrescaler(32767);  
  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();  
}
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
