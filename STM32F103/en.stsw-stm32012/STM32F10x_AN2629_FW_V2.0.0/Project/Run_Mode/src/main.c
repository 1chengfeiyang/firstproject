/**
  ******************************************************************************
  * @file Run_Mode/src/main.c 
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
#include "main.h"
#include "usart.h"
#include "rtc.h"


/** @addtogroup Run_Mode
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t TimeDisplay; 
  	 
/* Private function prototypes -----------------------------------------------*/
void GPIO_AllAinConfig(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void UART_Configuration(void);
void SetSysClock(void);
void SetSysClockTo8(void);
void SetSysClockTo72(void);
void SwitchSystemClock(void);

/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int main()
{
	uint32_t rtcCounter;
	
	/* System Clocks Configuration */
	RCC_Configuration();
	
	/* NVIC configuration */
	NVIC_Configuration();
	
	
	/* RCC configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, DISABLE);
				 
	#ifdef ALL_PERIPHERIALS_ENABLE
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, ENABLE);
	#endif

	#ifdef UART_ONLY
		/* Enable clock on UART */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,ENABLE);
		/* Enable clock for BKP domain */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	#endif

	 PWR_BackupAccessCmd(ENABLE);			   

	 /* Configure all GPIO port pins in Analog Input mode */
	 GPIO_AllAinConfig();

	 /* Configures GPIO for USART1 using */	
	 GPIO_ConfigUSART();	
	
	 USART_Configuration();

	 /* read of the backup registers content */
	 rtcCounter = BKP_ReadBackupRegister(BKP_DR2) | BKP_ReadBackupRegister(BKP_DR1) << 16;		
	 
	 if(rtcCounter == 0x0000) 
	  {
	    /* Backup data register value is not correct or not yet programmed (when
	       the first time the program is executed) */
	
	    printf("\r\n\n RTC not yet configured....");
	   			  
	    /* RTC Configuration */
	    RTC_Configuration();
	
	     printf("\r\n RTC configured....");
	 
	    /* Adjust time by values entred by the user on the hyperterminal */
	    Time_Adjust();
	
	  }
	  else
	  {
	    /* Check if the Power On Reset flag is set */
	    if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
	    {
	     printf("\r\n\n Power On Reset occurred....");
	    }
	    /* Check if the Pin Reset flag is set */
	    else if(RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
	    {
	     printf("\r\n\n External Reset occurred....");
	    }
	
	    printf("\r\n No need to configure RTC...."); 
		
		/* Display the backup registers content */
		Time_Display(rtcCounter);
	    
		/* Wait for RTC registers synchronization */
	    RTC_WaitForSynchro();
	
	    /* Enable the RTC Second */  
	    RTC_ITConfig(RTC_IT_SEC, ENABLE);
	    /* Wait until last write operation on RTC registers has finished */
	    RTC_WaitForLastTask();
	
	  }
	
	  /* Clear reset flags */
	  RCC_ClearFlag();
	  
	  /* Display time in infinte loop */
	  printf("\n\r");
	  

	  /* Infinite loop */ 
	  while(1)
	  {
		#ifdef WFI_ON 
			__WFI();
		#endif			
	
		while(TimeDisplay==0);		
		rtcCounter = RTC_GetCounter();
			
		/* Display current time */		
	    Time_Display(rtcCounter);

		/* Save current time in Backup register */		
		BKP_WriteBackupRegister(BKP_DR1,(uint16_t)(rtcCounter >> 16)); 
		BKP_WriteBackupRegister(BKP_DR2,(uint16_t)rtcCounter);

		TimeDisplay = 0;
	  }
	  
}




/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{																
RCC_ClocksTypeDef RCC_ClockFreq;

/* System Clocks Configuration ---------------------------------------------*/
  RCC_DeInit();

#ifdef HSE_ENABLE
    /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

   /* Wait till HSE is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
  {
  }
#endif

 #ifdef PREFETCH_ON 
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	#else
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Disable);
	#ifdef HALF_CYCLE_ON 
		FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Enable);
		#else
		FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Disable);
	#endif
 #endif
  

				  
  /* PCLK2 = HCLK/2	 */
  #ifdef ABP2_DIV2
  RCC_PCLK2Config(RCC_HCLK_Div2); 			
  /* PCLK2 = HCLK/8	 */
  #elif defined ABP2_DIV8
  RCC_PCLK2Config(RCC_HCLK_Div8); 

  #endif

		
  /* PCLK1 = HCLK/4 */
  #ifdef ABP1_DIV4
  RCC_PCLK1Config(RCC_HCLK_Div4);
  /* PCLK1 = HCLK/8 */
  #elif defined ABP1_DIV8
  RCC_PCLK1Config(RCC_HCLK_Div8);

  #endif


  /* Select HSE or HSI as system clock source */
  SwitchSystemClock(); 

  /* Configure System clock */
  SetSysClock(); 

  RCC_GetClocksFreq(&RCC_ClockFreq);

    /* Enable PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* GPIOs Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						 RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

}



/**
  * @brief  Configure all GPIO port pins in Analog Input mode 
  *   (floating input trigger OFF)
  * @param  None
  * @retval : None
  */
void GPIO_AllAinConfig(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}



/**
  * @brief  Configures the NVIC.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{  
NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



/**
  * @brief  Sets the system clock.
  * @param  None
  * @retval : None
  */
void SetSysClock(void)
{
#ifdef HCLK_FREQ_8MHz  
  SetSysClockTo8();
#elif defined HCLK_FREQ_72MHz
  SetSysClockTo72();
#else 
  #error no frequency specified
#endif
}



/**
  * @brief  Sets the system clock to 8 MHz
  *   
  * @param  None
  * @retval : None
  */
void SetSysClockTo8(void)
{
#ifdef HSE_ENABLE
 	/* Flash 0 wait state */
  	FLASH_SetLatency(FLASH_Latency_0);
#endif
}



/**
  * @brief  Sets the system clock to 72 MHz
  * @param  None
  * @retval : None
  */
void SetSysClockTo72(void)
{
#ifdef HSE_ENABLE
  /* Disable PLL */ 
  RCC_PLLCmd(DISABLE);

/* Configure PLL **************************************************************/
  /* Flash 2 wait state */
  FLASH_SetLatency(FLASH_Latency_2);
								 
  /* HCLK = SYSCLK */
  RCC_HCLKConfig(RCC_SYSCLK_Div1); 

  /* PLLCLK = 8MHz * 9 = 72 MHz */
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

  /* Enable PLL */ 
  RCC_PLLCmd(ENABLE);

  /* Wait till PLL is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {
  }

/* Switch system clock to PLL *************************************************/ 
  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  /* Wait till PLL is used as system clock source */
  while(RCC_GetSYSCLKSource() != 0x08)
  {
  }
#endif
}



/**
  * @brief  Sets the system clock to 8 MHz
  * @param  None
  * @retval : None
  */
void SwitchSystemClock(void)
{
/* Switch system clock *************************************************/
  #ifdef HSE_ENABLE
  /* Select HSE as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

  /* Wait till HSE is used as system clock source */
  while(RCC_GetSYSCLKSource() != 0x04)
  {
  }
  #endif

  #ifdef HSI_ENABLE
  /* Select HSI as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

  /* Wait till HSI is used as system clock source */
  while(RCC_GetSYSCLKSource() != 0x00)
  {
  }
  #endif
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
