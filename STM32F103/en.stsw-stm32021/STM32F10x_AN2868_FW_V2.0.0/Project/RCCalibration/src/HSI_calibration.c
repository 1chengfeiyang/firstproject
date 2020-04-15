/**
  ******************************************************************************
  * @file RCCalibration/src/HSI_calibration.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  HSI Calibration basic routines
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
#include "HSI_calibration.h"

/* Private variables--------------------------------------------------------- */
__IO uint32_t IC1_ValueHSI = 0;
uint32_t N_OverflowCounter = 0;
__IO int32_t Real_RC_Frequency=0;
__IO uint32_t MeasurementDone=0;
uint32_t CurrentPeriodFreq=0;
__IO uint32_t Real_RC_Frequency_cumul = 0;
__IO uint32_t PeriodCount=0;
uint32_t APB1_UserPeriph=0;
uint32_t APB2_UserPeriph=0;
FunctionalState BA_bit = DISABLE;
FunctionalState CC0_Bit= DISABLE;

/** @addtogroup RCCalibration
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void RCC_Configuration(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Restore the RCC user configuration (configuration before
  *   calibration procedure)
  * @param  None
  * @retval : None
  */
void Restore_RCC_UserConfiguration(void)
{
  /* Call your default RCC configuration here, for exemple: RCC_Configuration(); */
   RCC_Configuration();
}



/**
  * @brief  Calibrates the internal RC and returns the current frequency 
  *   after calibration.
  * @param  None
  * @retval : None
  */
void HSI_FreqMeasure(void)
{  
  /* If the counter has overflowed */
  if((TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)&&(MeasurementDone == 0)) 
  {
    /* Clear TIM3 counter update interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    
    /* Increment N_OverflowCounter if the overflow condition has been occured */
    N_OverflowCounter ++;
  }
      
  /* If the Input capture 1 interrupt has been occured: new period occured */
  if((TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)&&(MeasurementDone == 0))
  {	
    /* Clear TIM3 Capture compare 1 interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    
    /* Get the Input Capture 1 value of timer TIM3 */
    IC1_ValueHSI = TIM_GetCapture1(TIM3);
             
    /* Compute the frequency */
   CurrentPeriodFreq = Ref_Frequency * (IC1_ValueHSI + ((N_OverflowCounter-1) * 0xFFFF));
    
    if(PeriodCount!=0)
    {
      /* Add the current frequency to previous cumulation */
      Real_RC_Frequency_cumul = Real_RC_Frequency_cumul + CurrentPeriodFreq;
    
      /* If the number of desired period is reached, compute the frequency average */
      if(PeriodCount==NbOfPeriod)
      {    
        /* Compute the HSI frequency average of the the current HSITRIM configuration */
        Real_RC_Frequency = (int32_t)(Real_RC_Frequency_cumul / NbOfPeriod);
        /* Initialize the frequency cumulation variable */
        Real_RC_Frequency_cumul = 0;
        /* Initialize the measured period number */
        PeriodCount=0;
        /* Set the "measurement done": the frequency of all desired periods has 
           been measured */
        MeasurementDone = 1;
      }
    }
    
    /* Initialize N_OverflowCounter */
    N_OverflowCounter = 0;
    /* Increment the number of measured periods */
    PeriodCount ++;
  }
}



/**
  * @brief  Calibrates the internal RC with the minimum error found 
  *   and returns the frequency error after calibration.
  * @param  None
  * @retval : The HSI frequency value after calibration
  */
int32_t HSI_CalibrateMinError(void)
{
   NVIC_InitTypeDef  NVIC_InitStructureForHSI; 
   int32_t FrequencyAfterCalib = 0;
   uint8_t Calibration_value=0;
   int32_t CurrentError=0;
   uint32_t PreviousError = 1000000; /* large value */
   uint8_t HSITRIM=0;
   
   /* System clock configuration for HSI calibration process */
   RCC_ConfigurationForHSI();

    /* Enable the TIM3 global Interrupt */
   NVIC_InitStructureForHSI.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructureForHSI.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructureForHSI.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructureForHSI.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructureForHSI);

   /* Configure the GPIO ports for HSI calibration process */
   GPIO_ConfigurationForHSI();

#ifdef  USE_Reference_RTC   
   /* RTC configuration for HSI calibration process if the clock souce is RTC */
   RTC_ConfigurationForHSI();
  
   /* output the RTC clock with frequency divided by 64 on the Tamper pin */
   BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
#endif   
  
   /* Configure the timer TIM3 for HSI calibration process */
   TIM_ConfigurationForHSI();

   /* RC Frequency measurement for different values 0 to 31 of HSITRIM[4:0] bits
      in RCC_CR register */
   for (HSITRIM=0; HSITRIM<32; HSITRIM++)
   {
     /* Reset "measurement done" */
     MeasurementDone = 0;
  
     /* Set the HSITRIM[4:0] bits to HSITRIM value to be ready for measurement */
     RCC_AdjustHSICalibrationValue(HSITRIM);
    
     /* Start Frequency measurement for current HSITRIM value:
     Enable TIM3 and its interrupts */
     TIM_Cmd(TIM3, ENABLE); 
     TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_Update, ENABLE);
    
     /* Wait for all periods of the current frequency were measured */
     while (MeasurementDone == 0)
     {
     }
     
     /* Initialize the counted periods after measurement */
     PeriodCount=0;
     
     /* Disable TIM3 */
     TIM_Cmd(TIM3, DISABLE); 
    
     /* Disable TIM3 interrupts to avoid to enter again in the interrpt routine */
     TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_Update, DISABLE);
     
     /* Compute current frequency Error corresponding to the current HSITRIM value */
     if((Real_RC_Frequency - 8000000) < 0)
     {
       CurrentError = 8000000 - Real_RC_Frequency;
     }
     else
     {
       CurrentError = Real_RC_Frequency - 8000000;
     }
     
     /* Get the best calibration value to have the nearest frequency value to 
        8000000Hz of the HSI in this chip */
     if(PreviousError > CurrentError)
     {
       PreviousError = CurrentError;
       Calibration_value = HSITRIM;
       FrequencyAfterCalib = Real_RC_Frequency;
     }
   }
   
    /* Set HSITRIM bits corresponding to the nearest frequency */
    RCC_AdjustHSICalibrationValue(Calibration_value);
   
    /* Disable TIM3 and its interrupts */
    NVIC_InitStructureForHSI.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructureForHSI.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructureForHSI);

#ifdef  USE_Reference_RTC     
    /* Restore user configuration of RTC output on the Tamper pin after calibration */
    BKP_RTCOutputConfig(CC0_Bit);
    
    /* Restore the user configuration of the access to BKP Domain after calibration */
    PWR_BackupAccessCmd(BA_bit);
#endif
    
    /* Restore RCC User configuration after calibration process */
    Restore_RCC_UserConfiguration();
                      
    /* Return the RC frequency after calibration */
    return (FrequencyAfterCalib);
}



/**
  * @brief  Calibrates the internal RC with the maximum allowed Error 
  *   value fixed by the user. If this value was not found, this
  *   function sets the RC frequency to default value (HSITRIM = 16).
  * @param AllowedErrorMax: maximum absolute value allowed of the HSI 
  *   frequency Error given in Hz.
  * @param Freq: pointer to a uint32_t variable that will contain the value  
  *   of the HSI frequency after calibration.
  * @retval : - An ErrorStatus enumuration value:
  * @param SUCCESS: an error with value =< AllowedErrorMax was
  *   found.
  * @param ERROR: an error with value =< AllowedErrorMax was
  *   not found.                  
  */
ErrorStatus HSI_CalibrateFixedError(uint32_t AllowedErrorMax, int32_t* Freq)
{
   NVIC_InitTypeDef  NVIC_InitStructureForHSI; 
   int32_t Error=0;
   int8_t HSITRIM=16;
   int8_t Sign= 1;
   int8_t i;
   ErrorStatus CalibrationStatus = ERROR;
 
   /* System clock configuration for HSI calibration process */
   RCC_ConfigurationForHSI();

   /* Enable the TIM3 global Interrupt for HSI calibration process */
   NVIC_InitStructureForHSI.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructureForHSI.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructureForHSI.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructureForHSI.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructureForHSI);

   /* Configure the GPIO ports for HSI calibration process */
   GPIO_ConfigurationForHSI();

#ifdef  USE_Reference_RTC   
   /* RTC configuration for HSI calibration process */
   RTC_ConfigurationForHSI();
  
   /* output the RTC clock with frequency divided by 64 on the Tamper pin */
   BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
#endif   
  
   /* Configure the timer for HSI calibration process */
   TIM_ConfigurationForHSI();

   /* RC Frequency measurement for different values 0 to 31 of HSITRIM[4:0] bits
      in RCC_CR register */
   for(i=0; i<32; i++)
   {
     /* Reset "measurement done" */
     MeasurementDone = 0;
     
     /* Compute next HSITRIM value for frequency measurments */
     /* HSITRIM = 16 then 15, 17, 14, 18, 13, 19 etc until HSITRIM = 31 */
     HSITRIM = HSITRIM + (i*Sign);
     Sign *= (-1);
       
     /* Set the HSITRIM[4:0] bits to HSITRIM value to be ready for measurements */
     RCC_AdjustHSICalibrationValue(HSITRIM);
    
     /* Start Frequency measurement for current HSITRIM value:
     Enable TIM3 and its interrupts */
     TIM_Cmd(TIM3, ENABLE); 
     TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_Update, ENABLE);
    
     /* Wait for all periods of the current frequency were measured */
     while (MeasurementDone == 0)
     {
     }
     
     /* Initialize the counted periods after measurement */
     PeriodCount=0;
     
     /* Disable TIM3 */
     TIM_Cmd(TIM3, DISABLE); 
    
     /* Disable TIM3 interrupts to avoid to enter again in the interrpt routine */
     TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_Update, DISABLE);
     
     /* Compute current frequency Error */
     if(((int32_t)Real_RC_Frequency - 8000000) < 0)
     {
       Error = 8000000 - Real_RC_Frequency;
     }
     else
     {
       Error = Real_RC_Frequency - 8000000;
     }
     /* If an error less or equal to value settled by the user */
     if((uint32_t)Error <= AllowedErrorMax )
     {
       CalibrationStatus = SUCCESS; /* The calibration has succeed */
       break; /* stop searching and measurments for frequencies */
     }
   }

   *Freq = Real_RC_Frequency;
   
   /* If the frequncy error settled by the user was not found in the chip */
   if(CalibrationStatus == ERROR)
   {
     /* Set the HSITRIM[4:0] bits to HSITRIM default value */      
     RCC_AdjustHSICalibrationValue(16);
   }
   
   /* Disable TIM3 and its interrupts */
   NVIC_InitStructureForHSI.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructureForHSI.NVIC_IRQChannelCmd = DISABLE;
   NVIC_Init(&NVIC_InitStructureForHSI);

#ifdef  USE_Reference_RTC     
   /* Restore user configuration of RTC output on the Tamper pin */
   BKP_RTCOutputConfig(CC0_Bit);
    
   /* Restore the user configuration of the access to BKP Domain */
   PWR_BackupAccessCmd(BA_bit);
#endif
    
   /* Restore RCC User configuration */
   Restore_RCC_UserConfiguration();
   
   /* Return the calibration status: failed or succeed */
   return (CalibrationStatus);

}



/**
  * @brief  Configures the GPIOs Pins.
  * @param  None
  * @retval : None
  */
void RTC_ConfigurationForHSI(void)
{
    /* Check if the backup domain write protection was enabled by
       the user application */
    if(PWR->CR&0x100)
    {
      BA_bit = ENABLE; /* The backup domain write protection was enabled by
                          the user application*/
    }
    /* Check if the RTC clock / 64 on Tamper pin was used by the user application */
    if(BKP->RTCCR&0x0080)
    {
      CC0_Bit = ENABLE; /* The RTC clock was used by the user application */
    }
    
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);
  
    /* Wait till LSE is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {
    }

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);
}


/**
  * @brief  Configures the TIM to mesure the frequency.
  * @param  None
  * @retval : None
  */
void TIM_ConfigurationForHSI(void)
{
  /* TIM3 configuration: Input Capture mode: slave mode reset ---
     The tamper pin (PC13) is connected to TIM3 CH1 pin (PA.06),          
     The Rising edge is used as active edge,
     The TIM3 CCR1 is used to compute the RC frequency value
  ------------------------------------------------------------- */
    TIM_ICInitTypeDef  TIM_ICInitStructureForHSI;
    
    TIM_DeInit(TIM3);
    TIM_ICStructInit(&TIM_ICInitStructureForHSI);
  
    TIM_ICInitStructureForHSI.TIM_Channel = TIM_Channel_1;           
    TIM_ICInitStructureForHSI.TIM_ICPolarity = TIM_ICPolarity_Rising ;      
    TIM_ICInitStructureForHSI.TIM_ICSelection = TIM_ICSelection_DirectTI;       
    TIM_ICInitStructureForHSI.TIM_ICPrescaler = TIM_ICPSC_DIV1;       
    TIM_ICInitStructureForHSI.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3, &TIM_ICInitStructureForHSI);
   
    /* Select the TIM3 Input Trigger: TI1FP1 */
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
   
    /* Select the slave Mode: Reset Mode */
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
   
    /* Enable the Master/Slave Mode */
    TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
    
    /* Disable TIM3 and its interrupts to avoid unwanted frequency measurements */
    TIM_Cmd(TIM3, DISABLE); 
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_Update, DISABLE);
  
    /* Reset all flags on TIM3_SR register */
    TIM3->SR = 0x0000;
    
}  



/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : the Peripheral used by the user before the calibration
  */
void RCC_ConfigurationForHSI(void)
{ 
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* HCLK = SYSCLK = 8MHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
      
    /* PCLK2 = HCLK = 8MHz */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK = 8MHz */
    RCC_PCLK1Config(RCC_HCLK_Div1);
    
    /* Flash 0 wait state */
    FLASH_SetLatency(FLASH_Latency_0);

    /* Select HSI as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    
    /* Get the used user clocks on APB1 before calibration */
    APB1_UserPeriph = RCC->APB1ENR;
    /* Get the used user clocks on APB2 before calibration */
    APB2_UserPeriph = RCC->APB2ENR;    

    /* Enable (add to user clocks) TIM3, Buckup and PWR clocks */
    RCC_APB1PeriphClockCmd( APB1_UserPeriph | RCC_APB1Periph_TIM3 |
                            RCC_APB1Periph_BKP  |
                            RCC_APB1Periph_PWR , ENABLE);

    /* Enable (add to user clocks)  GPIOA, GPIOC and AFIO clocks */
    RCC_APB2PeriphClockCmd( APB2_UserPeriph | RCC_APB2Periph_GPIOA |
                            RCC_APB2Periph_GPIOC |
                            RCC_APB2Periph_AFIO, ENABLE);
}


/**
  * @brief  Configures the GPIOs Pins.
  * @param  None
  * @retval : None
  */
void GPIO_ConfigurationForHSI(void)
{
    GPIO_InitTypeDef GPIO_InitStructureForHSI;

    /* GPIOA Configuration: for TIM3 channel 1 input for frequency measurment */
    GPIO_InitStructureForHSI.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructureForHSI.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructureForHSI); 

#ifdef  USE_Reference_RTC      
    /* GPIOC Configuration: for Tamper pin (RTC_frequency /64) */
    GPIO_InitStructureForHSI.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructureForHSI.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructureForHSI.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructureForHSI); 
#endif    
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
