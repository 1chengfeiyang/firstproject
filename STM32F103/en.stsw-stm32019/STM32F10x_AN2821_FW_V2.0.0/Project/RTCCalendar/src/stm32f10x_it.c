/**
  ******************************************************************************
  * @file RTC/src/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
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
#include "stm32f10x_it.h"
#include "main.h"


/** @addtogroup RTC
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t DisplayDateFlag;
uint8_t AlarmStatus;
extern __IO uint32_t TimingDelay;
uint8_t FlagKey;
uint8_t TamperEvent;
__IO uint16_t DutyCycle = 0;
float f32_Frequency = 0;
uint8_t BatteryRemoved;
uint16_t SummerTimeCorrect;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{  
 if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/**
  * @brief  This function handles PVD_IRQHandler .
  * @param  None
  * @retval : None
  */
void PVD_IRQHandler(void)
{
  BKP_TamperPinCmd(DISABLE);

  EXTI_ClearITPendingBit(EXTI_Line16);
  NVIC_ClearPendingIRQ(PVD_IRQn);
}

/**
  * @brief  This function handles TAMPER_IRQHandler .
  * @param  None
  * @retval : None
  */
 void TAMPER_IRQHandler(void)
{
  BKP_TamperPinCmd(DISABLE);

  TamperEvent = 1;

  BKP_ClearITPendingBit();

  BKP_ClearFlag();

  NVIC_ClearPendingIRQ(TAMPER_IRQn);
}

/**
  * @brief  This function handles RTC_IRQHandler .
  * @param  None
  * @retval : None
  */
void RTC_IRQHandler(void)
{
  uint8_t Month,Day;
  uint16_t Year;
  
  Month = BKP_ReadBackupRegister(BKP_DR2);
  Day = BKP_ReadBackupRegister(BKP_DR3);
  Year = BKP_ReadBackupRegister(BKP_DR4);
  NVIC_ClearPendingIRQ(RTC_IRQn);
  RTC_ClearITPendingBit(RTC_IT_SEC);
  
  /* If counter is equal to 86399: one day was elapsed */
  /* This takes care of date change and resetting of counter in case of
  power on - Run mode/ Main supply switched on condition*/
  if(RTC_GetCounter() == 86399)
  {
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* Reset counter value */
    RTC_SetCounter(0x0);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Increment the date */
    DateUpdate();
  }

  if((RTC_GetCounter()/3600 == 1)&&(((RTC_GetCounter()%3600)/60) == 59)&&
     (((RTC_GetCounter()%3600)%60) == 59))
  {
    /* March Correction */
    if((Month == 3) && (Day >24))
    {
      if(WeekDay(Year,Month,Day)==0)
      {
        if((SummerTimeCorrect & 0x8000) == 0x8000)
        {
          RTC_SetCounter(RTC_GetCounter() + 3601);
         
          /* Reset March correction flag */
          SummerTimeCorrect &= 0x7FFF;
         
          /* Set October correction flag  */
          SummerTimeCorrect |= 0x4000;
          SummerTimeCorrect |= Year;
          BKP_WriteBackupRegister(BKP_DR7,SummerTimeCorrect);
        }
      }
    }
      /* October Correction */
    if((Month == 10) && (Day >24))
    {
      if(WeekDay(Year,Month,Day)==0)
      {
        if((SummerTimeCorrect & 0x4000) == 0x4000)
        {
          RTC_SetCounter(RTC_GetCounter() - 3599);
          
          /* Reset October correction flag */
          SummerTimeCorrect &= 0xBFFF;
          
          /* Set March correction flag  */
          SummerTimeCorrect |= 0x8000;
          SummerTimeCorrect |= Year;
          BKP_WriteBackupRegister(BKP_DR7,SummerTimeCorrect);
        }
      }
    }
  }
}

/**
  * @brief  This function handles EXTI0_IRQHandler .
  * @param  None
  * @retval : None
  */
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line0);
  
    /*  Right Function  */
    FlagKey |=0x08;
  }
}

/**
  * @brief  This function handles EXTI1_IRQHandler .
  * @param  None
  * @retval : None
  */
void EXTI1_IRQHandler(void)
{
 
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line1);
    
    /*  Left Function  */
    FlagKey |=0x10;
  }
}

/**
  * @brief  This function handles EXTI9_5_IRQHandler .
  * @param  None
  * @retval : None
  */
void EXTI9_5_IRQHandler(void)
{
  uint32_t DelayBat;
  
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line8);
    
    /* Up Function  */
    FlagKey |=0x02;
  }

  /* Battery Removal and Restore detection  */
  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line7);
    
    /* Large delay to prevent rebounce (multiple spikes) at GPIO Input */
    for(DelayBat=0;DelayBat<2000000;DelayBat++);
    
    /* Battery Removal and Restore detection */
    if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))
    {
      /* Battery Restored  */
      BatteryRemoved=2; 
    }    
    if(!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)))
    {
      /* Battery Removed  */	
      BatteryRemoved=1; 
    }
  }
}
 
/**
  * @brief  This function handles TIM2_IRQHandler .
  * @param  None
  * @retval : None
  */
void TIM2_IRQHandler(void)
{
  __IO uint16_t IC2Value = 0;

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM2);

  if(IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = (TIM_GetCapture1(TIM2) * 100) / IC2Value;

    /* Frequency computation */
    f32_Frequency = ((float)TimerFrequency) / ((float)IC2Value);

    /* TIM  counter disable */
    TIM_Cmd(TIM2, DISABLE);
    /* Disable the CC2 Interrupt Request */
    TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
  }
  else
  {
    DutyCycle = 0;
    f32_Frequency = 0;
  }
}

/**
  * @brief  This function handles EXTI15_10_IRQHandler .
  * @param  None
  * @retval : None
  */

 void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line12);
  
    /*  Sel Function  */
    FlagKey |=0x01;
  }

  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line14);
   
    /*  Down Function  */
    FlagKey |=0x04;
  }
}

/**
  * @brief  This function handles RTCAlarm_IRQHandler .
  * @param  None
  * @retval : None
  */
void RTCAlarm_IRQHandler(void)
{
   /* Clear the Alarm Pending Bit */
   RTC_ClearITPendingBit(RTC_IT_ALR);

   if((BKP_ReadBackupRegister(BKP_DR8)==BKP_ReadBackupRegister(BKP_DR2)) && \
     (BKP_ReadBackupRegister(BKP_DR9)==BKP_ReadBackupRegister(BKP_DR3)) && \
      (BKP_ReadBackupRegister(BKP_DR10)==BKP_ReadBackupRegister(BKP_DR4)))
   {
     AlarmStatus = 1;
   }

  /* Clear the EXTI Line 17 */
  EXTI_ClearITPendingBit(EXTI_Line17);
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (RTC), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles RTC interrupt request.
  * @param  None
  * @retval : None
  */
/*void RTC_IRQHandler(void)
{
}*/

/**
  * @}
  */ 



/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
