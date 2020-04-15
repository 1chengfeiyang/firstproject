/**
  ******************************************************************************
  * @file FullHalfStepMode/src/main.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main program body
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
#include "StepperMotor.h"
#include <stdio.h>



/** @addtogroup FullHalfStepMode
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
  /* Private define ------------------------------------------------------------*/
/* -------------------- Stepper modes entering function ---------------- */
/******* Define the Rotation Direction *******/
#define RotationDirection_CW
//#define RotationDirection_CCW

/******* Define the Step Mode *********/
#define Half_Step
//#define Full_Step

/******* Define the Decay Mode ********/
//#define ControlSlow_Current
#define ControlFast_Current

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 

ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
    
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int main(void)
{

  /* System Clocks Configuration */
  RCC_Configuration();
  
  /* NVIC Configuration */
  NVIC_Configuration();
  
  /* Activate the driver */
  Stepper_Cmd(ENABLE);
  
  /* Driver control pin configuration */
  Stepper_PinControlConfig();
  
  /* Disable the initialization of the driver */ 
  Stepper_ResetDisable();
      
  /* -----------Modes selection: Rotation direction, Step mode, Decay mode---------------*/ 
#ifdef RotationDirection_CW
  Stepper_SetRotationDirection(Stepper_RotationDirection_CW);
#endif /* RotationDirection_CW */

#ifdef RotationDirection_CCW
  Stepper_SetRotationDirection(Stepper_RotationDirection_CCW);
#endif /* RotationDirection_CCW */
  
#ifdef Half_Step
  Stepper_SelectMode(Stepper_Half);
#endif  /* Half_Step */
  
#ifdef Full_Step
  Stepper_SelectMode(Stepper_Full);
#endif  /* Full_Step */ 

#ifdef ControlSlow_Current
  Stepper_SetControlMode(Stepper_ControlSlow);
#endif  /* ControlSlow_Current */  

#ifdef ControlFast_Current
  Stepper_SetControlMode(Stepper_ControlFast);
#endif   /* ControlFast_Current */ 
  
  
  /* Start the stepper motor */   
  Stepper_Start(ENABLE);
  
  /* Peripherals configuration */
  Stepper_Init();
  
  while (1)
  {
  }   
}



/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

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

}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{ 


  NVIC_SetPriorityGrouping(7); /* 0 bits for pre-emption priority 4 bits for
subpriority*/
  NVIC_SetPriority(DMA1_Channel2_IRQn, 0x01); /* 0x01 = 0x0 << 3 | (0x1 & 0x7), prority
is 0x01 << 4 */
 NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  
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

  while (1)
  {
    
  }	
}
#endif
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
