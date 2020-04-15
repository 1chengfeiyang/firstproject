/**
  ******************************************************************************
  * @file RCCalibration/src/main.c 
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
#include "HSI_calibration.h"


/** @addtogroup RCCalibration
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
  /* Uncomment this line to activate demo 1: calibration with fixed Error */
  /* Comment this line to activate demo 2: calibration with minimum Error found */
//#define USE_HSI_Fixed_Error 

/* Incomment this line to output HSI on pin MCO (PA8) */
#define OUTPUT_RC_ON_MCO_FOR_DEBUG
 
 /* GPIOs definition */
#define LED_Green      GPIO_Pin_5
#define LED_Red        GPIO_Pin_6
#define HSITRIM_5LEDs (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 \
                       | GPIO_Pin_4) 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
int32_t Current_RC_Frequency = 0;
ErrorStatus CalibStatus = ERROR;
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void DisplayHSITRIMbits(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int main(void)
{
   /* User RCC configuration */
   RCC_Configuration();
    
   /* User GPIO configuration */
   GPIO_Configuration();
   
   /* Initialize GPIOC.0 to GPIOC.6 for LEDs */
   GPIO_ResetBits(GPIOC, HSITRIM_5LEDs| LED_Green | LED_Red );
   
#ifdef USE_HSI_Fixed_Error   
   /* Calibrate internal RC with the maximum error allowed in Hz */
   /* Fix the maximum value of the error frequency at +/- 14000Hz -> 0.17% */
   CalibStatus = HSI_CalibrateFixedError(14000, &Current_RC_Frequency);

#else   
   /* Calibrate internal RC and return its frequency in Hz after calibration */
   Current_RC_Frequency = HSI_CalibrateMinError();

#endif  
   
   /* Get the HSITRIM value from the RCC_CR register (5 bits) after calibration */
   DisplayHSITRIMbits();
   
#ifdef USE_HSI_Fixed_Error /* If the user selected the demo 1: calibration with */
   if(CalibStatus == SUCCESS) /* fixed Error */
   {
      /* The LED Green glows when the Calibration was done */
      GPIO_SetBits(GPIOC, LED_Green);
   }
   else
   {
     /* The LED Red glows when the Calibration was not done */
     GPIO_SetBits(GPIOC, LED_Red);
   }
#else  /* The user selected the demo 2: calibration with the minimum Error value found */
    /* The LED Green glows */
    GPIO_SetBits(GPIOC, LED_Green);
#endif     
   
   while(1)
   {
   }
}



/**
  * @brief  Displays HSITRIM bits on LEDs.
  * @param  None
  * @retval : None
  */
void DisplayHSITRIMbits(void)
{
   uint16_t tmp=0;
   uint16_t RCC_CR_HSITRIM_Value=0;
  
   RCC_CR_HSITRIM_Value = ((RCC->CR)>>3)&0x1F;
   tmp = GPIO_ReadInputData(GPIOC);
   tmp &= 0xFFE0;
   tmp |= RCC_CR_HSITRIM_Value;
   
   /* Write HSITRIM value on GPIOC Pin 0 to Pin 4 */
   GPIO_Write(GPIOC,tmp);
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

    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_1);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 4MHz * 9 = 36 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);

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

    /* Enable GPIOA and GPIOC clocks  */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC \
                           | RCC_APB2Periph_AFIO , ENABLE);
    
#ifdef  OUTPUT_RC_ON_MCO_FOR_DEBUG  
   /* Output the HSI frequency on MCO pin (GPIOA.8)*/ 
   RCC_MCOConfig(RCC_MCO_HSI);
#endif
}




/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{   
   GPIO_InitTypeDef GPIO_InitStructure;
   
   GPIO_InitStructure.GPIO_Pin = HSITRIM_5LEDs | LED_Green | LED_Red; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
#ifdef  OUTPUT_RC_ON_MCO_FOR_DEBUG
   /* Output HSI clock on MCO pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
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

/**
  * @}
  */ 




/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
