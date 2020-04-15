/**
  ******************************************************************************
  * @file    InfraRed/RC5_Decoding_TIM_EXTI/src/main.c 
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
#include "stm32f10x.h"
#include "stm32_eval.h"
#include "RC5_IR_Emul_Receiver.h"
#include <stdio.h>

/** @addtogroup STM32F10x_Infrared_Decoders
  * @{
  */

/** @addtogroup RC5_Decoding_TIM_EXTI
  * @{
  */ 

/** @addtogroup MAIN
  * @{
  */ 
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Table of different Addresses (devices) */
const char* RC5_Devices[32] = {
        "TV1" ,                  /* 0 */
        "TV2" ,                  /* 1 */
        "Video Text" ,           /* 2 */
        "Extension TV",          /* 3 */
        "LaserVideoPlayer",      /* 4 */
        "VCR1",                  /* 5 */
        "VCR2",                  /* 6 */
        "Reserved",              /* 7 */
        "Sat1",                  /* 8 */
        "Extension VCR",         /* 9 */
        "Sat2",                  /* 10 */
        "Reserved",              /* 11 */
        "CD Video",              /* 12 */
        "Reserved",              /* 13 */
        "CD Photo",              /* 14 */
        "Reserved",              /* 15 */
        "Preampli Audio 1",      /* 16 */
        "Tuner",                 /* 17 */
        "Analog Magneto",        /* 18 */
        "Preampli Audio 2",      /* 19 */
        "CD",                    /* 20 */
        "Rack Audio",            /* 21 */
        "Audio Sat Receiver",    /* 22 */
        "DDC Magneto",           /* 23 */
        "Reserved",              /* 24 */
        "Reserved",              /* 25 */
        "CDRW",                  /* 26 */
        "Reserved",              /* 27 */
        "Reserved",              /* 28 */
        "Reserved",              /* 29 */
        "Reserved",              /* 30 */
        "Reserved"               /* 31 */  	             
       };

/* Table of different commands (TV) */
const char* RC5_Commands[128] = {
        "Num0",                                       /* 0 */
        "Num1",                                       /* 1 */
        "Num2",                                       /* 2 */
        "Num3",                                       /* 3 */
        "Num4",                                       /* 4 */
        "Num5",                                       /* 5 */
        "Num6",                                       /* 6 */
        "Num7",                                       /* 7 */
        "Num8",                                       /* 8 */
        "Num9",                                       /* 9 */
        "TV Digits",                                  /* 10 */
        "TV Freq",                                    /* 11 */
        "TV StandBy",                                 /* 12 */
        "TV Mute On-Off",                             /* 13 */
        "TV Preference",                              /* 14 */
        "TV Display",                                 /* 15 */
        "Volume Up",                                  /* 16 */
        "Volume Down",                                /* 17 */
        "Brightness Up",                              /* 18 */
        "Brightness Down",                            /* 19 */
        "Color Saturation Up",                        /* 20 */
        "Color Saturation Down",                      /* 21 */
        "Bass Up",                                    /* 22 */
        "Bass Down",                                  /* 23 */
        "Treble Up",                                  /* 24 */
        "Treble Down",                                /* 25 */
        "Balance Right",                              /* 26 */
        "BalanceLeft",                                /* 27 */
        "TV Contrast Up",                             /* 28 */
        "TV Contrast Down",                           /* 29 */
        "TV Search Up",                               /* 30 */
        "TV tint-hue Down",                           /* 31 */
        "TV CH Prog Up",                              /* 32 */
        "TV CH ProgDown",                             /* 33 */
        "TV Last viewed program-channel",             /* 34 */
        "TV Select stereo sound channel-language",    /* 34 */
        "TV Spacial Stereo",                          /* 36 */
        "TV Stereo Mono",                             /* 37 */
        "TV Sleep Timer",                             /* 38 */
        "TV tint-hue Up",                             /* 39 */
        "TV RF Switch",                               /* 40 */
        "TV Store-VOTE",                              /* 41 */
        "TV Time",                                    /* 42 */
        "TV Scan Fwd Inc",                            /* 43 */
        "TV Decrement",                               /* 44 */
        "Reserved",                                   /* 45 */
        "TV Secondary control-menu",                  /* 46 */
        "TV Show Clock",                              /* 47 */
        "TV Pause",                                   /* 48 */
        "TV Erase Correct Entry",                     /* 49 */
        "TV Rewind",                                  /* 50 */                                 
        "TV Goto",                                    /* 51 */
        "TV Wind",                                    /* 52 */
        "TV Play",                                    /* 53 */
        "TV Stop",                                    /* 54 */
        "TV Record",                                  /* 55 */
        "TV External 1",                              /* 56 */                                 
        "TV External 2",                              /* 57 */
        "Reserved",                                   /* 58 */
        "TV Advance",                                 /* 59 */
        "TV TXT-TV toggle",                           /* 60 */
        "TV System StandBy",                          /* 61 */
        "TV Picture Crispener",                       /* 62 */
        "System Select",                              /* 63 */
        "Reserved",                                   /* 64 */
        "Reserved",                                   /* 65 */
        "Reserved",                                   /* 66 */
        "Reserved",                                   /* 67 */
        "Reserved",                                   /* 68 */
        "Reserved",                                   /* 69 */
        "TV Speech Music",                            /* 70 */
        "DIM Local Display",                          /* 71 */
        "Reserved",                                   /* 72 */
        "Reserved",                                   /* 73 */
        "Reserved",                                   /* 74 */
        "Reserved",                                   /* 75 */
        "Reserved",                                   /* 76 */
        "Increase Linear Control Setting",            /* 77 */
        "Decrease Linear Control Setting",            /* 78 */
        "TV Sound Scroll",                            /* 79 */
        "Step Up",                                    /* 80 */
        "Step Down",                                  /* 81 */
        "Menu On",                                    /* 82 */
        "Menu Off",                                   /* 83 */
        "AV Status",                                  /* 84 */
        "Step Left",                                  /* 85 */
        "Step Right",                                 /* 86 */
        "Acknowledge",                                /* 87 */
        "PIP On Off",                                 /* 88 */
        "PIP Shift",                                  /* 89 */
        "PIP Main Swap",                              /* 90 */
        "Strobe On Off",                              /* 91 */
        "Multi Strobe",                               /* 92 */
        "Main Frozen",                                /* 93 */
        "3div9 Multi scan",                           /* 94 */
        "PIPSelect",                                  /* 95 */                                 
        "MultiPIP",                                   /* 96 */
        "Picture DNR",                                /* 97 */
        "Main Stored",                                /* 98 */
        "PIP Strobe",                                 /* 99 */
        "Recall Main Stored Picture",                 /* 100 */
        "PIP Freeze",                                 /* 101 */
        "PIP Step Up",                                /* 102 */
        "PIP Step Down",                              /* 103 */
        "TV PIP Size",                                /* 104 */
        "TV Picture Scroll",                          /* 105 */
        "TV Actuate Colored Or Other Special Keys",   /* 106 */
        "TV Red",                                     /* 107 */
        "TV Green",                                   /* 108 */
        "TV Yellow",                                  /* 109 */
        "TV Cyan",                                    /* 110 */
        "TV Index White",                             /* 111 */
        "TV Next",                                    /* 112 */
        "TV Previous",                                /* 113 */
        "Reserved",                                   /* 114 */
        "Reserved",                                   /* 115 */
        "Reserved",                                   /* 116 */
        "Reserved",                                   /* 117 */
        "Sub Mode",                                   /* 118 */
        "Option Sub Mode",                            /* 119 */
        "Reserved",                                   /* 120 */
        "Reserved",                                   /* 121 */
        "TV Store Open Close",                        /* 122 */
        "Connect",                                    /* 123 */
        "Disconnect",                                 /* 124 */
        "Reserved",                                   /* 125 */
        "TV Movie Expand",                            /* 126 */
        "TV Parental Access"                          /* 127 */
   };

extern StatusYesOrNo RC5_FrameReceived;
RC5Frame_TypeDef RC5_Frame;
uint8_t RC5_TogglePrevious = 0;
StatusYesOrNo FirstTimeIssued = YES;
  
/* Private function prototypes -----------------------------------------------*/
void USART_Config(void);

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Initialize RC5 reception */
  RC5_Receiver_Init();
  
  /* Configure LED1 GPIO */
  STM_EVAL_LEDInit(LED1);
  
  /* USART configuration for hyperterminal display */
  USART_Config();
  
  /* Display Demo start */
  printf("\n\r\n                    STMICROELECTRONICS\n\r");
  printf("\n\r      ********** STM32 - RC5 IR RECEIVER DEMO **********\n\r");

  while (1)
  {
     /* If RC5 frame has been received, then decode it */
     if (RC5_FrameReceived == YES)
     {
        /* Turn On LED1  */
        STM_EVAL_LEDOn(LED1);
        
        /* Get the RC5 frame */
        RC5_Frame = RC5_Decode();
        
        /* If the first time RC5 frame is received */
        if(FirstTimeIssued == YES)
        {
          /* Get and invert the toggle bit received at the first time to allow 
            to enter the next condition at the first time */
          RC5_TogglePrevious = ~(RC5_Frame.ToggleBit)&0x01;

          /* Initialize the variable to avoid to enter in this section next time */
          FirstTimeIssued = NO;
        }
        
        /* If the toggle bit value has changed then display the RC5 frame */
        if(RC5_Frame.ToggleBit != RC5_TogglePrevious)
        {
          /* Print RC5 Toggle bit state */
          printf("\n\rRC5 Toggle bit: %d \n\r",RC5_Frame.ToggleBit);
      
          /* Print RC5 Device and address */
          printf("RC5 Address:    %s ",RC5_Devices[RC5_Frame.Address]);
          printf("(%d) \n\r",RC5_Frame.Address);
        
          /* Print RC5 command */
          printf("RC5 Command:    %s",RC5_Commands[RC5_Frame.Command]);
          printf("(%d) \n\r",RC5_Frame.Command);
          
          RC5_TogglePrevious = RC5_Frame.ToggleBit;
        }
        
        /* Turn Off LED1  */
        STM_EVAL_LEDOff(LED1);
     }
     
     /* Wait for interrupt */
     __WFI();
  }
}

/**
  * @brief  Configures USART peripheral.
  * @param  None
  * @retval None
  */
void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;

  STM_EVAL_COMInit(COM1, &USART_InitStructure);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {}

  return ch;
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
  
  while (1)
  {}
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
  

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
