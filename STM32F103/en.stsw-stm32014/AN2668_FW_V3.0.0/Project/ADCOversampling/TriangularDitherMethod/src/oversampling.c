/**
  ******************************************************************************
  * @file TriangularDitherMethod/src/oversampling.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains the oversampling functions body.
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
#include <stdio.h>
#include "oversampling.h"

#ifdef Thermal_Noise_Measure  
/* This parameter to compute the ADC thermal noise */ 
uint16_t ADC_ConvertedValue[Total_Samples_Number];
uint16_t ADC_Codes[Relevant_Samples_Number];
uint16_t ADC_Codes_Number[Relevant_Samples_Number];
#else
uint16_t ADC_ConvertedValue[ADC_Oversampling_Factor];
uint16_t TimerPulseWidth[BUFFER_PW_WIDTH];
#endif 
#ifdef Oversampling_Test
uint16_t ADC_Oversampled_Buffer[Oversampling_Test_Samples];
#endif


/** @addtogroup TriangularDitherMethod
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/    
/* Private function prototypes -----------------------------------------------*/
void Oversampling_InitBuffer ( uint16_t * Buffer , uint32_t size );

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Initializes a data buffer which size is passed in parameter 
  *   Input1         : buffer   
  *   Input2         : buffer size
  * @retval : None
  */
void Oversampling_InitBuffer( uint16_t * Buffer , uint32_t size )
{
  uint32_t index = 0;
  for( index = 0; index < size; index++)
  Buffer[index] = 0;
} 



/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval : None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {
  }
  return ch;
} 



/**
  * @brief  Initializes The different data buffers used in the 
  *   oversampling alogorithm according to the different cases:
  *   Thermal_Noise_Measure, Oversampling_Test 
  * @param uint16_t buffer   
  * @arg uint32_t: buffer size
  * @retval : None
  */
void  Oversampling_InitData( void )
{
  #ifdef  Thermal_Noise_Measure
  Oversampling_InitBuffer( ADC_ConvertedValue, Total_Samples_Number);
  Oversampling_InitBuffer( ADC_Codes, Relevant_Samples_Number);
  Oversampling_InitBuffer( ADC_Codes_Number, Relevant_Samples_Number);
  #else
  Oversampling_InitBuffer( ADC_ConvertedValue, ADC_Oversampling_Factor); 
  /* Init the Capture compare values for the triangular waveform generation */
  Oversampling_ComputePulseWidthBuffer();
  #endif
  #ifdef Oversampling_Test
  Oversampling_InitBuffer ( ADC_Oversampled_Buffer, Oversampling_Test_Samples);
  #endif  
}  



/**
  * @brief  Determines the number of each code bin included in the 
  *   samples. 
  * @param uint16_t buffer conatining the code bins  
  * @arg uint16_t buffer containing the number of the corresponding code
  * @arg uint32_t: Code bins Number 
  * @retval : None
  */
#ifdef Thermal_Noise_Measure
void  Oversampling_TestADCThermalNoise( __IO uint16_t *ADC_Code, __IO uint16_t *ADC_Code_Number)
{
  uint32_t index = 0;
  uint32_t i = 0; 
  uint32_t pointeur = 0; 
  
  do
  {  
    if ( index == 0)
     {
      ADC_Code[0] = ADC_ConvertedValue[0];
      ADC_Code_Number[0] = 1;
     } 
     else
     {
       i = 0; 
       do
        {
          if( ADC_ConvertedValue[index] == ADC_Code[i])
          {
            ADC_Code_Number[i]++;
            break;
          } 
          i++;
        } while( i<= pointeur);
        
        if( i == (pointeur+1))
        {
          pointeur++;
          ADC_Code[pointeur] = ADC_ConvertedValue[index];
          ADC_Code_Number[pointeur] = 1;
        } 
    } /* end else */  
    index++;
  } while( index < Total_Samples_Number);
  
 Oversampling_CopyConvValuesToFile(  ADC_Code , 100 );
 
 printf( "The occurance of each ADC code is as follow:\n");
        
 Oversampling_CopyConvValuesToFile(  ADC_Code_Number , 100 );

}  
#endif



/**
  * @brief  Gives the ADC oversampled conversion result  
  * @param  None 
  * @retval : Oversampled Result
  */
uint16_t Oversampling_GetConversion ( void )
{
  uint32_t index = 0;
  uint32_t result = 0;
  
  for( index = 0; index < ADC_Oversampling_Factor ; index++)
    result += ADC_ConvertedValue[index];
  
  result = (result + 1 ) >> 1;
  
  /* This line is to compensate the offset AOP VIN+ = 3.3V */
  result =  OFFSET_3V - result; 
  
 return ( (uint16_t)result );
    
}  



/**
  * @brief  Copies the ADC converted value into a text file  
  *   Input1         : Buffer
  *   Input2         : Buffer size
  * @retval : None
  */
void Oversampling_CopyConvValuesToFile ( __IO uint16_t *Tab, uint32_t size )
{    
  uint32_t index = 0;

  for ( index = 0; index < size ; index++ )
   {
	 printf("%x\r\n", Tab[index]);
   }  
   printf ("End\n\r");
}



/**
  * @brief  Computes the Pulse width buffer that is used to update the 
  *   Compare register of Timer 3 used for the triangular waveform
  *   generation
  * @param  None   
  * @retval : None
  */
#ifndef Thermal_Noise_Measure
void Oversampling_ComputePulseWidthBuffer ( void)
{
  uint32_t index = 0; 
  uint32_t Half_Size = ( ADC_Oversampling_Factor >> 1 );
   
   TimerPulseWidth[0] = TIM_PWM_STEP;
   
  for( index = 1 ; index < Half_Size - 1 ; index++)
    TimerPulseWidth[ index] = TimerPulseWidth[index - 1] + TIM_PWM_STEP;
  
 for( index = 0; index < Half_Size - 1 ; index++)
    TimerPulseWidth[Half_Size + index] = TimerPulseWidth[Half_Size - index-2];
 
 TimerPulseWidth[Half_Size - 1 ] = TIM_PWM_PERIOD;
 
} 
#endif

 
/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
