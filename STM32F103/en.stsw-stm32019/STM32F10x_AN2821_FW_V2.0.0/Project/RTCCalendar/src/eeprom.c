/**
  ******************************************************************************
  * @file RTC/src/eeprom.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file provides the EEPROM emulation firmware functions
  *        used for Time Stamping in Clock/Calendar Application
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

/* Private variables ---------------------------------------------------------*/
uint16_t Value;

/** @addtogroup RTC
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Erases EEPROM Page
  * @param  None
  * @retval : Status of the last operation (Flash write or erase) done during
  *   EEPROM formating
  */
FLASH_Status EE_Format(void)
{
  FLASH_Status FlashStatus = FLASH_COMPLETE;
  
  TamperNumber = 1;
  BKP_WriteBackupRegister(BKP_DR5,TamperNumber);

  /* Erase Page */
  FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDRESS);
  
  /* If erase operation was failed, a Flash error code is returned */
  if (FlashStatus != FLASH_COMPLETE)
  {
    return FlashStatus;
  }
  
  return FlashStatus;
}



/**
  * @brief  Writes variable data in EEPROM.
  * @param Data: 16 bit data to be written
  * @retval : - Success or error status:
  * @param FLASH_COMPLETE: on success,
  * @param PAGE_FULL: if valid page is full
  * @param Flash error code: on write Flash error
  */
uint16_t EE_WriteVariable(uint16_t Data)
{
  uint32_t LoggingTimer;
  FLASH_Status FlashStatus = FLASH_COMPLETE; 
  /* Get the valid Page start Address */
  uint32_t Address;
  uint32_t PageEndAddress;

  Address = (uint32_t)(PAGE0_BASE_ADDRESS);

  /* Get the valid Page end Address */
  PageEndAddress = (uint32_t)(PAGE0_END_ADDRESS);

  /* Check each active page address starting from begining */
  while (Address < PageEndAddress)
  {
    /* Verify if Address and Address+2 contents are 0xFFFFFFFF */
    if ((*(__IO uint32_t*)Address) == 0xFFFFFFFF)
    {
      /* Set variable data */
      FlashStatus = FLASH_ProgramHalfWord(Address, Data);
      /* If program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_COMPLETE)
      {
        return FlashStatus;
      }
      /* Return program operation status */
      return FlashStatus;
    }
    else
    {
      /* Next address location */
      Address = Address + 2;
    }
  }
  
  LCD_Clear(Red);
  LCD_SetBackColor(Red);
  LCD_DisplayString(Line4,Column5,"MEMORY FULL");
  LCD_DisplayString(Line5,Column4,"ERASING LOG");
  EE_Format();
  LoggingTimer=RTC_GetCounter();
  
  while((RTC_GetCounter() - LoggingTimer) < 2)
  {
  }
  
  MenuInit();
  return PAGE_FULL;
}



/**
  * @brief  Returns the stored variable data
  *   
  * @param  None
  * @retval : - Success or error status:
  * @param 0: if variable was found
  * @param 1: if the variable was not found
  */
uint16_t EE_ReadValue(void)
{
  uint16_t Value;
  uint8_t DisplayArrayPos=0;
  uint16_t ReadStatus = 1;
  uint32_t Address,PageStartAddress;
  uint8_t DisplayLine;

  /*Buffer for storing time stamp read from the flash memory and to be
  displayed on the LCD screen*/
  uint8_t EventLogArray[20]={0,0,0,0,255,0,0,255,0,0,0,0,0,0,10,0,0,10,0,0};
  
  DisplayLine=0;
  /* Get the valid Page start Address */
  PageStartAddress = PAGE0_BASE_ADDRESS;

  /* Get the valid Page end Address */
  Address = PageStartAddress;
  Value = *(__IO uint16_t*)(Address);
  
  /*If first element is emplty means whole page is empty */
  if(Value == 0xFFFF)
  {
    LCD_DisplayString(Line4,Column5,"NO STORED");
    LCD_DisplayString(Line5,Column2,"TAMPER EVENTS");
    return(ReadStatus);
  }
  
  LCD_SetTextColor(Yellow);
  
  /* Check each active page address starting from beginning */
  while (((*(__IO uint16_t*)(Address))!=0xFFFF) && (Address < PAGE0_END_ADDRESS))
  {
    if(DisplayArrayPos == 4)
    {
      DisplayArrayPos = 5;
    }
    if(DisplayArrayPos == 7)
    {
      DisplayArrayPos = 8;
    }
    if(DisplayArrayPos == 11)
    {
      DisplayArrayPos = 12;
    }
    if(DisplayArrayPos == 14)
    {
      DisplayArrayPos = 15;
    }
    if(DisplayArrayPos == 17)
    {
      DisplayArrayPos = 18;
    }
    
    Value = (*(__IO uint16_t*)(Address));

    /* In case variable value is read, reset ReadStatus flag */
    ReadStatus = 0;
    Address=Address +2;
    EventLogArray[DisplayArrayPos]=Value/10;
    EventLogArray[DisplayArrayPos+1]=Value%10;
    
    if(DisplayArrayPos==18)
    {
      LCD_DisplayStringLine(DisplayLine,EventLogArray);
      DisplayArrayPos=0;
      DisplayLine=DisplayLine + 24;
      
      /* If Line9 is already reached start rewriting from the beginning */
      if(DisplayLine == 240)
      {
        LCD_SetTextColor(White);
        /*Draw Vertical Line Starting from:0: X Cood,287:Y-Cood, 260:Length*/
        LCD_DrawLine(0,287,260,Vertical);
        /*Draw Vertical Line Starting from:0: X Cood,287:Y-Cood, 260:Length*/
        LCD_DrawLine(0,125,260,Vertical);
 
        if((*(__IO uint16_t*)(Address)) != 0xFFFF)
        {
          while(ReadKey() != SEL)
          {
          }
        
          LCD_SetTextColor(Yellow);
          LCD_Clear(Blue2);
          DisplayLine=0;
        }
      }
    }
    else
    {
      DisplayArrayPos=DisplayArrayPos+2;
    }
  }
  
  LCD_SetTextColor(White);
  /*Draw Vertical Line Starting from:0: X Cood,287:Y-Cood, 260:Length*/
  LCD_DrawLine(0,287,260,Vertical);
  /*Draw Vertical Line Starting from:0: X Cood,287:Y-Cood, 260:Length*/
  LCD_DrawLine(0,125,260,Vertical);
  /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
  return ReadStatus;
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
