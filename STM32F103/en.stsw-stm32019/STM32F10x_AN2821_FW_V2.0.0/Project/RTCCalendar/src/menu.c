/**
  ******************************************************************************
  * @file RTC/src/menu.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains the basic routines for MENU implementation                    
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

/* Private variables----------------------------------------------------------*/
uint8_t DisplayCurrentPos;
uint8_t DisplayPrevPos;
uint32_t Dummy;
uint16_t Counter=0;
uint8_t ArrayTime[8]={0,0,0,0,0,0,0,0};
uint8_t CounterPosition=0;
uint8_t MenuLevelPointer=1;
uint8_t TotalMenuPointer=1;
uint8_t TamperNumber; /* Keeps the number of tamper Events already happened */
uint8_t AlarmDate=0;
uint8_t TimeDateDisplay=0;
uint32_t TimerFrequency;


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
  * @brief  This Function is executed when Joystick Up Button is pressed
  * @param  None
  * @retval : None
  *   
  */
void UpFunction(void)
{
  uint8_t CounterMax = 0, CounterMin = 0;

  /* Determine maximum and minimum allowed values for the elements of time */
  if(TotalMenuPointer == 1 || TotalMenuPointer == 2)
  {
    CounterMin = 0;
   
    if(CounterPosition == 0)
    {
      if(ArrayTime[1]<4)
      {
        CounterMax=2;
      }
      else
      {
        CounterMax=1;
      }
    }
    else if(CounterPosition==1)
    {
      if(ArrayTime[0]==2)
      {
        CounterMax=3;
      }
      else
      {
        CounterMax=9;
      }
    }
    else if(CounterPosition==2 || CounterPosition==4)
    {
      CounterMax=5;
    }
    else if(CounterPosition==3 || CounterPosition==5)
    {
      CounterMax=9;
    }
  }
  /* Determine maximum and minimum allowed values for the elements of Date */
  else if(TotalMenuPointer==3)
  {
    if(CounterPosition==0)
    {
      if(ArrayTime[1]>1)
      {
        CounterMax=2;
      }
      else
      {
        CounterMax=3;
      }
      if(ArrayTime[1]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==1)
    {
      if(ArrayTime[0]==3)
      {
        CounterMax=1;
      }
      else
      {
        CounterMax=9;
      }
      if(ArrayTime[0]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==2)
    {
      if(ArrayTime[3]>2)
      {
        CounterMax=0;
      }
      else
      {
        CounterMax=1;
      }
      if(ArrayTime[3]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==3)
    {
      if(ArrayTime[2]==1)
      {
        CounterMax=2;
      }
      else
      {
        CounterMax=9;
      }
      if(ArrayTime[2]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==4)
    {
      CounterMax=2;
      CounterMin=2;
    }
    else if( CounterPosition==5 || CounterPosition==6 || CounterPosition==7)
    {
      CounterMax=9;
      CounterMin=0;
    }
  }
  else
  {
    CounterMax=9;
    CounterMin=0;
  }
  
  Counter=ArrayTime[CounterPosition];
  Counter++;
  
  if(Counter>CounterMax)
  {
    Counter=CounterMin;
  }
  
  ArrayTime[CounterPosition]=Counter;
  LCD_DisplayChar(Line4,Column6-(DisplayCurrentPos*16),\
                    ArrayTime[CounterPosition] + 0x30);
}



/**
  * @brief  This Function is executed when Joystick Down Button is pressed
  * @param  None
  * @retval : None
  */
void DownFunction(void)
{
  uint8_t CounterMax=0, CounterMin=0;
  
  /* Determine maximum and minimum allowed values for the elements of time */
  if(TotalMenuPointer==1 || TotalMenuPointer==2)
  {
    CounterMin=0;
    if(CounterPosition==0)
    {
      if(ArrayTime[1]<4)
      {
        CounterMax=2;
      }
      else
      {
        CounterMax=1;
      }
    }
    else if(CounterPosition==1)
    {
      if(ArrayTime[0]==2)
      {
        CounterMax=3;
      }
      else
      {
         CounterMax=9;
      }
    }
    else if(CounterPosition==2 || CounterPosition==4)
    {
      CounterMax=5;
    }
    else if(CounterPosition==3 || CounterPosition==5)
    {
      CounterMax=9;
    }
  }
  /* Determine maximum and minimum allowed values for the elements of date*/
  else if(TotalMenuPointer==3)
  {
    if(CounterPosition==0)
    {
      if(ArrayTime[1]>1)
      {
        CounterMax=2;
      }
      else
      {
        CounterMax=3;
      }

      if(ArrayTime[1]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==1)
    {
      if(ArrayTime[0]==3)
      {
        CounterMax=1;
      }
      else
      {
        CounterMax=9;
      }

      if(ArrayTime[0]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==2)
    {
      if(ArrayTime[3]>2)
      {
        CounterMax=0;
      }
      else
      {
        CounterMax=1;
      }
      
      if(ArrayTime[3]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==3)
    {
      if(ArrayTime[2]==1)
      {
        CounterMax=2;
      }
      else
      {
        CounterMax=9;
      }
      
      if(ArrayTime[2]==0)
      {
        CounterMin=1;
      }
      else
      {
        CounterMin=0;
      }
    }
    else if(CounterPosition==4)
    {
      CounterMax=2;
      CounterMin=2;
    }
    else if(CounterPosition==5 || CounterPosition==6 \
              || CounterPosition==7)
    {
      CounterMax=9;
      CounterMin=0;
    }
  }
  else
  {
    CounterMax=9;
    CounterMin=0;
  }
  
  Counter=ArrayTime[CounterPosition];
  
  if(Counter==CounterMin)
  {
    Counter=CounterMax+1;
  }
  
  Counter-=1;
  ArrayTime[CounterPosition]=Counter;
  LCD_DisplayChar(Line4,Column6-(DisplayCurrentPos*16),\
                  ArrayTime[CounterPosition] + 0x30);
}



/**
  * @brief  This Function is executed when Joystick Right Button is pressed
  * @param  None
  * @retval : None
  */
void RightFunction(void)
{
  uint8_t PrevElement=0;

  switch(MenuLevelPointer)
  {
    case 1: TotalMenuPointer++;
            if(TotalMenuPointer > 9)
            {
              TotalMenuPointer=1;
            }
            if(TotalMenuPointer==2)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            } 
            else if (TotalMenuPointer==3)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            }
            else if (TotalMenuPointer==1)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            }
            else if(TotalMenuPointer==4)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            }
            else if(TotalMenuPointer==5)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            }
            else if(TotalMenuPointer==6)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            }
            else if(TotalMenuPointer==7)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            }
            else if(TotalMenuPointer==8)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line7,Column15,"60Hz");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
            }
            else if(TotalMenuPointer==9)
            {
              LCD_SetTextColor(Green);
              LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
              LCD_DisplayString(Line3,Column5,"FEATURES");
              LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
              LCD_SetTextColor(White);
              LCD_SetBackColor(Green);
              LCD_DisplayString(Line9,Column5,"CALIBRATE");
              LCD_SetBackColor(Blue2);
              LCD_DisplayString(Line1,Column1,"Time");
              LCD_DisplayString(Line1,Column7,"Alarm");
              LCD_DisplayString(Line1,Column14,"Date");
              LCD_DisplayString(Line4,Column1,"LowPower");
              LCD_DisplayString(Line4,Column11,"Tamper");
              LCD_DisplayString(Line7,Column0,"32768Hz");
              LCD_DisplayString(Line7,Column9,"50Hz");
              LCD_DisplayString(Line7,Column15,"60Hz");
            }
        break;
        
    case 2: if(TotalMenuPointer==1 || TotalMenuPointer==2 \
                          || TotalMenuPointer==3)
            {
              CounterPosition++;

              if(TotalMenuPointer==1 || TotalMenuPointer==2)
              {
                if(CounterPosition==6)
                {
                  CounterPosition=0;
                  PrevElement=5;
                }
                else
                {
                  PrevElement=CounterPosition-1;
                }
              }
              else if(TotalMenuPointer==3)
              {
                if(CounterPosition==8)
                {
                  CounterPosition=0;
                  PrevElement=7;
                }
                else
                {
                  PrevElement=CounterPosition-1;
                }
              }
                  
              Counter=ArrayTime[CounterPosition];
                  
              if(TotalMenuPointer == 1 || TotalMenuPointer == 2)
              {
                if(DisplayCurrentPos==0x07)
                {
                  DisplayCurrentPos=255;
                }
              }
              else if(TotalMenuPointer == 3)
              {
                if(DisplayCurrentPos==0x09)
                {
                  DisplayCurrentPos=0xFF;
                }
              }

              if(CounterPosition==2 || CounterPosition==4)
              {
                DisplayCurrentPos+=2;
                DisplayPrevPos=DisplayCurrentPos-2;
              }
              else
              {
                DisplayCurrentPos++;
                DisplayPrevPos=DisplayCurrentPos-1;
              }
                
              if(TotalMenuPointer==1 || TotalMenuPointer==2)
              {
                if(DisplayPrevPos==255)
                {
                  DisplayPrevPos=7;
                }
              }
              else if(TotalMenuPointer==3)
              {
                if(DisplayPrevPos==255)
                {
                  DisplayPrevPos=9;
                }
              }

              LCD_DisplayCount(Line4,Column6-((DisplayPrevPos)*16),0,\
                                    ArrayTime[PrevElement] + 0x30);
              LCD_DisplayCount(Line4,Column6-(DisplayCurrentPos*16),1,\
                                    ArrayTime[CounterPosition] + 0x30);
            }

             if((TotalMenuPointer==9 || TotalMenuPointer==12) && MenuLevelPointer==2)
             {
                if(TotalMenuPointer==9)
                  {
                    TotalMenuPointer=12;
                  }
                  else if(TotalMenuPointer==12)
                  {
                    TotalMenuPointer=9;
                  }
                 if(TotalMenuPointer==9)
                  {
                    LCD_SetBackColor(Green);
                    LCD_DisplayString(Line4,Column2,"MANUAL");
                    LCD_SetBackColor(Blue2);
                    LCD_DisplayString(Line4,Column12,"AUTO");
                  }
                  else if(TotalMenuPointer==12)
                  {
                    LCD_SetBackColor(Green);
                    LCD_DisplayString(Line4,Column12,"AUTO");
                    LCD_SetBackColor(Blue2);
                    LCD_DisplayString(Line4,Column2,"MANUAL");
                  }
              }

              if(TotalMenuPointer==11 || TotalMenuPointer==10)
              {
                if(TotalMenuPointer==11)
                {
                  TotalMenuPointer=10;
                }
                else if(TotalMenuPointer==10)
                {
                   TotalMenuPointer=11;
                }
                if(TotalMenuPointer==10)
                {
                  LCD_SetBackColor(Green);
                  LCD_DisplayString(Line4,Column4,"STOP");
                  LCD_SetBackColor(Blue2);
                  LCD_DisplayString(Line4,Column10,"STANDBY");
                }
                else if(TotalMenuPointer==11)
                {
                  LCD_SetBackColor(Green);
                  LCD_DisplayString(Line4,Column10,"STANDBY");
                  LCD_SetBackColor(Blue2);
                  LCD_DisplayString(Line4,Column4,"STOP");
                }
               }

               if(TotalMenuPointer==5 || TotalMenuPointer==6)
               {
                 if(TotalMenuPointer==5)
                 {
                   TotalMenuPointer=6;
                 }
                 else if(TotalMenuPointer==6)
                 {
                   TotalMenuPointer=5;
                 }
                 if(TotalMenuPointer==5)
                 {
                   LCD_SetBackColor(Green);
                   LCD_DisplayString(Line4,Column4,"VIEW");
                   LCD_SetBackColor(Blue2);
                   LCD_DisplayString(Line4,Column10,"ERASE");
                 }
                 else if(TotalMenuPointer==6)
                 {
                   LCD_SetBackColor(Green);
                   LCD_DisplayString(Line4,Column10,"ERASE");
                   LCD_SetBackColor(Blue2);
                   LCD_DisplayString(Line4,Column4,"VIEW");
                  }
                }
              break;
              
    case 3: if(TotalMenuPointer==9)
            {
              CounterPosition++;

              if(CounterPosition==3)
              {
                CounterPosition=0;
                PrevElement=2;
              }
              else
              {
                PrevElement=CounterPosition-1;
              }
               
              Counter=ArrayTime[CounterPosition];
               
              if(DisplayCurrentPos==0x02)
              {
                DisplayCurrentPos=0xFF;
              }
               
              DisplayCurrentPos++;
              DisplayPrevPos=DisplayCurrentPos-1;
               
              if(DisplayPrevPos==255)
              {
                DisplayPrevPos=2;
              }
              
              LCD_DisplayCount(Line4,Column6-((DisplayPrevPos)*16),0,\
                                  ArrayTime[PrevElement] + 0x30);
              LCD_DisplayCount(Line4,Column6-(DisplayCurrentPos*16),1,\
                                  ArrayTime[CounterPosition] + 0x30);
            }
          break;
  }
}



/**
  * @brief  This Function is executed when Left Button is pressed
  * @param  None
  * @retval : None
  *   
  */
void LeftFunction(void)
{
  uint8_t PrevElement=0;
  
  switch(MenuLevelPointer)
  {
    case 1: TotalMenuPointer--;
           if(TotalMenuPointer==0)
           {
             TotalMenuPointer=9;
           }

           if(TotalMenuPointer==2)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if (TotalMenuPointer==3)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if (TotalMenuPointer==1)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if (TotalMenuPointer==4)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if(TotalMenuPointer==5)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if(TotalMenuPointer==6)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if(TotalMenuPointer==7)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if(TotalMenuPointer==8)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line7,Column15,"60Hz");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
           }
           else if(TotalMenuPointer==9)
           {
             LCD_SetTextColor(Green);
             LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
             LCD_DisplayString(Line3,Column5,"FEATURES");
             LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
             LCD_SetTextColor(White);
             LCD_SetBackColor(Green);
             LCD_DisplayString(Line9,Column5,"CALIBRATE");
             LCD_SetBackColor(Blue2);
             LCD_DisplayString(Line1,Column1,"Time");
             LCD_DisplayString(Line1,Column7,"Alarm");
             LCD_DisplayString(Line1,Column14,"Date");
             LCD_DisplayString(Line4,Column1,"LowPower");
             LCD_DisplayString(Line4,Column11,"Tamper");
             LCD_DisplayString(Line7,Column0,"32768Hz");
             LCD_DisplayString(Line7,Column9,"50Hz");
             LCD_DisplayString(Line7,Column15,"60Hz");
           }
         break;
         
    case 2: if(TotalMenuPointer == 1 || TotalMenuPointer == 2 \
                      || TotalMenuPointer == 3)
            {
              CounterPosition--;

              if(CounterPosition==0xFF)
              {
                if(TotalMenuPointer == 1 || TotalMenuPointer == 2)
                {
                  CounterPosition=5;
                  PrevElement=0;
                }
                else if(TotalMenuPointer == 3)
                {
                  CounterPosition=7;
                  PrevElement=0;
                }
              }
              else
              {
                PrevElement=CounterPosition+1;
              }

              if(CounterPosition==3 || CounterPosition==1)
              {
                DisplayCurrentPos=DisplayCurrentPos-2;
                DisplayPrevPos=DisplayCurrentPos+2;
              }
              else
              {
                DisplayCurrentPos=DisplayCurrentPos-1;
                DisplayPrevPos=DisplayCurrentPos+1;
              }

              Counter=ArrayTime[CounterPosition];
              
              if(DisplayCurrentPos==0xFF)
              {
                if(TotalMenuPointer == 1 || TotalMenuPointer == 2)
                {
                  DisplayCurrentPos=7;
                }
                else if (TotalMenuPointer == 3)
                {
                  DisplayCurrentPos=9;
                }
              }

              LCD_DisplayCount(Line4,Column6-((DisplayPrevPos)*16),0,\
                            ArrayTime[PrevElement] + 0x30);
              LCD_DisplayCount(Line4,Column6-(DisplayCurrentPos*16),1,\
                            ArrayTime[CounterPosition] + 0x30);
            }

            if((TotalMenuPointer==9 || TotalMenuPointer==12) && MenuLevelPointer==2)
            {
              if(TotalMenuPointer==9)
              {
                TotalMenuPointer=12;
              }
              else if(TotalMenuPointer==12)
              {
                TotalMenuPointer=9;
              }
   
              if(TotalMenuPointer==9)
              {
                LCD_SetBackColor(Green);
                LCD_DisplayString(Line4,Column2,"MANUAL");
                LCD_SetBackColor(Blue2);
                LCD_DisplayString(Line4,Column12,"AUTO");
              }
              else if(TotalMenuPointer==12)
              {
                LCD_SetBackColor(Green);
                LCD_DisplayString(Line4,Column12,"AUTO");
                LCD_SetBackColor(Blue2);
                LCD_DisplayString(Line4,Column2,"MANUAL");
              }
            }

            if(TotalMenuPointer==11 || TotalMenuPointer==10)
            {
              if(TotalMenuPointer==11)
              {
                TotalMenuPointer=10;
              }
              else if(TotalMenuPointer==10)
              {
                TotalMenuPointer=11;
              }

              if(TotalMenuPointer==10)
              {
                LCD_SetBackColor(Green);
                LCD_DisplayString(Line4,Column4,"STOP");
                LCD_SetBackColor(Blue2);
                LCD_DisplayString(Line4,Column10,"STANDBY");
              }
              else if(TotalMenuPointer==11)
              {
                LCD_SetBackColor(Green);
                LCD_DisplayString(Line4,Column10,"STANDBY");
                LCD_SetBackColor(Blue2);
                LCD_DisplayString(Line4,Column4,"STOP");
              }
            }
     
            if(TotalMenuPointer==5 || TotalMenuPointer==6)
            {
              if(TotalMenuPointer==5)
                {
                  TotalMenuPointer=6;
                }
                else if(TotalMenuPointer==6)
                {
                  TotalMenuPointer=5;
                }
            
              if(TotalMenuPointer==5)
              {
                LCD_SetBackColor(Green);
                LCD_DisplayString(Line4,Column4,"VIEW");
                LCD_SetBackColor(Blue2);
                LCD_DisplayString(Line4,Column10,"ERASE");
              }
              else if(TotalMenuPointer==6)
              {
                LCD_SetBackColor(Green);
                LCD_DisplayString(Line4,Column10,"ERASE");
                LCD_SetBackColor(Blue2);
                LCD_DisplayString(Line4,Column4,"VIEW");
              }
            }
          break;

    case 3: if(TotalMenuPointer==9)
            {
              CounterPosition--;

              if(CounterPosition==0xFF)
              {
                CounterPosition=2;
                PrevElement=0;
              }
              else
              {
                PrevElement=CounterPosition+1;
              }

              DisplayCurrentPos=DisplayCurrentPos-1;
              DisplayPrevPos=DisplayCurrentPos+1;
              Counter=ArrayTime[CounterPosition];

              if(DisplayCurrentPos==0xFF)
              {
                DisplayCurrentPos=2;
              }

              LCD_DisplayCount(Line4,Column6-((DisplayPrevPos)*16),0,\
                                    ArrayTime[PrevElement] + 0x30);
              LCD_DisplayCount(Line4,Column6-(DisplayCurrentPos*16),1,\
                                    ArrayTime[CounterPosition] + 0x30);
            }
          break;
  }
}



/**
  * @brief  This Function is executed when Sel Button is pressed
  * @param  None
  * @retval : None
  *   
  */
void SelFunction(void)
{
  uint32_t EventTimer;
  uint8_t ArrayPos;
  uint16_t CalibrationValue;
  uint8_t DisplayPos=0;
  
  MenuLevelPointer++;
  
  if(MenuLevelPointer !=0)
  {
    TimeDateDisplay=1;
  }
  
  if(MenuLevelPointer==1)
  {
    BKP_TamperPinCmd(DISABLE);
    LCD_Clear(Blue2);
    LCD_SetTextColor(Green);
    LCD_DisplayString(Line0,Column1,"CLOCK/CALENDAR SET");
    LCD_DisplayString(Line3,Column5,"FEATURES");
    LCD_DisplayString(Line6,Column1,"CLOCK SOURCE SELECT");
    LCD_SetTextColor(White);
    LCD_SetBackColor(Green);
    LCD_DisplayString(Line1,Column1,"Time");
    LCD_SetBackColor(Blue2);
    LCD_DisplayString(Line1,Column7,"Alarm");
    LCD_DisplayString(Line1,Column14,"Date");
    LCD_DisplayString(Line4,Column1,"LowPower");
    LCD_DisplayString(Line4,Column11,"Tamper");
    LCD_DisplayString(Line7,Column0,"32768Hz");
    LCD_DisplayString(Line7,Column9,"50Hz");
    LCD_DisplayString(Line7,Column15,"60Hz");
    LCD_DisplayString(Line9,Column5,"CALIBRATE");
    RightLeftIntExtOnOffConfig(ENABLE);
  }

  /* Enter the System Time */
  if(MenuLevelPointer==2 && TotalMenuPointer==1)
  {
    UpDownIntOnOffConfig(ENABLE);
    LCD_Clear(Blue2);
    LCD_SetTextColor(Green);
    LCD_DisplayString(Line2,Column6,"Set Time ");
    LCD_SetTextColor(White);
    /* Display two colons (:) for seperating hours, minutes and seconds */
    LCD_DisplayChar(Line4,Column8,ASCII_COLON);
    LCD_DisplayChar(Line4,Column11,ASCII_COLON);
    InitialiseTime();
    LCD_DisplayCount(Line4,Column6,1,ArrayTime[0]+0x30);
    LCD_SetBackColor(Blue2);
    
    for(ArrayPos=1;ArrayPos<=5;ArrayPos++)
    {
      if(!((ArrayPos==2) || (ArrayPos==4)))
      {
        DisplayPos ++;
      }
      else
      {
        DisplayPos+=2;
      }
      LCD_DisplayChar(Line4,Column6-(16*DisplayPos),ArrayTime[ArrayPos]+0x30);
    }
    
    LCD_DisplayString(Line6,Column6,"To Quit");
    LCD_DisplayString(Line7,Column0,"Do not press any key");
    LCD_DisplayString(Line8,Column3,"for 8 seconds");
    LCD_SetBackColor(Green);
  }

  /* Enter the Alarm Time */
  if(MenuLevelPointer==2 && TotalMenuPointer==2)
  {
    UpDownIntOnOffConfig(ENABLE);
    LCD_Clear(Blue2);
    LCD_SetTextColor(Green);
    LCD_DisplayString(Line2,Column2,"Set Alarm Time");
    LCD_SetTextColor(White);
    /* Display two colons (:) for seperating hours, minutes and seconds */
    LCD_DisplayChar(Line4,Column8,ASCII_COLON);
    LCD_DisplayChar(Line4,Column11,ASCII_COLON);
    InitialiseTime();
    LCD_DisplayCount(Line4,Column6,1,ArrayTime[0]+0x30);
    LCD_SetBackColor(Blue2);
    
    for(ArrayPos=1;ArrayPos<=5;ArrayPos++)
    {
      if(!((ArrayPos==2) || (ArrayPos==4)))
      {
        DisplayPos ++;
      }
      else
      {
        DisplayPos+=2;
      }
      LCD_DisplayChar(Line4,Column6-(16*DisplayPos),ArrayTime[ArrayPos]+0x30);
    }
    
    LCD_DisplayString(Line6,Column6,"To Quit");
    LCD_DisplayString(Line7,Column0,"Do not press any key");
    LCD_DisplayString(Line8,Column3,"for 8 seconds");
    LCD_SetBackColor(Green);
  }

  /* Enter the System Date/Alarm Date */
  if(MenuLevelPointer==2 && TotalMenuPointer==3)
  {
    UpDownIntOnOffConfig(ENABLE);
    LCD_Clear(Blue2);
    LCD_SetTextColor(Green);
    
    if(AlarmDate==0)
    {
      LCD_DisplayString(Line2,Column7,"Set Date ");
    }
    else
    {
      LCD_DisplayString(Line2,Column4,"Set Alarm Date ");
    }
    
    LCD_SetTextColor(White);
    InitialiseDate();
    LCD_DisplayChar(Line4,Column8,ASCII_SLASH);
    LCD_DisplayChar(Line4,Column11,ASCII_SLASH);
    LCD_DisplayCount(Line4,Column6,1,(ArrayTime[0]+0x30));
    LCD_SetBackColor(Blue2);
    
    for(ArrayPos=1;ArrayPos <= 7;ArrayPos++)
    {
      if(!((ArrayPos==2) || (ArrayPos==4)))
      {
        DisplayPos++;
      }
      else
      {
        DisplayPos+=2;
      }

      LCD_DisplayChar(Line4,Column6-(16*DisplayPos),ArrayTime[ArrayPos]+0x30);
    }
    
    LCD_DisplayString(Line6,Column6,"To Quit");
    LCD_DisplayString(Line7,Column0,"Do not press any key");
    LCD_DisplayString(Line8,Column3,"for 8 seconds");
    LCD_SetBackColor(Green);
  }


  /* Enter into Low Power Mode(STOP/STANDBY) selection (LowPowerMode is selected) */
  if(MenuLevelPointer==2 && TotalMenuPointer==4)
  {
    LCD_Clear(Blue2);
    LCD_SetBackColor(Green);
    LCD_DisplayString(Line4,Column4,"STOP");
    LCD_SetBackColor(Blue2);
    LCD_DisplayString(Line4,Column10,"STANDBY");
    TotalMenuPointer=10;
  }

  /* Enter into STOP MODE */
  if(MenuLevelPointer==3 && TotalMenuPointer==10)
  {
    LCD_Clear(Blue2);
    LCD_DisplayString(Line3,Column2,"MCU IN STOP MODE");
    LCD_DisplayString(Line6,Column1,"PRESS SEL TO EXIT");
    RightLeftIntExtOnOffConfig(DISABLE);
    /* LED D4 Glows */
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
    PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI);
    ReturnFromStopMode();
    CheckForDaysElapsed();
  }

  /* Enter into STANDBY MODE  */
  if(MenuLevelPointer==3 && TotalMenuPointer==11)
  {
    LCD_Clear(Blue2);
    LCD_DisplayString(Line3,Column0-5,"MCU IN STANDBY MODE");
    LCD_DisplayString(Line6,Column0-2,"PRESS WAKEUP TO EXIT");
    /* Enable Wake up pin for waking the micro from standby mode */
    PWR_WakeUpPinCmd(ENABLE);
    /* Enter into standby mode  */
    PWR_EnterSTANDBYMode();
  }

  /* Tamper events section  */
  if(MenuLevelPointer==2 && TotalMenuPointer==5)
  {
    LCD_Clear(Blue2);
    LCD_SetBackColor(Green);
    LCD_DisplayString(Line4,Column4,"VIEW");
    LCD_SetBackColor(Blue2);
    LCD_DisplayString(Line4,Column10,"ERASE");
  }

  /*Read Tamper events and display on the LCD*/
  if(MenuLevelPointer==3 && TotalMenuPointer==5)
  {
    SelIntExtOnOffConfig(DISABLE);
    LCD_Clear(Blue2);
    EE_ReadValue();

    while(ReadKey() != SEL)
    {
    }
    MenuInit();
  }

  /*Erase Tamper Log */
  if(MenuLevelPointer==3 && TotalMenuPointer==6)
  {
    SelIntExtOnOffConfig(DISABLE);
    EE_Format();
    LCD_Clear(Red);
    LCD_SetBackColor(Red);
    LCD_DisplayString(Line4,Column1,"TAMPER LOG ERASED");
    TamperNumber=0;
    BKP_WriteBackupRegister(BKP_DR5,TamperNumber);
    EventTimer= RTC_GetCounter();
    while((RTC_GetCounter()-EventTimer) < 2)
    {
    }
    MenuInit();
  }

 /* Clock Source Select - Selection_32768Hz   */
  if(MenuLevelPointer==2 && TotalMenuPointer==6)
  {
    BKP_WriteBackupRegister(BKP_DR1, CONFIGURATION_RESET);
    RTC_Configuration();
    MenuInit();
  }

 /* Clock Source Select - Selection_50Hz   */
  if(MenuLevelPointer==2 && TotalMenuPointer==7)
  {
    BKP_WriteBackupRegister(BKP_DR1, CONFIGURATION_RESET);
    RTC_Configuration();
    MenuInit();
  }
   /* Clock Source Select - Selection_50Hz   */
  if(MenuLevelPointer==2 && TotalMenuPointer==8)
  {
    BKP_WriteBackupRegister(BKP_DR1, CONFIGURATION_RESET);
    RTC_Configuration();
    MenuInit();
  }

  /* Clock Calibration Menu */
  if(MenuLevelPointer==2 && TotalMenuPointer==9)
  {
    /* Calibration is done only in case external crystal oscillator - 32 KHz is used */
    if(BKP_ReadBackupRegister(BKP_DR6) == 1)
    {
      LCD_Clear(Blue2);
      LCD_SetBackColor(Green);
      LCD_DisplayString(Line4,Column2,"MANUAL");
      LCD_SetBackColor(Blue2);
      LCD_DisplayString(Line4,Column12,"AUTO");
    }
    else
    {
      /* Probably 50/60 Hz external clock source is selected */
      LCD_Clear(Red);
      LCD_SetBackColor(Red);
      LCD_DisplayString(Line3,Column1, "No Calibration in");
      LCD_DisplayString(Line4,Column1,"case of 50/60Hz  ");
      LCD_DisplayString(Line6,Column1,"clock source");
    }
  }

  /* Manual Calibration Routine  */
  if(MenuLevelPointer==3 && TotalMenuPointer==9)
  {
    ManualClockCalibration();
  }

  /* Auto Calibration Routine */
  if(MenuLevelPointer==3 && TotalMenuPointer==12)
  {
    AutoClockCalibration();
  }
  
  if(MenuLevelPointer==4 && TotalMenuPointer==9)
  {
    CalibrationValue=(ArrayTime[0]*100)+(ArrayTime[1]*10)+ArrayTime[2];
  
    if(CalibrationValue<=121)
    {
      BKP_SetRTCCalibrationValue(CalibrationValue);
    }
    else
    {
      LCD_Clear(Red);
      LCD_SetBackColor(Red);
      LCD_DisplayString(Line4,Column4,"Invalid Value !");
      EventTimer= RTC_GetCounter();
      
      while(RTC_GetCounter()-EventTimer < 2)
      {
      }
    }
    MenuInit();
  }

  /*Set System time */
  if(MenuLevelPointer==3 && TotalMenuPointer==1)
  {
    SetTime((ArrayTime[0]*10)+ArrayTime[1],(ArrayTime[2]*10)+\
    ArrayTime[3],(ArrayTime[4]*10)+ArrayTime[5]);
    MenuInit();
    CounterPosition=0;
    UpDownIntOnOffConfig(DISABLE);
    TimeDateDisplay=0;
  }

 /*Set Alarm Time and Date*/
  if(MenuLevelPointer==3 && TotalMenuPointer==2)
  {
    SetAlarm((ArrayTime[0]*10)+ArrayTime[1],(ArrayTime[2]*10)+ArrayTime[3],\
      (ArrayTime[4]*10)+ArrayTime[5]);
    AlarmDate=1;
    MenuLevelPointer=1;
    TotalMenuPointer=3;
    
    for(ArrayPos=0;ArrayPos<7;ArrayPos++)
    {
      ArrayTime[ArrayPos]=0;
    }
    
    CounterPosition=0;
    DisplayCurrentPos=0;
    DisplayPrevPos=0;
    Counter=0;
    LCD_SetBackColor(Blue2);
    /* For entering Alarm date */
    SelFunction();
  }

  /*Set System date */
  if(MenuLevelPointer==3 && TotalMenuPointer==3)
  {
    SetDate((ArrayTime[0]*10)+ArrayTime[1],(ArrayTime[2]*10)+ArrayTime[3],\
      ((ArrayTime[4]*1000)+(ArrayTime[5]*100)+(ArrayTime[6]*10)+ArrayTime[7]));
    MenuInit();
    
    for(ArrayPos=0;ArrayPos<6;ArrayPos++)
    {
      ArrayTime[ArrayPos]=0;
    }
    
    CounterPosition=0;
    UpDownIntOnOffConfig(DISABLE);
    TimeDateDisplay=0;
  }
}



/**
  * @brief  Initialises ArrayTime[] buffer to current time
  * @param  None
  * @retval : None
  *   
  */
void InitialiseTime(void)
{
   ArrayTime[0]=s_TimeStructVar.HourHigh;
   ArrayTime[1]=s_TimeStructVar.HourLow;
   ArrayTime[2]=s_TimeStructVar.MinHigh;
   ArrayTime[3]=s_TimeStructVar.MinLow;
   ArrayTime[4]=s_TimeStructVar.SecHigh;
   ArrayTime[5]=s_TimeStructVar.SecLow;
 }



/**
  * @brief  Initialises ArrayTime[] buffer to current date
  * @param  None
  * @retval : None
  *   
  */
void InitialiseDate(void)
{
   ArrayTime[0]=s_DateStructVar.Day/10;
   ArrayTime[1]=s_DateStructVar.Day%10;
   ArrayTime[2]=s_DateStructVar.Month/10;
   ArrayTime[3]=s_DateStructVar.Month%10;
   ArrayTime[4]=s_DateStructVar.Year/1000;
   ArrayTime[5]=(s_DateStructVar.Year/100)%10;
   ArrayTime[6]=(s_DateStructVar.Year/10)%10;
   ArrayTime[7]=s_DateStructVar.Year%10;
}



/**
  * @brief  Menu Initialisation routine
  * @param  None
  * @retval : None
  *   
  */
void MenuInit(void)
{
  uint8_t ArrayCounter;
  LCD_Clear(Blue2);
  LCD_SetBackColor(Blue2);
  LCD_SetTextColor(Yellow);
  /*Draw a rectangle with-79:Start X-Cood,255:Start Y-Cood,100:Length,200:Width*/
  LCD_DrawRect(79,255,100,200);
  LCD_SetTextColor(Cyan);
  /*Draw a rectangle with-74:Start X-Cood,260:Start Y-Cood,110:Length,210:Width*/
  LCD_DrawRect(74,260,110,210);
  LCD_SetTextColor(Green);
  LCD_DisplayString(Line0,Column7,"STM32");
  LCD_DisplayString(Line1,Column2,"RTC AND CALENDAR");
  LCD_DisplayString(Line2,Column3,"IMPLEMENTATION");
  /*Display Colon : Ascii code:58 in decimal */
  LCD_DisplayChar(Line4,Column8,ASCII_COLON);
  LCD_DisplayChar(Line4,Column11,ASCII_COLON);
  LCD_SetTextColor(White);
  LCD_SetBackColor(Green);
  LCD_DisplayString(Line8,Column8,"MENU");
  LCD_SetBackColor(Blue2);
  DisplayDateFlag=1;
  Counter=0;
  MenuLevelPointer=0;
  TimeDateDisplay=0;
  TotalMenuPointer=1;
  DisplayCurrentPos=0;
  DisplayPrevPos=0;
  CounterPosition=0;
  
  for(ArrayCounter=0;ArrayCounter<6;ArrayCounter++)
  {
    ArrayTime[ArrayCounter]=0;
  }
  
  BKP_RTCOutputConfig(BKP_RTCOutputSource_None);
  SelIntExtOnOffConfig(ENABLE);
  UpDownIntOnOffConfig(DISABLE);
  RightLeftIntExtOnOffConfig(DISABLE);
  BKP_TamperPinCmd(ENABLE);
}



/**
  * @brief  Menu Management Routine
  * @param  None
  * @retval : None
  *   
  */
void MenuManage(void)
{
  uint32_t Timer;
  
  if(FlagKey & 0x01)
  {
    FlagKey &=0xFE;
    SelFunction();
  }
  
  if(FlagKey & 0x02)
  {
    FlagKey &=0xFD;
    UpFunction();
  }
  
  if(FlagKey & 0x04)
  {
    FlagKey &= 0xFB;
    DownFunction();
  }
  
 if(FlagKey & 0x08)
  {
    FlagKey &=0xF7;
    RightFunction();
  }
  
 if(FlagKey & 0x10)
  {
    FlagKey &=0xEF;
    LeftFunction();
  }
  
  /* Automatic return to the home screen if any of the joystick key
  is not pressed for 8 seconds */
  if(FlagKey==0x00 && (MenuLevelPointer == 1 || MenuLevelPointer==2) \
    /*&& (TotalMenuPointer !=9)*/)
  {
    Timer=RTC_GetCounter();

    while(ReadKey()==NOKEY)
    {
      if((RTC_GetCounter()-Timer)>=8)
      {
        MenuInit();
        break;
      }
    }
  }
}



/**
  * @brief  Enables or disables EXTI for the menu navigation keys :
  *   EXTI lines 8 and 14 which correpond respectively
  *   to Up and Down.
  * @param New State-can be ENABLE or DISABLE
  * @retval : None
  *   
  */
void UpDownIntOnOffConfig(FunctionalState NewState)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Initializes the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line 8 and 14 on falling edge */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = EXTI_Line8 |  EXTI_Line14;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  else
  {
    EXTI_ClearITPendingBit(EXTI_Line8  | EXTI_Line14 );
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = EXTI_Line8 | EXTI_Line14;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}



/**
  * @brief  Enables or disables EXTI for the menu navigation keys :
  *   EXTI lines 0 and 1 which correpond respectively
  *   to Right and Left.
  * @param NewState: New state of the navigation keys. This parameter
  *   can be: ENABLE or DISABLE.
  * @retval : None
  */
void RightLeftIntExtOnOffConfig(FunctionalState NewState)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Initializes the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line 0 and 1 on falling edge */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line 0 and 1 on falling edge */
  else
  {
    /* Clear the the EXTI line 0 and 1 interrupt pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1 );
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}



/**
  * @brief  Enables or disables EXTI for the menu navigation keys :
  *   EXTI lines  12
  *   "SEL"
  * @param NewState: New state of the navigation keys. This parameter
  *   can be: ENABLE or DISABLE.
  * @retval : None
  */
void SelIntExtOnOffConfig(FunctionalState NewState)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Initializes the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line  12  on falling edge */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = EXTI_Line12 ;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line 12 on falling edge */
  else
  {
    /* Clear the the EXTI line 12  interrupt pending bit */
    EXTI_ClearITPendingBit( EXTI_Line12 );
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = EXTI_Line12 ;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}



/**
  * @brief  Joystick interrupts NVIC Configuration
  * @param  None
  * @retval : None
  */
void NVIC_JoyStickConfig(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* Enable the EXTI9_5 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the EXTI1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the  EXTI15_10 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable EXTI_Line7 Interrupt */
  /* Used for Battery removal/restore simulated detection */
  EXTI_ClearITPendingBit( EXTI_Line7 );
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Line = EXTI_Line7 ;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}



/**
  * @brief  Reads key from demoboard.
  * @param  None
  * @retval : Return RIGHT, LEFT, SEL, UP, DOWN or NOKEY
  */
uint8_t ReadKey(void)
{
  /* "right" key is pressed */
  if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0))
  {
    while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) == Bit_RESET)
    {
    }
    return RIGHT;
  }
  /* "left" key is pressed */
  if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1))
  {
    while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1) == Bit_RESET)
    {
    }
    return LEFT;
  }
  /* "up" key is pressed */
  if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8))
  {
    while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == Bit_RESET)
    {
    }
    return UP;
  }
  /* "down" key is pressed */
  if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14))
  {
    while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14) == Bit_RESET)
    {
    }
    return DOWN;
  }
  /* "sel" key is pressed */
  if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))
  {
    while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12) == Bit_RESET)
    {
    }
    return SEL;
  }
  /* No key is pressed */
  else
  {
    return NOKEY;
  }
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
