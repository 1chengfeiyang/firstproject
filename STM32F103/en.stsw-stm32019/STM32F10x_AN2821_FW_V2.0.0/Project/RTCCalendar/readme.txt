/**
  @page RTC AN2821 RTC Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file RTCCalendar/Readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Readme file for Clock/Calendar implementation on STM32 RTC 
  *           peripheral Demo.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
   @endverbatim

@par description

This Firmware provides the demo of Clock/Calendar implementation on STM32 RTC
peripheral based on STM32F10X-128K-EVAL board.


@par Directory contents 

 - inc: containing the user header files  
    - RTCCalendar/inc/stm32f10x_conf.h    Library Configuration files
    - RTCCalendar/inc/stm32f10x_it.h      Interrupt handlers header files
    - RTCCalendar/inc/main.h 			        Main program header file
    - RTCCalendar/inc/application_rtc.h	  Basic application Routines header file
    - RTCCalendar/inc/clock_calendar.h	  Clock Calendar basic routines	header file
    - RTCCalendar/inc/menu.h			        basic routines for MENU implementation header file
    - RTCCalendar/inc/stm3210b_lcd.h	    LCD driver V2.0.0 header file.
    - RTCCalendar/inc/eeprom.h      	    EEPROM emulation firmware functions header file
                               
  - src: containing the user source files   
    - RTCCalendar/src/stm32f10x_it.c       Interrupt handlers
    - RTCCalendar/src/main.c               Main program file
    - RTCCalendar/src/application_rtc.c    Basic application Routines file
    - RTCCalendar/src/clock_calendar.c	   Clock Calendar basic routines
    - RTCCalendar/src/menu.c			         Basic routines for MENU implementation file
    - RTCCalendar/src/stm3210b_lcd.c	     LCD driver V2.0.0 routines
    - RTCCalendar/src/eeprom.c		         EEPROM emulation firmware functions file

@par Hardware and Software environment 

 - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL  evaluation 
    boards and can be easily tailored to any other supported device and development 
    board.

 - STM3210B-EVAL Set-up 
    - Backup battery connected.

@note
 - A page of STM32 Flash Memory is used for storing time stamps, hence a maximum 
   number of times the page is erased and rewritten is limited.
 - Auto Calibration Accuracy of RTC depends upon the accuracy of the HSE 8 Mhz crystal used.

@par How to use it ? 

In order to load the RTC Calendar code, you have do the following:

 - EWARM: 
    - Open the RTC_Calender.eww workspace
    - In the workspace toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE 
    - Open the RTC_Calender.rprj project
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)

 - RVMDK 
    - Open the RTC_Calender.Uv2 project
    - In the build toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)
    
@note

 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 
 * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
 */
