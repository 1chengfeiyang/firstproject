/**
  @page CurrentMeasurements AN2629 CurrentMeasurements Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file CurrentMeasurements/readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Description of the AN2629 Application note's Current Measurements.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
   @endverbatim

@par Example Description 

This example  describes how to measure the power consumption of the STM32F10xxx 
using the firmware provided in the Zip file that comes with this application note. 


@par Directory contents 

  - inc
    - CurrentMeasurements/inc/stm32f10x_conf.h Library Configuration file 
    - CurrentMeasurements/inc/stm32f10x_it.h   Interrupt handlers header file 
    - CurrentMeasurements/inc/hw_config.h      Hardware configuration header file 

  - src 
    - CurrentMeasurements/src/stm32f10x_it.c   Interrupt handlers 
    - CurrentMeasurements/src/main.c           Main program file
    - CurrentMeasurements/src/hw_config.c      Hardware configuration header file 
   
@par Hardware and Software environment  

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL evaluation 
    boards and can be easily tailored to any other supported device and development 
    board.
   
  
  - STM3210B-EVAL Set-up 
    - Sleep, Stop and Standby (With RTC OFF): the measurement of the power
      consumption is made by replacing jumper JP9 in the STM3210B-EVAL board by 
      an ampermeter and by powering the board from an external supply, or by 
      using the USB cable.
    - RTC powered by VBAT: the measurement is made by connecting an external power
      supply to the pin 2 of jumper J11. The ampermeter is then connected in series.
            
         
@par How to use it ? 

In order to load the Current Measurements code, you have do the following:

  - RVMDK 
    - Open the CurrentMeasurements.uv2 project
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

  - RIDE 
    - Open the CurrentMeasurements.rprj project
    - Rebuild all files: Project->Build Project
    - Load project image: Debug->start
    - Run program: Debug->Run(ctrl+F9)  
    
  - EWARMv5 
    - Open the CurrentMeasurements.eww workspace.
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

@note
 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.

 * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
 */
