/**
  @page Run_Mode AN2629 Run_Mode Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file Run_Mode/readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Description of the AN2629 Application note's Run_Mode.
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

This example describes how to use the clock configuration of the STM32F10xxx.
It describes also  how to use the STM32F10xxx backup registers, using a firmware 
found in the Zip file delivered with this application note. 


@par Directory contents 

  - inc
    - Run_Mode/inc/stm32f10x_conf.h    Library Configuration file 
    - Run_Mode/inc/stm32f10x_it.h      Interrupt handlers header file 
    - Run_Mode/inc/main.h              Main program file header file 
    - Run_Mode/inc/usart.h             This file contains all the functions prototypes for the USART 
    - Run_Mode/inc/rtc.h               This file contains all the functions prototypes for the rtc

  - src 
    - Run_Mode/src/stm32f10x_it.c      Interrupt handlers 
    - Run_Mode/src/main.c              Main program file
    - Run_Mode/src/usart.c             This file provides USART firmware functions
    - Run_Mode/src/rtc.c               This file provides RTC firmware functions
                  
   
@par Hardware and Software environment  

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL evaluation 
    boards and can be easily tailored to any other supported device and development 
    board.
   
  
  - STM3210B-EVAL Set-up 
  
    - Use this example with STM32F10xxx evaluation board: refer to the user
      manual UM0426:“STM3210B-EVAL evaluation board” to use the STM3210B-EVAL.
    - The USART1 signals (RX, TX) must be connected to a DB9 connector using an RS232
      transceiver.
    - A null-modem female/female RS232 cable must be connected between the DB9
      connector (CN6 on STM3210B-EVAL board) and the PC serial port.
    - After connecting the power supply and the JTAG tools, the power consumption 
      can be measured by replacing Jumper JP9 by an ampermeter.
            
         
@par How to use it ? 

In order to load the Run_Mode code, you have do the following:

  - RVMDK 
    - Open the Run_Mode.uv2 project
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

  - RIDE 
    - Open the Run_Mode.rprj project
    - Rebuild all files: Project->Build Project
    - Load project image: Debug->start
    - Run program: Debug->Run(ctrl+F9)  
    
  - EWARMv5 
    - Open the Run_Mode.eww workspace.
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
