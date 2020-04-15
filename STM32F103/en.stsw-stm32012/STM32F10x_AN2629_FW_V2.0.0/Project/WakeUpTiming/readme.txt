/**
  @page WakeUpTiming AN2629 WakeUpTiming Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file WakeUpTiming/readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Description of the AN2629 Application note's WakeUp Timing.
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

This example  describes how to measure the time required by the STM32F10xxx to
wake up from different low-power modes, using the firmware provided with this 
application note. 


@par Directory contents 

  - inc 
    - WakeUpTiming/inc/stm32f10x_conf.h     Library Configuration file 
    - WakeUpTiming/inc/stm32f10x_it.h       Interrupt handlers header file 
    - WakeUpTiming/inc/hw_config.h          Hardware configuration header file 

  - src 
    - WakeUpTiming/src/stm32f10x_it.c       Interrupt handlers 
    - WakeUpTiming/src/main.c               Main program file
    - WakeUpTiming/src/hw_config.c          Hardware configuration header file 
   
@par Hardware and Software environment  

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL  evaluation 
    boards and can be easily tailored to any other supported device and development 
    board.
   
  
  - STM3210B-EVAL Set-up 
    - Pin PC6 is set to one after wakeup from the low-power modes, so, to measure 
    the wakeup time, an oscilloscope should be connected across the PC6 and PA0 
    pins. The wakeup time is the time between the rising edge of PA0 (Wakeup button) 
    and the rising edge of PC6.
            
         
@par How to use it ? 

In order to load the WakeUp Timing code, you have do the following:

  - RVMDK 
    - Open the WakeUpTiming.uv2 project
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

  - RIDE 
    - Open the WakeUpTiming.rprj project
    - Rebuild all files: Project->Build Project
    - Load project image: Debug->start
    - Run program: Debug->Run(ctrl+F9)  
    
  - EWARMv5 
    - Open the WakeUpTiming.eww workspace.
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
