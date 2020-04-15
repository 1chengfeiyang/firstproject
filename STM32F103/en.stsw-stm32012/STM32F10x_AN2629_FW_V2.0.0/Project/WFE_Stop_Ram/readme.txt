/**
  @page WFE_Stop_Ram AN2629 WFE_Stop_Ram Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file WFE_Stop_Ram/readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    Description of the AN2629 Application note's WFE_Stop_RAM.
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

describes how to use the Wait for Event instruction and the Stop mode of the
STM32F10xxx, using the firmware found in the Zip file delivered with this 
application note. 
WFE_Stop_RAM example  is downloaded in Flash memory, but some functions 
are remapped in RAM at startup. How to remap code in RAM is explained in the 
toolset documentation.

@par Directory contents 

   
    - inc 
       - WFE_Stop_Ram/inc/stm32f10x_conf.h     Library Configuration file 
       - WFE_Stop_Ram/inc/stm32f10x_it.h       Interrupt handlers header file 

    - src 
      - WFE_Stop_Ram/src/stm32f10x_it.c        Interrupt handlers 
      - WFE_Stop_Ram/src/main.c                Main program file
  
@par Hardware and Software environment  

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL  evaluation 
    boards and can be easily tailored to any other supported device and development 
    board.
   
  
  - STM3210B-EVAL Set-up 
  
    - Use this example with STM32F10xxx evaluation board: refer to the user manual 
      UM0426: STM3210B-EVAL evaluation board to use the STM3210B-EVAL.
    - After connecting the power supply and the JTAG tools, the power consumption 
      can be measured by replacing Jumper JP9 by an ampermeter.
            
         
@par How to use it ? 

In order to load the WFE_Stop_RAM code, you have do the following:

  - RVMDK 
    - Open the WFE_Stop_Ram.uv2 project
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)
 
  - EWARMv5 
    - Open the WFE_Stop_Ram.eww workspace.
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
