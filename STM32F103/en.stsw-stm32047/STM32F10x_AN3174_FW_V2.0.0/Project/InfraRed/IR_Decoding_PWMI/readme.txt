/**
  @page InfraRed_Decoding_PWMI AN3174 InfraRed Decoding uisng TIM PWMI mode Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2012 STMicroelectronics *******************
  * @file    InfraRed/IR_Decoding_PWMI/readme.txt 
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    25-January-2012
  * @brief   Description of the AN3174 "Implementing receivers for infrared remote 
  *          control protocols using STM32F10xxx microcontrollers".
  ******************************************************************************
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
  ******************************************************************************
   @endverbatim

@par Description

This directory contains a set of sources files and pre-configured projects that 
describes how to implement an IR receiver in software using STM32F10xxx 
microcontrollers.

@par Directory contents 

 - "InfraRed\IR_Decoding_PWMI\inc": contains the InfraRed IR Decoding_PWMI firmware header files 
    - InfraRed/IR_Decoding_PWMI/inc/ir_decode.h           This file contains all the functions prototypes for the IR firmware library.
    - InfraRed/IR_Decoding_PWMI/inc/sirc_decode.h         This file contains all the functions prototypes for the Sirc firmware library.   
    - InfraRed/IR_Decoding_PWMI/inc/rc5_decode.h          This file contains all the functions prototypes for the RC5 firmware library. 
    - InfraRed/IR_Decoding_PWMI/inc/stm32f10x_conf.h      Library Configuration file
    - InfraRed/IR_Decoding_PWMI/inc/stm32f10x_it.h        Header for stm32f10x_it.c    

 - "InfraRed\IR_Decoding_PWMI\MDK-ARM": contains pre-configured project for MDK-ARM toolchain

 - "InfraRed\IR_Decoding_PWMI\RIDE": contains pre-configured project for RIDE toolchain

 - "InfraRed\IR_Decoding_PWMI\HiTOP": contains pre-configured project for HiTOP toolchain

 - "InfraRed\IR_Decoding_PWMI\EWARM": contains pre-configured project for EWARM toolchain
 
 - "InfraRed\IR_Decoding_PWMI\TrueSTUDIO": contains pre-configured project for TrueSTUDIO toolchain

 - "InfraRed\IR_Decoding_PWMI\src": contains the IAP firmware source files
    - InfraRed/IR_Decoding_PWMI/src/main.c               Main program
    - InfraRed/IR_Decoding_PWMI/src/stm32f10x_it.c       Interrupt handlers
    - InfraRed/IR_Decoding_PWMI/src/ir_decode.c          This file provides the high level functions.
    - InfraRed/IR_Decoding_PWMI/src/rc5_decode.c         This file provides all the RC5 firmware functions.
    - InfraRed/IR_Decoding_PWMI/src/ir_commands.c        This file provides  the IR command Tables.    
    - InfraRed/IR_Decoding_PWMI/src/system_stm32f10x.c   STM32F10x system source file
    
@par Hardware and Software environment  

  - This example runs on STM32F10x Connectivity line, High-Density, High-Density 
    Value line, Medium-Density, XL-Density, Medium-Density Value line, Low-Density
    and Low-Density Value line Devices.
  
  - This example has been tested with STMicroelectronics STM32100E-EVAL (High-Density 
    Value line) evaluation boards and can be easily tailored to any other supported 
    device and development board.    
    
@par How to use it ? 

In order to make the program work, you must do the following:

1. Program the internal Flash with the IR_Decoding_PWMI (see below) " 
2. Choose a protocol from the list.
3. Press on the remote control to send a command.
      
In order to load the InfraRed code, you have do the following:
 - EWARM:
    - Open the IR_Decoding_PWMI.eww workspace
    - In the workspace toolbar select the infrared protocol:
        - IR_RC5_Decoder       : to configure the project for RC5 protocol.
        - IR_SIRC_Decoder      : to configure the project for SIRC protocol.
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE:
    - Open the IR_Decoding_PWMI.rprj project
    - In the configuration toolbar(Project->properties) select the infrared protocol:
        - IR_RC5_Decoder       : to configure the project for RC5 protocol.
        - IR_SIRC_Decoder      : to configure the project for SIRC protocol.
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)

 - MDK-ARM:
    - Open the IR_Decoding_PWMI.uvproj project
    - In the build toolbar select the infrared protocol:
        - IR_RC5_Decoder       : to configure the project for RC5 protocol.
        - IR_SIRC_Decoder      : to configure the project for SIRC protocol.
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

 - HiTOP:
    - Open the HiTOP toolchain.
    - Browse to open the IR_Decoding_PWMI.htp
        - IR_RC5_Decoder       : to configure the project for RC5 protocol.
        - IR_SIRC_Decoder      : to configure the project for SIRC protocol.
    - A "Download application" window is displayed, click "cancel".
    - Rebuild all files: Project->Rebuild all
    - Load project image : Click "ok" in the "Download application" window.
    - Run program: Debug->Run 

 - TrueSTUDIO:
    - Open the TrueSTUDIO toolchain.
    - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace 
      directory.
    - Click on File->Import, select General->'Existing Projects into Workspace' 
      and then click "Next". 
    - Browse to the TrueSTUDIO workspace directory and select the project: 
        - IR_RC5_Decoder       : to load the project for RC5 protocol.
        - IR_SIRC_Decoder      : to load the project for SIRC protocol.
    - Under Windows->Preferences->General->Workspace->Linked Resources, add 
      a variable path named "CurPath" which points to the folder containing
      "Libraries", "Project" and "Utilities" folders.
    - Rebuild all project files: Select the project in the "Project explorer" 
      window then click on Project->build project menu.
    - Run program: Select the project in the "Project explorer" window then click 
      Run->Debug (F11)

    
@note
 - Low-density Value line devices are STM32F100xx microcontrollers where the 
   Flash memory density ranges between 16 and 32 Kbytes.
 - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx 
   microcontrollers where the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density Value line devices are STM32F100xx microcontrollers where
   the Flash memory density ranges between 64 and 128 Kbytes.  
 - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx 
   microcontrollers where the Flash memory density ranges between 64 and 128 Kbytes.
 - High-density Value line devices are STM32F100xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 512 and 1024 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
 
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */
