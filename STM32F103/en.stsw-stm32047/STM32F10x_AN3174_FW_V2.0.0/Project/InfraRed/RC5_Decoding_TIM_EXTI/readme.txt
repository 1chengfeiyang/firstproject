
/**
  @page RC5_Decoding_TIM_EXTI AN3174 RC5 Decoding using TIM and EXTI Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2012 STMicroelectronics *******************
  * @file    InfraRed/RC5_Decoding_TIM_EXTI/readme.txt 
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

This application note shows how to implement a RC5 receiver by software using
STM32 microcontrollers. The demo consists of receiving RC5 messages and then sent
it to the hyperterminal. 
USART1 is used for this purpose to connect the board to the PC. Please configure the 
hyperterminal as the following:

 -  115200 baud,
 -  8-bit data,
 -  No parity,
 -  No Flow Control.


@par Directory contents 

 - "InfraRed\IR_Decoding_PWMI\inc": contains the InfraRed IR Decoding_PWMI firmware header files   
    - InfraRed/RC5_Decoding_TIM_EXTI/inc/RC5_IR_Emul_Receiver.h    This file contains all the functions prototypes for the RC5 firmware library. 
    - InfraRed/RC5_Decoding_TIM_EXTI/inc/stm32f10x_conf.h          Library Configuration file
    - InfraRed/RC5_Decoding_TIM_EXTI/inc/stm32f10x_it.h            Header for stm32f10x_it.c    

 - "InfraRed/RC5_Decoding_TIM_EXTI\MDK-ARM": contains pre-configured project for MDK-ARM toolchain

 - "InfraRed/RC5_Decoding_TIM_EXTI\RIDE": contains pre-configured project for RIDE toolchain

 - "InfraRed/RC5_Decoding_TIM_EXTI\HiTOP": contains pre-configured project for HiTOP toolchain

 - "InfraRed/RC5_Decoding_TIM_EXTI\EWARM": contains pre-configured project for EWARM toolchain
 
 - "InfraRed/RC5_Decoding_TIM_EXTI\TrueSTUDIO": contains pre-configured project for TrueSTUDIO toolchain

 - "InfraRed/RC5_Decoding_TIM_EXTI\src": contains the IAP firmware source files
    - InfraRed/RC5_Decoding_TIM_EXTI/src/main.c                    Main program
    - InfraRed/RC5_Decoding_TIM_EXTI/src/stm32f10x_it.c            Interrupt handlers
    - InfraRed/RC5_Decoding_TIM_EXTI/src/RC5_IR_Emul_Receiver.c    This file provides all the RC5 firmware functions.
    - InfraRed/RC5_Decoding_TIM_EXTI/src/system_stm32f10x.c        STM32F10x system source file


@par Hardware and Software environment

  - This example runs on STM32F10x High-Density, High-Density Value line, 
    Medium-Density, Connectivity line, Medium-Density Value line, Low-Density 
    and Low-Density Value line Devices.
  
  - This example has been tested with STMicroelectronics STM32100E-EVAL (STM32F10x 
    High-Density Value line), STM32100B-EVAL(STM32F10x Medium-Density Value line), 
    STM3210B-EVAL (STM32F10x Medium-Density) and STM3210E-EVAL (STM32F10x High-Density) 
    evaluation boards. 
    The STM3210C-EVAL board (STM32F10x Connectivity Line) is not supported by the demo. 
    However Connectivity Line product supports the demo.
   
  - STM32100B-EVAL, STM3210B-EVAL, STM3210C-EVAL and STM3210E-EVAL Set-up 
     - Connect the IR module output to GPIOB.01.
     - Connect the board (USART1) to the PC via RS232 cable.

  - STM32100E-EVAL Set-up 
     - Connect PC.06 to GPIOB.01 and use the IR module already available on board.
     - Connect the board (USART1) to the PC via RS232 cable.
           
@par How to use it 

In order to load the RC5_Receiver_Demo demo, you have do the following:
  
 - MDK-ARM 
    - Open the RC5_Receiver_Demo.uvopt project
    - In the build toolbar select the project config:
        - STM32100E-EVAL: to configure the project for STM32 High-Density Value-line devices
        - STM32100B-EVAL: to configure the project for STM32 Medium-Density Value-line devices
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
        - STM3210x-Connectivity: to configure the project for STM32 Connectivity-Line devices
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

 - EWARM 
    - Open the RC5_Receiver_Demo.eww workspace.
    - In the workspace toolbar select the project config:
        - STM32100E-EVAL: to configure the project for STM32 High-Density Value-line devices
        - STM32100B-EVAL: to configure the project for STM32 Medium-Density Value-line devices
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
        - STM3210C-EVAL: to configure the project for STM32 Connectivity-Line devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE
    - Open the RC5_Receiver_Demo.rprj project.
    - In the configuration toolbar(Project->properties) select the project config:
        - STM32100B-EVAL: to configure the project for STM32 Medium-Density Value-line devices
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
        - STM3210x-Connectivity: to configure the project for STM32 Connectivity Line devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)  

 - HiTOP
    - Open the HiTOP toolchain.
    - Browse to open the HiTOP directory and select the project: 
        - STM32100E-EVAL: to load the project for STM32 High-density Value line devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
        - STM3210C-EVAL: to load the project for STM32 Connectivity Line devices
    - A "Download application" window is displayed, click "cancel".
    - Rebuild all files: Project->Rebuild all
    - Load project image : Click "ok" in the "Download application" window.
    - Run the "RESET_GO_MAIN" script to set the PC at the "main"
    - Run program: Debug->Go(F5).

 - TrueSTUDIO 
    - Open the TrueSTUDIO toolchain.
    - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace 
      directory.
    - Click on File->Import, select General->'Existing Projects into Workspace' 
      and then click "Next". 
    - Browse to the TrueSTUDIO workspace directory and select the project: 
        - STM32100E-EVAL: to load the project for STM32 High-density Value line devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
        - STM3210C-EVAL: to load the project for STM32 Connectivity Line devices
    - Under Windows->Preferences->General->Workspace->Linked Resources, add 
      a variable path named "CurPath" which points to the folder containing
      "Libraries", "Project" and "Utilities" folders.
    - Rebuild all project files: Select the project in the "Project explorer" 
      window then click on Project->build project menu.
    - Run program: Select the project in the "Project explorer" window then click 
      Run->Debug (F11)
                 
    @note 
     - The free Lite version of TrueSTUDIO do not support printf() redirection.
       For more information, see "note.txt" under "RC5_Decoding_TIM_EXTI\TrueSTUDIO".  

@note the JTAG connector should be removed and power off/on should be applied 
otherwise the system remains halted

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
