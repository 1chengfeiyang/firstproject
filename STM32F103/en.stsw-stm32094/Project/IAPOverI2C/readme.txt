/**
  @page IAPOverI2C IAPOverI2C
  
  @verbatim
  ******************** (C) COPYRIGHT 2010 STMicroelectronics *******************
  * @file IAPOverI2C/readme.txt 
  * @author   MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief    In Application Programming over I2C example description.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
   @endverbatim

@par Description

This directory contains a set of source files and pre-configured projects that 
describes how to load an application into Flash memory using In-Application 
Programming (IAP through I2C).

@par Directory contents 

 - inc: contains the header files
  - IAPOverI2C/inc/commands.h          Header for commands.c
  - IAPOverI2C/inc/stm32f10x_conf.h    Library Configuration files
  - IAPOverI2C/inc/stm32f10x_it.h      Interrupt handlers header files
  - IAPOverI2C/inc/config.h            Header for config.c
  - IAPOverI2C/inc/main.h              Header for main.c
     
 - src: contains the source files 
  - IAPOverI2C/src/stm32f10x_it.c    Interrupt handlers
  - IAPOverI2C/src/main.c            Main program
  - IAPOverI2C/src/commands.c        Contains the I2C commands' routines
  - IAPOverI2C/src/config.c          Contains the configuration routines                                     
   
@par Hardware and Software environment

  - This example runs on STM32F10x Connectivity line, High-Density, Medium-Density, 
    XL-Density, Medium-Density Value line, Low-Density and Low-Density Value line Devices.
  
  - The example has been tested with STMicroelectronics STM32100B-EVAL (Medium-Density
    Value line), STM3210C-EVAL (Connectivity line), STM3210E-EVAL (High-Density and
    XL-Density) and STM3210B-EVAL (Medium-Density) evaluation boards and can be easily
    tailored to any other supported device and development board.
  
  - Connect I2C1 SCL pin (PB.6) to the Master(Aardvark or another STM32 I2C1) SCL pin. 
  
  - Connect I2C1 SDA pin (PB.7) to the Master(Aardvark or another STM32 I2C1) SDA pin. 
 
  - Check that a pull-up resistor is connected on one I2C SDA pin
  
  - Check that a pull-up resistor is connected on one I2C SCL pin
  
  - Don't forget to connect the GNDs

@par How to use it ?  

In order to make the program work, you must do the following:

1. Load the IAP driver in the STM32 Target Board (see below)
2. Hold down the Key push-button to enter the IAP mode
3. Run program 


In order to load the code, you have do the following:

 - EWARMv5 (v5.50.5) 
    - Open the IAP_I2C.eww workspace
    - In the workspace toolbar select the project config:
        - STM3210E-EVAL_XL: to load the project for STM32 XL-density devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices 
        - STM3210C-EVAL: to load the project for Connectivity line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE (v7) 
    - Open the IAP_I2C.rprj project
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210E-EVAL_XL: to load the project for STM32 XL-density devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices 
        - STM3210C-EVAL: to load the project for Connectivity line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)

 - MDK-ARM (v4.11 and later) 
    - Open the IAP_I2C.uvproj project
    - In the build toolbar select the project config:
        - STM3210E-EVAL_XL: to load the project for STM32 XL-density devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices 
        - STM3210C-EVAL: to load the project for Connectivity line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5) 

  - HiTOP (v5.32)
   - Open the HiTOP toolchain.
   - Browse to open the IAP_I2C.htp
   - A "Download application" window is displayed, click "cancel".
   - Rebuild all files: Project->Rebuild all
   - Load project image : Click "ok" in the "Download application" window.
   - Run the "RESET_GO_MAIN" script to set the PC at the "main"
   - Run program: Debug->Go(F5).    

 - TrueSTUDIO (v1.4.0 and later):
    - Open the TrueSTUDIO toolchain.
    - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace 
      directory.
    - Click on File->Import, select General->'Existing Projects into Workspace' 
      and then click "Next". 
    - Browse to the TrueSTUDIO workspace directory and select the project: 
        - STM3210E-EVAL_XL: to load the project for STM32 XL-density devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices 
        - STM3210C-EVAL: to load the project for Connectivity line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
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
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 512 and 1024 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
   
   
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */
