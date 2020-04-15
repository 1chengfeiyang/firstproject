/**
  @page template template

  @verbatim
  ******************** (C) COPYRIGHT 2010 STMicroelectronics *******************
  * @file template/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/15/2010
  * @brief   Description of the template directory.
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

This directory contains a set of sources files that build the application to be
loaded into Flash memory using In-Application Programming over I2C.

To build such application, some special configuration has to be peformed:
1. Set the program load address at 0x08001000, using your toolchain linker file
2. Relocate the vector table at address 0x08001000, using the "NVIC_SetVectorTable"
   function.
 
The SysTick example provided within the STM32F10x Standard Peripherals library 
examples is used as illustration.
This example configures the SysTick to generate a time base equal to 1 ms.
The system clock is set to 72 MHz, the SysTick is clocked by the AHB clock (HCLK).
A "Delay" function is implemented based on the SysTick end-of-count event.
Four LEDs are toggled with a timing defined by the Delay function.


@par Directory contents 

 - "template\EWARMv5": This folder contains a pre-configured project file 
                       that produces a binary image of SysTick example to 
                       be loaded with IAP.

 - "template\RIDE": This folder contains a pre-configured project file 
                    that produces a binary image of SysTick example to be 
                    loaded with IAP.

 - "template\MDK-ARM": This folder contains a pre-configured project file 
                     that produces a binary image of SysTick example to be 
                     loaded with IAP.
                           
 - "template\inc": contains the header files 
  - template/     - template/inc/stm32f10x_conf.h    Library Configuration file
  - template/     - template/inc/stm32f10x_it.h      Header for stm32f10x_it.c
  - template/     - template/inc/main.h              Header for main.c
     
 - "template\src": contains the binary_template firmware source files 
  - template/     - template/src/main.c              Main program
  - template/     - template/src/stm32f10x_it.c      Interrupt handlers


@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, Medium-Density and Low-Density 
    Devices.
  
  - This example has been tested with STMicroelectronics STM3210E-EVAL 
    (STM32F10x High-Density) and STM3210B-EVAL(STM32F10x Medium-Density) 
    evaluation boards and can be easily tailored to any other supported device
    and development board.
        
    - STM3210E-EVAL Set-up 
    - Use LED1, LED2, LED3 and LED4 connected respectively to PF.06, PF0.7, PF.08
      and PF.09 pins

    - STM3210B-EVAL Set-up  
    - Use LED1, LED2, LED3 and LED4 connected respectively to PC.06, PC.07, PC.08
      and PC.09 pins 

      
@par How to use it ?  

In order to load the SysTick example with the IAP, you must do the following:

 - EWARMv5 (v5.50.5)
    - Open the SysTick.eww workspace
    - In the workspace toolbar select the project config:
        - STM3210E-EVAL_XL: to load the project for STM32 XL-density devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices 
        - STM3210C-EVAL: to load the project for Connectivity line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all
    - A binary file "Project.bin" will be generated under "\STM3210E-EVAL\exe",
      "\STM3210E-EVAL\exe", "\STM3210B-EVAL\exe", "\STM32100B-EVAL\exe", 
      "\STM3210E-EVAL_XL\exe" folders depending on the selected configuration
    - Generate a dfu image from bin file using the DFU file manager.
    - Check "I want to generate a DFU file from S19,HEX,BIN file->OK."
    - Set the value of the target ID to 0.
    - Press the multibin button-> Set the Address to 0x1000. 
    - Choose the binary file to be converted->Add to list-> OK.
    - Then press the "GENERATE" button->enter the corresponding dfu name->OK.  
    - Finally load the DFU image with "DfuSe Demonstration" application.

 - RIDE (v7)
    - Open the SysTick.rprj project
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210E-EVAL_XL: to load the project for STM32 XL-density devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices 
        - STM3210C-EVAL: to load the project for Connectivity line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
    - Rebuild all files: Project->build project
    - Go to "template\Binary" directory and run "hextobin.bat"
    - A binary file "Project.bin" will be generated under "\STM3210E-EVAL",
      "\STM3210E-EVAL", "\STM3210B-EVAL", "\STM32100B-EVAL", "\STM3210E-EVAL_XL"
       folders depending on the selected configuration 
        - Generate a dfu image from bin file using the DFU file manager.
    - Check "I want to generate a DFU file from S19,HEX,BIN file->OK."
    - Set the value of the target ID to 0.
    - Press the multibin button-> Set the Address to 0x1000. 
    - Choose the binary file to be converted->Add to list-> OK.
    - Then press the "GENERATE" button->enter the corresponding dfu name->OK.  
    - Finally load the DFU image with "DfuSe Demonstration" application.


 - MDK-ARM (v4.11 and later)
    - Open the Project.uvproj project
    - In the build toolbar select the project config:
        - STM3210E-EVAL_XL: to load the project for STM32 XL-density devices
        - STM32100B-EVAL: to load the project for STM32 Medium-density Value line devices 
        - STM3210C-EVAL: to load the project for Connectivity line devices
        - STM3210B-EVAL: to load the project for STM32 Medium-density devices
        - STM3210E-EVAL: to load the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all target files
    - Go to "template\Binary" directory and run "axftobin.bat"
    - A binary file "Project.bin" 
      will be generated under "\STM3210E-EVAL", "\STM3210E-EVAL", "\STM3210B-EVAL",
      "\STM32100B-EVAL", "\STM3210E-EVAL_XL" folders depending on the selected configuration 
    - Generate a dfu image from bin file using the DFU file manager.
    - Check "I want to generate a DFU file from S19,HEX,BIN file->OK."
    - Set the value of the target ID to 0.
    - Press the multibin button-> Set the Address to 0x1000. 
    - Choose the binary file to be converted->Add to list-> OK.
    - Then press the "GENERATE" button->enter the corresponding dfu name->OK.  
    - Finally load the DFU image with "DfuSe Demonstration" application.

- HiTOP (v5.32)
   - Open the HiTOP toolchain.
   - Browse to open the Project.htp
   - A "Download application" window is displayed, click "cancel".
   - Rebuild all files: Project->Rebuild all.
   - Go to "template\Binary" directory and run "hex2bin.bat"
   - A Project.bin will be generated under the project directory
   - Generate a dfu image from bin file using the DFU file manager.
   - Check "I want to generate a DFU file from S19,HEX,BIN file->OK."
   - Set the value of the target ID to 0.
   - Press the multibin button-> Set the Address to 0x1000. 
   - Choose the binary file to be converted->Add to list-> OK.
   - Then press the "GENERATE" button->enter the corresponding dfu name->OK.  
   - Finally load the DFU image with "DfuSe Demonstration" application.

 - TrueSTUDIO (v1.4.0 and later)
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
    - Go to "template\Binary" directory and run "TrueSTUDIO_elf2bin.bat"
    - A binary file "project.bin" ill be generated under "\STM3210E-EVAL", 
      "\STM3210E-EVAL", "\STM3210B-EVAL", "\STM32100B-EVAL", "\STM3210E-EVAL_XL"
      folders depending on the selected configuration 
    - Generate a dfu image from bin file using the DFU file manager.
    - Check "I want to generate a DFU file from S19,HEX,BIN file->OK."
    - Set the value of the target ID to 0.
    - Press the multibin button-> Set the Address to 0x1000. 
    - Choose the binary file to be converted->Add to list-> OK.
    - Then press the "GENERATE" button->enter the corresponding dfu name->OK.  
    - Finally load the DFU image with "DfuSe Demonstration" application.


@note
 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.

 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */
