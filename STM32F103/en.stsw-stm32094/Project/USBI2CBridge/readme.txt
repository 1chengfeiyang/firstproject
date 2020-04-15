/**
  @page USBI2CBridge USBI2CBridge 
  
  @verbatim
  ******************** (C) COPYRIGHT 2010 STMicroelectronics *******************
  * @file USBI2CBridge/readme.txt 
  * @author   MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief    USB/I2C  bridge (based on the DFU) description.
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

This demo presents an update of the Device Firmware Upgrade demo. 
The STM32F10xxx programmed with this driver is a bridge USB/I2C that allows 
sending IAP commands to an STM32 slave device.   

@par Directory contents 

 - inc: contains the header files     
 - src: contains the source files 
 - EWARMv5: contains pre-configured project for EWARM5 toolchain
 - RIDE: contains pre-configured project for RIDE toolchain
 - RVMDK: contains pre-configured project for RVMDK toolchain          
   
@par Hardware and Software environment

  - This example runs on STM32F10x Connectivity line, High-Density, Medium-Density 
    ,XL-Density and Low-Density Devices.
  
  - This example has been tested with  STM3210B-EVAL (Medium density), STM3210C-EVAL 
   (Connectivity line) and STM3210E-EVAL (High-Density and XL-Density) evaluation 
    boards and can be easily tailored to any other supported device and development board.  

   - Connect PB.6 (bridge board) to PB.6 (Target board)

   - Connect PB.7 (bridge board) to PB.7 (Target board)
  	
   - Connect the GNDs
        
    - STM3210B-EVAL Set-up 
        - Jumper JP1 (USB disconnect) should be connected in position 2-3.

    - STM3210E-EVAL Set-up 
        - Jumper JP14 (USB disconnect) should be connected in position 2-3.

    - STM3210C-EVAL Set-up 
        - None. 


@par How to use it ?  

In order to make the program work, you must do the following:

1. Generate a dfu image from bin file using the DFU file manager.
(The bin file is generated from the project in the 'template' subdirectory.)
   - Check "I want to generate a DFU file from S19,HEX,BIN file->OK."
   - Set the value of the target ID to 0.
   - Press the multibin button-> Set the Address to 0x1000. 
   - Choose the binary file to be converted->Add to list-> OK.
   - Then press the "GENERATE" button->enter the corresponding dfu name->OK.

2. Load the IAP I2C project into Target board.
3. Load the Bridge project into bridge board.
     
In order to do this, you have to do the following:
 + EWARMv5
    - Open the Bridge.eww workspace.
    - In the workspace toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
        - STM3210C-EVAL: to configure the project for STM32 Connectivity line devices
        - STM3210E-EVAL_XL: to configure the project for STM32 XL-Density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5).

- HiTOP
   - Open the HiTOP toolchain.
   - Browse to open Bridge.htp
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
        - STM3210C-EVAL: to configure the project for STM32 Connectivity line devices
        - STM3210E-EVAL_XL: to configure the project for STM32 XL-Density devices
   - A "Download application" window is displayed, click "cancel".
   - Rebuild all files: Project->Rebuild all
   - Load project image : Click "ok" in the "Download application" window.
   - Run the "RESET_GO_MAIN" script to set the PC at the "main"
   - Run program: Debug->Go(F5). 

 + TrueSTUDIO (v1.4.0 and later):
    - Open the TrueSTUDIO toolchain.
    - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace 
      directory.
    - Click on File->Import, select General->'Existing Projects into Workspace' 
      and then click "Next". 
    - Browse to the TrueSTUDIO workspace directory and select the project: 
        - STM3210E_EVAL_XL: to load the project for STM32 XL-density devices
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


@note: 
The Bridge project compiled for Medium-Density devices allow programming Medium-Density 
and Medium-Density Value line Devices.

The Bridge project compiled for High-Density devices allow programming High-Density 
Devices.

The Bridge project compiled for XL-Density devices allow programming XL-Density 
Devices.

The Bridge project compiled for Connectivity line devices allow programming Connectivity 
line Devices.

4. Reset the Target board with keeping Key push button pressed at reset.

5. Reset the Bridge board and run the "DfuSe Demonstration" application.

6. The target must appear in the target area in the DFU Se demo interface.

7. Select the target(I2C_FlashSTM32).

8. To upgrade the content of the user flash area: choose the dfu file in the target board using "CHOOSE..."
button and then press the "UPGRADE" button. 

@note

 - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx 
   microcontrollers where the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx 
   microcontrollers where the Flash memory density ranges between 64 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 512 and 1024 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
   
   
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */
