/**
  @page RCCalibration AN2868 RCCalibration Readme File
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file RCCalibration/readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    readme file for AN2868 STM32 HSI calibration firmware.
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

This AN describes a firmware that calibrates the internal RC (HSI) using an accurate 
reference frequency. The RTC/64 source clock is used as an accurate frequency source
for the calibration processus.


@par Directory contents 

  - RCCalibration/inc/stm32f10x_conf.h  Library Configuration file
  - RCCalibration/src/stm32f10x_it.c    Interrupt handlers
  - RCCalibration/inc/stm32f10x_it.h    Interrupt handlers header file
  - RCCalibration/src/main.c            Main program


@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL evaluation 
    boards and can be easily tailored to any other supported device and development 
    board. 
    
  - Connect PA.6 (TIM3 input capture 1) to PC.13 (tamper pin): in case of using
    the RTC/64 source clock. In case of using other refernec frequency, connect 
    the source to PA.8 (The source voltage levels have to correspond to levels 
    allowed for this pin).
     
  - Connect an oscilloscope probe to MCO pin (PA.8) to measure the RC frequency.
  
  - Connect LEDs as follows:
    - Green LED on PC.05.
    - Red LED on PC.06.
    - Five yellow LEDs from PC.00 to PC.04 to show HSTRIM bits value (5bits).
      
@par How to use it ? 

In order to load the RC Calibration code, you have do the following:

 - EWARM: 
    - Open the RC_Calibration.eww workspace
    - In the workspace toolbar select the project config:
        - STM3210B: to configure the project for STM32 Medium-density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE 
    - Open the RC_Calibration.rprj project
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210B: to configure the project for STM32 Medium-density devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)

 - RVMDK 
    - Open the RC_Calibration.Uv2 project
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
