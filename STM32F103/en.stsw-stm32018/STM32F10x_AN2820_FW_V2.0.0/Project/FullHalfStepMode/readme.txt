/**
  @page FullHalfStepMode AN2820 FullHalfStepMode Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2008 STMicroelectronics *******************
  * @file FullHalfStepMode/readme.txt 
  * @author   MCD Application Team
  * @version  V2.0.0
  * @date     04/27/2009
  * @brief    readme file for STM32 Driving Bipolar Stepper Motors AN2820.
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

This application note describes how to achieve a compact size, high speed, low 
coast with less resources when driving the bipolar stepper motors using the STM32 
cortex-M3 based microcontrollers. It presents a simple method to implement the 
full and half operating modes in order to control the stepper motors

@par Directory contents 

  - inc: containing the user header files  
    - FullHalfStepMode/inc/stm32f10x_conf.h    Library Configuration files
    - FullHalfStepMode/inc/stm32f10x_it.h      Interrupt handlers header files
    - FullHalfStepMode/inc/StepperMotor.h      Motor control header file
                               
  - src: containing the user source files    
    - FullHalfStepMode/src/stm32f10x_it.c      Interrupt handlers
    - FullHalfStepMode/src/main.c              main program
    - FullHalfStepMode/src/StepperMotor.c      Motor control source routines


@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL evaluation 
    boards and can be easily tailored to any other supported device and development 
    board.
  
  - STM3210B-EVAL Set-up 
    - There is no specific hardware (single PCB board with one reference) used in 
      this Application Note.The validation was performed using a connection between
      an evaluation board for the L6208N DMOS driver ( ST board)And STM3210B-EVAL 
      (the STM32F10x controls the L6208N DMOS driver).For getting clear picture 
      concerning the hardware connection, please refer to the Figure.8,  in the 
      current Application Note documentation.
      
@par How to use it ? 

In order to load the FullHalfStepMode code, you have do the following:

 - EWARM: 
    - Open the FullHalf_Step_Mode.eww workspace
    - In the workspace toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE 
    - Open the FullHalf_Step_Mode.rprj project
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)

 - RVMDK 
    - Open the FullHalf_Step_Mode.Uv2 project
    - In the build toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
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
