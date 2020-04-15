
/**
  @page USARTInterruptMethod AN3070 USARTInterruptMethod Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2010 STMicroelectronics *******************
  * @file USARTInterruptMethod/readme.txt 
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date  01/04/2010
  * @brief  Description of the AN3070 Application note's USARTInterruptMethod.
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

This example describes how to manage the DE (Driver Enable) signal using the 
USART interrupts to meet the RS485 & IO-Link timing specification.
The user can select the CPU frequency 72MHz or 24MHz (for debug and timing 
measurements purpose) by commenting/uncommenting the following define in the
  - src/main.c file: #define HCLK_FREQ_72MHz.
The USART1 is used to transfer a buffer of 4 bytes at 230kps.


@par Directory contents 

  - USARTInterruptMethod/inc/stm32f10x_conf.h  Library Configuration file
  - USARTInterruptMethod/src/stm32f10x_it.c    Interrupt handlers
  - USARTInterruptMethod/inc/stm32f10x_it.h    Interrupt handlers header file
  - USARTInterruptMethod/src/main.c            Main program



@par Hardware and Software environment  

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210B-EVAL  evaluation 
    board and can be easily tailored to any other supported device and development 
    board except the STM3210C-EVAL board because the USART1 is not vailable on the
    board.
   
  - STM3210B-EVAL Set-up 
    - Connect the channel 1 probe of the oscilloscope to GPIOC.6 (DE signal).   
    - Connect the channel 2 probe of the oscilloscope to GPIOA.9 (USART1 TX pin).
    - Connect the channel 3 probe of the oscilloscope to GPIOA.8 (MCO pin).
            
         
@par How to use it ? 

In order to load the USARTInterruptMethod code, you have do the following:

- RVMDK (v4.00)
    - Open the USARTInterruptMethod.uvopt project
    - In the build toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

 - EWARMv5 (v5.40) 
    - Open the USARTInterruptMethod.eww workspace.
    - In the workspace toolbar select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE (RIDE7)
    - Open the USARTInterruptMethod.rprj project.
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)  

 - HiTOP (v5.31)
    - Open the HiTOP toolchain.
    - Browse to open the USARTInterruptMethod.htp
    - A "Download application" window is displayed, click "cancel".
    - Rebuild all files: Project->Rebuild all
    - Load project image : Click "ok" in the "Download application" window.
    - Run the "RESET_GO_MAIN" script to set the PC at the "main"
    - Run program: Debug->Go(F5).

@note
 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.

 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */
