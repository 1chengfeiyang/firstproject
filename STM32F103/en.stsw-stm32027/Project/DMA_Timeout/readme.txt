/**
  @page DMATimeout AN3109 DMA_Timeout Readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file  DMA_Timeout/readme.txt 
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date  12/15/2009
  * @brief  Description of the DMA_Timeout implementation for the USART.
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

This example provides a DMA_Timeout implementation method(Connect USART Rx with a Timer 
input capture). 

In this example, data are transmitted from USART_Tx into USART_Rx.
the USART_Tx is transmitter using DMA and the USART2 is  receiver using DMA 

The USART_Tx can be USART1 or USART3 depending on the STMicroelectronics EVAL board 
you are using and the USART_Rx is USART2.

USART_Tx Transmit DMA is configured to transfer only 5 data and USART2 Receive DMA is 
configured to receive 300 data. 

Timer2 is used in slave reset mode where the counter is reinitialized in response
to rising edges on an input capture (TIM2 Channel2) connected to the USART receive pin.


The TIM2CLK frequency is set to 72 MHz, the Prescaler is 1 so the TIM2 counter
clock is 36 MHz. 
Timeout = CCR1_Val/TIM2 counter clock 


Timer2 will generate an output compare interrupt signaling the timeout by toggling the PC.07.

USART1 and USART2 configured as follow:
  - BaudRate = 460800 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled


@par Directory contents 

  - DMA_Timeout/inc/stm32f10x_conf.h   Library Configuration file
  - DMA_Timeout/inc/platform_config.h  Evaluation board specific configuration file
  - DMA_Timeout/inc/stm32f10x_it.h     Interrupt handlers header file
  - DMA_Timeout/src/stm32f10x_it.c     Interrupt handlers
  - DMA_Timeout/src/main.c             Main program

@par Hardware and Software environment 

  - This example runs on STM32F10x Connectivity line, High-Density, Medium-Density 
    and Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210C-EVAL (STM32F10x 
    Connectivity line), STM3210E-EVAL (STM32F10x High-Density) and STM3210B-EVAL
    (STM32F10x Medium-Density) evaluation boards and can be easily tailored to
    any other supported device and development board.

   - STM3210C-EVAL Set-up 
     - Connect USART3 Tx pin (PC.10) to USART2 Rx pin (PD.06)
     - Connect USART2 Rx (PD.06) pin to TIM2 Channel2 (PA1).
     - Visualize the USART2 Rx and PC.7 signals using an oscilloscope. (the timeout
       is the time between the last rising edge on the USART2 RX and the PC.7
       toggling).
      @note In this case USART3 Tx and USART2 Rx pins are remapped by software on 
      PC.10 and PD.06 respectively. 
      Make sure that jumper JP3 is open or in position 1-2. 
      
  - STM3210E-EVAL Set-up 

     - Connect USART1 Tx pin (PA.09) to USART2 Rx pin (PA.03)
     - Connect USART2 Rx pin to TIM2 Channel2 (PA1).
     - Visualize the USART2 Rx and PC.7 signals using an oscilloscope. (the timeout
       is the time between the last rising edge on the USART2 RX and the PC.7
       toggling). 

     
  - STM3210B-EVAL Set-up 
     - Connect USART1 Tx pin (PA.09) to USART2 Rx pin (PD.06)
     - Note: in this case USART2 Rx pin is remapped by software on PD.06. 
     - Connect USART2 Rx pin to TIM2 Channel2 (PA1).
     - Visualize the USART2 Rx and PC.7 signals using an oscilloscope. (the timeout
       is the time between the last rising edge on the USART2 RX and the PC.7
       toggling). 


@par How to use it ?

 - RVMDK (v4.00)
    - Open the DMA_Timeout.uvopt project
    - In the build toolbar select the project config:
        - STM3210C-EVAL: to configure the project for STM32 Connectivity line devices
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)

 - EWARMv5 (v5.40) 
    - Open the DMA_Timeout.eww workspace.
    - In the workspace toolbar select the project config:
        - STM3210C-EVAL: to configure the project for STM32 Connectivity line devices
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 - RIDE (RIDE7)
    - Open the DMA_Timeout.rprj project.
    - In the configuration toolbar(Project->properties) select the project config:
        - STM3210C-EVAL: to configure the project for STM32 Connectivity line devices
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
    - Rebuild all files: Project->build project
    - Load project image: Debug->start(ctrl+D)
    - Run program: Debug->Run(ctrl+F9)  

 - HiTOP (v5.31)
    - Open the HiTOP toolchain.
    - Browse to open the DMA_Timeout.htp
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
 
 * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
 */


