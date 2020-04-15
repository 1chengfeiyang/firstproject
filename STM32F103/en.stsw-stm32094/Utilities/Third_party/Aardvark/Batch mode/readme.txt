/**
  @page Batch Mode Batch mode

  @verbatim
  ******************** (C) COPYRIGHT 2010 STMicroelectronics *******************
  * @file Batch Mode/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/15/2010
  * @brief   Description of the Bath mode directory.
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

This directory contains a set of XML files that represent some commands to 
be sent by the Aardvark tool to the STM32 Target.

@par Directory contents

iap_i2c_erase_page.xml: allows erasing the 7 first pages in the user space flash memory.  
iap_i2c_erase_usm.xml: allows erasing all the user space flash memory (beginning from 
0x8001000)
iap_i2c_goto_user_code.xml: allows jumping to the user code.
iap_i2c_write_16.xml: allows writing 16 data bytes beginning from address 0x8001000.
iap_i2c_systick.xml: allows programming the Systick example in the standard fimrware library 
compiled for Medium-density devices.

You can create your own XML files based on the guidelines provided within the application note 
document. 

  
 * <h3><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h3>
 */
