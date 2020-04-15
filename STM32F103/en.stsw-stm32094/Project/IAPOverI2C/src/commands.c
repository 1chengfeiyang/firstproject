/**
  ******************************************************************************
  * @file IAPOverI2C/src/commands.c
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief  This file provides the different commands' routines.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */


/* Includes ------------------------------------------------------------------*/
#include "commands.h"
#include "stm32f10x.h"
#include "config.h"
#include "main.h"


/** @addtogroup IAPOverI2C
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t opcode;
extern EventStatus i2c_event;
uint8_t  I2C1_Buffer_Tx[1200];
uint16_t I2C1_Buffer_Rx[1200];
extern __IO uint16_t Tx_Idx , Rx_Idx;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Read opcode command transferred from the Master.
  * @param  None
  * @retval - The received opcode
  */
uint8_t Read_Opcode(void)
{
    /* Private variables ---------------------------------------------------------*/
    __IO uint8_t Opcode;
    Opcode = I2C1_Buffer_Rx[0];
    return (Opcode);
}



/**
  * @brief  This function reads the Address memory  transferred from
  *   Master.
  * @param  None
  * @retval -The address transfered from the Master
  */
uint32_t Read_Add(void)
{
    uint32_t Add_High1,Add_High0,Add_Low1,Add_Low0,Add_High,Add_Low,Add;

    Add_High1 = I2C1_Buffer_Rx[1] ;
    Add_High0 = I2C1_Buffer_Rx[2];
    Add_Low1  = I2C1_Buffer_Rx[3];
    Add_Low0  = I2C1_Buffer_Rx[4];

    Add_High1 = Add_High1 << 24;
    Add_High0 = Add_High0 << 16;
    Add_Low1 = Add_Low1 << 8;
    Add_High = Add_High1 | Add_High0;
    Add_Low =Add_Low1 | Add_Low0;
    Add= Add_High | Add_Low;
    return(Add);
}


/**
  * @brief Read the number of bytes to be read or written/Or the number
  *   of pages to be erased.
  * @param  None
  * @retval -the number of data bytes or pages
  */
uint16_t Read_Byte_Page_Number(void)
{
    /* Private variables ---------------------------------------------------------*/
    uint16_t Numbr_HL = 0x0000, Numbr_H = 0x0000, Numbr_L = 0x0000;

    Numbr_H = I2C1_Buffer_Rx[5] ;
    Numbr_L = I2C1_Buffer_Rx[6];

    Numbr_H= Numbr_H << 8;
    Numbr_HL=Numbr_H|Numbr_L ;
    return(Numbr_HL);
}

/**
  * @brief  This function ensures the read_memory_command including the reading add,
  *   number of data to read and the transfer of data from STM32 to host
  * @param none
  * @retval none
  */
void Read_Memory_Command(void)
{
    /* Private variables ---------------------------------------------------------*/
    uint32_t *Add_Flash;
    uint16_t Byte_Number;
    uint16_t Index=0;
    uint8_t *Aux_Add;
    __IO uint32_t Temp;

    /* wait until receiving opcode + address + number of data */
    while (Rx_Idx!= 0);
    I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
    /* Address reading */
    Add_Flash = (uint32_t*)Read_Add();
    /* reading the number of data to be read */
    Byte_Number = Read_Byte_Page_Number();

    for (Index=0; Index<(Byte_Number) ;Index++)
    {
        Aux_Add = (uint8_t*)Add_Flash;
        I2C1_Buffer_Tx[Index] = *(Aux_Add + Index) ;
    }
    /* Enable  the I2C_IT_EVT after the I2C1_Buffer_Tx is fully filled */
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR));
    /* Enable EVT IT in order to launch data transmission */
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    /* Clear ADDR flag by reading SR2 register */
    Temp = I2C1->SR2 ;
    while (Tx_Idx !=0);
    /* Reset opcode */
    opcode=0;
}

/**
  * @brief  This function writes a number of data beginning from a provided Flash address.
  * @param none
  * @retval none
  */
void Write_Memory_Command(void)
{
    uint16_t WriteCounter= 0;
    uint16_t Number_Bytes_Transferred = 0;
    uint32_t Add_Flash,DATA_SIZE_IN_PAGE = 0;
    uint16_t Idx = 0;


    /* Wait until receiving Flash address to  write into + number of data to be written 
      + the data to be written */
    while (Rx_Idx!= 0);

    /* Read the address  */
    Add_Flash = Read_Add();

    /* Read the number of bytes to be written  */
    Number_Bytes_Transferred = Read_Byte_Page_Number();
    if (Number_Bytes_Transferred <= PAGE_SIZE)
    {
        DATA_SIZE_IN_PAGE = 1;
    }
    else
    {
        if ((Number_Bytes_Transferred%PAGE_SIZE)==0)
        {
            DATA_SIZE_IN_PAGE= Number_Bytes_Transferred/PAGE_SIZE;
        }
        else
        {
            DATA_SIZE_IN_PAGE=(uint32_t) (Number_Bytes_Transferred/PAGE_SIZE)+1;
        }
    }

    for (Idx=4; Idx<(Number_Bytes_Transferred+8)/2; Idx++)
    {
        I2C1_Buffer_Rx[2*Idx+1]=I2C1_Buffer_Rx[2*Idx+1]<<8;
        I2C1_Buffer_Rx[2*Idx]=I2C1_Buffer_Rx[2*Idx]|I2C1_Buffer_Rx[2*Idx+1];
    }

#if defined USE_STM3210B_EVAL  || USE_STM32100B_EVAL

  Erase_Page(Add_Flash,DATA_SIZE_IN_PAGE);

#elif defined USE_STM3210E_EVAL ||  USE_STM3210C_EVAL

  if (((Add_Flash/(PAGE_SIZE/(PAGE_SIZE/Number_Bytes_Transferred)))&1)==0)
    {
        Erase_Page(Add_Flash,DATA_SIZE_IN_PAGE); 
    }
#endif  



    for (WriteCounter = 0; (WriteCounter < (Number_Bytes_Transferred)/2 ); WriteCounter++)
    {
        FLASH_ProgramHalfWord((Add_Flash+2*WriteCounter), I2C1_Buffer_Rx[2*(WriteCounter+4)]);
    }

    opcode=0;

}



/**
  * @brief   Performs erase page based on the Flash address and
  * the number of pages to be erased.
  * @param ADD_FLASH: adress in the first page to be erased
  * @param page_number: the number of pages to be erased.
  * @retval None
  */
void Erase_Page(uint32_t Add_Flash,uint16_t Page_Number)
{
    /* Private variables ---------------------------------------------------------*/
    __IO uint32_t EraseCounter = 0x00;
    uint32_t BASE_ADDRESS_PAGE;
    __IO uint32_t page_index;

    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    /* Erase the FLASH pages */

    page_index =(uint32_t) (Add_Flash-0x08000000)/PAGE_SIZE;
    BASE_ADDRESS_PAGE=(uint32_t)(0x08000000 + page_index * PAGE_SIZE);

    for (EraseCounter=0;(EraseCounter < Page_Number);EraseCounter++ )
    {
        FLASH_ErasePage(BASE_ADDRESS_PAGE+EraseCounter*PAGE_SIZE);
    }


}


/**
  * @brief   Erase page command.
  * @param none
  * @retval :   none
  */
void Erase_Page_Command(void)
{
    uint16_t Page_Number;
    uint32_t Add_Flash;
    /* Read the Flash  address */
    Add_Flash= (uint32_t)Read_Add();
    /* Read the number of page to be erased*/
    Page_Number = Read_Byte_Page_Number();
    /* Erase the corresponding page(s) */
    Erase_Page(Add_Flash,Page_Number);
    /* Reset opcode */
    opcode=0;
}



/**
  * @brief  Erase the user space memory (from 0x8001000 to the
  *   flash memory end address)".
  * @param  none.
  * @retval :  none.
  */
void User_Space_Memory_Erase_Command(void)
{

    /* Erase the user space memory */
    Erase_Page(USER_START_ADDRESS,(TOTAL_PAGE_NUMBER-((uint32_t) (USER_START_ADDRESS-0x08000000)/PAGE_SIZE)));
    opcode=0;
}
/**
  * @}
  */



/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
