/**
  ******************************************************************************
  * @file  USBI2CBridge/src/i2c_flash.c
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date     09/15/2010
  * @brief  This file provides a set of functions needed to manage the
  *                      I2C communication.
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
#include "i2c_flash.h"
#include "i2c_if.h"
#include "stm32f10x.h"



/** @addtogroup USBI2CBridge
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  ClockSpeed           400000
#define  OPC_READ             (uint8_t)(0x03)
#define  OPC_WREN             (uint8_t)(0x06)

#define  BufferSize           0x408
#define  I2C_SLAVE_ADDRESS7   0x30
#define I2C1_DR_Address       0x40005410
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t  I2C1_Buffer_Tx[BufferSize];
uint8_t  I2C1_Buffer_Rx[BufferSize];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Initializes the peripherals used by the I2C FLASH driver.
  * @param  None
  * @retval : None
  */
void I2C_FLASH_Init(void)
{
    I2C_InitTypeDef   I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    /* Enable peripheral clocks --------------------------------------------------*/
    /* Enable I2C1 and I2C2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 , ENABLE);
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*Reset the configuration of the I2C interface*/
    I2C_DeInit(I2C1);

    /* DMA1 channel6 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)I2C1_Buffer_Tx;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = BufferSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
    /* Enable I2C1 DMA */


    /* DMA1 channel7 configuration ---------------------------------------------*/
    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)I2C1_Buffer_Rx;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BufferSize;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);


    /* I2C1 configuration ------------------------------------------------------*/
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
    I2C_Init(I2C1, &I2C_InitStructure);
    /* Enable I2C  ------------------------------------------------------------*/
    I2C_Cmd(I2C1, ENABLE);
    I2C_DMACmd(I2C1, ENABLE);


}


/**
  * @brief  send opcode to the target device
  * @param  opcode,I2C1_Buffer[]
  * @param  opcode
  * @retval : I2C1_Buffer
  */
void i2c_send_opcode(uint8_t opcode,uint8_t I2C1_Buffer[])
{
    I2C1_Buffer[0] = opcode;
}



/**
  * @brief  This function sends the Address memory  to the target device.
  * @param ADD_FLASH
  * @param  I2C1_Buffer[]
  * @retval : I2C1_Buffer[]
  */
void i2c_send_add(uint32_t ADD_FLASH,uint8_t I2C1_Buffer[])
{

    uint8_t Add_High1,Add_High0,Add_Low1,Add_Low0;

    Add_Low0 = (uint8_t) ADD_FLASH;
    ADD_FLASH = ADD_FLASH>>8;
    Add_Low1 = (uint8_t) ADD_FLASH;
    ADD_FLASH = ADD_FLASH>>8;
    Add_High0 = (uint8_t) ADD_FLASH;
    ADD_FLASH = ADD_FLASH>>8;
    Add_High1 = (uint8_t) ADD_FLASH;

    I2C1_Buffer[1] = Add_High1;
    I2C1_Buffer[2] = Add_High0;
    I2C1_Buffer[3] = Add_Low1;
    I2C1_Buffer[4] = Add_Low0;

}


/**
  * @brief sends the number of bytes to be read or written.
  * @param byte_number
  * @param I2C1_Buffer[]
  * @retval  I2C1_Buffer[]
  */
void i2c_send_byte_number(uint16_t byte_number,uint8_t I2C1_Buffer[])
{

    uint8_t Numbr_H=0x0000;
    uint8_t Numbr_L=0x0000;

    Numbr_L = (uint8_t) byte_number;
    byte_number = byte_number>>8;
    Numbr_H = (uint8_t) byte_number;

    I2C1_Buffer[5] = Numbr_H;
    I2C1_Buffer[6] = Numbr_L;

}




/**
  * @brief  Reads a block of data from the FLASH.
  * @param pBuffer : pointer to the buffer that receives the data read
  *   from the FLASH.
  * @param ReadAddr : FLASH's internal address to read from.
  * @param NumByteToRead : number of bytes to read from the FLASH.
  * @retval : None
  */
void I2C_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint32_t add_flash;
    __IO uint32_t  Rx1_Idx=0;

    /*----------------------------1st sequence----------------------------------*/
    i2c_send_opcode(OPC_READ,I2C1_Buffer_Tx);
    add_flash = ReadAddr + 0x08000000;
    i2c_send_add(add_flash,I2C1_Buffer_Tx);
    i2c_send_byte_number(NumByteToRead,I2C1_Buffer_Tx);
    /* Initialize the DMA Transmit counter: transmit opcode + address + number of data to be read */
    DMA1_Channel6->CNDTR = 7;
    /* Enable DMA1 Channel6 */
    DMA_Cmd(DMA1_Channel6, ENABLE);
    /* Send I2C1 START condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on I2C1 EV5 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send I2C2 slave Address for write */
    I2C_Send7bitAddress(I2C1, I2C_SLAVE_ADDRESS7, I2C_Direction_Transmitter);
    /* Test on I2C1 EV6 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* DMA1 Channel6 transfer complete test */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC6));
    DMA_ClearFlag(DMA1_FLAG_TC6);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    DMA_Cmd(DMA1_Channel6, DISABLE);

    /*-------------------------------2nd sequence---------------------------------*/
    DMA1_Channel7->CNDTR = (BufferSize-8);

    /*Enable the last transfer bit*/
    I2C_DMALastTransferCmd(I2C1, ENABLE);
     /* Send I2C1 START condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on I2C1 EV5 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send slave Address for write */
    I2C_Send7bitAddress(I2C1, I2C_SLAVE_ADDRESS7, I2C_Direction_Receiver );
    /* Test on I2C1 EV7 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    DMA_Cmd(DMA1_Channel7, ENABLE);
    /* DMA1 Channel6 transfer complete test */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC7));
    DMA_ClearFlag(DMA1_FLAG_TC7);
    DMA_Cmd(DMA1_Channel7, DISABLE);
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);

    while (NumByteToRead--) /* while there is data to be read */
    {

        *pBuffer = I2C1_Buffer_Rx[Rx1_Idx++];
        /* Point to the next location where the byte read will be saved */
        pBuffer++;
    }

    /* Reset the receive counter */
    Rx1_Idx=0;

}



/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE
  *   cycle(Page WRITE sequence). The number of byte can't exceed
  *   the FLASH page size.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *   written into the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH,
  *   must be equal or less than "SPI_FLASH_PageSize" value.
  * @retval : None
  */
void I2C_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint32_t add_flash;
    __IO uint32_t Tx1_Idx = 0;
    /*----------------------------1st sequence----------------------------------*/
    i2c_send_opcode(OPC_WREN,I2C1_Buffer_Tx);
    add_flash = WriteAddr + 0x08000000;
    i2c_send_add(add_flash,I2C1_Buffer_Tx);
    i2c_send_byte_number(NumByteToWrite,I2C1_Buffer_Tx);
    /* Dummy byte */
    I2C1_Buffer_Tx[7] = 0xFF;
    Tx1_Idx = 0;
    while (NumByteToWrite--)
    {
        /* Send the current byte */
        I2C1_Buffer_Tx[Tx1_Idx + 8] =*pBuffer;
        /* Point on the next byte to be written */
        pBuffer++;
        Tx1_Idx++;
    }
    DMA1_Channel6->CNDTR = BufferSize;
    /* Enable DMA1 Channel6 */
    DMA_Cmd(DMA1_Channel6, ENABLE);
    /* Send I2C1 START condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on I2C1 EV5 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send I2C2 slave Address for write */
    I2C_Send7bitAddress(I2C1, I2C_SLAVE_ADDRESS7, I2C_Direction_Transmitter);
    /* Test on I2C1 EV6 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* DMA1 Channel6 transfer complete test */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC6));
    DMA_ClearFlag(DMA1_FLAG_TC6);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    DMA_Cmd(DMA1_Channel6, DISABLE);



}




/**
  * @}
  */




/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
