/**
  ******************************************************************************
  * @file IAPOverI2C/src/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "stm32f10x_it.h"
#include "main.h"

/** @addtogroup IAPOverI2C
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern EventStatus i2c_event;
__IO uint16_t Tx_Idx =0, Rx_Idx=0;
extern uint8_t  I2C1_Buffer_Tx[];
extern uint16_t I2C1_Buffer_Rx[];


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void  NMI_Handler(void)

{

}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}



/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}



/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}



/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}



/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}



/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}



/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}



/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (AN_example_src), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/


/**
  * @brief  This function handles I2C1 Event interrupt request.
  * @param  None
  * @retval : None
  */
void I2C1_EV_IRQHandler(void)
{

    switch (I2C_GetLastEvent(I2C1))
    {
        /* Slave Transmitter ---------------------------------------------------*/
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
    
      I2C_SendData(I2C1, I2C1_Buffer_Tx[Tx_Idx++]);
      break;


    case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:             /* EV3 */   

        /* Transmit I2C1 data */
        I2C_SendData(I2C1, I2C1_Buffer_Tx[Tx_Idx++]);
        break;



    /* Slave Receiver ------------------------------------------------------*/
    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:     /* EV1 */
    
      break;

    case I2C_EVENT_SLAVE_BYTE_RECEIVED:                /* EV2 */
        /* Store I2C1 received data */
        I2C1_Buffer_Rx[Rx_Idx++] = I2C_ReceiveData(I2C1);

        break;

    case I2C_EVENT_SLAVE_STOP_DETECTED:                /* EV4 */
        /* Clear I2C1 STOPF flag */
  
        I2C_Cmd(I2C1, ENABLE);
        Rx_Idx=0;
        i2c_event = EVENT_OPCOD_NOTYET_READ;
    break;

    default:
        break;
    }
}



/**
  * @brief  This function handles I2C1 Error interrupt request.
  * @param  None
  * @retval : None
  */
void I2C1_ER_IRQHandler(void)
{
    /* Check on I2C1 AF flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_AF))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
        Tx_Idx = 0;
        i2c_event = EVENT_OPCOD_NOTYET_READ;
    }


    /* Check on I2C1 AF flag and clear it */
    if (I2C_GetITStatus(I2C1, I2C_IT_BERR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
    }
}




/**
  * @}
  */


/**
  * @}
  */


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
