/**
  ******************************************************************************
  * @file IAPOverI2C/src/main.c
  * @author  MCD Application Team
  * @version  V1.0.0.
  * @date     09/15/2010
  * @brief  Main program body
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
#include "main.h"



/** @addtogroup IAPOverI2C
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  ClockSpeed      400000
#define  OPC_READ       (uint8_t)(0x03)
#define  OPC_WREN       (uint8_t)(0x06)
#define  OPC_ERPG       (uint8_t)(0x20)
#define  OPC_ERUSM      (uint8_t)(0x60)
#define  OPC_USRCD      (uint8_t)(0x77)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

EventStatus i2c_event= NOEVENT;
uint8_t opcode,value_push_button;
I2C_InitTypeDef   I2C_InitStructure;
pFunction Jump_To_Application;
uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void iap_i2c(void);


/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main (void)
{

  
    /* Configures the different system clocks */
    RCC_Configuration();
    /* Configures the different GPIO ports */
    GPIO_Configuration();
   
    /* Test if Key push-button on STM3210x-EVAL Board is pressed */
    value_push_button = Push_Button_Read();
    if (value_push_button == 0x00)
    {
        /* If Key is pressed ,Execute the IAP i2c routine*/
        iap_i2c();
    }
    /* Keep the user application running */
    if (value_push_button == 0x01)
    {
        /*The user code is called in iap_i2c function when the value of opcode is equal to OPC_USRCD */
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*) (USER_START_ADDRESS +4);
        Jump_To_Application = (pFunction) JumpAddress;
        /* Initialize user application's Stack Pointer */
        __set_MSP(* ( __IO uint32_t* ) USER_START_ADDRESS);
        Jump_To_Application();
    }

	return (0);



}


/**
  * @brief  In application programming routine.
  * @param  None
  * @retval : None
  */
void iap_i2c(void)
{


     /* Unlock the Flash */
    FLASH_Unlock();
    /* Configures NVIC and Vector Table base location */
    NVIC_Configuration();
    I2C_DeInit(I2C1);
    /* I2C1 configuration ------------------------------------------------------*/
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
    I2C_Init(I2C1, &I2C_InitStructure);


    while (1)
    {

        /* Enable I2C1 event and buffer interrupts */
        I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
        /* Enable I2C1 Error interrupts */
        I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

        /* Only when there is an event and the the opcode is not yet read, the value
          of opcode must be tested */
        if (i2c_event == EVENT_OPCOD_NOTYET_READ)
        {
            /* Read opcode */
            opcode = Read_Opcode();
            i2c_event = EVENT_OPCOD_READ; /* The opcode has been already read */
        }

        switch (opcode)
        {
        case OPC_READ:
        
            Read_Memory_Command();
            break;

        case OPC_WREN:
        
            Write_Memory_Command();
            break;

        case OPC_ERPG:
        
            Erase_Page_Command();
            break;

        case OPC_ERUSM:
        
            User_Space_Memory_Erase_Command();
            break;

        case OPC_USRCD:
        
            /* Jump to user application */
            JumpAddress = *(__IO uint32_t*) (USER_START_ADDRESS +4);
            Jump_To_Application = (pFunction) JumpAddress;
            /* Initialize user application's Stack Pointer */
            __set_MSP(*(__IO uint32_t*) USER_START_ADDRESS);
            Jump_To_Application();

            break;
        default:
        break;

        }
    }

}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}
#endif

/**
  * @}
  */



/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/








