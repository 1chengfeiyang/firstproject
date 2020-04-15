/**
  ******************************************************************************
  * @file    hal_uart.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    14-April-2014
  * @brief   UART Driver Low Power Protocol
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx.h"
#include "hal_uart_driver_configuration.h"
#include "hal_uart_interfaces.h"
#include "hal_uart_driver_definition.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/**
 *	List of modules that shall grant the notification to be sent to the application
 *
 *	The UART driver decodes on its own the header of the received packet
 *	Even though the application did not yet request to receive the payload, the UART driver
 *	starts reception
 *	The payload receive notification cannot be sent until all these modules gave acceptance
 *
 *	It supports only 8 members. To support more members, the type of ubRxPayloadRequest shall be changed
 *	When a new member is added, the definition of ENABLE_PAYLOAD_NOTIFICATION shall be updated
 */
typedef enum{
	eApplication,		/**<  The application has requested to receive the payload */
	eBLEModule			/**<  The payload has been fully received and is ready to be sent to the application */
}eRxPayloadNotificationRequester_t;

/**
 *	States to defined which part of the packet the Host is waiting for
 *
 *	The UART driver decodes on its own the header of the received packet
 *	Even though the application did not yet request to receive the payload, the UART driver
 *	starts reception
 *	The first receive request from the application shall be the Header. The next receive request from the application
 *	shall be the payload
 *	These two states track which part of the packet the Host is expecting.
 */
typedef enum{
	eHostHeaderToRx,	/**<  The application is expecting to receive the header */
	eHostPayloadToRx	/**<  The application is expecting to receive the payload */
}eHostRxState_t;

/**
 *	States to defined which part of the packet is being received by the UART driver
 *
 * The UART low power protocol requires specific actions on different steps while receiving a packet
 * This is the list of steps where dedicated actions shall be taken
 */
typedef enum{
	eHeaderToRx,		/**<  The UART driver is receiving the header */
	ePayloadToRx,		/**<  The UART driver is receiving the payload */
	eLast2ndByteToRx,	/**<  The application is receiving the last second byte of the payload (n-1) */
	eLastByteToRx		/**<  The application is receiving the last byte of the payload */
}eRxState_t;

/**
 *	List of event being tracked by the UART driver
 *
 * The UART low power protocol requires some acknowledge on different steps while transmitting a packet
 * This is the list of steps where dedicated actions shall be taken.
 * There are all based on CTS interrupt at different time of the UART activity
 */
typedef enum{
	eRemoteTxrequest,		/**<  The UART driver is listening for a Remote transmit request */
	eLastSentByteAck,		/**<  The UART driver is waiting for the ack of the last byte sent  of the payload */
	eLast2ndSentByteAck,	/**<  The UART driver is waiting for the ack of the last second byte sent (n-1) of the payload */
	eAckToTxRequest			/**<  The UART driver is waiting for the ack of a transmit request */
}eWCTSEventWaited_t;

/**
 *	Event being tracked by the UART Interrupt Handler when sending a packet
 *
 * The UART low power protocol requires dedicated actions when sending the last two byte of the packet
 */
typedef enum{
	ePayloadSent,	/**<  The second last byte (n-1) has been shifted out from the UART */
	eLastByteSent	/**<  The last byte has been shifted out from the UART */
}eUSART_TC_EventWaited_t;

/**
 *	Context of the UART driver
 *
 * The UART Low power driver stores several informations to handle properly the reception and transmission of a message
 * These are all stored in this structure instead of being scattered across several variables
 */
typedef struct{
	pf_HAL_UART_PhyDriverEvent_Handler_t	pf_PhyDriverEvent_Handler;	/**<  BLE Event handler to be called to notify end of either RX or TX */
	uint32_t								uwTxFlag;					/**<  This flag is tracking the transmit activity to handle low power protocol */
	uint32_t								uwRxFlag;					/**<  This flag is tracking the receive activity  to handle low power protocol */
	eWCTSEventWaited_t						eWCTSEventWaited;			/**<  The type of event expected on the CTS interrupt  */
	eUSART_TC_EventWaited_t					eUSART_TC_EventWaited;		/**<  The part of the packet being shifted out the UART  */
	eRxState_t								eRxState;					/**<  The UART driver Rx state machine to apply the low power protocol */
	eHostRxState_t							eHostRxState;				/**<  The part of the packet the BLE application is expected */
	uint32_t								uwWCTS_TxStarted;			/**<  Used in the CTS interrupt handler to know when transmit is ongoing */
	uint16_t								uhTxSize;					/**<  The size from the header of the transmit packet */
	uint16_t								uhRxSize;					/**<  The size from the header of the receive packet */
	uint8_t*								pTxBuffer;					/**<  The address of the transmit buffer */
	uint8_t*								pRxBuffer;					/**<  The address of the receive buffer */
}sUartContext_t;


/* Private macros ------------------------------------------------------------*/

/**
 *	Macro to disable HW CTS flow control
 *
 *	The macro disables the CTS HW flow control in the UART block
 */
#define	DISABLE_HW_CTS(UART_SELELECTED)	(UART_SELELECTED->CR3 &= (~USART_HardwareFlowControl_CTS))

/**
 *	Macro to enable HW CTS flow control
 *
 *	The macro is enables the CTS HW flow control in the UART block
 */
#define	ENABLE_HW_CTS(UART_SELELECTED)		(UART_SELELECTED->CR3 |= (USART_HardwareFlowControl_CTS))

/**
 *	Macro to set a GPIO
 *
 *	The macro sets the GPIO_PIN in GPIO_PORT
 */
#define	SET_GPIO(GPIO_PORT, GPIO_PIN)	(GPIO_PORT->BSRRL = GPIO_PIN)

/**
 *	Macro to read the GPIO level
 *
 *	The macro reads the GPIO_PIN level on GPIO_PORT
 */
#define	READ_GPIO_INPUTBIT(GPIO_PORT, GPIO_PIN)	(GPIO_PORT->IDR & GPIO_PIN)


/* Private defines -----------------------------------------------------------*/

/**
 *	Packet Header size
 *
 *	It defines the packet header size (in bytes) that the UART driver is expected at the start of a message
 */
#define	PACKET_HEADER_SIZE			((uint32_t)2)

/**
 *	Condition to notify the payload reception to the BLE module
 *
 *	To enable the payload notification, all modules shall set their bit
 */
#define	ENABLE_PAYLOAD_NOTIFICATION		0x03


/* Private variables ---------------------------------------------------------*/

/**
 *	UART context
 *
 *	Variable to hold the UART context to be used in the UART driver
 */
static volatile sUartContext_t sUartContext;

/**
 *	Flag to lock the Rx payload notification to the BLE module
 *
 *	This is a container of all modules that shall allow the Rx payload notification to the BLE module
 */
static volatile uint8_t	ubRxPayloadRequest;


/* Private function prototypes -----------------------------------------------*/
static void dma_rx_handler(void);
static void uart_handler(void);
static void uart_init(void);
static void gpio_init(void);
static void dma_init(void);
static void wcts_interrupt_init(void);
static void wcts_interrupt_enable(eWCTSEventWaited_t eWCTSEventWaited);
static void wcts_interrupt_disable(void);
static void set_wrts_gpio_mode(GPIOMode_TypeDef eGPIO_mode);
static void RxPayloadNotification(eRxPayloadNotificationRequester_t eRxPayloadNotificationRequester);
static void SetGPIO_PUPD(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, GPIOPuPd_TypeDef GPIOPuPd);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Set the GPIO PU/PD configuration
  *
  * @note   This API shall never be called in a PRIMASK critical section as it would re-enable
  * 		the PRIMASK bit before this would be expected by the calling function
  *
  * @param 	GPIO_Pin_Source : The GPIO pin on which the PU/PD configuration shall apply
  *
  * @param 	GPIOPuPd : The PU/PD configuration that shall be applied
  *
  * @retval None
  */
static void SetGPIO_PUPD(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, GPIOPuPd_TypeDef GPIOPuPd)
{
	/**
	 * This is a temporary register to avoid any unwanted transient state when performing read/modify/write
	 * operation on the destination register
	 */
	uint32_t uwTempRegister;

	/**
	 * The access to the GPIO register may be colliding with an access from the application
	 * The operation is protected by a critical section to avoid any corruption
	 */
	USART_ENTER_CRITICAL_SECTION;
	uwTempRegister = GPIOx->PUPDR;
	uwTempRegister &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)GPIO_Pin_Source * 2));
	uwTempRegister |= (((uint32_t)GPIOPuPd) << ((uint16_t)GPIO_Pin_Source * 2));
	GPIOx->PUPDR = uwTempRegister;
	USART_EXIT_CRITICAL_SECTION;

	return;
}

/**
  * @brief  Rx Payload Application Notification
  *
  * @note	The payload notification cannot be sent before the BLE application module
  * 		has requested to receive the payload
  * 		The UART driver is receiving the payload on its own based on the length
  * 		decoded in the header. It may happen the UART driver has received the payload before
  * 		the BLE uart module has made the request
  * 		The payload notification shall be notified to the BLE application when both :
  * 		a) The BLE application made the request to receive the payload
  * 		b) The UART driver has received the payload
  *
  * 		This API shall never be called in a PRIMASK critical section as it would re-enable
  * 		the PRIMASK bit before this would be expected by the calling function.
  *
  * @param  eRxPayloadNotificationRequester : The module that is ready to notify the payload to the BLE application
  *
  * @retval None
  */
static void RxPayloadNotification(eRxPayloadNotificationRequester_t eRxPayloadNotificationRequester)
{
	sHAL_UART_PhyDriverEvent_t sPhyDriverEvent;

	/**
	 * The container may be access from two different contexts
	 * The operation is protected by a critical section to avoid any corruption
	 */
	USART_ENTER_CRITICAL_SECTION;
	ubRxPayloadRequest |= (1<<eRxPayloadNotificationRequester);
	USART_EXIT_CRITICAL_SECTION;

	if(ubRxPayloadRequest == ENABLE_PAYLOAD_NOTIFICATION)
	{
		/* notify payload reception */
		ubRxPayloadRequest = 0;

		sPhyDriverEvent.ePhyDriverNotification = eUART_RxDone;
		sPhyDriverEvent.uwParam1 = (uint32_t)sUartContext.pRxBuffer;
		HAL_UART_Msg_Handler(sUartContext.pf_PhyDriverEvent_Handler, sPhyDriverEvent);
	}

	return;
}

/**
  * @brief  Set the RTS GPIO Mode
  *
  * @note	This API shall never be called in a PRIMASK critical section as it would re-enable
  * 		the PRIMASK bit before this would be expected by the calling function.
  *
  * @param  eGPIO_mode : The mode to be used with the RTS GPIO pin
  *
  * @retval None
  */
static void set_wrts_gpio_mode(GPIOMode_TypeDef eGPIO_mode)
{
	/**
	 * This is a temporary register to avoid any unwanted transient state when performing read/modify/write
	 * operation on the destination register
	 */
	uint32_t uwTempRegister;

	/**
	 * The access to the GPIO register may be colliding with an access from the application
	 * The operation is protected by a critical section to avoid any corruption
	 */
	USART_ENTER_CRITICAL_SECTION;
	uwTempRegister = USART_WRTS_GPIO_PORT->MODER;
	uwTempRegister  &= ~(0x03 << (USART_WRTS_PIN_SOURCE * 2));
	uwTempRegister |= (((uint32_t)eGPIO_mode) << (USART_WRTS_PIN_SOURCE * 2));
	USART_WRTS_GPIO_PORT->MODER = uwTempRegister;
	USART_EXIT_CRITICAL_SECTION;

	return;
}


/**
  * @brief  Disable CTS interrupt
  *
  * @note	The CTS interrupt is always kept enable in the NVIC. It is disable only in the EXTI block
  *
  * 		This API shall never be called in a PRIMASK critical section as it would re-enable
  * 		the PRIMASK bit before this would be expected by the calling function.
  *
  * @param  None
  *
  * @retval None
  */
static void wcts_interrupt_disable(void)
{
	/**
	 * The access to the EXTI register may be colliding with an access from the application
	 * The operation is protected by a critical section to avoid any corruption
	 */
	USART_ENTER_CRITICAL_SECTION;
	EXTI->IMR &= ~USART_WCTS_EXTI_LINE;	/**< Mask out CTS interrupt in EXTI block */
	USART_EXIT_CRITICAL_SECTION;

	/**
	 * As the register is only written, the operation cannot collide with other access
	 * The operation is out of critical section
	 */
	EXTI->PR = USART_WCTS_EXTI_LINE;	/**< Clear pending bit in EXTI block */

	return;
}

/**
  * @brief  Enable CTS interrupt
  *
  * @note	The UART low power protocol is using the CTS line to request transmission
  * 		and to ack packet reception (in combination with HW UART flow control)
  * 		When handling the CTS line as low power control, the actions are taken
  * 		based on the level of the line
  * 		The STM32 supports only edge detection with the EXTI block to generate an interrupt
  * 		To limit the number of access dynamically to the EXTI block, the line is configured
  * 		to generate an interrupt on both rising and falling edge (that means on any change of the IO level)
  * 		When the CTS interrupt is enable, it is passed as parameter which event the FW is waiting for
  * 		Based on the expected event, the FW knows what is the expected level.
  * 		In order to not miss the changing level when the CTS pulse is very short, the FW is:
  * 		a) First enabling CTS changing state detection
  * 		b) Checking the IO level in case the pulse
  *
  *
  * 		This API shall never be called in a PRIMASK critical section as it would re-enable
  * 		the PRIMASK bit before this would be expected by the calling function.
  *
  * @param  eWCTSEventWaited
  *
  * @retval None
  */
static void wcts_interrupt_enable(eWCTSEventWaited_t eWCTSEventWaited)
{
	/**
	 *  Start of critical section
	 */
	USART_ENTER_CRITICAL_SECTION;

	if((eWCTSEventWaited != eRemoteTxrequest) ||(sUartContext.uwWCTS_TxStarted == DISABLE))
	{
		sUartContext.eWCTSEventWaited = eWCTSEventWaited;

		/**
		 *  Enable WCTS interrupt in EXTI block
		 */
		EXTI->IMR |= USART_WCTS_EXTI_LINE;

		switch(eWCTSEventWaited)
		{
			case eLastSentByteAck:
				/**
				 * Check WCTS GPIO line
				 */
				if(READ_GPIO_INPUTBIT(USART_WCTS_GPIO_PORT, USART_WCTS_PIN) == Bit_SET)
				{
					/**
					 * Edge has been missed - Set the bit manually
					 */
					EXTI->SWIER = USART_WCTS_EXTI_LINE;
				}
				break;

			case eLast2ndSentByteAck:
			case eAckToTxRequest:
			case eRemoteTxrequest:
				/**
				 * Check WCTS GPIO line
				 */
				if(READ_GPIO_INPUTBIT(USART_WCTS_GPIO_PORT, USART_WCTS_PIN) == Bit_RESET)
				{
					/**
					 * Edge has been missed - Set the bit manually
					 */
					EXTI->SWIER = USART_WCTS_EXTI_LINE;
				}
				break;

			default:
				break;
		}

	}

	/**
	 * end of critical section
	 */
	USART_EXIT_CRITICAL_SECTION;

	return;
}

/**
  * @brief  Initialize EXTI
  *
  * @note	This API sets the NVIC and the static configuration of the EXTI block
  *
  * 		This API shall never be called in a PRIMASK critical section
  *
  * @param  None
  *
  * @retval None
  */
static void wcts_interrupt_init(void)
{
	USART_ENTER_CRITICAL_SECTION;
	/**
	 * Select WCTS on EXTI
	 */
	SYSCFG_EXTILineConfig(USART_WCTS_EXTI_PORT, USART_WCTS_EXTI_PIN);

	/**
	 * Enable both edge
	 */
	EXTI->RTSR |= USART_WCTS_EXTI_LINE;
	EXTI->FTSR |= USART_WCTS_EXTI_LINE;
	USART_EXIT_CRITICAL_SECTION;

	/**
	 * Set NVIC priority
	 */
	NVIC_SetPriority(USART_WCTS_NVIC_VECTOR, NVIC_UART_WCTS_IT_PRIORITY);

	/**
	 *	Enable NVIC interrupt
	 */
	NVIC_EnableIRQ(USART_WCTS_NVIC_VECTOR);

	return;
}

/**
  * @brief  Initialize DMA
  *
  * @note	This API sets the NVIC and the static configuration of the DMA block
  *
  * @param  None
  *
  * @retval None
  */
static void dma_init(void)
{
	DMA_InitTypeDef sDMA_InitStruct;

	/**
	 * Common DMA configuration
	 */
	sDMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	sDMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	sDMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	sDMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	sDMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	sDMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	sDMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	sDMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART_BLE->DR);

	/**
	 * Configure UART2 TX path
	 */
	sDMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Init(DMA_USART_TX_CHANNEL, &sDMA_InitStruct);

	/**
	 * Configure UART2 RX path
	 */
	sDMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Init(DMA_USART_RX_CHANNEL, &sDMA_InitStruct);
	DMA_ITConfig(DMA_USART_RX_CHANNEL, DMA_IT_TC, ENABLE);

	/**
	 * Set NVIC priority
	 */
	NVIC_SetPriority(DMA_USART_RX_NVIC_VECTOR, NVIC_UART_DMA_IT_PRIORITY);

	/**
	 * Enable NVIC interrupt
	 */
	NVIC_EnableIRQ(DMA_USART_RX_NVIC_VECTOR);

    return;
}

/**
  * @brief  Initialize UART
  *
  * @note	This API sets the NVIC and the static configuration of the UART block
  *
  * @param  None
  *
  * @retval None
  */
static void uart_init(void)
{
	USART_InitTypeDef sUSART_InitStructure;

	/**
	 * Enable UART block
	 */
	USART_Cmd(USART_BLE, ENABLE);

    /**
     * Clear TC Flag
     */
   	USART_BLE->SR &= (~USART_FLAG_TC);

	/**
	 * OverSample = 8
	 */
	USART_OverSampling8Cmd(USART_BLE, ENABLE);

    /**
    *  USART configuration
    * BaudRate = 921600 baud
    * Word Length = 8 Bits
    * One Stop Bit
    * No parity
    * CTS/RTS Hardware flow control enabled.
    * Receive and transmit enabled
    */
	sUSART_InitStructure.USART_BaudRate = USART_BAUDRATE;
	sUSART_InitStructure.USART_WordLength = USART_WordLength_8b;
	sUSART_InitStructure.USART_StopBits = USART_StopBits_1;
	sUSART_InitStructure.USART_Parity = USART_Parity_No;
	sUSART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
	sUSART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART_BLE, &sUSART_InitStructure);

    /**
     * Wait IDLE character to be sent
     */
    while(!(USART_BLE->SR & (USART_FLAG_TC)));

    /**
     * Clear TC Flag
     */
   	USART_BLE->SR &= (~USART_FLAG_TC);

	/**
	 * Enable TC interrupt
	 */
	USART_BLE->CR1 |= USART_FLAG_TC;

	/**
	 * Set NVIC priority
	 */
	NVIC_SetPriority(USART_NVIC_VECTOR, NVIC_UART_UART_IT_PRIORITY);

	/**
	 * Enable USART interrupt in NVIC
	 */
	NVIC_EnableIRQ(USART_NVIC_VECTOR);

	/**
	 * enable DMA in UART2 for receive and transmit
	 */
	USART_DMACmd(USART_BLE, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

    return;
}

/**
  * @brief  Initialize GPIO configuration
  *
  * @note   All registers from a port used by the BLE UART shall not be changed by the application when
  * 		this API is running
  *
  * @param  None
  *
  * @retval None
  */
static void gpio_init(void)
{
	GPIO_InitTypeDef sGPIO_InitStruct;

	/**
	 * Select Alternate Function
	 */
	GPIO_PinAFConfig(USART_TX_GPIO_PORT, USART_TX_PIN_SOURCE, USART_TX_AF);
	GPIO_PinAFConfig(USART_RX_GPIO_PORT, USART_RX_PIN_SOURCE, USART_RX_AF);
	GPIO_PinAFConfig(USART_WCTS_GPIO_PORT, USART_WCTS_PIN_SOURCE, USART_WCTS_AF);
	GPIO_PinAFConfig(USART_WRTS_GPIO_PORT, USART_WRTS_PIN_SOURCE, USART_WRTS_AF);

	/**
	 * UART TX Pin
	 */
	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	sGPIO_InitStruct.GPIO_Pin = USART_TX_PIN;
	GPIO_Init(USART_TX_GPIO_PORT,&sGPIO_InitStruct);

	/**
	 * UART RX Pin
	 */
	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	sGPIO_InitStruct.GPIO_Pin = USART_RX_PIN;
	GPIO_Init(USART_RX_GPIO_PORT,&sGPIO_InitStruct);

	/**
	 * UART CTS Pin
	 */
	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	sGPIO_InitStruct.GPIO_Pin = USART_WCTS_PIN;
	GPIO_Init(USART_WCTS_GPIO_PORT,&sGPIO_InitStruct);

	/**
	 * UART RTS Pin
	 */
	sGPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	sGPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	SET_GPIO(USART_WRTS_GPIO_PORT, USART_WRTS_PIN);
	sGPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	sGPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	sGPIO_InitStruct.GPIO_Pin = USART_WRTS_PIN;
	GPIO_Init(USART_WRTS_GPIO_PORT,&sGPIO_InitStruct);

	/** Once the IOs are configured, the wakeup pin shall be disabled to removed the forced Input PD state on CTS line
	 * This shall be done after the IOs is configured to avoid intermediate Input Nopull state
	 */
	PWR_WakeUpPinCmd(USART_WAKEUP_PIN_SELECTED, DISABLE);

    return;
}

/**
  * @brief  DMA Rx Handler
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void dma_rx_handler(void)
{
	sHAL_UART_PhyDriverEvent_t sPhyDriverEvent;

	sUartContext.uwRxFlag = ENABLE;

	DMA_ClearITPendingBit(DMA_USART_RX_CHANNEL_FLAG);

	sUartContext.uhRxSize = *(uint16_t*)sUartContext.pRxBuffer;

	switch(sUartContext.eRxState)
	{
		case eHeaderToRx:
			/**
			 * The DMA could be used only to receive a payload with more that two bytes
			 */
			if(sUartContext.uhRxSize > 2)
			{
				/**
				 * Go with DMA to receive other data
				 */
				sUartContext.eRxState = ePayloadToRx;
				DMA_Cmd(DMA_USART_RX_CHANNEL, DISABLE);
				DMA_USART_RX_CHANNEL->CNDTR = sUartContext.uhRxSize-PACKET_HEADER_SIZE;
				DMA_USART_RX_CHANNEL->CMAR = (uint32_t)(sUartContext.pRxBuffer+PACKET_HEADER_SIZE);
				DMA_Cmd(DMA_USART_RX_CHANNEL, ENABLE);

				sPhyDriverEvent.ePhyDriverNotification = eUART_RxDone;
				sPhyDriverEvent.uwParam1 = (uint32_t)sUartContext.pRxBuffer;
				HAL_UART_Msg_Handler(sUartContext.pf_PhyDriverEvent_Handler, sPhyDriverEvent);
			}
			else
			{
				/**
				 * Go with USART interrupt to receive end of packet
				 */
				sUartContext.eRxState = eLast2ndByteToRx;
				/**
				 * Enable RX interrupt
				 */
				USART_BLE->CR1 |= USART_FLAG_RXNE;
			}

			HAL_UART_HSclkRequest(eUART_Rx, eUART_HSclkEnable);

			break;

		case ePayloadToRx:
			/**
			 * Go with USART interrupt to receive end of packet
			 */
			sUartContext.eRxState = eLast2ndByteToRx;
			/**
			 * Enable RX interrupt
			 */
			USART_BLE->CR1 |= USART_FLAG_RXNE;
			break;

		default:
			break;
	}

	return;
}

/**
  * @brief  UART BLE Handler
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void uart_handler(void)
{
	uint8_t		*plocalRxbuffer;
	uint16_t	localRxSize;

	if( (USART_BLE->SR & USART_FLAG_TC) && (USART_BLE->CR1 & USART_FLAG_TC) )
	{
		/**
		 * Clear TC Flag
		 */
	   	USART_BLE->SR = (~USART_FLAG_TC);

		if(sUartContext.eUSART_TC_EventWaited == ePayloadSent)
		{
			wcts_interrupt_enable(eLast2ndSentByteAck);
			sUartContext.eUSART_TC_EventWaited = eLastByteSent;
		}
		else
		{
			sUartContext.eUSART_TC_EventWaited = ePayloadSent;
		}
	}

	if( (USART_BLE->SR & USART_FLAG_RXNE) && (USART_BLE->CR1 & USART_FLAG_RXNE) )
	{
		switch(sUartContext.eRxState)
		{
			case eLast2ndByteToRx:
				SetGPIO_PUPD(USART_WCTS_GPIO_PORT, USART_WCTS_PIN_SOURCE, GPIO_PuPd_UP);
				plocalRxbuffer = sUartContext.pRxBuffer;
				localRxSize = sUartContext.uhRxSize;
				*(plocalRxbuffer + localRxSize) = USART_BLE->DR;

				/**
				 * Go with USART interrupt to receive end of packet
				 */
				sUartContext.eRxState = eLastByteToRx;
				break;

			case eLastByteToRx:
				/**
				 * Stop flow
				 */
				SET_GPIO(USART_WRTS_GPIO_PORT, USART_WRTS_PIN);
				set_wrts_gpio_mode(GPIO_Mode_OUT);

				/**
				 * Disable the DMAR_Request
				 */
				USART_BLE->CR3 &= (~USART_DMAReq_Rx);
				/**
				 * read byte
				 */
				plocalRxbuffer = sUartContext.pRxBuffer;
				localRxSize = sUartContext.uhRxSize;
				*(plocalRxbuffer + localRxSize + 1) = USART_BLE->DR;
				sUartContext.uwRxFlag = DISABLE;
				/**
				 * Disable RX interrupt
				 */
				USART_BLE->CR1 &= (~USART_FLAG_RXNE);
				if(sUartContext.uwTxFlag == ENABLE)
				{
					/**
					 * Enable receiving path
					 */
					set_wrts_gpio_mode(GPIO_Mode_AF);
				}
				else
				{
					/**
					 * Enable WCTS interrupt
					 */
					wcts_interrupt_enable(eRemoteTxrequest);
				}

				sUartContext.eRxState = eHeaderToRx;
				/**
				 * Enable the DMAR_Request
				 */
				USART_BLE->CR3 |= USART_DMAReq_Rx;

				HAL_UART_HSclkRequest(eUART_Rx, eUART_HSclkDisable);

				RxPayloadNotification(eBLEModule);
				break;

			default:
				break;
		}
	}

	return;
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Allocate all resources to get the UART ready to be used
  *
  * @note   The UART implementation is using a DMA and supporting the Low Power Control Mechanism
  *
  * @param  Param1     The handler to be called when data are received over UART
  *
  * @retval None
  */
void HAL_UART_uart_open(pf_HAL_UART_PhyDriverEvent_Handler_t pf_PhyDriverEvent_Handler)
{
	/**
	 * UART context configuration
	 */
	sUartContext.pf_PhyDriverEvent_Handler = pf_PhyDriverEvent_Handler;
	sUartContext.uwRxFlag = DISABLE;
	sUartContext.uwTxFlag = DISABLE;
	sUartContext.eRxState = eHeaderToRx;
	sUartContext.eWCTSEventWaited = eRemoteTxrequest;
	sUartContext.eHostRxState = eHostHeaderToRx;
	sUartContext.uwWCTS_TxStarted = DISABLE;
	sUartContext.eUSART_TC_EventWaited = ePayloadSent;

	ubRxPayloadRequest = 0;

	/**
	 * Configure UART block
	 */
	uart_init();

	/**
	 * Configure DMA1
	 */
	dma_init();

	/**
	 * Configure GPIO
	 */
	gpio_init();

	/**
	 * Initialize EXTI block for WCTS interrupt
	 */
	wcts_interrupt_init();

	/**
	 * Enable WCTS interrupt
	 */
	wcts_interrupt_enable(eRemoteTxrequest);

	return;
}

/**
  * @brief  Interface to the application to receive data from UART
  *
  * @note	The API is called twice to receive the packet in two steps
  * 		1) The header
  * 		2) The payload
  * 		The UART driver receives the full packet as soon as the application requests the header
  * 		It is the responsability of the application to make sure both the header and the payload are
  * 		located consecutively in the same buffer
  * 		As the UART driver may receives the payload before the application requests for it, the notification
  * 		for the reception of the payload is delayed until requested by the application
  *
  * @param  pbuffer: The address of the buffer to store the data received. The application shall make sure the payload is received
  * 				  in the same buffer and consecutively to the header
  *
  * @retval None
  */
void HAL_UART_receive_data(uint8_t * pbuffer)
{
	if(sUartContext.eHostRxState == eHostHeaderToRx)
	{
		sUartContext.eHostRxState = eHostPayloadToRx;

		/**
		 * Receive the header
		 */
		sUartContext.pRxBuffer = pbuffer;

		DMA_Cmd(DMA_USART_RX_CHANNEL, DISABLE);
		DMA_USART_RX_CHANNEL->CNDTR = PACKET_HEADER_SIZE;
		DMA_USART_RX_CHANNEL->CMAR = (uint32_t)pbuffer;
		DMA_Cmd(DMA_USART_RX_CHANNEL, ENABLE);
	}
	else
	{
		sUartContext.eHostRxState = eHostHeaderToRx;

		/**
		 * Receive the payload
		 */
		RxPayloadNotification(eApplication);
	}
    return;
}

/**
  * @brief  Interface to the application to send data over UART
  *
  * @note
  *
  * @param  pbuffer: The address of the buffer holding the data to send
  *
  * @param  uhSize: The size of data to send
  *
  * @retval None
  */
void HAL_UART_send_data(uint8_t * pbuffer, uint16_t uhSize)
{
	/**
	 * Enable transmit flag
	 */
	sUartContext.uwTxFlag = ENABLE;

	sUartContext.uwWCTS_TxStarted = ENABLE;

	/**
	 * Disable WCTS interrupt
	 */
	wcts_interrupt_disable();

	sUartContext.uhTxSize = uhSize;

	/**
	 * Enable WCTS HW flow control
	 */
	ENABLE_HW_CTS(USART_BLE);

	/**
	 * Request High Speed clock
	 */
	HAL_UART_HSclkRequest(eUART_Tx, eUART_HSclkEnable);

	/**
	 *  Setup  DMA to be ready to send data
	 */
	sUartContext.pTxBuffer = pbuffer;
	DMA_Cmd(DMA_USART_TX_CHANNEL, DISABLE);
	DMA_USART_TX_CHANNEL->CMAR = (uint32_t)pbuffer;
	DMA_USART_TX_CHANNEL->CNDTR = sUartContext.uhTxSize-1;

	/**
	 * Check WCTS GPIO line
	 */
	if(READ_GPIO_INPUTBIT(USART_WCTS_GPIO_PORT, USART_WCTS_PIN) == Bit_SET)
	{
		/**
		 * Wakeup remote
		 */

		/**
		 * Enable receiving path
		 */
		set_wrts_gpio_mode(GPIO_Mode_AF);

		/**
		 * Wait for ack
		 */
		wcts_interrupt_enable(eAckToTxRequest);
	}
	else
	{
	    DMA_Cmd(DMA_USART_TX_CHANNEL, ENABLE);	/**< Enable DMA to start transmission */

	    /**
	     * Enable receiving path
	     */
		set_wrts_gpio_mode(GPIO_Mode_AF);

		/**
		 * remove PU
		 */
		SetGPIO_PUPD(USART_RX_GPIO_PORT, USART_RX_PIN_SOURCE, GPIO_PuPd_NOPULL);
		SetGPIO_PUPD(USART_WCTS_GPIO_PORT, USART_WCTS_PIN_SOURCE, GPIO_PuPd_NOPULL);
	}

    return;
}

/**
  * @brief  CTS interrupt handler
  *
  * @note	The Low Power Protocol handles the CTS line as either hardware flow control or low power control signal
  * 		The CTS interrupt handler manages the low power control signaling
  *
  * @param  None
  *
  * @retval None
  */
void HAL_UART_wcts_handler(void)
{
	sHAL_UART_PhyDriverEvent_t sPhyDriverEvent;
	uint8_t		*plocalTxbuffer;
	uint16_t	localTxSize;

	/**
	 * Clear pending bit and mask out interrupt
	 */
	wcts_interrupt_disable();

	switch(sUartContext.eWCTSEventWaited)
	{
		case eLast2ndSentByteAck:
			sUartContext.uwTxFlag = DISABLE;

			/**
			 * Stop flow
			 */
			SET_GPIO(USART_WRTS_GPIO_PORT, USART_WRTS_PIN);
			set_wrts_gpio_mode(GPIO_Mode_OUT);

			DISABLE_HW_CTS(USART_BLE);

			if(sUartContext.uwRxFlag == DISABLE)
			{
				SetGPIO_PUPD(USART_WCTS_GPIO_PORT, USART_WCTS_PIN_SOURCE, GPIO_PuPd_UP);
			}
			wcts_interrupt_enable(eLastSentByteAck);
			plocalTxbuffer = sUartContext.pTxBuffer;
			localTxSize =  sUartContext.uhTxSize;
			USART_BLE->DR = *(uint16_t*)(plocalTxbuffer + localTxSize - 1);

			break;

		case eRemoteTxrequest:
			sUartContext.uwRxFlag = ENABLE;

			HAL_UART_HSclkRequest(eUART_Rx, eUART_HSclkEnable);

			/**
			 * Enable receiving path
			 */
			set_wrts_gpio_mode(GPIO_Mode_AF);

			SetGPIO_PUPD(USART_WCTS_GPIO_PORT, USART_WCTS_PIN_SOURCE, GPIO_PuPd_NOPULL);
			SetGPIO_PUPD(USART_RX_GPIO_PORT, USART_RX_PIN_SOURCE, GPIO_PuPd_NOPULL);
			break;

		case eAckToTxRequest:
		    DMA_Cmd(DMA_USART_TX_CHANNEL, ENABLE);	/**< Enable DMA to start transmission */

			SetGPIO_PUPD(USART_WCTS_GPIO_PORT, USART_WCTS_PIN_SOURCE, GPIO_PuPd_NOPULL);
			SetGPIO_PUPD(USART_RX_GPIO_PORT, USART_RX_PIN_SOURCE, GPIO_PuPd_NOPULL);
			break;

		case eLastSentByteAck:
			sUartContext.uwWCTS_TxStarted = DISABLE;

			HAL_UART_HSclkRequest(eUART_Tx, eUART_HSclkDisable);

			wcts_interrupt_enable(eRemoteTxrequest);

			/**
			 * Notify end of transmit
			 */
			sPhyDriverEvent.ePhyDriverNotification = eUART_TxDone;
			HAL_UART_Msg_Handler(sUartContext.pf_PhyDriverEvent_Handler, sPhyDriverEvent);
			break;

		default:
			break;
	}

	return;
}

/**
  * @brief  Interface to the application to configure properly the UART driver before entering standby mode
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void HAL_UART_EnterStandByMode(void)
{
	uint32_t uwTempRegister;

	/**
	 * Disable falling edge detector on WCTS line
	 */
	EXTI->FTSR &= ~USART_WCTS_EXTI_LINE;

	/**
	 * Set PD on WCTS input
	 */
	uwTempRegister = USART_WCTS_GPIO_PORT->PUPDR;
	uwTempRegister &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)USART_WCTS_PIN_SOURCE * 2));
	uwTempRegister |= (((uint32_t)GPIO_PuPd_DOWN) << ((uint16_t)USART_WCTS_PIN_SOURCE * 2));
	USART_WCTS_GPIO_PORT->PUPDR = uwTempRegister;

	while(USART_WCTS_GPIO_PORT->IDR & USART_WCTS_PIN);

	PWR_WakeUpPinCmd(USART_WAKEUP_PIN_SELECTED, ENABLE);

	return;
}

/**
  * @brief  Interface to the application to configure properly the UART driver when moving out of standby mode
  *
  * @note	This API is called only when the application was expecting to enter standby mode but finally did not succeed
  *
  * @param  None
  *
  * @retval None
  */
void HAL_UART_ExitStandByMode(void)
{
	uint32_t uwTempRegister;

	/**
	 * Set PU on WCTS input
	 */
	uwTempRegister = USART_WCTS_GPIO_PORT->PUPDR;
	uwTempRegister &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)USART_WCTS_PIN_SOURCE * 2));
	uwTempRegister |= (((uint32_t)GPIO_PuPd_UP) << ((uint16_t)USART_WCTS_PIN_SOURCE * 2));
	USART_WCTS_GPIO_PORT->PUPDR = uwTempRegister;

	/**
	 * Enable falling edge detector on WCTS line
	 */
	EXTI->FTSR |= USART_WCTS_EXTI_LINE;

	return;
}

/**
  * @brief  Mapping of the CTS interrupt handler to the NVIC when using either PA0 or PD3
  *
  * @note	In all other cases, the mapping shall be done by the application as the IO may be shared
  * 		with others functionalities
  *
  * @param  None
  *
  * @retval None
  */
#if (USART_CTS_PIN_SELECTED == PA0)
void EXTI0_IRQHandler(void)
{
	HAL_UART_wcts_handler();

	return;
}
#elif (USART_CTS_PIN_SELECTED == PD3)
void EXTI3_IRQHandler(void)
{
	HAL_UART_wcts_handler();

	return;
}
#endif

/**
  * @brief  Mapping of the DMA RX interrupt handler to the NVIC
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
#if (USART_SELECTED == BLE_ON_USART1)
void DMA1_Channel5_IRQHandler(void)
#elif (USART_SELECTED == BLE_ON_USART2)
void DMA1_Channel6_IRQHandler(void)
#elif (USART_SELECTED == BLE_ON_USART3)
void DMA1_Channel3_IRQHandler(void)
#endif
{
	dma_rx_handler();

	return;
}

/**
  * @brief  Mapping of the UART interrupt handler to the NVIC
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
#if (USART_SELECTED == BLE_ON_USART1)
void USART1_IRQHandler(void)
#elif (USART_SELECTED == BLE_ON_USART2)
void USART2_IRQHandler(void)
#elif (USART_SELECTED == BLE_ON_USART3)
void USART3_IRQHandler(void)
#endif
{
	uart_handler();

	return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
