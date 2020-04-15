/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of other
 * contributors to this software may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * 4. This software must only be used in a processor manufactured by Nordic
 * Semiconductor ASA, or in a processor manufactured by a third party that
 * is used in combination with a processor manufactured by Nordic Semiconductor.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
// @todo: @file scope comment block missing.

#ifndef APP_UART_STREAM_H__
#define APP_UART_STREAM_H__

#include <stdbool.h>
#include <stdint.h>

/**@brief Enumeration of supported baud rates. */ 
typedef enum
{
    UART_BAUD_RATE_1200,     /**< Baud rate 1200. */
    UART_BAUD_RATE_2400,     /**< Baud rate 2400. */
    UART_BAUD_RATE_4800,     /**< Baud rate 4800. */
    UART_BAUD_RATE_9600,     /**< Baud rate 9600. */
    UART_BAUD_RATE_14400,    /**< Baud rate 14400. */
    UART_BAUD_RATE_19200,    /**< Baud rate 19200. */
    UART_BAUD_RATE_28800,    /**< Baud rate 28800. */
    UART_BAUD_RATE_38400,    /**< Baud rate 38400. */
    UART_BAUD_RATE_57600,    /**< Baud rate 57600. */
    UART_BAUD_RATE_76800,    /**< Baud rate 76800. */
    UART_BAUD_RATE_115200,   /**< Baud rate 115200. */
    UART_BAUD_RATE_230400,   /**< Baud rate 230400. */
    UART_BAUD_RATE_250000,   /**< Baud rate 250000. */
    UART_BAUD_RATE_460800,   /**< Baud rate 460800. */
    UART_BAUD_RATE_921600,   /**< Baud rate 921600. */
    UART_BAUD_RATE_1000000,  /**< Baud rate 1000000. */
    UART_BAUD_RATE_MAX       /**< Enumeration upper bound. */
} app_uart_stream_baud_rate_t;

/**@brief UART communication structure holding configuration settings for the peripheral.
 */
typedef struct
{
    uint8_t                     rx_pin_no;      /**< RX pin number. */
    uint8_t                     tx_pin_no;      /**< TX pin number. */
    uint8_t                     rts_pin_no;     /**< RTS pin number, only used if flow control is enabled. */
    uint8_t                     cts_pin_no;     /**< CTS pin number, only used if flow control is enabled. */
    bool                        use_parity;     /**< Even parity if TRUE, no parity if FALSE. */
    app_uart_stream_baud_rate_t baud_rate;      /**< Baud rate configuration. */
} app_uart_stream_comm_params_t;

/**@brief Event callback function events. */ 
typedef enum
{
    APP_UART_STREAM_TX_DONE,            /**< An event indicating that TX stream has been transmitted. */
    APP_UART_STREAM_RX_RDY,             /**< An event indicating that RX stream has been received. */        
    APP_UART_STREAM_RX_OVERFLOW,        /**< An event indicating that RX data has been discarded due to lack of free RX memory. Either lack of RX buffer or RX buffer overflow would occur */         
    APP_UART_STREAM_ERROR,              /**< An error in the app_uart_stream module has occured. */
    APP_UART_STREAM_EVT_TYPE_MAX        /**< Enumeration upper bound. */    
} app_uart_stream_evt_type_t;

/**@brief Struct containing events from the UART module.
 */
typedef struct
{
    app_uart_stream_evt_type_t event;       /**< Type of event. */
    
    uint32_t                  param1;       /**< Event specific parameter. @todo: this is begin of RX-buffer with APP_UART_STREAM_RX_RDY */
    uint32_t                  param2;       /**< Event specific parameter. @todo: this is length of data in RX-buffer with APP_UART_STREAM_RX_RDY */    
} app_uart_stream_evt_t;

typedef void (*app_uart_stream_event_handler_t)(app_uart_stream_evt_t event);

/**@brief Function for opening and initializing the UART module.
 *
 * @warning Must not be called if has been allready opened. 
 *
 * @param[in]     p_comm_params     Pin and communication parameters or NULL to use driver defined default values.
 * @param[in]     event_handler     Function to be called in case of an event.
 *
 * @retval      NRF_SUCCESS               Operation success. 
 * @retval      NRF_ERROR_INVALID_STATE   Operation failure. Module in invalid state.
 * @retval      NRF_ERROR_NO_MEM          Operation failure. No memory available.
 * @retval      NRF_ERROR_INVALID_PARAM   Operation failure. If the given parameters are not supported by the uart driver. 
 */
uint32_t app_uart_stream_open(const app_uart_stream_comm_params_t * p_comm_params);

/**@brief Function for registering a generic event handler.
 *
 * @note Multiple registration requests will overwrite any possible existing registration. 
 *
 * @param[in] event_handler         The function to be called by the XXX layer upon an event.
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.    
 */
uint32_t app_uart_stream_evt_handler_reg(app_uart_stream_event_handler_t event_handler);

/**@brief Function for closing the UART module. 
 *
 * @note Can be called multiple times and also for not opened module.
 * 
 * @retval      NRF_SUCCESS         Operation success.
 */
uint32_t app_uart_stream_close(void);
                              
/**@brief Write a bytestream to the UART.
 *
 * @param[in] p_buffer              Pointer to the buffer to transmit.
 * @param[in] length                Buffer length in bytes.
 *
 * @retval NRF_SUCCESS              Operation success. Stream was added to the transmission queue 
 *                                  and event will be send upon transmission completion. 
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. Module in invalid state.  
 * @retval NRF_ERROR_NO_MEM         Operation failure. Transmission queue is full and stream was not 
 *                                  added to the transmission queue. User should wait for a 
 *                                  appropriate event prior issuing this operation again.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.  
 */
uint32_t app_uart_stream_write(const uint8_t * p_buffer, uint16_t length);

/**@brief Set RX-stream buffer where the RX data is appended and enable the PHY flow and reception of data.
 *
 * @param[in] p_buffer      Pointer to the RX buffer where to receive.
 * @param[in] num_of_bytes  Number of bytes to append to the RX buffer prior sending the
 *                          @ref APP_UART_STREAM_RX_RDY event
 * @param[in] header        Boolean specifying if the data to be received is considered part of a
 *                          header frame. If set to false, the uart stream driver should enter low
 *                          power mode when complete packet has been received.
 * 
 * @note When @ref num_of_bytes has been received an event @ref APP_UART_STREAM_RX_READY will be send, 
 * PHY flow will be turned OFF and no more data is appended to the RX buffer.
 *
 * @note The actual length of the RX buffer is atleast the size of @ref num_of_bytes 
 *
 * @note Can be called multiple times, and also for not opened module, possible existing buffer is replaced with the new one. 
 *
 * @retval NRF_SUCCESS              Operation success. 
 * @retval NRF_ERROR_BUSY           Operation failure. RX buffer append in progress.  
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.   
 */
uint32_t app_uart_stream_rx_buffer_set(uint8_t * p_buffer, uint16_t num_of_bytes, bool header);

#endif // APP_UART_STREAM_H__
