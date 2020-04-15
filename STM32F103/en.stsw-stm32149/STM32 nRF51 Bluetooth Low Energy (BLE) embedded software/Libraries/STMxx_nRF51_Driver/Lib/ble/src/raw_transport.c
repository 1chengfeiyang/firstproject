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

#include "hal_transport.h"
#include <stdbool.h> 
#include <string.h>
#include "hal_transport_config.h"
#include "app_uart_stream.h"
#include "app_error.h"
#include "app_util.h"

typedef enum 
{
    TRANSPORT_RECEIVE_PKT_HEADER,
    TRANSPORT_RECEIVE_PKT_DATA
} transport_receive_pkt_state_t; 

static bool     m_tx_buffer_in_use = false;                   /**< Indicates if transmission buffer is in use. */
static bool     m_rx_buffer_in_use = false;                   /**< Indicates if reception buffer is in use. */
static uint8_t  m_tx_buffer[HCI_TRANSPORT_PKT_BUFFER_SIZE];   /**< Transmission buffer. */
static uint8_t  m_rx_buffer[HCI_TRANSPORT_PKT_BUFFER_SIZE];   /**< Reception buffer. */

static transport_receive_pkt_state_t   m_receive_state;       /**< Receive state. */
static hci_transport_event_handler_t   m_event_handler   = NULL;       /**< Transport event handler callback function. */
static hci_transport_tx_done_handler_t m_tx_done_handler = NULL;     /**< Transport tx done handler callback function. */

static void uart_stream_event_handle(app_uart_stream_evt_t uart_stream_event)
{
    if (uart_stream_event.event == APP_UART_STREAM_RX_RDY)
    {
        if (m_receive_state == TRANSPORT_RECEIVE_PKT_HEADER)
        {
            uint8_t  * data            = (uint8_t *)uart_stream_event.param1;
            
            // @todo: Evaluate to use data_len to trigger state changes instead of depending
            //        on rx_pkt_consume function to reset state and setting buffer.
            // uint16_t   data_len        = (uint16_t)uart_stream_event.param2;

            // Expect next packet to be data.
            m_receive_state = TRANSPORT_RECEIVE_PKT_DATA;
            
            // Decode packet length data
            uint16_t decoded_data_len = uint16_decode(data);

            if (decoded_data_len > (HCI_TRANSPORT_PKT_DATA_SIZE))
            {
                APP_ERROR_CHECK_BOOL(false);
            }

            // Copy the header to the first bytes of the rx buffer.
            memcpy(&m_rx_buffer[0], data, HCI_TRANSPORT_PKT_HEADER_SIZE);

            // Set uart stream module to receive data packet.
            // 4 bytes allocated for packet size info.
            uint32_t err_code = app_uart_stream_rx_buffer_set(
                                            &m_rx_buffer[HCI_TRANSPORT_PKT_HEADER_SIZE], 
                                            decoded_data_len, false);
            
            (void)err_code;

            if (m_event_handler != NULL)
            {
                // Generate a transport event indicating that we are finished
                // receiving a packet.
                hci_transport_evt_t event;
                event.evt_type = HCI_TRANSPORT_RX_STARTED;

                // Call application event callback function.
                m_event_handler(event);
            }
    
        }
        else if (m_receive_state == TRANSPORT_RECEIVE_PKT_DATA)
        {

            // Mark rx buffer as in use.  
            // Only marked as in use when a whole packet is received, header+data. This is
            // done to prevent consume to be able to free something that is not fully 
            // received.          
            m_rx_buffer_in_use = true; 
            
            if (m_event_handler != NULL)
            {
                // Generate a transport event indicating that we are finished 
                // receiving a packet.
                hci_transport_evt_t event;
                event.evt_type = HCI_TRANSPORT_RX_RDY;
            
                // Call application event callback function.
                m_event_handler(event);
            }
        }
    }
    else if (uart_stream_event.event == APP_UART_STREAM_TX_DONE)
    {
        // Call application tx done event callback function.
        if (m_tx_done_handler != NULL)
        {
            // Generate a transport event indicating that we are finished 
            // transmitting a packet.
            hci_transport_tx_done_result_t event = HCI_TRANSPORT_TX_DONE_SUCCESS;

            m_tx_done_handler(event);
        }
    }   
}


uint32_t hci_transport_evt_handler_reg(hci_transport_event_handler_t event_handler)
{
    // Check that handler is valid.
    if (event_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }    

    m_event_handler = event_handler;

    return NRF_SUCCESS;
}


uint32_t hci_transport_tx_done_register(hci_transport_tx_done_handler_t event_handler)
{
    // Check that handler is valid.
    if (event_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    m_tx_done_handler = event_handler;
    
    return NRF_SUCCESS;
}


uint32_t hci_transport_open(void)
{
    
    // Register callback function for uart_stream events.
    uint32_t err_code = app_uart_stream_evt_handler_reg(uart_stream_event_handle);
    if (err_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }
    
    // Initilize uart_stream to start receving header.
    m_receive_state = TRANSPORT_RECEIVE_PKT_HEADER;
    
    err_code = app_uart_stream_rx_buffer_set(&m_rx_buffer[0], HCI_TRANSPORT_PKT_HEADER_SIZE, true);
    if (err_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }
    
    return NRF_SUCCESS;
}


uint32_t hci_transport_close(void)
{
    // Reset callback handlers.
    m_event_handler   = NULL;
    m_tx_done_handler = NULL;

    return NRF_SUCCESS;
}    


uint32_t hci_transport_tx_alloc(uint8_t ** pp_memory)
{
    if (pp_memory == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    if (m_tx_buffer_in_use)
    {
        return NRF_ERROR_NO_MEM;
    }
    


    // Return address to first byte after the transport pkt header placeholder.
    *pp_memory         = m_tx_buffer + HCI_TRANSPORT_PKT_HEADER_SIZE;
    m_tx_buffer_in_use = true;

    return NRF_SUCCESS;
}


uint32_t hci_transport_tx_free(void)
{
    m_tx_buffer_in_use = false;
    
    return NRF_SUCCESS;
}


uint32_t hci_transport_pkt_write(const uint8_t * p_buffer, uint16_t length)
{
    if (p_buffer == NULL)
    {
        return NRF_ERROR_NULL; 
    }
    
    if (!m_tx_buffer_in_use)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > (HCI_TRANSPORT_PKT_BUFFER_SIZE - HCI_TRANSPORT_PKT_HEADER_SIZE))
    {
        // length exceeds available memory provided by the transport module.
        return NRF_ERROR_DATA_SIZE;
    }

    // Calculate base pointer of the provided buffer.
    uint8_t * p_base = (uint8_t *)(p_buffer - HCI_TRANSPORT_PKT_HEADER_SIZE);
    
    // Verify that the memory address is the allocated in m_tx_buffer.
    if (p_base != m_tx_buffer)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    
    // Add length info to the packet in front of the data.
    (void)uint16_encode(length, p_base);    

    return app_uart_stream_write(p_base, length + HCI_TRANSPORT_PKT_HEADER_SIZE);        
}


uint32_t hci_transport_rx_pkt_extract(uint8_t ** pp_buffer, uint16_t * p_length)
{
    if (pp_buffer == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_length == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (!m_rx_buffer_in_use)
    {
        return NRF_ERROR_NO_MEM;   
    }

    // Extract packet length from rx buffer.
    *p_length = uint16_decode(&m_rx_buffer[0]);
    *pp_buffer = &m_rx_buffer[HCI_TRANSPORT_PKT_HEADER_SIZE];
    
    return NRF_SUCCESS;
}


uint32_t hci_transport_rx_pkt_consume(uint8_t * p_buffer)
{
    // If there is nothing to consume
    if (!m_rx_buffer_in_use)
    {
        return NRF_ERROR_NO_MEM;
    }

    // Calculate base pointer of the provided buffer.
    uint8_t * p_base = (uint8_t *)(p_buffer - HCI_TRANSPORT_PKT_HEADER_SIZE);
    
    // Verify that the memory address is the allocated in m_tx_buffer.
    if (p_base != m_rx_buffer)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    // Release rx buffer for use.
    m_rx_buffer_in_use = false;

    // Expect next packet to be header.
    m_receive_state = TRANSPORT_RECEIVE_PKT_HEADER;

    uint32_t err_code = app_uart_stream_rx_buffer_set(p_base, HCI_TRANSPORT_PKT_HEADER_SIZE, true);
    
    (void)err_code;
    
    return NRF_SUCCESS;
}

