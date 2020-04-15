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

#include "ble_encode_access.h"
#include "ble_encode_transport.h"
#include <stdbool.h>
#include <stdio.h>
#include "ble.h"
#include "ble_rpc_defines.h"
#include "ble_app.h"
#include "hal_transport.h"
#include "nrf_error.h"
#include "app_error.h"
#include "compiler_abstraction.h"

static ble_command_resp_decode_callback_t m_cmd_resp_decode_callback;   /**< BLE command response decode callback function. */
static ble_encode_cmd_resp_handler_t      m_ble_cmd_resp_handler;       /**< BLE command response application callback function. */
static ble_encode_event_handler_t         m_ble_event_handler;          /**< BLE event application callback function. */  
static const uint8_t *                    mp_evt_buffer;                /**< Pointer to begin of received BLE event buffer. */  
static uint16_t                           m_evt_length;                 /**< Length of data in bytes in BLE event buffer. */  
static ble_encode_cmd_write_mode_t        m_cmd_write_mode;             /**< @ref ble_encode_transport_cmd_write API command mode. */                                      
static volatile uint32_t                  m_event_flags;                /**< Variable for storing boolean event flags. */

#define FLAG_CMD_RESP_HANDLER_REGISTERED (1u << 0)                      /**< Flag for determining is command response handler registered or not. */
#define FLAG_EVT_AVAILABLE_FOR_POP       (1u << 1u)                     /**< Flag for determining is event available for application extraction. */
#define FLAG_CMD_WRITE_INPROGRESS        (1u << 2u)                     /**< Flag for determining is command write in progress. */


/**@brief Function for processing BLE command response.
 * 
 * @param[in] p_buffer  Pointer to the begin of command response after packet type field.
 * @param[in] length    Length of data in bytes. 
 */
static __INLINE void ble_command_response_process(const uint8_t * p_buffer, uint32_t length)
{    
    // @note: System design does not allow any valid use case for the callback to be NULL  
    // which will imply design error that must be fixed at compile time.    
    APP_ERROR_CHECK_BOOL(m_cmd_resp_decode_callback != NULL);      
    const uint32_t result_code = m_cmd_resp_decode_callback(p_buffer, length);
    
    // @note: Relevant flags must be cleared before notifying the application of command response 
    // reception due to the fact that application can call back within the same context.
    m_event_flags &= ~(FLAG_CMD_WRITE_INPROGRESS | FLAG_CMD_RESP_HANDLER_REGISTERED);

    // @note: System design does not allow any valid use case for the callback to be NULL  
    // which will imply design error that must be fixed at compile time.    
    APP_ERROR_CHECK_BOOL(m_ble_cmd_resp_handler != NULL);          
    m_ble_cmd_resp_handler(result_code);    
}


/**@brief Function for processing BLE event.
 * 
 * @param[in] p_buffer  Pointer to the begin of event after packet type field.
 * @param[in] length    Length of data in bytes. 
 */
static __INLINE void ble_event_process(const uint8_t * p_buffer, uint32_t length)
{
    // @note: System design does not allow any valid use case for the callback to be NULL  
    // which will imply design error that must be fixed at compile time.    
    APP_ERROR_CHECK_BOOL(m_ble_event_handler != NULL);      
    
    mp_evt_buffer = p_buffer;
    m_evt_length  = length;
    
    m_event_flags |= FLAG_EVT_AVAILABLE_FOR_POP;    
    m_ble_event_handler(BLE_ENCODE_EVT_RDY);
}


/**@brief Function for processing RX packet ready event from transport layer.
 */
static __INLINE void rx_packet_ready_event_process(void)
{    
    uint8_t * p_buffer;
    uint16_t  length;
    
    // @note: This implementation is based on system design where max 1 RX buffer is available.
    // On any other design this system will fail as multiple received events can override existing 
    // received event if application does not pop the event out within the current context. 
    if (!(m_event_flags & FLAG_EVT_AVAILABLE_FOR_POP))
    {
        uint32_t err_code = hci_transport_rx_pkt_extract(&p_buffer, &length);
        // @note: System design does not allow any valid use case for the RX packet extract failure 
        // which will imply design error that must be fixed at compile time.
        APP_ERROR_CHECK(err_code);  
        
        const uint8_t packet_type = p_buffer[0];    // @todo: use #define for 0
        
        switch (packet_type)
        {
            case BLE_RPC_PKT_RESP:
            case BLE_RPC_PKT_DTM_RESP:
                // Adjust buffer begin pointer and length values after bypassing the packet type 
                // field. 
                ble_command_response_process(&(p_buffer[1]), --length);  // @todo: use #define for 1
                
                // @todo: consider moving consume prior application completion event.
                err_code = hci_transport_rx_pkt_consume(p_buffer);
                // @note: System design does not allow any valid use case for the RX packet consume 
                // failure which will imply design error that must be fixed at compile time.
                APP_ERROR_CHECK(err_code);                      
                break;
                
            case BLE_RPC_PKT_EVT:
                // Adjust buffer begin pointer and length values after bypassing the packet type 
                // field.             
                ble_event_process(&(p_buffer[1]), --length); // @todo: use #define for 1
                break;
                
            default:
                // @note: Should never happen.
                APP_ERROR_HANDLER(packet_type);
                break;
        }    
    }
    else
    {
        // @note: Should never happen.
        APP_ERROR_HANDLER(m_event_flags);    
    }    
}


/**@brief Function for processing events from transport layer.
 * 
 * @param[in] event     Transport layer event to process.
 */
static void transport_event_process(hci_transport_evt_t event)
{     
    switch (event.evt_type)
    {
        case HCI_TRANSPORT_RX_RDY:
            rx_packet_ready_event_process();
            break;
        case HCI_TRANSPORT_RX_STARTED:
            break;
        default:
            // @note: Should never happen.
            APP_ERROR_HANDLER(event.evt_type);
            break;
    }    
}


/**@brief Function for processing TX done event from transport layer.
 *
 * @param[in] result    TX done event result code. 
 */
static void transport_tx_done_process(hci_transport_tx_done_result_t result)
{
    APP_ERROR_CHECK_BOOL(result == HCI_TRANSPORT_TX_DONE_SUCCESS);
    
    // Actions are only executed if no command response is required for the transmitted command.
    if (m_cmd_write_mode == BLE_ENCODE_WRITE_MODE_NO_RESP)
    {    
        // @note: System design does not allow any valid use case for the callback to be NULL  
        // which will imply design error that must be fixed at compile time.    
        APP_ERROR_CHECK_BOOL(m_cmd_resp_decode_callback != NULL);      
        const uint32_t result_code = m_cmd_resp_decode_callback(NULL, 0);
        
        // @note: Relevant flags must be cleared before notifying the application of command 
        // processing completion due to the fact that application can call back within the same 
        // context.
        m_event_flags &= ~(FLAG_CMD_WRITE_INPROGRESS | FLAG_CMD_RESP_HANDLER_REGISTERED);

        // @note: System design does not allow any valid use case for the callback to be NULL  
        // which will imply design error that must be fixed at compile time.    
        APP_ERROR_CHECK_BOOL(m_ble_cmd_resp_handler != NULL);          
        m_ble_cmd_resp_handler(result_code);    
    }
}


uint32_t ble_encode_open(void)
{   
    uint32_t err_code = hci_transport_evt_handler_reg(transport_event_process);    
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    err_code = hci_transport_tx_done_register(transport_tx_done_process);    
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                    

    return hci_transport_open();
}


uint32_t ble_encode_close(void)
{
    m_event_flags              = 0;
    m_evt_length               = 0;  
    m_ble_cmd_resp_handler     = NULL;
    m_cmd_resp_decode_callback = NULL;
    m_ble_event_handler        = NULL;
    mp_evt_buffer              = NULL;
    m_cmd_write_mode           = BLE_ENCODE_WRITE_MODE_MAX;    
    
    return hci_transport_close();
}


uint32_t ble_encode_cmd_resp_handler_reg(ble_encode_cmd_resp_handler_t ble_command_resp_handler)
{
    if (ble_command_resp_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }
        
    uint32_t err_code;
    
    if (!(m_event_flags & FLAG_CMD_RESP_HANDLER_REGISTERED))
    {
        m_event_flags         |= FLAG_CMD_RESP_HANDLER_REGISTERED;
        m_ble_cmd_resp_handler = ble_command_resp_handler;
        err_code               = NRF_SUCCESS;    
    }
    else
    {
        err_code = NRF_ERROR_BUSY;
    }
    
    return err_code;
}


uint32_t ble_encode_evt_handler_register(ble_encode_event_handler_t ble_event_handler)
{
    if (ble_event_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    m_ble_event_handler = ble_event_handler;
    
    return NRF_SUCCESS;
}


uint8_t * ble_encode_transport_tx_alloc(void)
{
    uint8_t * p_buffer;
    uint32_t err_code;

    // This should be called only from main application context and not from interrupt, otherwise it
    // will block the system.
    do
    {
        err_code = hci_transport_tx_alloc(&p_buffer);

        //__WFE();
    }
    while (err_code == NRF_ERROR_NO_MEM);

    
    return p_buffer;
}


void ble_encode_transport_tx_free(void)
{
    const uint32_t err_code = hci_transport_tx_free();
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);        
}


void ble_encode_transport_cmd_write(const uint8_t *                    p_buffer, 
                                    uint32_t                           length, 
                                    ble_encode_cmd_write_mode_t        cmd_write_mode,                                    
                                    ble_command_resp_decode_callback_t cmd_resp_decode_callback)
{    
    APP_ERROR_CHECK_BOOL(!(m_event_flags & FLAG_CMD_WRITE_INPROGRESS));    
    APP_ERROR_CHECK_BOOL(p_buffer != NULL);
    APP_ERROR_CHECK_BOOL(length != 0);    
    APP_ERROR_CHECK_BOOL((cmd_write_mode == BLE_ENCODE_WRITE_MODE_RESP) || 
                         (cmd_write_mode == BLE_ENCODE_WRITE_MODE_NO_RESP));        
    APP_ERROR_CHECK_BOOL(cmd_resp_decode_callback != NULL);        
    
    m_event_flags             |= FLAG_CMD_WRITE_INPROGRESS;
    m_cmd_write_mode           = cmd_write_mode;
    m_cmd_resp_decode_callback = cmd_resp_decode_callback;
    
    const uint32_t err_code = hci_transport_pkt_write(p_buffer, length);
    // @note: Should never fail as system design allows only 1 command to be in progress at any 
    // time.
    APP_ERROR_CHECK(err_code);    
}                                    


uint32_t ble_encode_event_pop(ble_evt_t * p_event, uint32_t * p_event_len)
{
    uint32_t err_code;
    
    if (p_event_len == NULL)
    {
        return NRF_ERROR_NULL;
    }    

    if (m_event_flags & FLAG_EVT_AVAILABLE_FOR_POP)
    {
        m_event_flags &= ~FLAG_EVT_AVAILABLE_FOR_POP;    
        
        err_code = ble_event_dec(mp_evt_buffer, m_evt_length, p_event, p_event_len);
        // @note: Should never happen. 
        APP_ERROR_CHECK_BOOL((err_code == NRF_SUCCESS) || (err_code == NRF_ERROR_DATA_SIZE));
        
        if ((err_code == NRF_SUCCESS) && (p_event != NULL))
        {
            // @note: (p_event != NULL) check needs to be included in order to cover the p_event 
            // length query use case.
        
            // @note: Decrement buffer pointer to original received from 
            // @ref hci_transport_rx_pkt_extract.
            --mp_evt_buffer;
            err_code = hci_transport_rx_pkt_consume((uint8_t *)mp_evt_buffer);
            // @note: System design does not allow any valid use case for the RX packet consume failure 
            // which will imply design error that must be fixed at compile time.
            APP_ERROR_CHECK(err_code);                      
        }
    }
    else
    {
        err_code = NRF_ERROR_NO_MEM;
    }
            
    return err_code;
}
