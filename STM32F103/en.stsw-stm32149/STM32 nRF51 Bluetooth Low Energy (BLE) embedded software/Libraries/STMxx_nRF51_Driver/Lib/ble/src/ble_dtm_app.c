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
#include "ble_dtm_app.h"
#include <stdint.h>
#include "ble_serialization.h"
#include "hal_transport_config.h"
#include "ble_encode_transport.h"
#include "app_error.h"
#include "nrf_error.h"
#include "app_uart_stream.h"


/**@brief Command response callback function for @ref sd_ble_gap_adv_start BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t dtm_init_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;
    
    const uint32_t err_code = ble_dtm_init_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();
    
    return result_code;
}


uint32_t ble_dtm_init(app_uart_stream_comm_params_t * p_uart_comm_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_DTM_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_dtm_init_req_enc(p_uart_comm_params, 
                                                   &(p_buffer[1]), 
                                                   &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.    
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   dtm_init_rsp_dec);    

    return NRF_SUCCESS;
} 

