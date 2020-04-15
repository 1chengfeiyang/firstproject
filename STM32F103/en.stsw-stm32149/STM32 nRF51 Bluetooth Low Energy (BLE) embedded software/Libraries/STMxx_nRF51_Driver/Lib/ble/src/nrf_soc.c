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
#include <stdint.h>
#include <string.h>
#include "ble_encode_transport.h"
#include "nrf_soc_app.h"
#include "hal_transport_config.h"
#include "nrf_error_soc.h"
#include "app_error.h"
#include "ble_rpc_defines.h"

static void * mp_out_param;

/**@brief Command processing complete function for @ref sd_power_system_off BLE command.
 *
 * Callback for returning the command response return code.
 *
 * @param[in] p_buffer NULL as no command response for the command send.
 * @param[in] length   0 as no command response for the command send.  
 *
 * @retval NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN
 */
static uint32_t cmd_write_complete(const uint8_t * p_buffer, uint32_t length)
{
    APP_ERROR_CHECK_BOOL(p_buffer == NULL);    
    APP_ERROR_CHECK_BOOL(length == 0);        
    
    ble_encode_transport_tx_free();
    
    return NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN;
}


uint32_t sd_power_system_off(void)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.
    
    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = power_system_off_req_enc(&(p_buffer[1]), &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    // @note: Increment buffer length as internally managed packet type field must be included.        
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_NO_RESP, 
                                   cmd_write_complete);    
    
    return NRF_SUCCESS;
} 


/**@brief Command response callback function for @ref sd_temp_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */

static uint32_t mw_temp_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = temp_get_rsp_dec(p_buffer,
                              length,
                              &result_code,
                              (int32_t *)mp_out_param);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_temp_get(int32_t * p_temp)
{
    mp_out_param = p_temp;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = temp_get_req_enc(p_temp,
                                               &(p_buffer[1]),
                                               &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   mw_temp_get_rsp_dec);

    return NRF_SUCCESS;
}
