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

#include "ble.h" 
#include "ble_rpc_defines.h"
#include <stdint.h>
#include "ble_app.h" 
#include "ble_encode_transport.h"
#include "hal_transport_config.h"
#include "app_error.h"

/**@brief Structure containing @ref sd_ble_uuid_encode output parameters. */
typedef struct
{
    uint8_t * p_uuid_le_len;                                    /**< @ref sd_ble_uuid_encode appearance p_uuid_le_len output parameter. */    
    uint8_t * p_uuid_le;                                        /**< @ref sd_ble_uuid_encode appearance p_uuid_le output parameter. */        
} ble_uuid_encode_out_params_t;

/**@brief Structure containing @ref sd_ble_tx_buffer_count_get output parameters. */
typedef struct
{
    uint8_t * p_count;                                    /**< @ref sd_ble_tx_buffer_count_get p_count output parameter. */
} ble_tx_buffer_count__get_out_params_t;

/**@brief Union containing BLE command output parameters. */
typedef union
{
    ble_uuid_encode_out_params_t     ble_uuid_encode_out_params;    /**< @ref sd_ble_uuid_encode output parameters. */
    ble_tx_buffer_count__get_out_params_t ble_tx_buffer_count_get_out_params;    /**< @ref sd_ble_uuid_encode output parameters. */
} ble_command_output_params_t;
    
static ble_command_output_params_t m_output_params;             /**< BLE command output parameters. */

static void * mp_out_params[3];

/**@brief Command response callback function for @ref sd_ble_uuid_encode BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t uuid_encode_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = 
        ble_uuid_encode_rsp_dec(p_buffer, 
                                length, 
                                m_output_params.ble_uuid_encode_out_params.p_uuid_le_len,
                                m_output_params.ble_uuid_encode_out_params.p_uuid_le,
                                &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                    
    
    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_uuid_encode(ble_uuid_t const * const p_uuid, 
                            uint8_t * const          p_uuid_le_len, 
                            uint8_t * const          p_uuid_le)
{
    m_output_params.ble_uuid_encode_out_params.p_uuid_le_len = p_uuid_le_len;
    m_output_params.ble_uuid_encode_out_params.p_uuid_le     = p_uuid_le;    

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_uuid_encode_req_enc(p_uuid, 
                                                      p_uuid_le_len, 
                                                      p_uuid_le, 
                                                      &(p_buffer[1]),
                                                      &buffer_length); 
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   uuid_encode_rsp_dec);    

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_tx_buffer_count_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t tx_buffer_count_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_tx_buffer_count_get_rsp_dec(p_buffer,
                              length,
                              (uint8_t **)&mp_out_params[0],
                              &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_tx_buffer_count_get(uint8_t* p_count)
{
    mp_out_params[0] = p_count;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_tx_buffer_count_get_req_enc(p_count,
                                                              &(p_buffer[1]),
                                                              &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   tx_buffer_count_get_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_uuid_vs_add BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t uuid_vs_add_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_uuid_vs_add_rsp_dec(p_buffer,
                              length,
                              (uint8_t **)&mp_out_params[0],
                              &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_uuid_vs_add(ble_uuid128_t const * const p_vs_uuid, uint8_t * const p_uuid_type)
{
    mp_out_params[0] = p_uuid_type;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_uuid_vs_add_req_enc(p_vs_uuid,p_uuid_type,
                                                      &(p_buffer[1]),
                                                      &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   uuid_vs_add_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_uuid_decode BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t uuid_decode_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_uuid_decode_rsp_dec(p_buffer,
                              length,
                              (ble_uuid_t **)&mp_out_params[0],
                              &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_uuid_decode(uint8_t uuid_le_len, uint8_t const * const p_uuid_le, ble_uuid_t * const p_uuid)
{
    mp_out_params[0] = p_uuid;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_uuid_decode_req_enc(uuid_le_len, p_uuid_le, p_uuid,
                                                      &(p_buffer[1]),
                                                      &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   uuid_decode_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_version_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t version_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_version_get_rsp_dec(p_buffer,
                              length,
                              (ble_version_t *)mp_out_params[0],
                              &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_version_get(ble_version_t * p_version)
{
    mp_out_params[0] = p_version;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_version_get_req_enc(p_version,
                                                      &(p_buffer[1]),
                                                      &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   version_get_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_enable BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t enable_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_enable_rsp_dec(p_buffer,
                              length,
                              &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_enable(ble_enable_params_t * p_ble_enable_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_enable_req_enc(p_ble_enable_params,
                                                      &(p_buffer[1]),
                                                      &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   enable_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_opt_set BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t opt_set_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_opt_set_rsp_dec(p_buffer,
                              length,
                              &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_opt_set(uint32_t opt_id, ble_opt_t const *p_opt)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_opt_set_req_enc(opt_id, p_opt,
                                                      &(p_buffer[1]),
                                                      &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   opt_set_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_opt_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t opt_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;
    uint32_t opt_id;

    const uint32_t err_code = ble_opt_get_rsp_dec(p_buffer,
                              length,
                              &opt_id,
                              (ble_opt_t *)mp_out_params[0],
                              &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    (void)opt_id;

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_opt_get(uint32_t opt_id, ble_opt_t *p_opt)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    mp_out_params[0] = p_opt;
    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_opt_get_req_enc(opt_id, p_opt,
                                                      &(p_buffer[1]),
                                                      &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   opt_get_rsp_dec);

    return NRF_SUCCESS;
}
