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
#include "ble_gattc.h" 
#include "ble_gattc_app.h"
#include "ble_rpc_defines.h"
#include "ble_encode_transport.h"
#include "hal_transport_config.h"
#include "app_error.h"

/**@brief Command response callback function for @ref sd_ble_gattc_primary_services_discover BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_primary_services_discover_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_primary_services_discover_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_primary_services_discover(uint16_t conn_handle,
                                                uint16_t start_handle,
                                                ble_uuid_t const *const  p_srvc_uuid)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_primary_services_discover_req_enc(conn_handle,
                                                                          start_handle,
                                                                          p_srvc_uuid,
                                                                          &(p_buffer[1]),
                                                                          &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_primary_services_discover_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_relationships_discover BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_relationships_discover_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_relationships_discover_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_relationships_discover(uint16_t conn_handle,
                                             ble_gattc_handle_range_t const * const p_handle_range)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_relationships_discover_req_enc(conn_handle,
                                                                       p_handle_range,
                                                                       &(p_buffer[1]),
                                                                       &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_relationships_discover_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_characteristics_discover BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_characteristics_discover_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_characteristics_discover_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_characteristics_discover(uint16_t conn_handle,
                                             ble_gattc_handle_range_t const * const p_handle_range)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_characteristics_discover_req_enc(conn_handle,
                                                                         p_handle_range,
                                                                         &(p_buffer[1]),
                                                                         &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_characteristics_discover_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_descriptors_discover BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_descriptors_discover_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_descriptors_discover_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_descriptors_discover(uint16_t conn_handle,
                                           ble_gattc_handle_range_t const * const p_handle_range)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_descriptors_discover_req_enc(conn_handle,
                                                                         p_handle_range,
                                                                         &(p_buffer[1]),
                                                                         &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_descriptors_discover_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_char_value_by_uuid_read BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_char_value_by_uuid_read_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_char_value_by_uuid_read_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
   APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_char_value_by_uuid_read(uint16_t conn_handle,
                                              ble_uuid_t const *const  p_uuid, 
                                              ble_gattc_handle_range_t const * const p_handle_range)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_char_value_by_uuid_read_req_enc(conn_handle,
                                                                        p_uuid,
                                                                        p_handle_range,
                                                                        &(p_buffer[1]),
                                                                        &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_char_value_by_uuid_read_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_read BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_read_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_read_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
   APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_read(uint16_t conn_handle,
                           uint16_t handle,
                           uint16_t offset)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_read_req_enc(conn_handle,
                                                     handle,
                                                     offset,
                                                     &(p_buffer[1]),
                                                     &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_read_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_char_values_read BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_char_values_read_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_char_values_read_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
   APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_char_values_read(uint16_t conn_handle,
                                       uint16_t const * const p_handles,
                                       uint16_t handle_count)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_char_values_read_req_enc(conn_handle,
                                                                 p_handles,
                                                                 handle_count,
                                                                 &(p_buffer[1]),
                                                                 &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_char_values_read_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_write BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_write_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_write_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_write(uint16_t conn_handle,
                            ble_gattc_write_params_t const *const p_write_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_write_req_enc(conn_handle,
                                                      p_write_params,
                                                      &(p_buffer[1]),
                                                      &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_write_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gattc_hv_confirm BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gattc_hv_confirm_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gattc_hv_confirm_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}

uint32_t sd_ble_gattc_hv_confirm(uint16_t conn_handle,
                                 uint16_t handle)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]            = BLE_RPC_PKT_CMD;
    uint32_t buffer_length = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gattc_hv_confirm_req_enc(conn_handle,
                                                           handle,
                                                           &(p_buffer[1]),
                                                           &buffer_length);
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gattc_hv_confirm_rsp_dec);

    return NRF_SUCCESS;
}
