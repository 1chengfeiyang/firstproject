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

#include "ble_gatts.h" 
#include <stdint.h>
#include <stddef.h>
#include "ble_rpc_defines.h"
#include "ble_encode_transport.h"
#include "ble_gatts_app.h"
#include "hal_transport_config.h"
#include "app_error.h"

/**@brief Structure containing @ref sd_ble_gatts_service_add output parameters. */
typedef struct
{
    uint16_t * p_handle;                                            /**< @ref sd_ble_gatts_service_add p_handle output parameter. */    
} gatts_service_add_out_params_t;

/**@brief Structure containing @ref sd_ble_gatts_characteristic_add output parameters. */
typedef struct
{
    uint16_t  * p_handles;                          /**< @ref sd_ble_gatts_characteristic_add p_handles output parameter. */
} gatts_char_add_out_params_t;

/**@brief Structure containing @ref sd_ble_gatts_sys_attr_get output parameters. */
typedef struct
{
    uint8_t *  p_sys_attr_data;                                     /**< @ref sd_ble_gatts_sys_attr_get p_sys_attr_data output parameter. */    
    uint16_t * p_len;                                               /**< @ref sd_ble_gatts_sys_attr_get p_len output parameter. */    
} gatts_sys_attr_get_out_params_t;

/**@brief Structure containing @ref sd_ble_gatts_value_set output parameters. */
typedef struct
{
    uint16_t * p_len;                                               /**< @ref sd_ble_gatts_value_set p_len output parameter. */    
} gatts_value_set_out_params_t;
    
/**@brief Union containing BLE command output parameters. */
typedef union
{
    gatts_service_add_out_params_t  gatts_service_add_out_params;   /**< @ref sd_ble_gatts_service_add output parameters. */  
    gatts_char_add_out_params_t     gatts_char_add_out_params;      /**< @ref sd_ble_gatts_characteristic_add output parameters. */  
    gatts_sys_attr_get_out_params_t gatts_sys_attr_get_out_params;  /**< @ref sd_ble_gatts_sys_attr_get output parameters. */
    gatts_value_set_out_params_t    gatts_value_set_out_params;     /**< @ref sd_ble_gatts_value_set output parameters. */
    ble_gatts_hvx_params_t          gatts_hvx_out_params;
} gatts_command_output_params_t;

static gatts_command_output_params_t m_output_params;               /**< BLE command output parameters. */

//Pointer for sd calls output params
static void * mp_out_params[3];

/**@brief Command response callback function for @ref sd_ble_gatts_sys_attr_set BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_sys_attrset_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gatts_sys_attr_set_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gatts_sys_attr_set(uint16_t              conn_handle, 
                                   uint8_t const * const p_sys_attr_data, 
                                   uint16_t              len)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gatts_sys_attr_set_req_enc(conn_handle, 
                                                             p_sys_attr_data,
                                                             len,
                                                             &(p_buffer[1]), 
                                                             &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_sys_attrset_reply_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gatts_hvx BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_hvx_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gatts_hvx_rsp_dec(p_buffer, length, &result_code,
                                        (uint16_t * *)& m_output_params.gatts_hvx_out_params.p_len);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gatts_hvx(uint16_t conn_handle, ble_gatts_hvx_params_t const * const p_hvx_params)
{
    m_output_params.gatts_hvx_out_params.p_len = (p_hvx_params) ? p_hvx_params->p_len : NULL;
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gatts_hvx_req_enc(conn_handle, 
                                                    p_hvx_params, 
                                                    &(p_buffer[1]), 
                                                    &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_hvx_reply_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gatts_service_add BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_service_add_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = 
        ble_gatts_service_add_rsp_dec(p_buffer, 
                                      length, 
                                      m_output_params.gatts_service_add_out_params.p_handle,
                                      &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();
    
    return result_code;
}


uint32_t sd_ble_gatts_service_add(uint8_t                  type, 
                                  ble_uuid_t const * const p_uuid, 
                                  uint16_t * const         p_handle)
{
    m_output_params.gatts_service_add_out_params.p_handle = p_handle;
    
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gatts_service_add_req_enc(type, 
                                                            p_uuid,
                                                            p_handle,
                                                            &(p_buffer[1]), 
                                                            &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_service_add_reply_rsp_dec);    

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gatts_service_add BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_service_changed_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = NRF_SUCCESS;

    const uint32_t err_code = ble_gatts_service_changed_rsp_dec(p_buffer, 
                                                                length, 
                                                                &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();
    
    return result_code;
}

uint32_t sd_ble_gatts_service_changed(uint16_t  conn_handle, 
                                      uint16_t  start_handle, 
                                      uint16_t  end_handle)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gatts_service_changed_req_enc(conn_handle, 
                                                                start_handle,
                                                                end_handle,
                                                                &(p_buffer[1]), 
                                                                &buffer_length);

    APP_ERROR_CHECK(err_code);                

    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_service_changed_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gatts_include_add BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_include_add_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = NRF_SUCCESS;

    const uint32_t err_code = 
        ble_gatts_include_add_rsp_dec(p_buffer, 
                                      length, 
                                      (uint16_t *) &mp_out_params[0],
                                      &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_gatts_include_add(uint16_t          service_handle,
                                  uint16_t          inc_serv_handle,
                                  uint16_t * const  p_include_handle)
{
    mp_out_params[0] = p_include_handle;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gatts_include_add_req_enc(service_handle, 
                                                            inc_serv_handle,
                                                            p_include_handle,
                                                            &(p_buffer[1]), 
                                                            &buffer_length);

    APP_ERROR_CHECK(err_code);                

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_include_add_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gatts_characteristic_add BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_char_add_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gatts_characteristic_add_rsp_dec(
                                p_buffer, 
                                length, 
                                &m_output_params.gatts_char_add_out_params.p_handles,
                                &result_code);                                
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();
    
    return result_code;
}


uint32_t sd_ble_gatts_characteristic_add(uint16_t                          service_handle, 
                                         ble_gatts_char_md_t const * const p_char_md, 
                                         ble_gatts_attr_t const * const    p_attr_char_value, 
                                         ble_gatts_char_handles_t * const  p_handles)
{
    m_output_params.gatts_char_add_out_params.p_handles     = (uint16_t*)p_handles;
                                
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;                                                                
    const uint32_t err_code = ble_gatts_characteristic_add_req_enc(service_handle,
                                                                   p_char_md,
                                                                   p_attr_char_value,
                                                                   p_handles,
                                                                   &(p_buffer[1]), 
                                                                   &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_char_add_reply_rsp_dec);    

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gatts_descriptor_add BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_descriptor_add_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = NRF_SUCCESS;

    const uint32_t err_code = 
        ble_gatts_descriptor_add_rsp_dec(p_buffer, 
                                      length, 
                                      (uint16_t *) &mp_out_params[0],
                                      &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_gatts_descriptor_add(uint16_t                       char_handle,
                                     ble_gatts_attr_t const *const  p_attr,
                                     uint16_t * const               p_handle)
{
    mp_out_params[0] = p_handle;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gatts_descriptor_add_req_enc(char_handle, 
                                                               p_attr,
                                                               p_handle,
                                                               &(p_buffer[1]), 
                                                               &buffer_length);

    APP_ERROR_CHECK(err_code);                

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_descriptor_add_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gatts_rw_authorize_reply BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_rw_authorize_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = NRF_SUCCESS;

    const uint32_t err_code = ble_gatts_rw_authorize_reply_rsp_dec(p_buffer, 
                                                                   length,
                                                                   &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_gatts_rw_authorize_reply(uint16_t       conn_handle,
    ble_gatts_rw_authorize_reply_params_t const *const  p_rw_authorize_reply_params)
{

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gatts_rw_authorize_reply_req_enc(conn_handle, 
                                                                   p_rw_authorize_reply_params,
                                                                   &(p_buffer[1]), 
                                                                   &buffer_length);

    APP_ERROR_CHECK(err_code);                

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_rw_authorize_reply_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gatts_value_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_value_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    // Order of arguments is different from ble_gatts_value_get_req_enc()
    // p_data is argument number 3 while p_len is argument number 4
    const uint32_t err_code = ble_gatts_value_get_rsp_dec(p_buffer,
                                                          length,
                                                          (uint8_t **)&mp_out_params[1],
                                                          (uint16_t **)&mp_out_params[0],
                                                          &result_code);

    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;
}

uint32_t sd_ble_gatts_value_get(uint16_t         handle, 
                                uint16_t         offset, 
                                uint16_t * const p_len, 
                                uint8_t  * const p_data)
{
    mp_out_params[0] = p_len;
    mp_out_params[1] = p_data;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;

    const uint32_t err_code = ble_gatts_value_get_req_enc(handle,
                                                          offset,
                                                          p_len,
                                                          p_data,
                                                          &(p_buffer[1]), 
                                                          &buffer_length);

    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_value_get_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gatts_value_set BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_value_set_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gatts_value_set_rsp_dec(
                                p_buffer, 
                                length, 
                                m_output_params.gatts_value_set_out_params.p_len,
                                &result_code);                                
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gatts_value_set(uint16_t              handle, 
                                uint16_t              offset, 
                                uint16_t * const      p_len, 
                                uint8_t const * const p_value)
{
    m_output_params.gatts_value_set_out_params.p_len = p_len;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;                                                                
    const uint32_t err_code = ble_gatts_value_set_req_enc(handle,
                                                          offset,
                                                          p_len,
                                                          p_value,
                                                          &(p_buffer[1]), 
                                                          &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_value_set_reply_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gatts_sys_attr_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gatts_sysattr_get_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gatts_sys_attr_get_rsp_dec(
                                p_buffer, 
                                length, 
                                m_output_params.gatts_sys_attr_get_out_params.p_sys_attr_data,
                                m_output_params.gatts_sys_attr_get_out_params.p_len,
                                &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
                                    
    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gatts_sys_attr_get(uint16_t         conn_handle, 
                                   uint8_t * const  p_sys_attr_data, 
                                   uint16_t * const p_len)
{
    m_output_params.gatts_sys_attr_get_out_params.p_sys_attr_data = p_sys_attr_data;
    m_output_params.gatts_sys_attr_get_out_params.p_len           = p_len;    
    
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;                                                                
    const uint32_t err_code = ble_gatts_sys_attr_get_req_enc(conn_handle,
                                                             p_sys_attr_data,
                                                             p_len,
                                                             &(p_buffer[1]), 
                                                             &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gatts_sysattr_get_reply_rsp_dec);    

    return NRF_SUCCESS;
}
