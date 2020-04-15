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

#include "ble_gap.h" 
#include <stdint.h>
#include "ble_rpc_defines.h"
#include "ble_encode_transport.h"
#include "ble_gap_app.h"
#include "hal_transport_config.h"
#include "app_error.h"

/**@brief Structure containing @ref sd_ble_gap_device_name_get output parameters. */
typedef struct
{
    uint8_t *  p_dev_name;                                              /**< @ref sd_ble_gap_device_name_get p_dev_name output parameter. */    
    uint16_t * p_len;                                                   /**< @ref sd_ble_gap_device_name_get p_len output parameter. */    
} gap_device_name_get_output_params_t;

/**@brief Structure containing @ref sd_ble_gap_appearance_get output parameters. */
typedef struct
{
    uint16_t * p_appearance;                                            /**< @ref sd_ble_gap_appearance_get p_appearance output parameter. */    
} gap_appearance_get_output_params_t;

/**@brief Structure containing @ref sd_ble_gap_ppcp_get output parameters. */
typedef struct
{
    ble_gap_conn_params_t * p_conn_params;                              /**< @ref sd_ble_gap_ppcp_get p_conn_params output parameter. */    
} gap_ppcp_get_out_params_t;

/**@brief Union containing BLE command output parameters. */
typedef union
{
    gap_device_name_get_output_params_t gap_device_name_get_out_params; /**< @ref sd_ble_gap_device_name_get output parameters. */
    gap_appearance_get_output_params_t  gap_appearance_get_out_params;  /**< @ref sd_ble_gap_appearance_get output parameters. */
    gap_ppcp_get_out_params_t           gap_ppcp_get_out_params;        /**< @ref sd_ble_gap_ppcp_get output parameters. */    
} gap_command_output_params_t;

static gap_command_output_params_t m_output_params;                     /**< BLE command output parameters. */

static void * mp_out_params[1];

/**@brief Command response callback function for @ref sd_ble_gap_adv_start BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_adv_start_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;
    
    const uint32_t err_code = ble_gap_adv_start_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();
    
    return result_code;
}


uint32_t sd_ble_gap_adv_start(ble_gap_adv_params_t const * const p_adv_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_adv_start_req_enc(p_adv_params, 
                                                        &(p_buffer[1]), 
                                                        &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.    
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_adv_start_rsp_dec);    

    return NRF_SUCCESS;
} 


/**@brief Command response callback function for @ref ble_gap_device_name_get_req_enc BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_device_name_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = 
        ble_gap_device_name_get_rsp_dec(p_buffer, 
                                        length, 
                                        m_output_params.gap_device_name_get_out_params.p_dev_name,
                                        m_output_params.gap_device_name_get_out_params.p_len,
                                        &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();
    
    return result_code;
}


uint32_t sd_ble_gap_device_name_get(uint8_t * const p_dev_name, uint16_t * const p_len)
{
    m_output_params.gap_device_name_get_out_params.p_dev_name = p_dev_name;
    m_output_params.gap_device_name_get_out_params.p_len      = p_len;    

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_device_name_get_req_enc(p_dev_name, 
                                                              p_len, 
                                                              &(p_buffer[1]), 
                                                              &buffer_length); 
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.        
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_device_name_get_rsp_dec);    
    
    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_appearance_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_appearance_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = 
        ble_gap_appearance_get_rsp_dec(p_buffer, 
                                       length, 
                                       m_output_params.gap_appearance_get_out_params.p_appearance,                                       
                                       &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_appearance_get(uint16_t * const p_appearance)
{
    m_output_params.gap_appearance_get_out_params.p_appearance = p_appearance;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_appearance_get_req_enc(p_appearance, 
                                                             &(p_buffer[1]), 
                                                             &buffer_length); 
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.            
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_appearance_get_rsp_dec);    
    
    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_device_name_set BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_device_name_set_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gap_device_name_set_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_device_name_set(ble_gap_conn_sec_mode_t const * const p_write_perm, 
                                    uint8_t const * const                 p_dev_name, 
                                    uint16_t                              len)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_device_name_set_req_enc(p_write_perm, 
                                                              p_dev_name, 
                                                              len, 
                                                              &(p_buffer[1]), 
                                                              &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.        
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_device_name_set_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_appearance_set BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_appearance_set_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gap_appearance_set_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_appearance_set(uint16_t appearance)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_appearance_set_req_enc(appearance, 
                                                             &(p_buffer[1]), 
                                                             &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.            
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_appearance_set_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_ppcp_set BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_ppcp_set_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gap_ppcp_set_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_ppcp_set(ble_gap_conn_params_t const * const p_conn_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_ppcp_set_req_enc(p_conn_params, 
                                                       &(p_buffer[1]), 
                                                       &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.            
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_ppcp_set_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_adv_data_set BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_adv_data_set_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gap_adv_data_set_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_adv_data_set(uint8_t const * const p_data, 
                                 uint8_t               dlen, 
                                 uint8_t const * const p_sr_data, 
                                 uint8_t               srdlen)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_adv_data_set_req_enc(p_data, 
                                                           dlen, 
                                                           p_sr_data, 
                                                           srdlen, 
                                                           &(p_buffer[1]), 
                                                           &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_adv_data_set_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_conn_param_update BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_conn_param_update_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gap_conn_param_update_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_conn_param_update(uint16_t conn_handle, 
                                      ble_gap_conn_params_t const * const p_conn_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_conn_param_update_req_enc(conn_handle, 
                                                                p_conn_params, 
                                                                &(p_buffer[1]), 
                                                                &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_conn_param_update_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_disconnect BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_disconnect_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gap_disconnect_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_disconnect(uint16_t conn_handle, uint8_t hci_status_code)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_disconnect_req_enc(conn_handle, 
                                                         hci_status_code, 
                                                         &(p_buffer[1]), 
                                                         &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_disconnect_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_sec_info_reply BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_sec_info_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code;

    const uint32_t err_code = ble_gap_sec_info_reply_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK_BOOL(err_code != NRF_ERROR_INVALID_DATA);                                                    
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_sec_info_reply(uint16_t                          conn_handle, 
                                   ble_gap_enc_info_t const * const  p_enc_info, 
                                   ble_gap_sign_info_t const * const p_sign_info)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_sec_info_reply_req_enc(conn_handle, 
                                                             p_enc_info, 
                                                             p_sign_info,                                                          
                                                             &(p_buffer[1]), 
                                                             &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_sec_info_reply_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_sec_params_reply BLE command.
 *
 * Callback for decoding the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_sec_params_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_sec_params_reply_rsp_dec(p_buffer, length, &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_sec_params_reply(uint16_t                           conn_handle, 
                                     uint8_t                            sec_status, 
                                     ble_gap_sec_params_t const * const p_sec_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_sec_params_reply_req_enc(conn_handle, 
                                                               sec_status, 
                                                               p_sec_params,                                                          
                                                               &(p_buffer[1]), 
                                                               &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_sec_params_reply_rsp_dec);    

    return NRF_SUCCESS;
}


/**@brief Command response callback function for @ref sd_ble_gap_ppcp_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_ppcp_get_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_ppcp_get_rsp_dec(p_buffer, 
                                 length, 
                                 m_output_params.gap_ppcp_get_out_params.p_conn_params, 
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);
    
    ble_encode_transport_tx_free();

    return result_code;    
}


uint32_t sd_ble_gap_ppcp_get(ble_gap_conn_params_t * const p_conn_params)
{
    m_output_params.gap_ppcp_get_out_params.p_conn_params = p_conn_params;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();
    
    // @note: No NULL check required as error checked by called module as defined in API 
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;    
    const uint32_t err_code = ble_gap_ppcp_get_req_enc(p_conn_params, 
                                                       &(p_buffer[1]), 
                                                       &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);                
    
    // @note: Increment buffer length as internally managed packet type field must be included.                
    ble_encode_transport_cmd_write(p_buffer, 
                                   (++buffer_length), 
                                   BLE_ENCODE_WRITE_MODE_RESP, 
                                   gap_ppcp_get_reply_rsp_dec);    

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_address_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_address_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_address_get_rsp_dec(p_buffer,
                                 length,
                                 (ble_gap_addr_t *)mp_out_params[0],
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_address_get(ble_gap_addr_t * const p_addr)
{
    mp_out_params[0] = p_addr;

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_address_get_req_enc(p_addr,
                                                       &(p_buffer[1]),
                                                       &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_address_get_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_address_set BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_address_set_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_address_set_rsp_dec(p_buffer,
                                 length,
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_address_set(uint8_t addr_cycle_mode, ble_gap_addr_t const * const p_addr)
{

    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_address_set_req_enc(addr_cycle_mode,
                                                          p_addr,
                                                          &(p_buffer[1]),
                                                          &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_address_set_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_adv_stop BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_adv_stop_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_adv_stop_rsp_dec(p_buffer,
                                 length,
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_adv_stop(void)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_adv_stop_req_enc(&(p_buffer[1]),
                                                       &buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_adv_stop_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_auth_key_reply BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_auth_key_reply_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_auth_key_reply_rsp_dec(p_buffer,
                                 length,
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_auth_key_reply(uint16_t conn_handle, uint8_t key_type, uint8_t const * const key)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_auth_key_reply_req_enc(conn_handle, key_type, key,
                                                             &(p_buffer[1]),&buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_auth_key_reply_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_authenticate BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_authenticate_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_authenticate_rsp_dec(p_buffer,
                                 length,
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_authenticate(uint16_t conn_handle, ble_gap_sec_params_t const * const p_sec_params)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_authenticate_req_enc(conn_handle, p_sec_params,
                                                             &(p_buffer[1]),&buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_authenticate_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_conn_sec_get BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_conn_sec_get_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_conn_sec_get_rsp_dec(p_buffer,
                                 length,
                                 (ble_gap_conn_sec_t **)&mp_out_params[0],
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_conn_sec_get(uint16_t conn_handle, ble_gap_conn_sec_t * const p_conn_sec)
{
    mp_out_params[0] = p_conn_sec;
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_conn_sec_get_req_enc(conn_handle, p_conn_sec,
                                                             &(p_buffer[1]),&buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_conn_sec_get_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_rssi_start BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_rssi_start_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_rssi_start_rsp_dec(p_buffer,
                                 length,
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_rssi_start(uint16_t conn_handle)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_rssi_start_req_enc(conn_handle,
                                                             &(p_buffer[1]),&buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_rssi_start_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_rssi_stop BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_rssi_stop_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_rssi_stop_rsp_dec(p_buffer,
                                 length,
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_rssi_stop(uint16_t conn_handle)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_rssi_stop_req_enc(conn_handle,
                                                             &(p_buffer[1]),&buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_rssi_stop_rsp_dec);

    return NRF_SUCCESS;
}

/**@brief Command response callback function for @ref sd_ble_gap_tx_power_set BLE command.
 *
 * Callback for decoding the output parameters and the command response return code.
 *
 * @param[in] p_buffer  Pointer to begin of command response buffer.
 * @param[in] length    Length of data in bytes.
 *
 * @return Decoded command response return code.
 */
static uint32_t gap_tx_power_set_rsp_dec(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t result_code = 0;

    const uint32_t err_code = ble_gap_tx_power_set_rsp_dec(p_buffer,
                                 length,
                                 &result_code);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    ble_encode_transport_tx_free();

    return result_code;
}


uint32_t sd_ble_gap_tx_power_set(int8_t tx_power)
{
    uint8_t * p_buffer = ble_encode_transport_tx_alloc();

    // @note: No NULL check required as error checked by called module as defined in API
    // documentation.

    p_buffer[0]             = BLE_RPC_PKT_CMD;
    uint32_t buffer_length  = HCI_TRANSPORT_PKT_DATA_SIZE - 1u;
    const uint32_t err_code = ble_gap_tx_power_set_req_enc(tx_power,
                                                             &(p_buffer[1]),&buffer_length);
    // @note: Should never fail.
    APP_ERROR_CHECK(err_code);

    // @note: Increment buffer length as internally managed packet type field must be included.
    ble_encode_transport_cmd_write(p_buffer,
                                   (++buffer_length),
                                   BLE_ENCODE_WRITE_MODE_RESP,
                                   gap_tx_power_set_rsp_dec);

    return NRF_SUCCESS;
}

