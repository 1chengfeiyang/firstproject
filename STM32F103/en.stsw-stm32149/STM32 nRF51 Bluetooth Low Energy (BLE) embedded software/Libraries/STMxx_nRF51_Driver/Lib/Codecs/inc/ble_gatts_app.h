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
#ifndef BLE_GATTS_APP_H__
#define BLE_GATTS_APP_H__
/**@file
 *
 * @defgroup ble_gatts_app GATTS Application command request encoders and command response decoders
 * @{
 * @ingroup  ble_sdk_lib_serialization
 *
 * @brief    GATTS Application command request encoders and command response decoders.
 */
#include "ble_gatts.h"
#include "ble.h"

/**@brief Encodes @ref sd_ble_gatts_value_get command request.
 *
 * @sa @ref nrf51_value_get_encoding for packet format,
 *     @ref ble_gatts_value_get_rsp_dec for command response decoder.
 *
 * @param[in] handle         Attribute handle.
 * @param[in] offset         Offset in bytes to read from.
 * @param[in] p_value        Pointer to buffer where the requested value will be stored.
 *                           Can be NULL to calculate required size.
 * @param[in] p_value_len    Size of \p p_value buffer if \p p_value is not NULL.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @note  \p p_data_len and \p p_data will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gatts_value_get_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_value_get_req_enc(uint16_t               handle,
                                     uint16_t               offset,
                                     uint16_t const * const p_value_len,
                                     uint8_t const * const  p_value,
                                     uint8_t * const        p_buf,
                                     uint32_t * const       p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_value_get command.
 *
 * @sa @ref nrf51_value_get_encoding for packet format,
 *     @ref ble_gatts_value_get_req_enc for command request encoder.
 *
 * @param[in]  p_buf           Pointer to beginning of command response packet.
 * @param[in]  packet_len      Length (in bytes) of response packet.
 * @param[out] pp_value         Pointer to pointera buffer where the requested value will be stored.
 * @param[in,out] pp_value_len  \c in: Size (in bytes) of \p p_value buffer.
 *                             \c out: Length of decoded contents of \p p_value.
 * @param[out] p_result_code   Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_DATA_SIZE       Length of \p p_value is too small to hold decoded
 *                                   value from response.
 */
uint32_t ble_gatts_value_get_rsp_dec(uint8_t const * const p_buf,
                                     uint32_t              packet_len,
                                     uint8_t * * const     pp_value,
                                     uint16_t * * const    pp_value_len,
                                     uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gatts_hvx command request.
 *
 * @sa @ref nrf51_gatts_hvx_encoding for packet format,
 *     @ref ble_gatts_hvx_rsp_dec for command response decoder.
 *
 * @param[in] conn_handle    Connection handle.
 * @param[in] p_hvx_params   Pointer to an HVx parameters structure to be encoded.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @note  \p p_hvx_params will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gatts_hvx_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_hvx_req_enc(uint16_t                             conn_handle,
                               ble_gatts_hvx_params_t const * const p_hvx_params,
                               uint8_t * const                      p_buf,
                               uint32_t * const                     p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_hvx command.
 *
 * @sa @ref nrf51_gatts_hvx_encoding for packet format,
 *     @ref ble_gatts_hvx_req_enc for command request encoder.
 *
 * @param[in]  p_buf            Pointer to beginning of command response packet.
 * @param[in]  packet_len       Length (in bytes) of response packet.
 * @param[out] p_result_code    Command result code.
 * @param[out] pp_bytes_written Pointer to pointer to location where number of bytes is written.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_hvx_rsp_dec(uint8_t const * const p_buf,
                               uint32_t              packet_len,
                               uint32_t * const      p_result_code,
                               uint16_t * * const    pp_bytes_written);

/**@brief Encodes @ref sd_ble_gatts_characteristic_add command request.
 *
 * @sa @ref nrf51_characteristics_add_encoding for packet format,
 *     @ref ble_gatts_characteristic_add_rsp_dec for command response decoder.
 *
 * @param[in] service_handle     Handle of the service where the characteristic is to be placed.
 *                               If @ref BLE_GATT_HANDLE_INVALID is used, it will be placed
 *                               sequentially.
 * @param[in] p_char_md          Pointer to a @ref ble_gatts_char_md_t structure, characteristic
 *                               metadata.
 * @param[in] p_attr_char_value  Pointer to a @ref ble_gatts_attr_t structure, corresponding to
 *                               the characteristic value.
 * @param[in] p_handles          Pointer to a @ref ble_gatts_char_handles_t structure, where the
 *                               assigned handles will be stored.
 * @param[in] p_buf              Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len      \c in: Size of \p p_buf buffer.
 *                               \c out: Length of encoded command packet.
 *
 * @note  \p p_handles will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gatts_characteristic_add_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_characteristic_add_req_enc
    (uint16_t                              service_handle,
    ble_gatts_char_md_t const * const      p_char_md,
    ble_gatts_attr_t const * const         p_attr_char_value,
    ble_gatts_char_handles_t const * const p_handles,
    uint8_t * const                        p_buf,
    uint32_t * const                       p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_characteristic_add command.
 *
 * @sa @ref nrf51_characteristics_add_encoding for packet format,
 *     @ref ble_gatts_characteristic_add_req_enc for command request encoder.
 *
 * @param[in]  p_buf              Pointer to beginning of command response packet.
 * @param[in]  packet_len         Length (in bytes) of response packet.
 * @param[out] pp_handles         Pointer to pointer to location where handles should be decoded.
 * @param[out] p_result_code      Pointer to command result code decode location.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_characteristic_add_rsp_dec(uint8_t const * const p_buf,
                                              uint32_t              packet_len,
                                              uint16_t * * const    pp_handles,
                                              uint32_t * const      p_result_code);


/**@brief Encodes @ref sd_ble_gatts_service_add command request.
 *
 * @sa @ref nrf51_gatts_service_add_encoding for packet format,
 *     @ref ble_gatts_service_add_rsp_dec for command response decoder.
 *
 * @param[in] type           Toggles between primary and secondary services,
 *                           see @ref BLE_GATTS_SRVC_TYPES.
 * @param[in] p_uuid         Pointer to service UUID.
 * @param[in] p_conn_handle  Pointer to a 16-bit word where the assigned handle will be stored.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @note  \p p_conn_handle will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gatts_service_add_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_service_add_req_enc(uint8_t                  type,
                                       ble_uuid_t const * const p_uuid,
                                       uint16_t const * const   p_conn_handle,
                                       uint8_t * const          p_buf,
                                       uint32_t * const         p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_service_add command.
 *
 * @sa @ref nrf51_gatts_service_add_encoding for packet format,
 *     @ref ble_gatts_service_add_req_enc for command request encoder.
 *
 * @param[in]  p_buf          Pointer to beginning of command response packet.
 * @param[in]  packet_len     Length (in bytes) of response packet.
 * @param[out] p_conn_handle  Connection handle.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_service_add_rsp_dec(uint8_t const * const p_buf,
                                       uint32_t              packet_len,
                                       uint16_t * const      p_conn_handle,
                                       uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gatts_sys_attr_set command request.
 *
 * @sa @ref nrf51_gatts_sys_attr_set_encoding for packet format,
 *     @ref ble_gatts_sys_attr_set_rsp_dec for command response decoder.
 *
 * @param[in] conn_handle        Connection handle.
 * @param[in] p_sys_attr_data    Pointer to a buffer (at least \p sys_attr_data_len bytes long)
 *                               containing the attribute value to write.
 * @param[in] sys_attr_data_len  Length (in bytes) of \p p_sys_attr_data.
 * @param[in] p_buf              Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len      \c in: Size of \p p_buf buffer.
 *                               \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_sys_attr_set_req_enc(uint16_t              conn_handle,
                                        uint8_t const * const p_sys_attr_data,
                                        uint16_t              sys_attr_data_len,
                                        uint8_t * const       p_buf,
                                        uint32_t * const      p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_sys_attr_set command.
 *
 * @sa @ref nrf51_gatts_sys_attr_set_encoding for packet format,
 *     @ref ble_gatts_sys_attr_set_req_enc for command request encoder.
 *
 * @param[in]  p_buf          Pointer to beginning of command response packet.
 * @param[in]  packet_len     Length (in bytes) of response packet.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_sys_attr_set_rsp_dec(uint8_t const * const p_buf,
                                        uint32_t              packet_len,
                                        uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gatts_value_set command request.
 *
 * @sa @ref nrf51_gatts_value_set_encoding for packet format,
 *     @ref ble_gatts_value_set_rsp_dec for command response decoder.
 *
 * @param[in] handle         Attribute handle.
 * @param[in] offset         Offset (in bytes) to write from.
 * @param[in] p_value_len    Pointer to length (in bytes) of \p p_value.
 * @param[in] p_value        Pointer to a buffer (at least \p value_len bytes long) containing the
 *                           attribute value to write.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_value_set_req_enc(uint16_t              handle,
                                     uint16_t              offset,
                                     uint16_t * const      p_value_len,
                                     uint8_t const * const p_value,
                                     uint8_t * const       p_buf,
                                     uint32_t * const      p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_value_set command.
 *
 * @sa @ref nrf51_gatts_value_set_encoding for packet format,
 *     @ref ble_gatts_value_set_req_enc for command request encoder.
 *
 * @param[in]  p_buf          Pointer to beginning of command response packet.
 * @param[in]  packet_len     Length (in bytes) of response packet.
 * @param[out] p_value_len    Length (in bytes) written.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_value_set_rsp_dec(uint8_t const * const p_buf,
                                     uint32_t              packet_len,
                                     uint16_t * const      p_value_len,
                                     uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gatts_sys_attr_get command request.
 *
 * @sa @ref nrf51_gatts_sys_attr_get_encoding for packet format,
 *     @ref ble_gatts_sys_attr_get_rsp_dec for command response decoder.
 *
 * @param[in]     conn_handle          Connection handle of the connection.
 * @param[in]     p_sys_attr_data      Pointer to buffer where updated information about system
 *                                     attributes will be stored. Can be NULL to calculate required
 *                                     size.
 * @param[in]     p_sys_attr_data_len  Size of p_sys_attr_data buffer if \p p_sys_attr_data is
 *                                     not NULL.
 * @param[in,out] p_buf                Pointer to buffer where encoded data command will
 *                                     be returned.
 * @param[in,out] p_buf_len            \c in: size of \p p_buf buffer.
 *                                     \c out: Length of encoded command packet.
 *
 * @note  \p p_sys_attr_data and \p p_sys_attr_data_len will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gatts_sys_attr_get_rsp_dec.
 *
 * @retval NRF_SUCCESS               Encoding success.
 * @retval NRF_ERROR_NULL            Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_sys_attr_get_req_enc(uint16_t               conn_handle,
                                        uint8_t const * const  p_sys_attr_data,
                                        uint16_t const * const p_sys_attr_data_len,
                                        uint8_t * const        p_buf,
                                        uint32_t *             p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_sys_attr_get command.
 *
 * @sa @ref nrf51_gatts_sys_attr_get_encoding for packet format,
 *     @ref ble_gatts_sys_attr_get_req_enc for command request encoder.
 *
 * @param[in] p_buf                    Pointer to beginning of command response packet.
 * @param[in] packet_len               Length (in bytes) of response packet.
 * @param[out] p_sys_attr_data         Pointer to a buffer where updated information about system
 *                                     attributes will be stored.
 * @param[in,out] p_sys_attr_data_len  \c in: Size (in bytes) of \p p_sys_attr_data buffer.
 *                                     \c out: Length of decoded contents of \p p_sys_attr_data.
 * @param[out] p_result_code           Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_DATA_SIZE       Length of \p p_sys_attr_data is too small to hold decoded
 *                                   value from response.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_sys_attr_get_rsp_dec(uint8_t const * const p_buf,
                                        uint32_t              packet_len,
                                        uint8_t * const       p_sys_attr_data,
                                        uint16_t * const      p_sys_attr_data_len,
                                        uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gatts_descriptor_add command request.
 *
 * @sa @ref nrf51_descriptor_add_encoding for packet format,
 *     @ref ble_gatts_descriptor_add_rsp_dec for command response decoder.
 *
 * @param[in] char_handle        Handle of the characteristic where the description is to be placed.
 *                               If @ref BLE_GATT_HANDLE_INVALID is used, it will be placed
 *                               sequentially.
 * @param[in] p_attr_md          Pointer to a @ref ble_gatts_attr_t structure, characteristic
 *                               metadata.
 * @param[in] p_handle           Pointer to a @ref ble_gatts_char_handles_t structure, where the
 *                               assigned handles will be stored.
 * @param[in] p_buf              Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len      \c in: Size of \p p_buf buffer.
 *                               \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_descriptor_add_req_enc(uint16_t                       char_handle,
                                          ble_gatts_attr_t const * const p_attr,
                                          uint16_t * const               p_handle,
                                          uint8_t * const                p_buf,
                                          uint32_t * const               p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_descriptor_add command.
 *
 * @sa @ref nrf51_descriptor_add_encoding for packet format,
 *     @ref ble_gatts_descriptor_add_req_enc for command request encoder.
 *
 * @param[in]  p_buf              Pointer to beginning of command response packet.
 * @param[in]  packet_len         Length (in bytes) of response packet.
 * @param[out] p_handle           Pointer to bufer where descriptor handle will be
                                  returned.
 * @param[out] p_result_code      Pointer to command result code decode location.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_descriptor_add_rsp_dec(uint8_t const * const p_buf,
                                          uint32_t              packet_len,
                                          uint16_t * const      p_handle,
                                          uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gatts_include_add command request.
 *
 * @sa @ref nrf51_include_add_encoding for packet format,
 *     @ref ble_gatts_include_add_rsp_dec for command response decoder.
 *
 * @param[in] service_handle     Handle of the service where the included service is to be placed.
 * @param[in] inc_srvc_handle    Handle of the included service
 * @param[in] p_include_handle   Pointer to Pointer to a 16-bit word where the assigned handle will be stored.
 * @param[in] p_buf              Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len      \c in: Size of \p p_buf buffer.
 *                               \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gatts_include_add_req_enc(uint16_t         service_handle,
                                       uint16_t         inc_srvc_handle,
                                       uint16_t * const p_include_handle,
                                       uint8_t * const  p_buf,
                                       uint32_t * const p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_include_add command.
 *
 * @sa @ref nrf51_include_add_encoding for packet format,
 *     @ref ble_gatts_include_add_req_enc for command request encoder.
 *
 * @param[in]  p_buf              Pointer to beginning of command response packet.
 * @param[in]  packet_len         Length (in bytes) of response packet.
 * @param[out] p_include_handle   Pointer to a 16-bit word where the assigned handle will be stored.
 * @param[out] p_result_code      Pointer to command result code decode location.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_include_add_rsp_dec(uint8_t const * const p_buf,
                                       uint32_t              packet_len,
                                       uint16_t * const      p_include_handle,
                                       uint32_t * const      p_result_code);


/**@brief Encodes @ref sd_ble_gatts_rw_authorize_reply command request.
 *
 * @sa @ref nrf51_gatts_rw_authorize_reply_encoding for packet format,
 *     @ref ble_gatts_rw_authorize_reply_rsp_dec for command response decoder.
 *
 * @param[in] conn_handle    Connection handle.
 * @param[in] p_reply_params Pointer to \ref ble_gatts_rw_authorize_reply_params_t
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_PARAM    Encoding failure. Invalid param provided in p_reply_params.
 */
uint32_t ble_gatts_rw_authorize_reply_req_enc(
    uint16_t conn_handle,
    ble_gatts_rw_authorize_reply_params_t const * const
    p_reply_params,
    uint8_t * const
    p_buf,
    uint32_t * const
    p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_rw_authorize_reply command.
 *
 * @sa @ref nrf51_gatts_rw_authorize_reply_encoding for packet format,
 *     @ref ble_gatts_rw_authorize_reply_req_enc for command request encoder.
 *
 * @param[in]  p_buf          Pointer to beginning of command response packet.
 * @param[in]  packet_len     Length (in bytes) of response packet.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_rw_authorize_reply_rsp_dec(uint8_t const * const p_buf,
                                              uint32_t              packet_len,
                                              uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gatts_service_changed command request.
 *
 * @sa @ref nrf51_gatts_service_chhanged_encoding for packet format,
 *     @ref ble_gatts_service_changed_rsp_dec for command response decoder.
 *
 * @param[in] conn_handle    Connection handle.
 * @param[in] start_handle   Start of affected attribute handle range.
 * @param[in] end_handle     End of affected attribute handle range.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_PARAM    Encoding failure. Invalid param provided in p_reply_params.
 */
uint32_t ble_gatts_service_changed_req_enc(uint16_t         conn_handle,
                                           uint16_t         start_handle,
                                           uint16_t         end_handle,
                                           uint8_t * const  p_buf,
                                           uint32_t * const p_buf_len);

/**@brief Decodes response to @ref sd_ble_gatts_service_changed command.
 *
 * @sa @ref nrf51_gatts_service_changed_encoding for packet format,
 *     @ref ble_gatts_service_changed_req_enc for command request encoder.
 *
 * @param[in]  p_buf          Pointer to beginning of command response packet.
 * @param[in]  packet_len     Length (in bytes) of response packet.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gatts_service_changed_rsp_dec(uint8_t const * const p_buf,
                                           uint32_t              packet_len,
                                           uint32_t * const      p_result_code);

/** @} */
#endif //BLE_GATTS_APP_H__


