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
#ifndef BLE_GAP_APP_H__
#define BLE_GAP_APP_H__

/**@file
 *
 * @defgroup ble_gap_app GAP Application command request encoders and command response decoders
 * @{
 * @ingroup  ble_sdk_lib_serialization
 *
 * @brief    GAP Application command request encoders and command response decoders.
 */
#include "ble.h"
#include "ble_gap.h"

/**
 * @brief Encodes @ref sd_ble_gap_address_get command request.
 *
 * @sa @ref nrf51_address_get_encoding for packet format,
 *     @ref ble_gap_address_get_rsp_dec for command response decoder.
 *
 * @param[in] p_address      Pointer to address.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @note  \p p_address  will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gap_address_get_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_address_get_req_enc(ble_gap_addr_t const * const p_address,
                                     uint8_t * const              p_buf,
                                     uint32_t * const             p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_address_get command.
 *
 * @sa @ref nrf51_address_get_encoding for packet format,
 *     @ref ble_gap_address_get_req_enc for command request encoder.
 *
 * @param[in] p_buf           Pointer to beginning of command response packet.
 * @param[in] packet_len      Length (in bytes) of response packet.
 * @param[out] p_address      Pointer to address.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gap_address_get_rsp_dec(uint8_t const * const  p_buf,
                                     uint32_t               packet_len,
                                     ble_gap_addr_t * const p_address,
                                     uint32_t * const       p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_address_set command request.
 *
 * @sa @ref nrf51_gap_addr_encoding for packet format,
 *     @ref ble_gap_address_set_rsp_dec for command response decoder.
 *
 * @param[in]     addr_cycle_mode      Address cycle mode.
 * @param[in]     p_addr               Pointer to address structure.
 * @param[in,out] p_buf                Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len            \c in: size of \p p_buf buffer.
 *                                     \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS               Encoding success.
 * @retval NRF_ERROR_NULL            Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_address_set_req_enc(uint8_t                      addr_cycle_mode,
                                     ble_gap_addr_t const * const p_addr,
                                     uint8_t * const              p_buf,
                                     uint32_t * const             p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_address_set command.
 *
 * @sa @ref nrf51_gap_addr_encoding for packet format,
 *     @ref ble_gap_address_set_req_enc for command request encoder.
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
uint32_t ble_gap_address_set_rsp_dec(uint8_t const * const p_buf,
                                     uint32_t              packet_len,
                                     uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_adv_data_set command request.
 *
 * @sa @ref nrf51_adv_set_encoding for packet format,
 *     @ref ble_gap_adv_data_set_rsp_dec for command response decoder.
 *
 * @param[in] p_data         Raw data to be placed in advertisement packet. If NULL, no changes
 *                           are made to the current advertisement packet data.
 * @param[in] dlen           Data length for p_data. Max size: @ref BLE_GAP_ADV_MAX_SIZE octets.
 *                           Should be 0 if p_data is NULL, can be 0 if p_data is not NULL.
 * @param[in] p_sr_data      Raw data to be placed in scan response packet. If NULL,
 *                           no changes are made to the current scan response packet data.
 * @param[in] srdlen         Data length for p_sr_data. Max size: @ref BLE_GAP_ADV_MAX_SIZE octets.
 *                           Should be 0 if p_sr_data is NULL, can be 0 if p_data is not NULL.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_adv_data_set_req_enc(uint8_t const * const p_data,
                                      uint8_t               dlen,
                                      uint8_t const * const p_sr_data,
                                      uint8_t               srdlen,
                                      uint8_t * const       p_buf,
                                      uint32_t * const      p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_adv_data_set command.
 *
 * @sa @ref nrf51_adv_set_encoding for packet format,
 *     @ref ble_gap_adv_data_set_req_enc for command request encoder.
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
uint32_t ble_gap_adv_data_set_rsp_dec(uint8_t const * const p_buf,
                                      uint32_t              packet_len,
                                      uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_adv_start command request.
 *
 * @sa @ref nrf51_adv_start_encoding for packet format,
 *     @ref ble_gap_adv_start_rsp_dec for command response decoder.
 *
 * @param[in] p_adv_params   Pointer to advertising parameters structure.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_adv_start_req_enc(ble_gap_adv_params_t const * const p_adv_params,
                                   uint8_t * const                    p_buf,
                                   uint32_t * const                   p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_adv_start command.
 *
 * @sa @ref nrf51_adv_start_encoding for packet format,
 *     @ref ble_gap_adv_start_req_enc for command request encoder.
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
uint32_t ble_gap_adv_start_rsp_dec(uint8_t const * const p_buf,
                                   uint32_t              packet_len,
                                   uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_tx_power_set command request.
 *
 * @sa @ref nrf51_tx_power_set_encoding for packet format,
 *     @ref ble_gap_tx_power_set_rsp_dec for command response decoder.
 *
 * @param[in]     tx_power   Radio transmit power in dBm (accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm).
 * @param[in]     p_buf      Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_tx_power_set_req_enc(int8_t           tx_power,
                                      uint8_t * const  p_buf,
                                      uint32_t * const p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_tx_power_set command.
 *
 * @sa @ref nrf51_tx_power_set_encoding for packet format,
 *     @ref ble_gap_adv_start_req_snc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] p_result_code Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gap_tx_power_set_rsp_dec(uint8_t const * const p_buf,
                                      uint32_t              packet_len,
                                      uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_appearance_get command request.
 *
 * @sa @ref nrf51_appearance_get_encoding for packet format,
 *     @ref ble_gap_appearance_get_rsp_dec for command response decoder.
 *
 * @param[in] p_appearance   Appearance (16-bit), see @ref BLE_APPEARANCES.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @note  \p p_appearance  will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gap_appearance_get_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_appearance_get_req_enc(uint16_t const * const p_appearance,
                                        uint8_t * const        p_buf,
                                        uint32_t * const       p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_appearance_get command.
 *
 * @sa @ref nrf51_appearance_get_encoding for packet format,
 *     @ref ble_gap_appearance_get_req_enc for command request encoder.
 *
 * @param[in] p_buf           Pointer to beginning of command response packet.
 * @param[in] packet_len      Length (in bytes) of response packet.
 * @param[out] p_appearance   Appearance (16-bit), see @ref BLE_APPEARANCES.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gap_appearance_get_rsp_dec(uint8_t const * const p_buf,
                                        uint32_t              packet_len,
                                        uint16_t * const      p_appearance,
                                        uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_appearance_set command request.
 *
 * @sa @ref nrf51_appearance_set_encoding for packet format,
 *     @ref ble_gap_appearance_set_rsp_dec for command response decoder.
 *
 * @param[in] p_appearance   Appearance (16-bit), see @ref BLE_APPEARANCES.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_appearance_set_req_enc(uint16_t         appearance,
                                        uint8_t * const  p_buf,
                                        uint32_t * const p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_appearance_set command.
 *
 * @sa @ref nrf51_appearance_set_encoding for packet format,
 *     @ref ble_gap_adv_start_req_snc for command request encoder.
 *
 * @param[in] p_buf           Pointer to beginning of command response packet.
 * @param[in] packet_len      Length (in bytes) of response packet.
 * @param[out] p_result_code  Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gap_appearance_set_rsp_dec(uint8_t const * const p_buf,
                                        uint32_t              packet_len,
                                        uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_device_name_get command request.
 *
 * @sa @ref nrf51_device_name_get_encoding for packet format,
 *     @ref ble_gap_device_name_get_rsp_dec for command response decoder.
 *
 * @param[in] p_dev_name     Pointer to an empty buffer where the UTF-8 <b>non NULL-terminated</b>
 *                           string will be placed. Set to NULL to obtain the complete device
 *                           name length.
 * @param[in] p_len          Length of the buffer pointed by p_dev_name.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @note  \p p_dev_name and \p  p_len will not be updated by the command
 *        request encoder. Updated values are set by @ref ble_gap_device_name_get_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_device_name_get_req_enc(uint8_t const * const  p_dev_name,
                                         uint16_t const * const p_dev_name_len,
                                         uint8_t * const        p_buf,
                                         uint32_t * const       p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_device_name_get command.
 *
 * @sa @ref nrf51_device_name_get_encoding for packet format,
 *     @ref ble_gap_device_name_get_req_enc for command request encoder.
 *
 * @param[in] p_buf              Pointer to beginning of command response packet.
 * @param[in] packet_len         Length (in bytes) of response packet.
 * @param[out] p_dev_name        Pointer to an empty buffer where the UTF-8
 *                               <b>non NULL-terminated</b> string will be placed.
 * @param[in,out] p_dev_namelen  Length of the buffer pointed by p_dev_name, complete device name
 *                               length on output.
 * @param[out] p_result_code     Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gap_device_name_get_rsp_dec(uint8_t const * const p_buf,
                                         uint32_t              packet_len,
                                         uint8_t * const       p_dev_name,
                                         uint16_t * const      p_dev_name_len,
                                         uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_device_name_set command request.
 *
 * @sa @ref nrf51_device_name_set_encoding for packet format,
 *     @ref ble_gap_device_name_set_rsp_dec for command response decoder.
 *
 * @param[in] p_write_perm   Write permissions for the Device Name characteristic see
 *                           @ref ble_gap_conn_sec_mode_t.
 * @param[in] p_dev_name     Pointer to a UTF-8 encoded, <b>non NULL-terminated</b> string.
 * @param[in] len            Length of the UTF-8, <b>non NULL-terminated</b> string pointed
 *                           to by p_dev_name in octets (must be smaller or equal
 *                           than @ref BLE_GAP_DEVNAME_MAX_LEN).
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_device_name_set_req_enc(ble_gap_conn_sec_mode_t const * const p_write_perm,
                                         uint8_t const * const                 p_dev_name,
                                         uint16_t                              len,
                                         uint8_t * const                       p_buf,
                                         uint32_t * const                      p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_device_name_set command.
 *
 * @sa @ref nrf51_device_name_set_encoding for packet format,
 *     @ref ble_gap_device_name_get_req_snc for command request encoder.
 *
 * @param[in] p_buf              Pointer to beginning of command response packet.
 * @param[in] packet_len         Length (in bytes) of response packet.
 * @param[out] p_result_code     Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gap_device_name_set_rsp_dec(uint8_t const * const p_buf,
                                         uint32_t              packet_len,
                                         uint32_t * const      p_result_code);

/**
 * @brief Encodes @ref sd_ble_gap_ppcp_set command request.
 *
 * @sa @ref nrf51_ppcp_set_encoding for packet format,
 *     @ref ble_gap_ppcp_set_rsp_dec for command response decoder.
 *
 * @param[in] p_conn_params  Pointer to a @ref ble_gap_conn_params_t structure with the
 *                           desired parameters.
 * @param[in] p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in,out] p_buf_len  \c in: Size of \p p_buf buffer.
 *                           \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_ppcp_set_req_enc(ble_gap_conn_params_t const * const p_conn_params,
                                  uint8_t * const                     p_buf,
                                  uint32_t * const                    p_buf_len);

/**
 * @brief Decodes response to @ref sd_ble_gap_ppcp_set command.
 *
 * @sa @ref nrf51_ppcp_set_encoding for packet format,
 *     @ref ble_gap_ppcp_set_req_enc for command request encoder.
 *
 * @param[in] p_buf              Pointer to beginning of command response packet.
 * @param[in] packet_len         Length (in bytes) of response packet.
 * @param[out] p_result_code     Command result code.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA    Decoding failure. Decoded operation code does not match
 *                                   expected operation code.
 */
uint32_t ble_gap_ppcp_set_rsp_dec(uint8_t const * const p_buf,
                                  uint32_t              packet_len,
                                  uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gap_conn_param_update command request.
 *
 * @sa @ref nrf51_gap_conn_param_update_encoding for packet format,
 *     @ref ble_gap_conn_param_update_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle     Connection handle of the connection.
 * @param[in]      p_conn_params   Pointer to desired connection parameters..
 * @param[in]      p_buf           Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len       \c in: size of \p p_buf buffer.
 *                                 \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_conn_param_update_req_enc(uint16_t                            conn_handle,
                                           ble_gap_conn_params_t const * const p_conn_params,
                                           uint8_t * const                     p_buf,
                                           uint32_t * const                    p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_conn_param_update command.
 *
 * @sa @ref nrf51_gap_conn_param_update_encoding for packet format,
 *     @ref ble_gap_conn_param_update_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_conn_param_update_rsp_dec(uint8_t const * const p_buf,
                                           uint32_t              packet_len,
                                           uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gap_disconnect command request.
 *
 * @sa @ref nrf51_disconnect_encoding for packet format,
 *     @ref ble_gap_disconnect_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle       Connection handle of the connection.
 * @param[in]      hci_status_code   HCI status code, see @ref BLE_HCI_STATUS_CODES.
 * @param[in]      p_buf             Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len         \c in: size of \p p_buf buffer.
 *                                   \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_disconnect_req_enc(uint16_t         conn_handle,
                                    uint8_t          hci_status_code,
                                    uint8_t * const  p_buf,
                                    uint32_t * const p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_disconnect command.
 *
 * @sa @ref nrf51_disconnect_encoding for packet format,
 *     @ref ble_gap_disconnect_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_disconnect_rsp_dec(uint8_t const * const p_buf,
                                    uint32_t              packet_len,
                                    uint32_t * const      p_result_code);


/**@brief Encodes @ref sd_ble_gap_rssi_stop command request.
 *
 * @sa @ref nrf51_rssi_stop for packet format,
 *     @ref ble_gap_rssi_stop_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle       Connection handle of the connection.
 * @param[in]      p_buf             Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len         \c in: size of \p p_buf buffer.
 *                                   \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_rssi_stop_req_enc(uint16_t         conn_handle,
                                   uint8_t * const  p_buf,
                                   uint32_t * const p_buf_len);

/**@brief Decodes response to @ref sd_ble__rssi_stop command.
 *
 * @sa @ref nrf51_rssi_stop for packet format,
 *     @ref ble_gap_rssi_stop_rsp_dec for command response decoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_rssi_stop_rsp_dec(uint8_t const * const p_buf,
                                   uint32_t              packet_len,
                                   uint32_t * const      p_result_code);




/**@brief Encodes @ref sd_ble_gap_ppcp_get command request.
 *
 * @sa @ref nrf51_gap_ppcp_get_encoding for packet format,
 *     @ref ble_gap_ppcp_get_rsp_dec for command response decoder.
 *
 * @param[in]      p_conn_params  Pointer to a @ref ble_gap_conn_params_t structure where the
 *                                parameters will be stored.
 * @param[in]      p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len      \c in: size of \p p_buf buffer.
 *                                \c out: Length of encoded command packet.
 *
 * @note  \p p_conn_params will not be updated by the command request encoder. Updated values are
 *        set by @ref ble_gap_ppcp_get_rsp_dec.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_ppcp_get_req_enc(ble_gap_conn_params_t const * const p_conn_params,
                                  uint8_t * const                     p_buf,
                                  uint32_t * const                    p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_ppcp_get command.
 *
 * @sa @ref nrf51_gap_ppcp_get_encoding for packet format,
 *     @ref ble_gap_ppcp_get_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] p_conn_params Pointer to a @ref ble_gap_conn_params_t structure where the parameters
 *                           will be stored.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_ppcp_get_rsp_dec(uint8_t const * const         p_buf,
                                  uint32_t                      packet_len,
                                  ble_gap_conn_params_t * const p_conn_params,
                                  uint32_t * const              p_result_code);

/**@brief Encodes @ref sd_ble_gap_auth_key_reply command request.
 *
 * @sa @ref nrf51_gap_auth_key_reply_encoding for packet format,
 *     @ref ble_gap_auth_key_reply_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle    Connection handle of the connection.
 * @param[in]      key_type       Key type which defines length of key data as defined for
 *                                @ref sd_ble_gap_auth_key_reply .
 * @param[in]      p_key          Pointer to a buffer which contains key
 * @param[in]      p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len      \c in: size of \p p_buf buffer.
 *                                \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_PARAM    Encoding failure. Incorrect param provided (key_type).
 */
uint32_t ble_gap_auth_key_reply_req_enc(uint16_t              conn_handle,
                                        uint8_t               key_type,
                                        uint8_t const * const p_key,
                                        uint8_t * const       p_buf,
                                        uint32_t * const      p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_auth_key_reply command.
 *
 * @sa @ref nrf51_gap_auth_key_reply_encoding for packet format,
 *     @ref ble_gap_sec_auth_key_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_auth_key_reply_rsp_dec(uint8_t const * const p_buf,
                                        uint32_t              packet_len,
                                        uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gap_sec_info_reply command request.
 *
 * @sa @ref nrf51_gap_sec_info_reply_encoding for packet format,
 *     @ref ble_gap_sec_info_reply_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle    Connection handle of the connection.
 * @param[in]      p_enc_info     Pointer to a @ref ble_gap_enc_info_t encryption information
 *                                structure.
 * @param[in]      p_sign_info    Pointer to a @ref ble_gap_sign_info_t signing information
 *                                structure.
 * @param[in]      p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len      \c in: size of \p p_buf buffer.
 *                                \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_sec_info_reply_req_enc(uint16_t                          conn_handle,
                                        ble_gap_enc_info_t const * const  p_enc_info,
                                        ble_gap_sign_info_t const * const p_sign_info,
                                        uint8_t * const                   p_buf,
                                        uint32_t * const                  p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_sec_info_reply command.
 *
 * @sa @ref nrf51_gap_sec_info_reply_encoding for packet format,
 *     @ref ble_gap_sec_info_reply_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_sec_info_reply_rsp_dec(uint8_t const * const p_buf,
                                        uint32_t              packet_len,
                                        uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gap_sec_params_reply command request.
 *
 * @sa @ref nrf51_sec_params_reply_encoding for packet format,
 *     @ref ble_gap_sec_params_reply_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle    Connection handle of the connection.
 * @param[in]      sec_status     Security status, see @ref BLE_GAP_SEC_STATUS.
 * @param[in]      p_sec_params   Pointer to a @ref ble_gap_sec_params_t security parameters
 *                                structure.
 * @param[in]      p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len      \c in: size of \p p_buf buffer.
 *                                \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_sec_params_reply_req_enc(uint16_t                           conn_handle,
                                          uint8_t                            sec_status,
                                          ble_gap_sec_params_t const * const p_sec_params,
                                          uint8_t * const                    p_buf,
                                          uint32_t * const                   p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_sec_params_reply command.
 *
 * @sa @ref nrf51_sec_params_reply_encoding for packet format,
 *     @ref ble_gap_sec_params_reply_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_sec_params_reply_rsp_dec(uint8_t const * const p_buf,
                                          uint32_t              packet_len,
                                          uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gap_authenticate command request.
 *
 * @sa @ref nrf51_ble_gap_authenticate_encoding for packet format,
 *     @ref ble_gap_authenticate_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle    Connection handle of the connection.
 * @param[in]      p_sec_params   Pointer to a @ref ble_gap_sec_params_t security parameters
 *                                structure.
 * @param[in]      p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len      \c in: size of \p p_buf buffer.
 *                                \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_authenticate_req_enc(uint16_t                           conn_handle,
                                      ble_gap_sec_params_t const * const p_sec_params,
                                      uint8_t * const                    p_buf,
                                      uint32_t * const                   p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_authenticate command.
 *
 * @sa @ref nrf51_ble_gap_authenticate_encoding for packet format,
 *     @ref ble_gap_authenticate_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_authenticate_rsp_dec(uint8_t const * const p_buf,
                                      uint32_t              packet_len,
                                      uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gap_adv_stop command request.
 *
 * @sa @ref nrf51_sd_ble_gap_adv_stop for packet format,
 *     @ref ble_gap_adv_stop_rsp_dec for command response decoder.
 *
 * @param[in]      p_buf          Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len      \c in: size of \p p_buf buffer.
 *                                \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_adv_stop_req_enc(uint8_t * const p_buf, uint32_t * const p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_adv_stop command.
 *
 * @sa @ref nrf51_sd_ble_gap_adv_stop for packet format,
 *     @ref ble_gap_adv_stop_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_adv_stop_rsp_dec(uint8_t const * const p_buf,
                                  uint32_t              packet_len,
                                  uint32_t * const      p_result_code);

/**@brief Encodes @ref sd_ble_gap_conn_sec_get command request.
 *
 * @sa @ref nrf51_conn_sec_get_encoding for packet format,
 *     @ref ble_gap_conn_sec_get_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle       Connection handle of the connection.
 * @param[in]      p_conn_sec        Pointer to \ref ble_gap_conn_sec_t which will be filled in
 *                                   response.
 * @param[in]      p_buf             Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len         \c in: size of \p p_buf buffer.
 *                                   \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_conn_sec_get_req_enc(uint16_t                         conn_handle,
                                      ble_gap_conn_sec_t const * const p_conn_sec,
                                      uint8_t * const                  p_buf,
                                      uint32_t * const                 p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_conn_sec_get command.
 *
 * @sa @ref nrf51_conn_sec_get_encoding for packet format,
 *     @ref ble_gap_conn_sec_get_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] pp_conn_sec   Pointer to pointer to \ref ble_gap_conn_sec_t which will be filled by
 *                           the decoded data (if present).
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_conn_sec_get_rsp_dec(uint8_t const * const        p_buf,
                                      uint32_t                     packet_len,
                                      ble_gap_conn_sec_t * * const pp_conn_sec,
                                      uint32_t * const             p_result_code);

/**@brief Encodes @ref sd_ble_gap_rssi_start command request.
 *
 * @sa @ref nrf51_rssi_start_encoding for packet format,
 *     @ref ble_gap_rssi_start_rsp_dec for command response decoder.
 *
 * @param[in]      conn_handle       Connection handle of the connection.
 * @param[in]      p_buf             Pointer to buffer where encoded data command will be returned.
 * @param[in, out] p_buf_len         \c in: size of \p p_buf buffer.
 *                                   \c out: Length of encoded command packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gap_rssi_start_req_enc(uint16_t         conn_handle,
                                    uint8_t * const  p_buf,
                                    uint32_t * const p_buf_len);

/**@brief Decodes response to @ref sd_ble_gap_rssi_start command.
 *
 * @sa @ref nrf51_rssi_start_encoding for packet format,
 *     @ref ble_gap_rssi_start_req_enc for command request encoder.
 *
 * @param[in]  p_buf         Pointer to beginning of command response packet.
 * @param[in]  packet_len    Length (in bytes) of response packet.
 * @param[out] result_code   Command response result code.
 *
 * @retval NRF_SUCCESS              Decoding success.
 * @retval NRF_ERROR_NULL           Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_DATA_SIZE      Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_INVALID_DATA   Decoding failure. Decoded operation code does not match expected
 *                                  operation code.
 */
uint32_t ble_gap_rssi_start_rsp_dec(uint8_t const * const p_buf,
                                    uint32_t              packet_len,
                                    uint32_t * const      p_result_code);

/** @} */
#endif
