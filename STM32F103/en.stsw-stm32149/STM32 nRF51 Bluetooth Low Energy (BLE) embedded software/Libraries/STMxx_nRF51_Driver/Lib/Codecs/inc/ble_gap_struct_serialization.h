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

uint32_t ble_gap_irk_enc(void const * const p_data,
                         uint8_t * const    p_buf,
                         uint32_t           buf_len,
                         uint32_t * const   p_index);

uint32_t ble_gap_irk_dec(uint8_t const * const p_buf,
                         uint32_t              buf_len,
                         uint32_t * const      p_index,
                         void * const          p_data);

uint32_t ble_gap_addr_enc(void const * const p_data,
                          uint8_t * const    p_buf,
                          uint32_t           buf_len,
                          uint32_t * const   p_index);

uint32_t ble_gap_addr_dec(uint8_t const * const p_buf,
                          uint32_t              buf_len,
                          uint32_t * const      p_index,
                          void * const          p_addr);

uint32_t ble_gap_sec_levels_enc(void const * const p_data,
                                uint8_t * const    p_buf,
                                uint32_t           buf_len,
                                uint32_t * const   p_index);

uint32_t ble_gap_sec_keys_enc(void const * const p_data,
                              uint8_t * const    p_buf,
                              uint32_t           buf_len,
                              uint32_t * const   p_index);

uint32_t ble_gap_enc_info_enc(void const * const p_data,
                              uint8_t * const    p_buf,
                              uint32_t           buf_len,
                              uint32_t * const   p_index);

uint32_t ble_gap_enc_info_dec(uint8_t const * const p_buf,
                              uint32_t              buf_len,
                              uint32_t * const      p_index,
                              void * const          p_enc_info);

uint32_t ble_gap_sign_info_dec(uint8_t const * const p_buf,
                               uint32_t              buf_len,
                               uint32_t * const      p_index,
                               void * const          p_sign_info);

uint32_t ble_gap_evt_auth_status_t_enc(void const * const p_data,
                                       uint8_t * const    p_buf,
                                       uint32_t           buf_len,
                                       uint32_t * const   p_index);

uint32_t ble_gap_conn_sec_mode_enc(void const * const p_void_sec_mode,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index);

uint32_t ble_gap_conn_sec_mode_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_sec_mode);

uint32_t ble_gap_conn_sec_t_enc(void const * const p_void_sec,
                                uint8_t * const    p_buf,
                                uint32_t           buf_len,
                                uint32_t * const   p_index);

uint32_t ble_gap_conn_sec_t_dec(uint8_t const * const p_buf,
                                uint32_t              buf_len,
                                uint32_t * const      p_index,
                                void * const          p_void_sec);

uint32_t ble_gap_evt_conn_sec_update_t_enc(void const * const p_void_conn_sec_update,
                                           uint8_t * const    p_buf,
                                           uint32_t           buf_len,
                                           uint32_t * const   p_index);

uint32_t ble_gap_evt_conn_sec_update_t_dec(uint8_t const * const p_buf,
                                           uint32_t              buf_len,
                                           uint32_t * const      p_index,
                                           void * const          p_void_conn_sec_update);

uint32_t ble_gap_evt_sec_info_request_t_enc(void const * const p_void_sec_info_request,
                                            uint8_t * const    p_buf,
                                            uint32_t           buf_len,
                                            uint32_t * const   p_index);

uint32_t ble_gap_evt_sec_info_request_t_dec(uint8_t const * const p_buf,
                                            uint32_t              buf_len,
                                            uint32_t * const      p_index,
                                            void * const          p_void_sec_info_request);
uint32_t ble_gap_evt_connected_t_enc(void const * const p_void_struct,
                                     uint8_t * const    p_buf,
                                     uint32_t           buf_len,
                                     uint32_t * const   p_index);

uint32_t ble_gap_sec_params_t_enc(void const * const p_void_struct,
                                  uint8_t * const    p_buf,
                                  uint32_t           buf_len,
                                  uint32_t * const   p_index);

uint32_t ble_gap_sec_params_t_dec(uint8_t const * const p_buf,
                                  uint32_t              buf_len,
                                  uint32_t * const      p_index,
                                  void * const          p_void_struct);

uint32_t ble_gap_evt_sec_params_request_t_enc(void const * const p_void_struct,
                                              uint8_t * const    p_buf,
                                              uint32_t           buf_len,
                                              uint32_t * const   p_index);

uint32_t ble_gap_evt_sec_params_request_t_dec(uint8_t const * const p_buf,
                                              uint32_t              buf_len,
                                              uint32_t * const      p_index,
                                              void * const          p_void_struct);

uint32_t ble_gap_conn_params_t_enc(void const * const p_void_conn_params,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index);

uint32_t ble_gap_conn_params_t_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_conn_params);

uint32_t ble_gap_evt_conn_param_update_t_enc(void const * const p_void_evt_conn_param_update,
                                             uint8_t * const    p_buf,
                                             uint32_t           buf_len,
                                             uint32_t * const   p_index);

uint32_t ble_gap_evt_conn_param_update_t_dec(uint8_t const * const p_buf,
                                             uint32_t              buf_len,
                                             uint32_t * const      p_index,
                                             void * const          p_void_evt_conn_param_update);

uint32_t ble_gap_evt_disconnected_t_enc(void const * const p_void_disconnected,
                                        uint8_t * const    p_buf,
                                        uint32_t           buf_len,
                                        uint32_t * const   p_index);

uint32_t ble_gap_evt_disconnected_t_dec(uint8_t const * const p_buf,
                                        uint32_t              buf_len,
                                        uint32_t * const      p_index,
                                        void * const          p_void_disconnected);

uint32_t ble_gap_opt_local_conn_latency_t_enc(void const * const p_void_local_conn_latency,
                                              uint8_t * const    p_buf,
                                              uint32_t           buf_len,
                                              uint32_t * const   p_index);

uint32_t ble_gap_opt_local_conn_latency_t_dec(uint8_t const * const p_buf,
                                              uint32_t              buf_len,
                                              uint32_t * const      p_index,
                                              void * const          p_void_local_conn_latency);

uint32_t ble_gap_opt_passkey_t_enc(void const * const p_void_passkey,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index);

uint32_t ble_gap_opt_passkey_t_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_passkey);

uint32_t ble_gap_opt_privacy_t_enc(void const * const p_void_privacy,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index);

uint32_t ble_gap_opt_privacy_t_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_privacy);
