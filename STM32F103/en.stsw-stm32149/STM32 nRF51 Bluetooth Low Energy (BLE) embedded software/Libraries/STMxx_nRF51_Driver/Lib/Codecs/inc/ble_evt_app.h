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
#ifndef BLE_EVT_APP_H__
#define BLE_EVT_APP_H__
/**@file
 *
 * @defgroup ble_evt_app Application event decoders
 * @{
 * @ingroup  ble_sdk_lib_serialization
 *
 * @brief    Application event decoders.
 */
#include "ble.h"

/**
 * @brief Decodes ble_evt_tx_complete event.
 *
 * @sa @ref nrf51_evt_tx_complete_encoding for packet format.
 *
 * If \p p_event is null, the required length of \p p_event is returned in \p p_event_len.
 *
 * @param[in] p_buf            Pointer to the beginning of an event packet.
 * @param[in] packet_len       Length (in bytes) of the event packet.
 * @param[in,out] p_event      Pointer to a \ref ble_evt_t buffer where the decoded event will be
 *                             stored. If NULL, required length will be returned in \p p_event_len.
 * @param[in,out] p_event_len  \c in: Size (in bytes) of \p p_event buffer.
 *                             \c out: Length of decoded contents of \p p_event.
 *
 * @retval NRF_SUCCESS               Decoding success.
 * @retval NRF_ERROR_NULL            Decoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH  Decoding failure. Incorrect buffer length.
 * @retval NRF_ERROR_DATA_SIZE       Decoding failure. Length of \p p_event is too small to
 *                                   hold decoded event.
 */
uint32_t ble_evt_tx_complete_dec(uint8_t const * const p_buf,
                                 uint32_t              packet_len,
                                 ble_evt_t * const     p_event,
                                 uint32_t * const      p_event_len);

/** @} */
#endif
