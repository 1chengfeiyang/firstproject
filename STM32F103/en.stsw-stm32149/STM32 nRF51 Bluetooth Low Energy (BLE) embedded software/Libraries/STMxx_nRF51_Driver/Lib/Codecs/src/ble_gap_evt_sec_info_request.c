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

#include "ble_gap_evt_app.h"
#include "ble_serialization.h"
#include "app_util.h"


uint32_t ble_gap_evt_sec_info_request_dec(uint8_t const * const p_buf,
                                          uint32_t              packet_len,
                                          ble_evt_t * const     p_event,
                                          uint32_t * const      p_event_len)
{
    uint32_t index = 0;
    uint32_t event_len;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_event_len);

    SER_ASSERT_LENGTH_LEQ(2 + 1 + 6 + 2 + 1, packet_len);

    event_len = SER_EVT_CONN_HANDLE_SIZE + sizeof (ble_gap_evt_sec_info_request_t);

    if (p_event == NULL)
    {
        *p_event_len = event_len;
        return NRF_SUCCESS;
    }
    SER_ASSERT(event_len <= *p_event_len, NRF_ERROR_DATA_SIZE);

    p_event->header.evt_id  = BLE_GAP_EVT_SEC_INFO_REQUEST;
    p_event->header.evt_len = event_len;

    uint16_dec(p_buf, packet_len, &index, &p_event->evt.gap_evt.conn_handle);

    ble_gap_evt_sec_info_request_t * p_sec_info_request =
        &(p_event->evt.gap_evt.params.sec_info_request);

    p_sec_info_request->peer_addr.addr_type = p_buf[index++];
    p_sec_info_request->peer_addr.addr[0]   = p_buf[index++];
    p_sec_info_request->peer_addr.addr[1]   = p_buf[index++];
    p_sec_info_request->peer_addr.addr[2]   = p_buf[index++];
    p_sec_info_request->peer_addr.addr[3]   = p_buf[index++];
    p_sec_info_request->peer_addr.addr[4]   = p_buf[index++];
    p_sec_info_request->peer_addr.addr[5]   = p_buf[index++];

    uint16_dec(p_buf, packet_len, &index, &p_sec_info_request->div);

    p_sec_info_request->enc_info  = (p_buf[index] >> 0) & 0x1;
    p_sec_info_request->id_info   = (p_buf[index] >> 1) & 0x1;
    p_sec_info_request->sign_info = (p_buf[index] >> 2) & 0x1;
    index++;

    SER_ASSERT_LENGTH_EQ(index, packet_len);

    *p_event_len = event_len;

    return NRF_SUCCESS;
}
