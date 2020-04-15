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
#include <string.h>
#include "ble_serialization.h"
#include "app_util.h"


uint32_t ble_gap_evt_auth_status_dec(uint8_t const * const p_buf,
                                     uint32_t              packet_len,
                                     ble_evt_t * const     p_event,
                                     uint32_t * const      p_event_len)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_event_len);

    uint32_t event_len = sizeof (ble_gap_evt_auth_status_t) +
                         sizeof (p_event->evt.gap_evt.conn_handle);

    if (p_event == NULL)
    {
        *p_event_len = event_len;
        return NRF_SUCCESS;
    }

    SER_ASSERT(event_len <= *p_event_len, NRF_ERROR_DATA_SIZE);

    SER_ASSERT_LENGTH_LEQ(2 + 1 + 1 + 1 + 1 + 1 + 2, packet_len);
    p_event->header.evt_len = event_len;
    uint16_dec(p_buf, packet_len, &index, &p_event->evt.gap_evt.conn_handle);

    ble_gap_evt_auth_status_t * p_decoded_evt = &(p_event->evt.gap_evt.params.auth_status);

    p_decoded_evt->auth_status = p_buf[index++];
    p_decoded_evt->error_src   = p_buf[index++];

    p_decoded_evt->sm1_levels.lv3 = (p_buf[index] >> 5) & 0x01;
    p_decoded_evt->sm1_levels.lv2 = (p_buf[index] >> 4) & 0x01;
    p_decoded_evt->sm1_levels.lv1 = (p_buf[index] >> 3) & 0x01;
    p_decoded_evt->sm2_levels.lv3 = (p_buf[index] >> 2) & 0x01;
    p_decoded_evt->sm2_levels.lv2 = (p_buf[index] >> 1) & 0x01;
    p_decoded_evt->sm2_levels.lv1 = (p_buf[index] >> 0) & 0x01;
    index++;

    p_decoded_evt->periph_kex.csrk      = (p_buf[index] >> 4) & 0x01;
    p_decoded_evt->periph_kex.address   = (p_buf[index] >> 3) & 0x01;
    p_decoded_evt->periph_kex.irk       = (p_buf[index] >> 2) & 0x01;
    p_decoded_evt->periph_kex.ediv_rand = (p_buf[index] >> 1) & 0x01;
    p_decoded_evt->periph_kex.ltk       = (p_buf[index] >> 0) & 0x01;
    index++;

    p_decoded_evt->central_kex.ltk       = (p_buf[index] >> 4) & 0x01;
    p_decoded_evt->central_kex.ediv_rand = (p_buf[index] >> 3) & 0x01;
    p_decoded_evt->central_kex.irk       = (p_buf[index] >> 2) & 0x01;
    p_decoded_evt->central_kex.address   = (p_buf[index] >> 1) & 0x01;
    p_decoded_evt->central_kex.csrk      = (p_buf[index] >> 0) & 0x01;
    index++;

    uint16_dec(p_buf, packet_len, &index, &p_decoded_evt->periph_keys.enc_info.div);

    SER_ASSERT_LENGTH_LEQ(index + BLE_GAP_SEC_KEY_LEN + 1 +
                          BLE_GAP_SEC_KEY_LEN + 1 + BLE_GAP_ADDR_LEN,
                          packet_len);
    memcpy(&p_decoded_evt->periph_keys.enc_info.ltk[0], &p_buf[index], BLE_GAP_SEC_KEY_LEN);
    index += BLE_GAP_SEC_KEY_LEN;

    p_decoded_evt->periph_keys.enc_info.ltk_len = (p_buf[index] >> 1);
    p_decoded_evt->periph_keys.enc_info.auth    = (p_buf[index] >> 0) & 0x01;
    index++;

    memcpy(&p_decoded_evt->central_keys.irk.irk[0], &p_buf[index], BLE_GAP_SEC_KEY_LEN);
    index += BLE_GAP_SEC_KEY_LEN;

    p_decoded_evt->central_keys.id_info.addr_type = p_buf[index++];

    memcpy(&p_decoded_evt->central_keys.id_info.addr[0], &p_buf[index], BLE_GAP_ADDR_LEN);
    index += BLE_GAP_ADDR_LEN;

    SER_ASSERT_LENGTH_EQ(index, packet_len);

    *p_event_len = event_len;

    return NRF_SUCCESS;
}
