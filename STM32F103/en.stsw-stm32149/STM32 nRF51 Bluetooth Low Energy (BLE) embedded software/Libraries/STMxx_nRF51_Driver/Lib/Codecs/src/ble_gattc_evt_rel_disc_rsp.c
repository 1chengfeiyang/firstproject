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

#include "ble_gattc_evt_app.h"
#include "ble_gattc_struct_serialization.h"
#include "ble_serialization.h"
#include "app_util.h"

#define BLE_GATTC_EVT_REL_DISC_RSP_COUNT_POSITION 6


uint32_t ble_gattc_evt_rel_disc_rsp_dec(uint8_t const * const p_buf,
                                        uint32_t              packet_len,
                                        ble_evt_t * const     p_event,
                                        uint32_t * const      p_event_len)
{
    uint32_t index         = 0;
    uint32_t event_len     = 0;
    uint16_t include_count = 0;

    uint32_t error_code = NRF_SUCCESS;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_event_len);

    SER_ASSERT_LENGTH_LEQ(8, packet_len);

    include_count = uint16_decode(&p_buf[BLE_GATTC_EVT_REL_DISC_RSP_COUNT_POSITION]);
    event_len     = (uint16_t) (offsetof(ble_evt_t, evt.gattc_evt.params.rel_disc_rsp.includes)) -
                    sizeof (ble_evt_hdr_t) + (include_count * sizeof (ble_gattc_include_t));

    if (p_event == NULL)
    {
        *p_event_len = event_len;
        return NRF_SUCCESS;
    }

    SER_ASSERT(event_len <= *p_event_len, NRF_ERROR_DATA_SIZE);

    p_event->header.evt_id = BLE_GATTC_EVT_REL_DISC_RSP;
    error_code             =
        uint16_t_dec(p_buf, packet_len, &index, &(p_event->evt.gattc_evt.conn_handle));
    SER_ASSERT(error_code == NRF_SUCCESS, error_code);
    error_code = uint16_t_dec(p_buf, packet_len, &index, &(p_event->evt.gattc_evt.gatt_status));
    SER_ASSERT(error_code == NRF_SUCCESS, error_code);
    error_code = uint16_t_dec(p_buf, packet_len, &index, &(p_event->evt.gattc_evt.error_handle));
    SER_ASSERT(error_code == NRF_SUCCESS, error_code);
    error_code =
        ble_gattc_evt_rel_disc_rsp_t_dec(p_buf, packet_len, &index,
                                         &(p_event->evt.gattc_evt.params.rel_disc_rsp));
    SER_ASSERT(error_code == NRF_SUCCESS, error_code);

    SER_ASSERT_LENGTH_EQ(index, packet_len);

    return error_code;
}
