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

#include "ble_gap_app.h"
#include <string.h>
#include "ble_serialization.h"
#include "ble_gap.h"
#include "ble_rpc_defines.h"
#include "app_util.h"

#define WHITELIST_ENCODE_LEN(p_whitelist) (1 + ((p_whitelist)->addr_count * (1 + BLE_GAP_ADDR_LEN)) \
                                           + 1 + ((p_whitelist)->irk_count * BLE_GAP_SEC_KEY_LEN))


static uint32_t whitelist_encode(uint8_t *                         p_packet,
                                 ble_gap_whitelist_t const * const p_whitelist)
{
    uint32_t index = 0, i = 0;

    p_packet[index++] = p_whitelist->addr_count;

    for (i = 0; i < p_whitelist->addr_count; i++)
    {
        p_packet[index++] = p_whitelist->pp_addrs[i]->addr_type;
        memcpy(&p_packet[index], &p_whitelist->pp_addrs[i]->addr[0], BLE_GAP_ADDR_LEN);
        index += BLE_GAP_ADDR_LEN;
    }

    p_packet[index++] = p_whitelist->irk_count;

    for (i = 0; i < p_whitelist->irk_count; i++)
    {
        memcpy(&p_packet[index], &p_whitelist->pp_irks[i]->irk[0], BLE_GAP_SEC_KEY_LEN);
        index += BLE_GAP_SEC_KEY_LEN;
    }

    return index;
}


uint32_t ble_gap_adv_start_req_enc(ble_gap_adv_params_t const * const p_adv_params,
                                   uint8_t * const                    p_buf,
                                   uint32_t * const                   p_buf_len)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_buf_len);

    SER_ASSERT_LENGTH_LEQ(index + 2, *p_buf_len);
    p_buf[index++] = SD_BLE_GAP_ADV_START;
    p_buf[index++] = (p_adv_params == NULL) ? RPC_BLE_FIELD_NOT_PRESENT : RPC_BLE_FIELD_PRESENT;

    if (p_adv_params != NULL)
    {
        SER_ASSERT_LENGTH_LEQ(index + 2, *p_buf_len);
        p_buf[index++] = p_adv_params->type;
        p_buf[index++] = (p_adv_params->p_peer_addr != NULL) ?
                         RPC_BLE_FIELD_PRESENT : RPC_BLE_FIELD_NOT_PRESENT;

        if (p_adv_params->p_peer_addr != NULL)
        {
            SER_ASSERT_LENGTH_LEQ(index + 1 + BLE_GAP_ADDR_LEN, *p_buf_len);
            p_buf[index++] = p_adv_params->p_peer_addr->addr_type;
            memcpy(&p_buf[index], &p_adv_params->p_peer_addr->addr[0], BLE_GAP_ADDR_LEN);
            index += BLE_GAP_ADDR_LEN;
        }

        SER_ASSERT_LENGTH_LEQ(index + 2, *p_buf_len);
        p_buf[index++] = p_adv_params->fp;
        p_buf[index++] = (p_adv_params->p_whitelist != NULL) ?
                         RPC_BLE_FIELD_PRESENT : RPC_BLE_FIELD_NOT_PRESENT;

        if (p_adv_params->p_whitelist != NULL)
        {
            ble_gap_whitelist_t * p_whitelist = p_adv_params->p_whitelist;

            SER_ERROR_CHECK(p_whitelist->addr_count <= BLE_GAP_WHITELIST_ADDR_MAX_COUNT,
                            NRF_ERROR_INVALID_PARAM);
            SER_ERROR_CHECK(p_whitelist->irk_count <= BLE_GAP_WHITELIST_IRK_MAX_COUNT,
                            NRF_ERROR_INVALID_PARAM);
            SER_ASSERT_LENGTH_LEQ(index + WHITELIST_ENCODE_LEN(p_whitelist), *p_buf_len);

            index += whitelist_encode(&p_buf[index], p_whitelist);
        }

        SER_ASSERT_LENGTH_LEQ(index + 4, *p_buf_len);
        index += uint16_encode(p_adv_params->interval, &p_buf[index]);
        index += uint16_encode(p_adv_params->timeout, &p_buf[index]);
    }

    *p_buf_len = index;

    return NRF_SUCCESS;
}


uint32_t ble_gap_adv_start_rsp_dec(uint8_t const * const p_buf,
                                   uint32_t              packet_len,
                                   uint32_t * const      p_result_code)
{
    return ser_ble_cmd_rsp_dec(p_buf, packet_len, SD_BLE_GAP_ADV_START, p_result_code);
}
