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

#include "ble_struct_serialization.h"
#include "ble_serialization.h"
#include "app_util.h"
#include "ble_types.h"
#include "ble_l2cap.h"
#include "ble.h"
#include "cond_field_serialization.h"
#include <string.h>


uint32_t ble_uuid_t_enc(void const * const p_void_uuid,
                        uint8_t * const    p_buf,
                        uint32_t           buf_len,
                        uint32_t * const   p_index)
{
    ble_uuid_t * p_uuid   = (ble_uuid_t *)p_void_uuid;
    uint32_t     err_code = NRF_SUCCESS;

    err_code = uint16_t_enc(&p_uuid->uuid, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint8_t_enc(&p_uuid->type, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_uuid_t_dec(uint8_t const * const p_buf,
                        uint32_t              buf_len,
                        uint32_t * const      p_index,
                        void * const          p_void_uuid)
{
    ble_uuid_t * p_uuid = (ble_uuid_t *)p_void_uuid;

    SER_ASSERT_LENGTH_LEQ(3, buf_len - *p_index);
    uint16_dec(p_buf, buf_len, p_index, &p_uuid->uuid);
    uint8_dec(p_buf, buf_len, p_index, &p_uuid->type);

    return NRF_SUCCESS;
}

uint32_t ble_uuid128_t_enc(void const * const p_void_uuid,
                           uint8_t * const    p_buf,
                           uint32_t           buf_len,
                           uint32_t * const   p_index)
{
    ble_uuid128_t * p_uuid   = (ble_uuid128_t *)p_void_uuid;
    uint32_t        err_code = NRF_SUCCESS;

    SER_ASSERT_LENGTH_LEQ(16, buf_len - *p_index);

    memcpy(&p_buf[*p_index], p_uuid->uuid128, sizeof (p_uuid->uuid128));

    *p_index += sizeof (p_uuid->uuid128);

    return err_code;
}

uint32_t ble_uuid128_t_dec(uint8_t const * const p_buf,
                           uint32_t              buf_len,
                           uint32_t * const      p_index,
                           void * const          p_void_uuid)
{
    ble_uuid128_t * p_uuid   = (ble_uuid128_t *)p_void_uuid;
    uint32_t        err_code = NRF_SUCCESS;

    SER_ASSERT_LENGTH_LEQ(16, buf_len - *p_index);

    memcpy(p_uuid->uuid128, &p_buf[*p_index], sizeof (p_uuid->uuid128));

    *p_index += sizeof (p_uuid->uuid128);

    return err_code;
}

uint32_t ble_l2cap_header_t_enc(void const * const p_void_header,
                                uint8_t * const    p_buf,
                                uint32_t           buf_len,
                                uint32_t * const   p_index)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);
    SER_ASSERT_NOT_NULL(p_void_header);

    ble_l2cap_header_t * p_header = (ble_l2cap_header_t *)p_void_header;
    uint32_t err_code = NRF_SUCCESS;

    err_code = uint16_t_enc(&(p_header->len), p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_enc(&(p_header->cid), p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_l2cap_header_t_dec(uint8_t const * const p_buf,
                                uint32_t              buf_len,
                                uint32_t * const      p_index,
                                void * const          p_void_header)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);
    SER_ASSERT_NOT_NULL(p_void_header);

    ble_l2cap_header_t * p_header = (ble_l2cap_header_t *)p_void_header;
    uint32_t err_code = NRF_SUCCESS;

    err_code = uint16_t_dec(p_buf, buf_len, p_index, &(p_header->len));
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_dec(p_buf, buf_len, p_index, &(p_header->cid));
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_l2cap_evt_rx_t_enc(void const * const p_void_evt_rx,
                                uint8_t * const    p_buf,
                                uint32_t           buf_len,
                                uint32_t * const   p_index)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);
    SER_ASSERT_NOT_NULL(p_void_evt_rx);

    ble_l2cap_evt_rx_t * p_evt_rx = (ble_l2cap_evt_rx_t *)p_void_evt_rx;
    uint32_t err_code = NRF_SUCCESS;

    err_code = ble_l2cap_header_t_enc(&(p_evt_rx->header), p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    SER_ASSERT_LENGTH_LEQ(p_evt_rx->header.len, buf_len - *p_index);
    memcpy(&p_buf[*p_index], p_evt_rx->data, p_evt_rx->header.len);
    *p_index += p_evt_rx->header.len;

    return err_code;
}

uint32_t ble_l2cap_evt_rx_t_dec(uint8_t const * const p_buf,
                                uint32_t              buf_len,
                                uint32_t * const      p_index,
                                uint32_t * const      p_struct_len,
                                void * const          p_void_evt_rx)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);
    SER_ASSERT_NOT_NULL(p_struct_len);

    ble_l2cap_evt_rx_t * p_evt_rx = (ble_l2cap_evt_rx_t *)p_void_evt_rx;
    uint32_t err_code = NRF_SUCCESS;

    uint32_t total_struct_len = *p_struct_len;

    /* Get data length */
    uint32_t tmp_index = *p_index;
    uint16_t len       = 0;

    err_code = uint16_t_dec(p_buf, buf_len, &tmp_index, &len);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    /* Update struct length */
    *p_struct_len = offsetof(ble_l2cap_evt_rx_t, data[0]);
    *p_struct_len += (uint8_t*)&(p_evt_rx->data[len]) - (uint8_t*)&(p_evt_rx->data[0]);

    /* Decode header and copy data */
    if (p_void_evt_rx != NULL)
    {
        SER_ASSERT_LENGTH_LEQ(*p_struct_len, total_struct_len);

        err_code = ble_l2cap_header_t_dec(p_buf, buf_len, p_index, &(p_evt_rx->header));
        SER_ASSERT(err_code == NRF_SUCCESS, err_code);

        SER_ASSERT_LENGTH_LEQ(p_evt_rx->header.len, buf_len - *p_index);
        memcpy(p_evt_rx->data, &p_buf[*p_index], p_evt_rx->header.len);
        *p_index += p_evt_rx->header.len;
    }

    return err_code;
}

uint32_t ble_enable_params_t_enc(void const * const p_data,
                                 uint8_t * const    p_buf,
                                 uint32_t           buf_len,
                                 uint32_t * const   p_index)
{
    ble_enable_params_t * p_enable_params = (ble_enable_params_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);

    p_buf[*p_index] = p_enable_params->gatts_enable_params.service_changed;
    (*p_index)++;

    return NRF_SUCCESS;
}

uint32_t ble_enable_params_t_dec(uint8_t const * const p_buf,
                                 uint32_t              buf_len,
                                 uint32_t * const      p_index,
                                 void * const          p_data)
{
    ble_enable_params_t * p_enable_params = (ble_enable_params_t *) p_data;

    SER_ASSERT_LENGTH_LEQ(sizeof (ble_enable_params_t), buf_len - *p_index);
    p_enable_params->gatts_enable_params.service_changed = p_buf[(*p_index)++];

    return NRF_SUCCESS;
}
