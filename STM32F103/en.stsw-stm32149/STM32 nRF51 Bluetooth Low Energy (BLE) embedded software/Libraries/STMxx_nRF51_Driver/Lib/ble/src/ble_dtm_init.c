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
#include "ble_dtm_app.h"
#include "dtm_rpc.h"
//#include "dtm_uart.h"
#include "nrf_error.h"
#include "ble_serialization.h"


#define MIN_BUFFER_LEN          5
#define RESERVED_FIXED_VALUE    0x00

/** @todo Document
 */
uint32_t ble_dtm_init_req_enc(app_uart_stream_comm_params_t const * const  p_uart_comm_params,
                              uint8_t                             * const  p_buf,
                              uint32_t                            * const p_buf_len)
{
    uint32_t index = 0;
    
    SER_ASSERT_NOT_NULL(p_uart_comm_params);
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_buf_len);
    
    SER_ASSERT_LENGTH_LEQ(MIN_BUFFER_LEN, *p_buf_len);

 //   p_buf[index++] = DTM_OP_CODE_INIT;
 //   p_buf[index++] = RESERVED_FIXED_VALUE;
    p_buf[index++] = p_uart_comm_params->tx_pin_no;
    p_buf[index++] = p_uart_comm_params->rx_pin_no;
    p_buf[index++] = p_uart_comm_params->baud_rate;
    
    *p_buf_len = index;
    return NRF_SUCCESS;
}



uint32_t ble_dtm_init_rsp_dec(uint8_t const * const p_buf, 
                              uint32_t              packet_len,
                              uint32_t * const      p_result_code)
{
    return ser_ble_cmd_rsp_dec(p_buf, packet_len, DTM_OP_CODE_INIT, p_result_code);
}

                              

