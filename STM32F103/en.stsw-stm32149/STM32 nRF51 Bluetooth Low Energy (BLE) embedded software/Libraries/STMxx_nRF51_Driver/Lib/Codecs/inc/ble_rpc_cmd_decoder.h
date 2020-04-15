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

/** @file
 *
 * @defgroup ble_rpc_cmd_decoder Command Decoder
 * @{
 * @ingroup ble_sdk_lib_serialization
 *
 * @brief   Decoder for serialized commands from Application Chip.
 *
 * @details This file contains declaration of common functions used for sending responses back to
 *          Application Chip after the command is processed, and function for processing commands
 *          received by the transport layer.
 */

#ifndef BLE_RPC_CMD_DECODER_H__
#define BLE_RPC_CMD_DECODER_H__

#include <stdint.h>

#define RPC_DECODER_LENGTH_CHECK(LEN, INDEX, CMD) if ( INDEX > LEN) \
        return ble_rpc_cmd_resp_send(CMD, NRF_ERROR_INVALID_LENGTH);

/**@brief Function for sending a Command Response packet to the Application Chip through the transport
 *        layer.
 *
 * @param[in] op_code          The op code of the command for which the Command Response is sent.
 * @param[in] status           The status field to be encoded into the Command Response.
 *
 * @retval NRF_SUCCESS         On successful write of Command Response, otherwise an error code.
 *                             If the transport layer returns an error code while sending
 *                             the Command Response, the same error code will be returned by this
 *                             function (see @ref hci_transport_pkt_write for the list of
 *                             error codes).
 */
uint32_t ble_rpc_cmd_resp_send(uint8_t op_code, uint32_t status);

/**@brief Function for sending a command response with additional data to the Application Chip through
 *        the transport layer.
 *
 * @param[in]   op_code        The op code of the command for which the Command Response is sent.
 * @param[in]   status         The status field to be encoded into the Command Response.
 * @param[in]   p_data         The data to be sent along with the status.
 * @param[in]   data_len       The length of the additional data.
 *
 * @retval      NRF_SUCCESS    On successful write of Command Response, otherwise an error code.
 *                             If the transport layer returns an error code while sending
 *                             the Command Response, the same error code will be returned by this
 *                             function (see @ref hci_transport_pkt_write for the list of
 *                             error codes).
 */
uint32_t ble_rpc_cmd_resp_data_send(uint8_t               op_code,
                                    uint8_t               status,
                                    const uint8_t * const p_data,
                                    uint16_t              data_len);

/**@brief Function for scheduling an RPC command event to be processed in main-thread.
 *
 * @details     The function will read the arrived packet from the transport layer
 *              which is passed for decoding by the rpc_cmd_decoder module.
 *
 * @param[in]   p_event_data   Event data. This will be NULL as rpc_evt_schedule
 *                             does not set any data.
 * @param[in]   event_size     Event data size. This will be 0 as rpc_evt_schedule
 *                             does not set any data.
 */
void ble_rpc_cmd_handle(void * p_event_data, uint16_t event_size);

#endif // BLE_RPC_CMD_DECODER_H__

/** @} */
