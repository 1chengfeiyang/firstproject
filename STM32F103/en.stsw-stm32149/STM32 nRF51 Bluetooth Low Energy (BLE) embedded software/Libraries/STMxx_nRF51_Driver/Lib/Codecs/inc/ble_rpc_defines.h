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
 * @defgroup rpc_cmd_defines Defines related to serialized BLE commands.
 * @{
 * @ingroup ble_sdk_lib
 *
 * @brief Defines for serialized BLE commands.
 *
 */
 
#ifndef BLE_RPC_DEFINES_H__
#define BLE_RPC_DEFINES_H__

#define RPC_PKT_TYPE_POS                        0                      /**< Position of Packet type in the serialized packet buffer.*/
#define RPC_PKT_OP_CODE_POS                     1                      /**< Position of Op Code in the serialized packet buffer.*/

#define RPC_CMD_OP_CODE_POS                     0                      /**< Position of the Op Code in the command buffer.*/
#define RPC_CMD_DATA_POS                        1                      /**< Position of the data in the command buffer.*/

#define RPC_CMD_RESP_PKT_TYPE_POS               0                      /**< Position of Packet type  in the command response buffer.*/
#define RPC_CMD_RESP_OP_CODE_POS                1                      /**< Position of the Op Code in the command response buffer.*/
#define RPC_CMD_RESP_STATUS_POS                 2                      /**< Position of the status field in the command response buffer.*/

#define RPC_DTM_CMD_OP_CODE_POS                 0                      /**< Position of the Op Code in the DTM command buffer.*/
#define RPC_DTM_DATA_POS                        1                      /**< Position of the data in the DTM command buffer.*/

#define RPC_DTM_RESP_OP_CODE_POS                1                      /**< Position of the Op Code in the DTM command response buffer.*/
#define RPC_DTM_RESP_STATUS_POS                 2                      /**< Position of the status field in the DTM command response buffer.*/

#define RPC_BLE_FIELD_LEN                       1                      /**< Optional field length size in bytes. */
#define RPC_BLE_FIELD_PRESENT                   0x01                   /**< Value to indicate that an optional field is encoded in the serialized packet, e.g. white list. */
#define RPC_BLE_FIELD_NOT_PRESENT               0x00                   /**< Value to indicate that an optional field is not encoded in the serialized packet. */

#define RPC_ERR_CODE_SIZE                       4                      /**< BLE API err_code size in bytes. */
#define BLE_PKT_TYPE_SIZE                       1                      /**< Packet type (@ref ble_rpc_pkt_type_t) field size in bytes. */
#define BLE_OP_CODE_SIZE                        1                      /**< Operation code field size in bytes. */

#define RPC_BLE_CMD_RESP_PKT_MIN_SIZE           6                      /**< Minimum length of a command response. */
#define RPC_BLE_PKT_MAX_SIZE                    596                    /**< Maximum size for a BLE packet on the HCI Transport layer. This value is the hci_mem_pool buffer size minus the HCI Transport size. @note This value must be aligned with TX_BUF_SIZE in hci_mem_pool_internal.h. */

/**@brief The types of packets. */
typedef enum
{
    BLE_RPC_PKT_CMD,                                                   /**< Command packet type. */
    BLE_RPC_PKT_RESP,                                                  /**< Command Response packet type. */
    BLE_RPC_PKT_EVT,                                                   /**< Event packet type. */
    BLE_RPC_PKT_DTM_CMD,                                               /**< DTM Command packet type. */
    BLE_RPC_PKT_DTM_RESP,                                              /**< DTM Response packet type. */
    BLE_RPC_PKT_TYPE_MAX                                               /**< Upper bound. */
} ble_rpc_pkt_type_t;

#endif // BLE_RPC_DEFINES_H__

/** @} */
