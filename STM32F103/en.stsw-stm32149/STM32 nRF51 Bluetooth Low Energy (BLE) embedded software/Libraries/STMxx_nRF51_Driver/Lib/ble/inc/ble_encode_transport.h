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
 
/**@file
 *
 * @defgroup XXX
 * @{
 * @ingroup  XXX
 *
 * @brief    XXX
 */

#include <stdint.h>

#ifndef BLE_ENCODE_TRANSPORT_H__
#define BLE_ENCODE_TRANSPORT_H__

/**@brief @ref ble_encode_transport_cmd_write API command mode definitions. */
typedef enum
{
    BLE_ENCODE_WRITE_MODE_RESP,     /**< Command written includes a corresponding command response. */
    BLE_ENCODE_WRITE_MODE_NO_RESP,  /**< Command written does not include a corresponding command response. */
    BLE_ENCODE_WRITE_MODE_MAX       /**< Enumeration upper bound. */      
} ble_encode_cmd_write_mode_t;

/**@brief Command response callback function type.
 *
 * Callback function for decoding the possible command response output parameteres and copying them 
 * to application and decoding the command response return code.
 *
 * @param[in] p_buffer                  Pointer to begin of command response buffer.
 * @param[in] length                    Length of data in bytes.  
 *
 * @return Decoded command response return code.
 */
typedef uint32_t (*ble_command_resp_decode_callback_t)(const uint8_t * p_buffer, uint32_t length);

/**@brief Function for allocating TX command memory.
 * 
 * @note If memory can't be acquired error check is executed as implies an error as not allowed by 
 *       the system design and must be fixed at compile time.
 * 
 * @return Pointer to the begin of the buffer allocated.
 */
uint8_t * ble_encode_transport_tx_alloc(void);

/**@brief Function for freeing TX command memory.
 *
 * @note Memory management works in FIFO principle meaning that free order must match the alloc 
 *       order. 
 * @note Does nothing if no TX command memory to be freed exists. 
 */
void ble_encode_transport_tx_free(void);

/**@brief Function for writing a command.
 *
 * @note Attempt to have multiple commands to be in progress at same time will lead to error check 
 *       as not allowed by the system design.
 * @note Error check is executed for validating @ref p_buffer against NULL.
 * @note Error check is executed for validating @ref length against 0. 
 * @note Error check is executed for validating @ref cmd_write_mode.  
 * @note Error check is executed for validating @ref cmd_resp_decode_callback against NULL.  
 * 
 * @param[in] p_buffer                  Pointer to begin of command buffer.
 * @param[in] length                    Length of data in bytes.  
 * @param[in] cmd_write_mode            Command write mode see @ref 
 *                                      ble_command_resp_decode_callback_t for details.
 * @param[in] cmd_resp_decode_callback  Command response callback.   
 */
void ble_encode_transport_cmd_write(const uint8_t *                    p_buffer, 
                                    uint32_t                           length, 
                                    ble_encode_cmd_write_mode_t        cmd_write_mode,
                                    ble_command_resp_decode_callback_t cmd_resp_decode_callback);

#endif // BLE_ENCODE_TRANSPORT_H__

/** @} */
