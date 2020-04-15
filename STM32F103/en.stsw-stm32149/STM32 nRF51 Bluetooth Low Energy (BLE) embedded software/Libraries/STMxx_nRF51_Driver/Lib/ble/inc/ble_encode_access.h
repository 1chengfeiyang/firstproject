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

#ifndef BLE_ENCODE_ACCESS_H__
#define BLE_ENCODE_ACCESS_H__

#include <stdint.h>
#include "ble.h"

/**@brief Command result callback function type.
 *
 * @param[in] result_code               Command result code.  
 */
typedef void (*ble_encode_cmd_resp_handler_t)(uint32_t result_code);

/**@brief Generic event callback function events. */
typedef enum
{
    BLE_ENCODE_EVT_RDY  /**< An event indicating that SoftDevice event is available for read. */
} ble_encode_evt_type_t;

/**@brief Event callback function type.
 *
 * @param[in] event                     The event occurred.  
 */
typedef void (*ble_encode_event_handler_t)(ble_encode_evt_type_t event);
 
/**@brief Function for opening the module.
 *
 * @warning Must not be called for a module which has been allready opened. 
 * 
 * @retval NRF_SUCCESS          Operation success.
 * @retval NRF_ERROR_INTERNAL   Operation failure. Internal error ocurred. 
 */
uint32_t ble_encode_open(void);

/**@brief Function for closing the module and resetting its internal state.
 *
 * @note Can be called multiple times and also for not opened module.
 * 
 * @retval NRF_SUCCESS          Operation success.  
 */
uint32_t ble_encode_close(void);

/**@brief Function for registering a BLE command response callback handler.
 *
 * @param[in] command_resp_handler      BLE command response handler.  
 * 
 * @retval NRF_SUCCESS          Operation success.  
 * @retval NRF_ERROR_BUSY       Operation failure. Command allready in progress.
 * @retval NRF_ERROR_NULL       Operation failure. NULL pointer supplied.     
 */
uint32_t ble_encode_cmd_resp_handler_reg(ble_encode_cmd_resp_handler_t command_resp_handler);

/**@brief Function for registering a generic event handler.
 *
 * @note Multiple registration requests will overwrite any possible existing registration. 
 *
 * @param[in] event_handler             The function to be called upon an event.
 *
 * @retval NRF_SUCCESS          Operation success.
 * @retval NRF_ERROR_NULL       Operation failure. NULL pointer supplied.    
 */
uint32_t ble_encode_evt_handler_register(ble_encode_event_handler_t event_handler);

/**@brief Function for extracting an received BLE event.
 *
 * If @ref p_event is NULL, the required length of @ref p_event is returned in @ref p_event_len.
 *
 * @param[out] p_event                  Pointer to memory where event specific data is copied. If 
 *                                      NULL, required length will be returned in @ref p_event_len.
 * @param[in,out] p_event_len           in: Size (in bytes) of @ref p_event buffer. 
 *                                      out: Length of decoded contents of @ref p_event. 
 *
 * @retval NRF_SUCCESS          Operation success. Event was copied to @ref p_event or in case @ref 
 *                              p_event is NULL @ref p_event_len is updated.
 * @retval NRF_ERROR_NO_MEM     Operation failure. No event available to extract.
 * @retval NRF_ERROR_DATA_SIZE  Operation failure. Length of @ref p_event is too small to hold 
 *                              decoded event. 
 * @retval NRF_ERROR_NULL       Operation failure. NULL pointer supplied.    
 */
uint32_t ble_encode_event_pop(ble_evt_t * p_event, uint32_t * p_event_len);

#endif // BLE_ENCODE_ACCESS_H__

/** @} */
