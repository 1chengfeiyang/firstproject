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
 * @defgroup XXXX
 * @{
 * @ingroup  YYYY
 *
 * @brief    ZZZZZ.
 */

#ifndef BLE_TIMER_H__
#define BLE_TIMER_H__

#include <stdint.h>

/**@brief Timer id type. */
typedef uint32_t ble_timer_id_t;

/**@brief BLE timeout handler type. */
typedef void (*ble_timer_timeout_handler_t)(void * p_context);

/**@brief Timer modes. */
typedef enum
{
    BLE_TIMER_MODE_SINGLE_SHOT, /**< The timer will expire only once. */
    BLE_TIMER_MODE_REPEATED     /**< The timer will restart each time it expires. */
} ble_timer_mode_t;

/**@brief Function for creating a timer instance.
 *
 * @param[out] p_timer_id       Id of the newly created timer.
 * @param[in]  mode             Timer mode.
 * @param[in]  timeout_handler  Function to be executed when the timer expires.
 *
 * @retval     NRF_SUCCESS               Timer was successfully created.
 * @retval     NRF_ERROR_INVALID_PARAM   Invalid parameter.
 * @retval     NRF_ERROR_INVALID_STATE   Timer module has not been initialized.
 * @retval     NRF_ERROR_NO_MEM          Maximum number of timers has already been reached.
 */
uint32_t ble_timer_create(ble_timer_id_t *            p_timer_id,
                          ble_timer_mode_t            mode,
                          ble_timer_timeout_handler_t timeout_handler);

/**@brief Function for starting a timer.
 *
 * @param[in]  timer_id         Id of timer to start.
 * @param[in]  timeout_ms       Number of millseconds to timeout event
 * @param[in]  p_context        General purpose pointer. Will be passed to the timeout handler when
 *                              the timer expires. 
 *
 * @retval     NRF_SUCCESS               Timer was successfully started.
 * @retval     NRF_ERROR_INVALID_PARAM   Invalid parameter.
 * @retval     NRF_ERROR_INVALID_STATE   Timer module has not been initialized, or timer has not 
 *                                       been created.
 * @retval     NRF_ERROR_NO_MEM          Timer operations queue was full.
 *
 * @note When calling this method on a timer which is already running, the second start operation
 *       will be ignored.
 */
uint32_t ble_timer_start(ble_timer_id_t timer_id, uint32_t timeout_ms, void * p_context);

/**@brief Function for stopping the specified timer.
 *
 * @param[in]  timer_id         Id of timer to stop.
 *
 * @retval     NRF_SUCCESS               Timer was successfully stopped.
 * @retval     NRF_ERROR_INVALID_PARAM   Invalid parameter.
 * @retval     NRF_ERROR_INVALID_STATE   Timer module has not been initialized, or timer has not 
 *                                       been created.
 * @retval     NRF_ERROR_NO_MEM          Timer operations queue was full.
 */
uint32_t ble_timer_stop(ble_timer_id_t timer_id);

#endif // BLE_TIMER_H__

/** @} */
