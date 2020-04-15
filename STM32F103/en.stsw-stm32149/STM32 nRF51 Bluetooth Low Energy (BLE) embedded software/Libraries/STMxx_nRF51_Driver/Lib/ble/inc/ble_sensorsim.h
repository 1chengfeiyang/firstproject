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
 * @defgroup ble_sdk_lib_sensorsim Sensor Data Simulator
 * @{
 * @ingroup ble_sdk_lib
 * @brief Functions for simulating sensor data.
 *
 * @details Currently only a triangular waveform simulator is implemented.
 */

#ifndef BLE_SENSORSIM_H__
#define BLE_SENSORSIM_H__

#include <stdint.h>
#include <stdbool.h>

/**@brief Triangular waveform sensor simulator configuration. */
typedef struct
{
    uint32_t min;                       /**< Minimum simulated value. */
    uint32_t max;                       /**< Maximum simulated value. */
    uint32_t incr;                      /**< Increment between each measurement. */
    bool     start_at_max;              /**< TRUE is measurement is to start at the maximum value, FALSE if it is to start at the minimum. */
} ble_sensorsim_cfg_t;

/**@brief Triangular waveform sensor simulator state. */
typedef struct
{
    uint32_t current_val;               /**< Current sensor value. */
    bool     is_increasing;             /**< TRUE if the simulator is in increasing state, FALSE otherwise. */
} ble_sensorsim_state_t;

/**@brief Function for initializing a triangular waveform sensor simulator.
 *
 * @param[out]  p_state  Current state of simulator.
 * @param[in]   p_cfg    Simulator configuration.
 */
void ble_sensorsim_init(ble_sensorsim_state_t *     p_state, 
                        const ble_sensorsim_cfg_t * p_cfg);

/**@brief Function for generating a simulated sensor measurement using a triangular waveform generator.
 *
 * @param[in,out]  p_state  Current state of simulator.
 * @param[in]      p_cfg    Simulator configuration.
 *
 * @return         Simulator output.
 */
uint32_t ble_sensorsim_measure(ble_sensorsim_state_t *     p_state,
                               const ble_sensorsim_cfg_t * p_cfg);

#endif // BLE_SENSORSIM_H__

/** @} */
