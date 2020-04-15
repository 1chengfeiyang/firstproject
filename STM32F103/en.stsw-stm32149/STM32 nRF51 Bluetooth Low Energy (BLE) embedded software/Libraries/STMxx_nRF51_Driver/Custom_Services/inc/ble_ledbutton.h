/**
  ******************************************************************************
  * @file    ble_ledbutton.h
  * @author  MCD Application Team
  * @version V1.0
  * @date    26-August-2014
  * @brief   Header file for ble_ledbutton.c module
  *
  * @note	The application must send BLE stack events to the LED BUTTON Service
  * 		It is not specified  by the SIG
  *       	It calls ble_ledbs_on_ble_evt() from the from the ble_stack_handler function.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/*
 *
 * @note The application must propagate BLE stack events to the LED BUTTON Service - NOT SIG but ST Created Service
 *       module by calling ble_ledbs_on_ble_evt() from the from the @ref ble_stack_handler function.
 *      This service is an experiementation for LED BUTTON Service:
 *					
 */

#ifndef BLE_LEDBUTTON_H__
#define BLE_LEDBUTTON_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"

#define BLE_UUID_LED_SERVICE                 				0x1906     /**< LED BUTTON service UUID. */
#define BLE_UUID_LED_CHAR    											  0x2B20     /**< LED1 characteristic UUID. */

#define BLE_UUID_BUTTON_SERVICE                 		0x1907     /**< BUTTON service UUID. */
#define BLE_UUID_BUTTON_CHAR    										0x2B23     /**< BUTTON characteristic UUID. */

#define LED_NO_SIGNAL																0x00
#define INITIAL_LED_SIGNAL													LED_NO_SIGNAL



/* BUTTON DECLARATION*/
/**@brief Button Service event type. */
typedef enum
{
    BLE_BUT_EVT_NOTIFICATION_ENABLED,                   /**< Button value notification enabled event. */
    BLE_BUT_EVT_NOTIFICATION_DISABLED                   /**< Button value notification disabled event. */
} ble_but_evt_type_t;

/**@brief Button Service event. */
typedef struct
{
    ble_but_evt_type_t evt_type;                        /**< Type of event. */
} ble_but_evt_t;

// Forward declaration of the ble_hrs_t type. 
typedef struct ble_but_s ble_but_t;

/**@brief Button Service event handler type. */
typedef void (*ble_but_evt_handler_t) (ble_but_t * p_but, ble_but_evt_t * p_evt);

/**@brief Button Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_but_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Button Service. */
    bool                         is_sensor_contact_supported;                          /**< Determines if sensor contact detection is to be supported. */
    ble_srv_cccd_security_mode_t but_attr_md;                                          /**< Initial security level for button service  attribute */
 } ble_but_init_t;

/**@brief Button Service structure. This contains various status information for the service. */
typedef struct ble_but_s
{
    ble_but_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     but_handles;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_but_t;

/**@brief Function for initializing the Button Service.
 *
 * @param[out]  p_but       Button Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_but_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_but_init(ble_but_t * p_hrs, const ble_but_init_t * p_but_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Button Service.
 *
 * @param[in]   p_button      Button Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_but_on_ble_evt(ble_but_t * p_but, ble_evt_t * p_ble_evt);

/**@brief Function for sending button push measurement if notification has been enabled.
 *
 * @details The application calls this function once a button is pushed only 
 *          If notification has been enabled, 
 *
 * @param[in]   p_but                    Heart Rate Service structure.
 * @param[in]   Button_Flag               New heart rate measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_button_id_pushed_send(ble_but_t * p_but, uint8_t button_id);

/***********************/
/* LED DECLARATION*/
/**@brief LED Service event type. */
typedef enum
{
    BLE_LED_EVT_SIGNAL_VALUE_UPDATED                     /**< LED SiGNAL VALUE Updated event. */
} ble_led_evt_type_t;

/**@brief LED Service event. */
typedef struct
{
    ble_led_evt_type_t evt_type;                        /**< Type of event. */
    union
    {
        uint8_t led_signal;                            /**< New led signal value. */
    } params;
} ble_led_evt_t;

// Forward declaration of the ble_led_t type. 
typedef struct ble_led_s ble_led_t;

/**@brief Led  Service event handler type. */
typedef void (*ble_led_evt_handler_t) (ble_led_t * p_ias, ble_led_evt_t * p_evt);

/**@brief Led Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_led_evt_handler_t evt_handler;                  /**< Event handler to be called for handling events Led Service. */
} ble_led_init_t;

/**@brief LED Service structure. This contains various status information for the
 *        service. */
typedef struct ble_led_s
{
    ble_led_evt_handler_t     evt_handler;              /**< Event handler to be called for handling events in the Led Service. */
    uint16_t                  service_handle;           /**< Handle of Immediate Alert Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t  led_signal_handles;      /**< Handles related to the led signal characteristic. */
	  uint16_t                  conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_led_t;

/**@brief Function for initializing the Immediate Alert Service.
 *
 * @param[out]  p_led       Led Service structure. This structure will have to be
 *                          supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_led_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_led_init(ble_led_t * p_led, const ble_led_init_t * p_led_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Immediate Alert Service.
 *
 * @param[in]   p_led      Led Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_led_on_ble_evt(ble_led_t * p_led, ble_evt_t * p_ble_evt);

#endif // BLE_LEDBUTTON_H__

/** @} */
