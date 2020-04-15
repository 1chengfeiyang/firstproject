/**
  ******************************************************************************
  * @file    ble_ledbutton.c
  * @author  MCD Application Team
  * @version V1.0
  * @date    26-August-2014
  * @brief   Main body
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

#include "ble_ledbutton.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "ble_util.h"
#include "blocking.h"
#include "app_error.h"


#define OPCODE_LENGTH  1                                                    /**< Length of opcode  */
#define HANDLE_LENGTH  2                                                    /**< Length of handle . */
#define MAX_BUTT_LEN    (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted  Measurement. */



/*BUTTON SERVICE MANAGEMENT */
/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_but       but Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_but_t * p_but, ble_evt_t * p_ble_evt)
{
    p_but->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_hrs       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_but_t * p_but, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_but->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the button characteristic.
 *
 * @param[in]   p_but         button Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_but_cccd_write(ble_but_t * p_but, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_but->evt_handler != NULL)
        {
            ble_but_evt_t evt;
            
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_BUT_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_BUT_EVT_NOTIFICATION_DISABLED;
            }
            
            p_but->evt_handler(p_but, &evt);
        }
    }
	
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_hrs       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write_but(ble_but_t * p_but, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if (p_evt_write->handle == p_but->but_handles.cccd_handle)
    {
        on_but_cccd_write(p_but, p_evt_write);
    }
}


void ble_but_on_ble_evt(ble_but_t * p_but, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_but, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_but, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write_but(p_but, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}




/**@brief Function for adding the Button Id characteristic.
 *
 * @param[in]   p_but        Button Service structure.
 * @param[in]   p_but_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t button_id_char_add(ble_but_t *            p_but,
                                                const ble_but_init_t * p_but_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
        
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_but_init->but_attr_md.cccd_write_perm;
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BUTTON_CHAR);
    
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_but_init->but_attr_md.read_perm;
    attr_md.write_perm = p_but_init->but_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 1;    //hrm_encode(p_hrs, INITIAL_VALUE_HRM, encoded_initial_hrm);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_BUTT_LEN;
    attr_char_value.p_value      = 0;//encoded_initial_hrm;
    
    return sd_ble_gatts_characteristic_add(p_but->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_but->but_handles);
}

uint32_t ble_but_init(ble_but_t * p_but, const ble_but_init_t * p_but_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_but->evt_handler                 = p_but_init->evt_handler;
    p_but->conn_handle                 = BLE_CONN_HANDLE_INVALID;
        
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BUTTON_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
                                        &ble_uuid, 
                                        &p_but->service_handle);
    APP_ERROR_CHECK(err_code);
    
    err_code = blocking_resp_wait();                                        
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add heart rate measurement characteristic
    err_code = button_id_char_add(p_but, p_but_init);
    APP_ERROR_CHECK(err_code);
    err_code = blocking_resp_wait();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    
    
    return NRF_SUCCESS;
}


uint32_t ble_button_id_pushed_send(ble_but_t * p_but, uint8_t button_id)
{
    uint32_t err_code;
    
    // Send value if connected and notifying
    if (p_but->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_but[MAX_BUTT_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;
        
			
			  encoded_but[0]=button_id;
        len     = 1;//hrm_encode(p_hrs, heart_rate, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));
        
        hvx_params.handle   = p_but->but_handles.value_handle;
        hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;
        hvx_params.p_len    = &hvx_len;
        hvx_params.p_data   = encoded_but;
        
        err_code = sd_ble_gatts_hvx(p_but->conn_handle, &hvx_params);
        APP_ERROR_CHECK(err_code);
        
        err_code = blocking_resp_wait();        
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
  //          err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

/******************************************************/
/* LED IMPLEMENTATION                                **/
/******************************************************/
/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_led       Led Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect_led(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
    p_led->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_led       led Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect_led(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
    ble_led_evt_t evt; 
	  UNUSED_PARAMETER(p_ble_evt);
    p_led->conn_handle = BLE_CONN_HANDLE_INVALID;
	  // Alert level written, call application event handler
        
        
        evt.evt_type           = BLE_LED_EVT_SIGNAL_VALUE_UPDATED;
        evt.params.led_signal = INITIAL_LED_SIGNAL;

        p_led->evt_handler(p_led, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_led       lED Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write_led(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
   
    if ((p_evt_write->handle == p_led->led_signal_handles.value_handle) && (p_evt_write->len == 1))
    {
        // Alert level written, call application event handler
        ble_led_evt_t evt;
        
        evt.evt_type           = BLE_LED_EVT_SIGNAL_VALUE_UPDATED;
        evt.params.led_signal = p_evt_write->data[0];

        p_led->evt_handler(p_led, &evt);
    }
	
}


void ble_led_on_ble_evt(ble_led_t * p_led, ble_evt_t * p_ble_evt)
{
         switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect_led(p_led, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect_led(p_led, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write_led(p_led, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }  
	
	
}


/**@brief Function for adding Alert Level characteristics.
 *
 * @param[in]   p_ias        Immediate Alert Service structure.
 * @param[in]   p_ias_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t led_signal_char_add(ble_led_t * p_led)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_led_signal;
    
    memset(&char_md, 0, sizeof(char_md));
    
   // char_md.char_props.write_wo_resp = 1;
    char_md.char_props.write = 1;
	  char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;
    
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_LED_CHAR);
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
 //   BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.write_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    initial_led_signal = INITIAL_LED_SIGNAL;
    
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = &initial_led_signal;
    
    return sd_ble_gatts_characteristic_add(p_led->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_led->led_signal_handles);
    
}

uint32_t ble_led_init(ble_led_t * p_led, const ble_led_init_t * p_led_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    if (p_led_init->evt_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    p_led->evt_handler = p_led_init->evt_handler;
    p_led->conn_handle = BLE_CONN_HANDLE_INVALID;
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_LED_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_led->service_handle);
    APP_ERROR_CHECK(err_code);
    
    err_code = blocking_resp_wait();    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
		//ADD Led Signal Characteristics
		err_code = led_signal_char_add(p_led);
		 APP_ERROR_CHECK(err_code);
        err_code = blocking_resp_wait();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
				
		return NRF_SUCCESS;
		    
}

















