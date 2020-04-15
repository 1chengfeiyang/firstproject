/**
  ******************************************************************************
  * @file    ble_app_main.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    29-August-2014
  * @brief   Main body for BLE app
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

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32l1xx.h"
#include "main.h"
#include "ble_types.h"
#include "ble_gap.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_hts.h"
#include "ble_ledbutton.h"
#include "ble_sensorsim.h"
#include "nordic_common.h"
#include "blocking.h"
#include "ble_advdata.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_bondmngr.h"
#include "pstorage.h"
#include "hal_timer.h"
#include "ble_encode_access.h"
#include "app_error.h"
#include "ble_nrf_soc.h"
#include "stm32_adafruit_spi_lcd.h"

#if(APP_L2CAP_TX_TEST == 1)
#include "ble_l2cap.h"
#endif

/* Private defines -----------------------------------------------------------*/
#define IS_SRVC_CHANGED_CHARACT_PRESENT		0                                           	/**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#if ((APP_HRS == 1) && (APP_HTS == 1))
#define DEVICE_NAME                          "HRM_HTS"										/**< Name of device. Will be included in the advertising data. */
#endif
#if ((APP_HRS == 1) && (APP_HTS == 0))
#define DEVICE_NAME                          "STM32_HRM"									/**< Name of device. Will be included in the advertising data. */
#endif
#if ((APP_HRS == 0) && (APP_HTS == 1))
#define DEVICE_NAME                          "STM32_HTS"									/**< Name of device. Will be included in the advertising data. */
#endif
#if ((APP_HRS == 0) && (APP_HTS == 0) && (APP_LEDBUTTON == 0))
#define DEVICE_NAME                          "STM32_ADV"									/**< Name of device. Will be included in the advertising data. */
#endif

#if ((APP_HRS == 0) && (APP_HTS == 0) && (APP_LEDBUTTON == 1))
#define DEVICE_NAME                          "LED_BUTT"										/**< Name of device. Will be included in the advertising data. */
#endif

#define MANUFACTURER_NAME                    "STMicroelectronics"                  			/**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                     40                                     		/**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                    		/**< The advertising timeout in units of seconds. */

#define BATTERY_LEVEL_MEAS_INTERVAL          25000                                  		/**< Battery level measurement interval (5s). */

#define MIN_BATTERY_LEVEL                    81                                             /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                    100                                            /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1                                              /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL             12500                                          /**< Heart rate measurement interval (5s). */

#define MIN_HEART_RATE                       50                                             /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                       180                                            /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                 5                                              /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                 25000                                          /**< RR interval interval (10s). */

#define MIN_RR_INTERVAL                      100                                            /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                      500                                            /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT                1                                              /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL     25000                                          /**< Sensor Contact Detected toggle interval (10s). */

#define TEMP_TYPE_AS_CHARACTERISTIC          0                                          	/**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */
#if ((APP_HRS == 1) && (APP_HTS == 1))
#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(250, UNIT_1_25_MS)               /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(250, UNIT_1_25_MS)              	/**< Maximum acceptable connection interval (1 second). */
#else
#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)               /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)              /**< Maximum acceptable connection interval (1 second). */
#endif

#define SLAVE_LATENCY                        0                                              /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)                /**< Connection supervisory timeout (4 seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       12500                                          /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        12500                                          /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */

#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                              /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                    30                                             /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                              /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                           /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                             /**< Maximum encryption key size. */

#define MAX_NUMBER_OF_EVT_TO_QUEUE			2
#define BLE_STACK_EVT_MSG_BUF_SIZE       	(sizeof(ble_evt_t) + (GATT_MTU_SIZE_DEFAULT))   /**< Size of BLE event message buffer. */
#define BLE_STACK_EVT_MSG_BUF_SIZE32       	((BLE_STACK_EVT_MSG_BUF_SIZE+3)/4)   			/**< Size of BLE event message buffer in 32bits word. */

/* Private variables ---------------------------------------------------------*/
static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;      	/**< Handle of the current connection. */
static ble_gap_sec_params_t                  m_sec_params;	                                /**< Security requirements for this application. */
static ble_bas_t                             m_bas;                                         /**< Structure used to identify the battery service. */
static ble_hrs_t                             m_hrs;                                         /**< Structure used to identify the heart rate service. */
static bool                                  m_rr_interval_enabled = true;                  /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static ble_sensorsim_cfg_t                   m_battery_sim_cfg;                             /**< Battery Level sensor simulator configuration. */
static ble_sensorsim_state_t                 m_battery_sim_state;                           /**< Battery Level sensor simulator state. */
static ble_sensorsim_cfg_t                   m_heart_rate_sim_cfg;                          /**< Heart Rate sensor simulator configuration. */
static ble_sensorsim_state_t                 m_heart_rate_sim_state;                        /**< Heart Rate sensor simulator state. */
static ble_sensorsim_cfg_t                   m_rr_interval_sim_cfg;                         /**< RR Interval sensor simulator configuration. */
static ble_sensorsim_state_t                 m_rr_interval_sim_state;                       /**< RR Interval sensor simulator state. */

static ble_hts_t                             m_hts;                                     	/**< Structure used to identify the health thermometer service. */
static bool                                  m_hts_meas_ind_conf_pending = false;       	/**< Flag to keep track of when an indication confirmation is pending. */

static uint8_t			                     m_battery_timer_id;                            /**< Battery timer. */
static uint8_t                       		 m_heart_rate_timer_id;                         /**< Heart rate measurement timer. */
static uint8_t                       		 m_rr_interval_timer_id;                        /**< RR interval timer. */
static uint8_t                       		 m_sensor_contact_timer_id;                     /**< Sensor contact detected timer. */

#if (APP_LEDBUTTON == 1)
static ble_but_t							m_but;
static ble_led_t							m_led;
#endif

/**
 * Support up to two events of size BLE_STACK_EVT_MSG_BUF_SIZE32
 * buffer shall be 32 bits aligned
 */
volatile static uint32_t	aEVTQueue[BLE_STACK_EVT_MSG_BUF_SIZE32 * MAX_NUMBER_OF_EVT_TO_QUEUE];
volatile static uint8_t		IndexOfNextFreeSlotInQueue;
volatile static uint8_t		IndexOfPendingEVTToExecute;
volatile static uint8_t		NumberOfPendingEVT;

static bool	bonds_delete = 0;


/* Private function prototypes -----------------------------------------------*/
static void Manual_hrs_value(void);
static void rr_interval_timeout_handler(void * p_context);
static void sensor_contact_detected_timeout_handler(void * p_context);
static void hts_sim_measurement(ble_hts_meas_t * p_meas);
static void gap_params_init(void);
static void advertising_init(void);
static void sensor_sim_init(void);
static void sec_params_init(void);

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void on_ble_evt(ble_evt_t * p_ble_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void bond_manager_error_handler(uint32_t nrf_error);
static void bond_manager_init(void);
static void timers_create(void);
static void encode_evt_process(ble_encode_evt_type_t event);
#if (APP_HTS == 1)
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t *p_evt);
#endif
#if (APP_HRS == 1)
static void application_timers_start(void);
#endif
#if ((APP_HRS == 1) || (APP_HTS == 1)|| (APP_LEDBUTTON == 1))
static void services_init(void);
#endif
#if (APP_LEDBUTTON == 1)
static void on_led_evt(ble_led_t * p_led, ble_led_evt_t * p_evt);
#endif

static void ble_stack_init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Function for initializing the BLE stack.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void ble_stack_init(void)
{
    ble_enable_params_t ble_enable_params;

    /*
     * Enable BLE stack
     */
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    sd_ble_enable(&ble_enable_params);
	blocking_resp_wait();

	return;
}

/**
  * @brief  Request a Heart Rate value to be provided
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void Manual_hrs_value(void)
{
#if (APP_HRS == 1)
	TaskExecutionRequest(eMAIN_Main_HeartRate_Id);
#endif

	return;
}

/**
  * @brief  Function for handling the RR interval timer timeout.
  *
  * @note	This function will be called each time the RR interval timer expires.
  *
  * @param  None
  *
  * @retval None
  */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;
        
        rr_interval = (uint16_t)ble_sensorsim_measure(&m_rr_interval_sim_state,
                                                      &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }

    return;
}

/**
  * @brief  Function for handling the Sensor Contact Detected timer timeout.
  *
  * @note	This function will be called each time the Sensor Contact Detected timer expires.
  *
  * @param  None
  *
  * @retval None
  */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;
    
    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);

    return;
}

/**
  * @brief  Function for populating simulated health thermometer measurement.
  *
  * @note
  *
  * @param  p_meas: Structure holding all temperature measurement informations
  *
  * @retval None
  */
static void hts_sim_measurement(ble_hts_meas_t * p_meas)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };

    uint32_t celciusX100;

    p_meas->temp_in_fahr_units = false;
    p_meas->time_stamp_present = true;
    p_meas->temp_type_present  = TEMP_TYPE_AS_CHARACTERISTIC ? false : true;

    celciusX100 = getTemperatureValue()*100;

    p_meas->temp_in_celcius.exponent = -2;
    p_meas->temp_in_celcius.mantissa = celciusX100;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
    p_meas->time_stamp               = time_stamp;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

    /*
     *  update simulated time stamp
     */
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}

/**
  * @brief  Function for the GAP initialization.
  *
  * @note	This function sets up all the necessary GAP (Generic Access Profile) parameters of the
  *			device including the device name, appearance, and the preferred connection parameters.
  * @param  None
  *
  * @retval None
  */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    
    err_code = blocking_resp_wait();    
    APP_ERROR_CHECK(err_code);    

#if (APP_HRS == 1)
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);
#else
	  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);
#endif
    
    err_code = blocking_resp_wait();        
    APP_ERROR_CHECK(err_code);
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = blocking_resp_wait();        
    APP_ERROR_CHECK(err_code);

    return;
}

/**
  * @brief  Function for initializing the Advertising functionality.
  *
  * @note	Encodes the required advertising data and passes it to the stack.
  *			Also builds a structure to be passed to the stack when starting advertising.
  *
  * @param  None
  *
  * @retval None
  */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
#if ((APP_HRS == 1) && (APP_HTS == 1) )
    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
				{BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE}
    };
#endif
#if ((APP_HRS == 1) && (APP_HTS == 0) )
    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };
#endif
#if ((APP_HRS == 0) && (APP_HTS == 1) )
    ble_uuid_t adv_uuids[] =
    {
       {BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE}
    };
#endif
#if (APP_LEDBUTTON == 1)
		ble_uuid_t adv_uuids[] =
    {
       {BLE_UUID_BUTTON_SERVICE, BLE_UUID_TYPE_BLE},
			 {BLE_UUID_LED_SERVICE, BLE_UUID_TYPE_BLE}
    };
#endif		

    /*
     *  Build and set advertising data
     */
    memset(&advdata, 0, sizeof(advdata));

#if ((APP_HRS == 1) || (APP_HTS == 1) || (APP_LEDBUTTON == 1))
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
 #endif

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    return;
}


#if (APP_LEDBUTTON == 1)
/**
  * @brief  Function for handling the LED events.
  *
  * @note	This function will be called for all LED Service events which are passed to the application.
  *
  *
  * @param  p_led: p_led   led Service structure.
  *
  * @param  p_evt: p_evt   Event received .
  *
  * @retval None
  */
static void on_led_evt(ble_led_t * p_led, ble_led_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_LED_EVT_SIGNAL_VALUE_UPDATED:
        	/*
        	 * change LED value : ON/OFF
        	 */
        	led_value( p_evt->params.led_signal);
            break;

        default:
            break;
    }
}
#endif

#if (APP_HTS == 1)
/**
  * @brief  Function for handling the Health Thermometer Service events.
  *
  * @note	This function will be called for all Health Thermometer Service events which are passed to the application.
  *
  *
  * @param  p_hts: p_hts   Health Thermometer Service structure.
  *
  * @param  p_evt: p_evt   Event received from the Health Thermometer Service.
  *
  * @retval None
  */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            /*
             * Indication has been enabled, send a single temperature measurement
             */
            acquireTemperatureData();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            break;
    }
}
#endif

#if ((APP_HRS == 1) || (APP_HTS == 1) || (APP_LEDBUTTON == 1))
/**
  * @brief  Function for initializing the services that will be used by the application.
  *
  * @note	Initialize the Heart Rate, Battery and Device Information services.
  *
  *
  * @param  None
  *
  * @retval None
  */
static void services_init(void)
{
    uint32_t       err_code;
#if (APP_HRS == 1)
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;
#endif
#if (APP_HTS == 1)
    ble_hts_init_t   hts_init;
#endif
#if (APP_LEDBUTTON == 1)
	  ble_but_init_t   but_init;
	  ble_led_init_t   led_init;
#endif	
    
#if (APP_HRS == 1)

    /*
     * Initialize Heart Rate Service
     */
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;
    
    memset(&hrs_init, 0, sizeof(hrs_init));
    
    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    /*
     * Here the sec level for the Heart Rate Service can be changed/increased.
     */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);
    
    /*
     *  Initialize Battery Service
     */
    memset(&bas_init, 0, sizeof(bas_init));
    
    /*
     * Here the sec level for the Battery Service can be changed/increased.
     */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    
    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
    

    /*
     * Initialize Device Information Service
     */
    memset(&dis_init, 0, sizeof(dis_init));
    
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
#endif

#if (APP_HTS == 1)
    /*
     * Initialize Health Thermometer Service
     */
    memset(&hts_init, 0, sizeof(hts_init));

    hts_init.evt_handler                 = on_hts_evt;
    hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    /*
     * Here the sec level for the Health Thermometer Service can be changed/increased.
     */
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hts_init.hts_meas_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_temp_type_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_temp_type_attr_md.write_perm);

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);
#endif

#if (APP_LEDBUTTON == 1)
    /*
     * Initialize Button Service
     */
    memset(&but_init, 0, sizeof(but_init));
    
    but_init.evt_handler = NULL;

    /*
     * Here the sec level for the Heart Rate Service can be changed/increased.
     */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&but_init.but_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&but_init.but_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&but_init.but_attr_md.write_perm);

    err_code = ble_but_init(&m_but, &but_init);
    APP_ERROR_CHECK(err_code);
		
	/*
     * Initialize Led Service
     */
      
    memset(&led_init, 0, sizeof(led_init));
    
    led_init.evt_handler = on_led_evt;
   
    /*
     * Here the sec level for the Heart Rate Service can be changed/increased.
     */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&but_init.but_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&but_init.but_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&but_init.but_attr_md.write_perm);

    err_code = ble_led_init(&m_led, &led_init);
    APP_ERROR_CHECK(err_code);
		
#endif

    return;
}
#endif

/**
  * @brief  Function for initializing the sensor simulators.
  *
  * @note
  *
  *
  * @param  None
  *
  * @retval None
  */
static void sensor_sim_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;
    
    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
    
    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;
    
    ble_sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
    
    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;
    
    ble_sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);

    return;
}

/**
  * @brief  Function for initializing the security parameters.
  *
  * @note
  *
  *
  * @param  None
  *
  * @retval None
  */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;

    return;
}

#if (APP_HRS == 1)
/**
  * @brief  Function for starting the application timers.
  *
  * @note
  *
  *
  * @param  None
  *
  * @retval None
  */
static void application_timers_start(void)
{
    HAL_TIMER_Start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL);
    HAL_TIMER_Start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL);
    HAL_TIMER_Start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL);
    HAL_TIMER_Start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL);

    return;
}
#endif

/**
  * @brief  Function for starting advertising.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void advertising_start(void)
{
    uint32_t err_code;
    ble_gap_adv_params_t m_adv_params;	/**< Parameters to be passed to the stack when starting advertising. */

    
    /*
     * Initialize advertising parameters (used when starting advertising)
     */
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           /**< Undirected advertisement */
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = blocking_resp_wait();        
    APP_ERROR_CHECK(err_code);

#if(APP_LCD == 1)
	LCD_SetTextColor(LCD_COLOR_RED); /**< Set Text color */
	LCD_DisplayStringLine(LCD_LINE_15, (uint8_t*)"  ADVERTISING ");	
#endif	

    return;
}

/**
  * @brief  Function for handling the Connection Parameters Module.
  *
  * @note	This function will be called for all events in the Connection Parameters Module which
  * 		are passed to the application.
  * 		All this function does is to disconnect. This could have been done by simply
  *			setting the disconnect_on_fail config parameter, but instead we use the event
  *			handler mechanism to demonstrate its use.
  *
  * @param  p_evt: Connection parameters
  *
  * @retval None
  */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    
        err_code = blocking_resp_wait();            
        APP_ERROR_CHECK(err_code);
    }

    return;
}

/**
  * @brief  Function for handling a Connection Parameters error.
  *
  * @note
  *
  * @param  nrf_error: Error code containing information about what went wrong.
  *
  * @retval None
  */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**
  * @brief  Function for initializing the Connection Parameters module.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);

    return;
}

/**
  * @brief  Function for handling the Application's BLE Stack events.
  *
  * @note
  *
  * @param  p_ble_evt:	Bluetooth stack event.
  *
  * @retval None
  */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            UserButtonHandlerRegister((pf_Main_UserButtonHandler_t)Manual_hrs_value);

			#if(APP_L2CAP_TX_TEST == 1)
            TaskExecutionRequest(eMAIN_Main_L2CAP_CID_Register_Id);
			#endif

 	 	 	 #if(APP_LCD == 1)
            LCD_SetTextColor(LCD_COLOR_BLUE);	/**< Set Text color */
            LCD_DisplayStringLine(LCD_LINE_15, (uint8_t*)"   CONNECTED   ");
			#endif
				
            break;
            
		#if(APP_L2CAP_TX_TEST == 1)
		case BLE_EVT_TX_COMPLETE:
			UpdateL2CAPBufferCount(p_ble_evt->evt.common_evt.params.tx_complete.count);

			TaskExecutionRequest(eMAIN_Main_L2CAP_TX_Packet_Id);

            break;
		#endif

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            TaskExecutionRequest(eMAIN_Main_GAP_Disc_Evt_Id);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            
            err_code = blocking_resp_wait();                                                       
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {                 
                HAL_TIMER_Delete(m_battery_timer_id);
                HAL_TIMER_Delete(m_heart_rate_timer_id);
                HAL_TIMER_Delete(m_rr_interval_timer_id);
                HAL_TIMER_Delete(m_sensor_contact_timer_id);

                sd_power_system_off();	/**< Put nRF51 to system-off mode */

                #if(APP_LCD == 1)
                LCD_SetTextColor(LCD_COLOR_RED);	/**< Set Text color */
                LCD_DisplayStringLine(LCD_LINE_15, (uint8_t*)"    OFF-MODE    ");
				#endif
            }
            break;
            
        default:
            break;
    }
}

/**
  * @brief  Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
  *
  * @note
  *
  * @param  p_ble_evt:	Bluetooth stack event.
  *
  * @retval None
  */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_bondmngr_on_ble_evt(p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_hts_on_ble_evt(&m_hts, p_ble_evt);
#if (APP_HRS == 1)
    ble_conn_params_on_ble_evt(p_ble_evt);
#endif
#if (APP_LEDBUTTON ==1)
    ble_but_on_ble_evt(&m_but, p_ble_evt);
	ble_led_on_ble_evt(&m_led, p_ble_evt);
#endif	
    on_ble_evt(p_ble_evt);
}

/**
  * @brief  Function for handling a Bond Manager error.
  *
  * @note
  *
  * @param  nrf_error:	Error code containing information about what went wrong.
  *
  * @retval None
  */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**
  * @brief  Function for the Bond Manager initialization.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void bond_manager_init(void)
{
    uint32_t            err_code;    
    ble_bondmngr_init_t bond_init_data;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);
    
    /*
     * Initialize the Bond Manager
     */
    bond_init_data.evt_handler             = NULL;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = bonds_delete;

    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
}

/**
  * @brief  Function for creating application timers.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
static void timers_create(void)
{        
    HAL_TIMER_Create(eTimerModuleID_Battery,
    				(uint8_t *)&m_battery_timer_id,
    				eTimerMode_Repeated,
    				0);

    HAL_TIMER_Create(eTimerModuleID_HeartRate,
    				(uint8_t *)&m_heart_rate_timer_id,
    				eTimerMode_Repeated,
    				0);

    HAL_TIMER_Create(eTimerModuleID_Interrupt,
    				(uint8_t *)&m_rr_interval_timer_id,
    				eTimerMode_Repeated,
    				(pf_HAL_TIMER_TimerCallBack_t)rr_interval_timeout_handler);

    HAL_TIMER_Create(eTimerModuleID_Interrupt,
    				(uint8_t *)&m_sensor_contact_timer_id,
    				eTimerMode_Repeated,
    				(pf_HAL_TIMER_TimerCallBack_t)sensor_contact_detected_timeout_handler);

    return;
}

/**
  * @brief  Function for processing ble encode layer events.
  *
  * @note	BLE encode layer events are forwardet to main application context.
  *
  * @param  event: Event occurred.
  *
  * @retval None
  */
static void encode_evt_process(ble_encode_evt_type_t event)
{
	uint32_t  event_len;
	uint32_t err_code;

    if (event == BLE_ENCODE_EVT_RDY)
    {
        event_len = BLE_STACK_EVT_MSG_BUF_SIZE;

        err_code = ble_encode_event_pop((ble_evt_t*)&aEVTQueue[IndexOfNextFreeSlotInQueue], &event_len);
        APP_ERROR_CHECK(err_code);

        IndexOfNextFreeSlotInQueue++;
    	IndexOfNextFreeSlotInQueue = (IndexOfNextFreeSlotInQueue % MAX_NUMBER_OF_EVT_TO_QUEUE);

        NumberOfPendingEVT++;

    	TaskExecutionRequest(eMAIN_Main_Hrs_Evt_Id);
    }
    else
    {
        APP_ERROR_HANDLER(event);
    }
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Function for processing ble hrs event
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
uint32_t ble_hrs_evt_process(void)
{
    while(NumberOfPendingEVT != 0)
    {
    	ble_evt_dispatch((ble_evt_t*)&aEVTQueue[IndexOfPendingEVTToExecute]);

        IndexOfPendingEVTToExecute++;
    	IndexOfPendingEVTToExecute = (IndexOfPendingEVTToExecute % MAX_NUMBER_OF_EVT_TO_QUEUE);

    	__disable_irq();
    	NumberOfPendingEVT--;
        __enable_irq();
    }
    
    return NRF_SUCCESS;
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    while(1);
}

/**
  * @brief  Request a battery level value to be provided
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
uint32_t battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)ble_sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }

    return NRF_SUCCESS;
}

/**
  * @brief  Function for handling the Heart rate measurement timer timeout.
  *
  * @note	This function will be called each time the heart rate measurement timer expires.
  * 		It will exclude RR Interval data from every third measurement.
  *
  * @param  None
  *
  * @retval None
  */
uint32_t heart_rate_meas_timeout_handler()
{
    static uint32_t cnt = 0;

    uint32_t err_code;
    uint16_t heart_rate;

    heart_rate = (uint16_t)ble_sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }

    /* Disable RR Interval recording every third heart rate measurement.
    * NOTE: An application will normally not do this. It is done here just for testing generation
    *       of messages without RR Interval measurements.
    */
    m_rr_interval_enabled = ((cnt % 3) != 0);

    return  NRF_SUCCESS;
}

/**
  * @brief  Function for simulating and sending one Temperature Measurement.
  *
  * @note
  *
  *
  * @param  None
  *
  * @retval None
  */
void temperature_measurement_send(void)
{
    ble_hts_meas_t simulated_meas;
    uint32_t       err_code;

    if (!m_hts_meas_ind_conf_pending)
    {
        hts_sim_measurement(&simulated_meas);

        err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);
        switch (err_code)
        {
            case NRF_SUCCESS:
            	/*
            	 * Measurement was successfully sent, wait for confirmation.
            	 */
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                /*
                 * Ignore error.
                 */
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}

#if (APP_LEDBUTTON == 1)
/**
  * @brief  send information button is pressed
  *
  * @note
  *
  * @param  BUTTON ID
  *
  * @retval None
  */
uint32_t ble_button_process(uint8_t button_id)
{
	uint32_t err_code;

    err_code = ble_button_id_pushed_send(&m_but, button_id);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }

    return NRF_SUCCESS;
}
#endif
/**
  * @brief  Function for BLE module initialization
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void ble_app_main_init_Reset(void)
{
    IndexOfNextFreeSlotInQueue = 0;
    IndexOfPendingEVTToExecute = 0;
    NumberOfPendingEVT = 0;

    blocking_init();

    ble_encode_evt_handler_register(encode_evt_process);	/**< Register handler for encode events. */

    ble_encode_open();		/**< Open encoder module. */
	
		ble_stack_init();

    timers_create();
    bond_manager_init();
    sensor_sim_init();
    sec_params_init();

    gap_params_init();
    advertising_init();
#if ((APP_HRS == 1) || (APP_HTS == 1) || (APP_LEDBUTTON == 1))
    services_init();
#endif
    conn_params_init();

#if (APP_HRS == 1)
    application_timers_start();
#endif
    advertising_start();

    return;
}

/**
  * @brief  Function for User button.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
void user_nvm_bond_delete(void)
{
	bonds_delete = 1;
	bond_manager_init();
	bonds_delete = 0;

	return;
}

#if(APP_L2CAP_TX_TEST == 1)
/**
  * @brief  Return the connection Handle.
  *
  * @note
  *
  * @param  None
  *
  * @retval None
  */
uint16_t GetConnectionHandle(void)
{
	return m_conn_handle;
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

