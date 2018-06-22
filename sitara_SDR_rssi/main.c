
/**
 *	Written by Phillip Smith, based on a demo by Nordic Semiconductor
 */

#include <stdint.h>
#include <string.h>
#include "stdlib.h"
#include "math.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

#include "cc1200.h"
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
// #include "cc1200-802154g-434mhz-2gfsk-50kbps-cw.h"
#include "cc1200-802154g-868mhz-2gfsk-50kbps-cw.h"
//#include "cc1200-ecobee-sensor-config.h"

#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"

#include "ble_nus.h"

//#define LEGACY_MODE_INTERFACE 1 				// remaps interface commands for compatibility with crowdsource app
#define CONVENIENCE_MODE_INTERFACE 1			// remaps interface commands for (hopefully) more intuitive use

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define PCA10056_USE_FRONT_HEADER		1											/**< Use the front header (P24) for the camera module. Requires SB10-15 and SB20-25 to be soldered/cut, as described in the readme. */

#define CONN_CFG_TAG                    1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "Sitara_SDR_4"                           		/**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)              /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(12, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static nrf_ble_gatt_t                   m_gatt;                                     /**< GATT module instance. */
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static uint16_t                         m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;  /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static uint8_t							m_BLE_sent_flag; // use with enum SENT, NOT_SENT
static uint8_t                          m_new_command_received = 0;
static uint8_t                          m_stream_mode_active = 0;
static uint8_t							m_receive_mode_active = 0;
static uint8_t							m_transmit_mode = 0;
static uint8_t                          m_new_phy;
static uint32_t							m_new_frequency; //in MHz
static char								m_register_cmd[7] = {0};
static uint8_t							m_RF_txbytes[100] = {0}; // this should be limited to the length of the TX FIFO

static uint32_t t_counter = 0;
// static volatile bool ble_spi_tx = false;

/***********************************************************/
/* additions to ble_nus service */
//to allow 2mbps connection and otherwise change parameters
typedef struct __attribute__((__packed__))
{
    uint16_t mtu;
    uint16_t con_interval;
    uint8_t  tx_phy;
    uint8_t  rx_phy;
}ble_nus_ble_params_info_t;

static ble_nus_ble_params_info_t        m_ble_params_info = {20, 50, 1, 1};

/***********************************************************/

#define GPIO2_INT_PIN NRF_GPIO_PIN_MAP(1,15)//1,15) // misnomer, this MAPS to GPIO2 which is technically CHFILT_VALID, which samples at 22us or about half the rate of MAGN_VALID
#define GPIO3_INT_PIN NRF_GPIO_PIN_MAP(0,2)// this actually maps to MAGN_VALID which has a sampling interval of about 11us or ~90ksps

enum {APP_CMD_NOCOMMAND = 0, APP_CMD_ACCESS_REGISTER = 1, APP_CMD_START_MEASUREMENT = 2, APP_CMD_STOP = 3,
      APP_CMD_CHANGE_FREQ = 4, APP_CMD_CHANGE_PHY = 5, /*APP_CMD_SEND_BLE_PARAMS = 6,*/ APP_CMD_TRANSMIT = 7, APP_CMD_RECEIVE = 8, APP_CMD_RESET = 9, APP_CMD_CLOCK = 10};

// TODO: create a structure where enum commands above reference strings that can be used for cmd_ack to be more helpful
// Should have it send cmd_ack + command name that was selected

enum {OFF = 0, MEASUREMENT_OPTION_RSSI_DIRECT = 1, MEASUREMENT_OPTION_RSSI_CALCULATED = 2, MEASUREMENT_OPTION_MAGNITUDE = 3, MEASUREMENT_OPTION_PHASE = 4}; // use with APP_CMD_START_MEASUREMENT
enum {TRANSMIT_MSG, TRANSMIT_MSG_REPEAT, TRANSMIT_CONTINOUS_TONE};
enum {CLOCK_READ, CLOCK_SET, CLOCK_DISABLE}; // USED WITH APP_CMD_CLOCK


enum {SENT, NOT_SENT};

#define ARR_SIZE(a) (sizeof(a) / sizeof((a)[0]))
uint8_t cmd_ack[] = "cmd ack";//{'c','m','d',' ','a','c','k'};

extern const cc1200_rf_cfg_t CC1200_RF_CFG;

// #define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps_cw
#define CC1200_RF_CFG cc1200_802154g_868mhz_2gfsk_50kbps_cw
//#define CC1200_RF_CFG cc1200_ecobee_sensor_config

/***********************************************************/
/*
 * 						BEGIN CODE
 */
/***********************************************************/


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



uint32_t ble_nus_ble_params_info_send(ble_nus_t * p_nus, ble_nus_ble_params_info_t * ble_params_info)
{
    uint8_t data_buf[1 + sizeof(ble_nus_ble_params_info_t)];
    ble_gatts_hvx_params_t hvx_params;

    VERIFY_PARAM_NOT_NULL(p_nus);

    if ((p_nus->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_nus->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint16_t length = 1 + sizeof(ble_nus_ble_params_info_t);

    data_buf[0] = 2;
    memcpy(&data_buf[1], ble_params_info, sizeof(ble_nus_ble_params_info_t));

    memset(&hvx_params, 0, sizeof(hvx_params));

    //hvx_params.handle = p_nus->img_info_handles.value_handle;
    hvx_params.p_data = data_buf;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint8_t selection = 0;
    //selection = p_data[0] - '0'; // convert to integer because ble nus sends ascii
    // ble_spi_tx = false;
#ifdef LEGACY_MODE_INTERFACE
    if(p_data[0] == '1' && p_data[1] == '1')
    	selection = APP_CMD_START_MEASUREMENT;
    if(p_data[0] == '0' && p_data[1] == '1')
    	selection = APP_CMD_TRANSMIT;
    else if(p_data[0] == '0')
    	selection = APP_CMD_STOP;
#elif CONVENIENCE_MODE_INTERFACE
    switch(p_data[0])
    {
    case 'x':
    	selection = APP_CMD_STOP;
    	break;
    case 'q':
    	selection = APP_CMD_STOP;
    	break;
    case 'a':
    	selection = APP_CMD_ACCESS_REGISTER;
    	break;
    case 'm':
    	selection = APP_CMD_START_MEASUREMENT;
    	break;
    case 'f':
    	selection = APP_CMD_CHANGE_FREQ;
    	break;
    case 'p':
    	selection = APP_CMD_CHANGE_PHY;
    	break;
    case 't':
    	selection = APP_CMD_TRANSMIT;
    	break;
    case 'T':
    	selection = APP_CMD_TRANSMIT;
    	break;
    case 'r':
    	selection = APP_CMD_RECEIVE;
    	break;
    case 'X':
    	selection = APP_CMD_RESET;
    	break;
    case 'c':
    	selection = APP_CMD_CLOCK;
    	break;
    }
#endif //

    switch(selection)
    {
        // Take picture
        case APP_CMD_ACCESS_REGISTER:
        	if(m_stream_mode_active)
        	{
        		NRF_LOG_ERROR("*Cannot access registers while streaming*\n");
        		m_new_command_received = APP_CMD_NOCOMMAND;
        	}
        	else
        	{
        		m_new_command_received = APP_CMD_ACCESS_REGISTER;
        		for (int i =0; (i < length && i < 7); i++)
        			m_register_cmd[i] = p_data[i+1]; // R/W,R1,R2,R3,R4,C1,C2 up to 7 chars 6 of which may be converted to hex
        	}
        	break;/*
        case APP_CMD_SEND_BLE_PARAMS:
            m_new_command_received = APP_CMD_SEND_BLE_PARAMS;
            break;*/
        
        case APP_CMD_START_MEASUREMENT:
        	if(m_stream_mode_active)
        	{
        		NRF_LOG_ERROR("*Already streaming*\n");
        		m_new_command_received = APP_CMD_NOCOMMAND;
        		break;
        	}
        	switch(p_data[1])
        	{
        	case 'm':
        		m_stream_mode_active = MEASUREMENT_OPTION_MAGNITUDE;
        		break;
        	case 'd':
        		m_stream_mode_active = MEASUREMENT_OPTION_RSSI_DIRECT;
        		break;
        	case 'c':
        		m_stream_mode_active = MEASUREMENT_OPTION_RSSI_CALCULATED;
        		break;
        	case 'p':
        		m_stream_mode_active = MEASUREMENT_OPTION_PHASE;
        		break;
        	default:
        		m_stream_mode_active = MEASUREMENT_OPTION_RSSI_DIRECT ; // we can change this to whatever will most frequently be used
        	}
        	ble_nus_string_send(&m_nus,cmd_ack,ARR_SIZE(cmd_ack));
            m_new_command_received = APP_CMD_START_MEASUREMENT;
            // TODO should maybe include another parameter for doppler RX eventually
            break;

        case APP_CMD_STOP:
            // ble_spi_tx = false;
            cc1200_cmd_strobe(CC1200_SIDLE);
            NRF_LOG_INFO("Done\r\n");
			m_stream_mode_active = OFF;
			ble_nus_string_send(&m_nus,cmd_ack,ARR_SIZE(cmd_ack));
            m_new_command_received = APP_CMD_STOP;
            break;
        
        case APP_CMD_CHANGE_FREQ:
        	if(length == 4) // frequence of the form f1,f2,f3 MHz
        	{
        		m_new_command_received = APP_CMD_CHANGE_FREQ;
        		m_new_frequency = 100000*(uint16_t)(p_data[1]-'0') + 10000*(p_data[2]-'0') + 1000*(p_data[3]- '0');
        	}
        	else if(length == 8) // frequency of the form f1,f2,f3 . f4, f5 f6 MHz
        	{
        		m_new_command_received = APP_CMD_CHANGE_FREQ;
        		m_new_frequency = 100000*(uint16_t)(p_data[1]-'0') + 10000*(p_data[2]-'0') + 1000*(p_data[3]- '0') + 100*(p_data[5]-'0') + 10*(p_data[6]- '0') + (p_data[7]- '0'); // KHz
        	}
            break;
        
        case APP_CMD_CHANGE_PHY:
            m_new_command_received = APP_CMD_CHANGE_PHY;
            m_new_phy = 2;//p_data[1];
            break;

        case APP_CMD_TRANSMIT:
            m_new_command_received = APP_CMD_TRANSMIT;
			ble_nus_string_send(&m_nus,cmd_ack,ARR_SIZE(cmd_ack));
			if(p_data[0] == 't')
			{
				m_transmit_mode = TRANSMIT_MSG;
	    		for (int i =1; (i < length && i < 100); i++) // being lazy, we want to limit it to the length of the TX buffer
	    			m_RF_txbytes[i] = p_data[i]; // R/W,R1,R2,R3,R4,C1,C2 up to 7 chars 6 of which may be converted to hex
			}
			else
				m_transmit_mode = TRANSMIT_CONTINOUS_TONE;
                // ble_spi_tx = true;
            break;

        case APP_CMD_RECEIVE:
			ble_nus_string_send(&m_nus,cmd_ack,ARR_SIZE(cmd_ack));
            m_new_command_received = APP_CMD_RECEIVE;
            break;

        case APP_CMD_CLOCK:
			ble_nus_string_send(&m_nus,cmd_ack,ARR_SIZE(cmd_ack));
			//TODO: need to include this function
            break;


        case APP_CMD_RESET:
        	NRF_LOG_INFO("\n****************\n System Reset\n****************\n");
			m_BLE_sent_flag = NOT_SENT;
			ble_nus_string_send(&m_nus,(uint8_t *)"\n****************\n System Reset\n****************\n\0",50);
			//while(m_BLE_sent_flag == NOT_SENT){}
			NRF_LOG_FLUSH();
			nrf_delay_ms(100);
			sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE); // not working yet
			NVIC_SystemReset();
            break;
        
        default: 
        	m_new_command_received = APP_CMD_NOCOMMAND;
            NRF_LOG_INFO("Unknown command!!\r\n");
            break;
    }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
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
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t  err_code;
    uint32_t elapsed_time;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            
            NRF_LOG_INFO("Connected\r\n");
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            m_stream_mode_active = false;
            m_ble_params_info.tx_phy = m_ble_params_info.rx_phy = 1;
            
            NRF_LOG_INFO("Disconnected\r\n");
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            uint16_t max_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
            uint16_t min_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;

            m_ble_params_info.con_interval = max_con_int;
            ble_nus_ble_params_info_send(&m_nus, &m_ble_params_info);
            NRF_LOG_INFO("Con params updated: CI %i, %i\r\n", (int)min_con_int, (int)max_con_int);
        } break;
       
        case BLE_GAP_EVT_PHY_UPDATE:
            m_ble_params_info.tx_phy = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
            m_ble_params_info.rx_phy = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;    
            ble_nus_ble_params_info_send(&m_nus, &m_ble_params_info);
            NRF_LOG_INFO("Phy update: %i, %i\r\n", (int)m_ble_params_info.tx_phy, (int)m_ble_params_info.rx_phy);
            break;
        
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        case BLE_GATTS_EVT_HVN_TX_COMPLETE: //we will use this to coordinate with PPI buffers
        	if(m_stream_mode_active)
        	{
        		elapsed_time = cc1200_resume_PPI(); // if nonzero then it represents number of 16MHz ticks since PPI was paused
        		if(elapsed_time)
        			ble_nus_string_send(&m_nus,(uint8_t*)&elapsed_time,sizeof(uint32_t)); // send 4 byte message with number of ticks
        	}
        	else
        		m_BLE_sent_flag = SENT;
        	break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg;

    clock_lf_cfg.source        = NRF_CLOCK_LF_SRC_XTAL;
    clock_lf_cfg.rc_ctiv       = 0;
    clock_lf_cfg.rc_temp_ctiv  = 0;
#ifdef S140
    clock_lf_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
#else
    clock_lf_cfg.accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM;
#endif

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum ATT MTU.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                 = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum event length.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                     = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        m_ble_params_info.mtu = m_ble_nus_max_data_len;
        ble_nus_ble_params_info_send(&m_nus, &m_ble_params_info);
        NRF_LOG_INFO("Data len is set to 0x%X(%d), or maybe %i\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len, p_evt->params.data_length);
    }
    NRF_LOG_INFO("ATT MTU exchange completed. central %i peripheral %i\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
            
        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS\r\n");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    err_code = ble_nus_string_send(&m_nus, data_array, index);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(CONN_CFG_TAG);
}


static void data_len_ext_set(bool status)
{
    uint8_t data_length = status ? (247 + 4) : (23 + 4);
    (void) nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, data_length);
}


static void gatt_mtu_set(uint16_t att_mtu)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, att_mtu);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, att_mtu);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



void gpio2_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	t_counter++;
	//NRF_LOG_INFO("We made it to the mag handler!\n"); // not needed, done via PPI instead
	if(t_counter%10000 ==0)
		NRF_LOG_INFO("We made it to the mag handler with counter %ld\n", t_counter);
	//NRF_LOG_FLUSH();
}

void gpio3_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	;
}

static void gpio_init(void)
{
	ret_code_t err_code;

	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_in_config_t gpio2_int_pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); //true sets to high accuracy, GPIO IN rather than PORT EVENT
	nrf_drv_gpiote_in_config_t gpio3_int_pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); //true sets to high accuracy, GPIO IN rather than PORT EVENT


	err_code = nrf_drv_gpiote_in_init(GPIO2_INT_PIN, &gpio2_int_pin_config, gpio2_int_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_in_init(GPIO3_INT_PIN, &gpio3_int_pin_config, gpio3_int_handler);
	APP_ERROR_CHECK(err_code);
}

// tunes CC1200 to m_new_frequency (in MHz) in 820-960 MHz band
// assumes there is no frequency offset
// TODO: sometimes the number is slightly off, may be a rounding error that needs to be fixed..
static int tune_frequency()
{
	registerSetting_t new_settings[CC1200_RF_CFG.size_of_register_settings];
	ret_code_t err_code = 0;
	uint8_t freq_tmp;
	uint32_t freq_vco = m_new_frequency * 0x1000/625; // b/c this is too big: 0xFFFF/10000; // assuming LO DIVIDER = 4, for frequencies in 820-960 band

	if ((m_new_frequency > 950000) || (m_new_frequency < 820000))
	{
		NRF_LOG_ERROR("%d MHz frequency out of range, no change.\n", m_new_frequency/1000);
		return 1;
	}

	cc1200_cmd_strobe(CC1200_SRES);

	// had to do it this stupid kludged way to get it to work, directly writing freq registers didn't work
	// could be that registers need to be written all at once instead of separate spi transfers as before
	for(int i = 0; i < CC1200_RF_CFG.size_of_register_settings; i++)
	{
		new_settings[i].addr = CC1200_RF_CFG.register_settings[i].addr;
		if(new_settings[i].addr == CC1200_FREQ2)
			new_settings[i].val = (uint8_t)(freq_vco >> 16);
		else if(new_settings[i].addr == CC1200_FREQ1)
			new_settings[i].val = (uint8_t)(freq_vco >> 8);
		else if(new_settings[i].addr == CC1200_FREQ0)
			new_settings[i].val = (uint8_t)(freq_vco);
		else
			new_settings[i].val = CC1200_RF_CFG.register_settings[i].val;
	}

	cc1200_write_reg_settings(new_settings, CC1200_RF_CFG.size_of_register_settings);

	NRF_LOG_DEBUG("wrote %X to FREQ2\n",(uint8_t)(freq_vco >> 16));
	NRF_LOG_DEBUG("wrote %X to FREQ1\n",(uint8_t)(freq_vco >> 8));
	NRF_LOG_DEBUG("wrote %X to FREQ0\n",(uint8_t)(freq_vco));

	// verify register received frequency settings
	freq_vco = 0;
	cc1200_read_register(CC1200_FREQ2,&freq_tmp);
	freq_vco |= freq_tmp;
	freq_vco <<= 8;
	cc1200_read_register(CC1200_FREQ1,&freq_tmp);
	freq_vco |= freq_tmp;
	freq_vco <<= 8;
	cc1200_read_register(CC1200_FREQ1,&freq_tmp);
	freq_vco |= freq_tmp;
	NRF_LOG_INFO("Changed frequency to: %d kHz\r\n", freq_vco *625/0x1000);

	cc1200_cmd_strobe(CC1200_SCAL); // should always calibrate before using new frequency

	return err_code;
}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    int32_t mean_sample, packet_number, capture_duration;
    int16_t previous, tmp, delta, sum_angle;

   	uint32_t reg_cmds; // stores commands used to register access
   	uint8_t reg_contents; // stores contents from read register access

   	uint16_t packet_count = 0;
    uint8_t *sample_buf = 0; // used to refer to the current SPI rx_buf when processing streaming data
   	uint8_t packed_msg[BLE_NUS_MAX_DATA_LEN] ={0}; // buffer to store the message while transmitting over BLE
   	uint8_t BLE_NUS_tx_num_bytes; // number of bytes to transmit, argument to BLE_NUS_string_send

    uint8_t status;
    uint8_t rxbytes;

    int32_t magnitude;
    float rss_value;
    float total_mag = 0.0;

   	log_init();
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_INFO("\n\n\n\n")
	NRF_LOG_INFO("****************************************************\n")
	NRF_LOG_INFO("                   Sitara SDR\r\n");
	NRF_LOG_INFO("****************************************************\n")
	NRF_LOG_FLUSH();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    
    data_len_ext_set(true);

    gatt_mtu_set(247);



    cc1200_init();
    gpio_init();

	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	NRF_LOG_INFO("CC1200 initialized\n");

	// start advertising after everything is set up..
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);


    // Enter main loop.
    for (;;)
    {

        ble_gap_phys_t gap_phys_settings;    

        switch(m_new_command_received) // process APP_CMDs
        {
            case APP_CMD_ACCESS_REGISTER:
                m_new_command_received = APP_CMD_NOCOMMAND;
                if(m_register_cmd[0] == 'r') // read
                {
                	m_register_cmd[0] = '0';
                	if(m_register_cmd[1] == '2' && m_register_cmd[2] == 'f')
                		m_register_cmd[5] = '\0'; // cut off any excess stuff that was received
                	else
                		m_register_cmd[3] = '\0';
                	reg_cmds = (int)strtol(m_register_cmd,NULL,16);
                	cc1200_read_register(reg_cmds, &reg_contents);
                	NRF_LOG_INFO("Register %X contains %X\n", reg_cmds,reg_contents);

                }
                else if(m_register_cmd[0] == 'w') // write
                {
                	m_register_cmd[0] = '0';
                	reg_cmds = (uint32_t)strtol(m_register_cmd,NULL,16);
                	NRF_LOG_INFO("Processing command %X\n", reg_cmds);
                	cc1200_write_register(reg_cmds>>8, (uint8_t)reg_cmds);
                	NRF_LOG_INFO("Wrote %X to register %X\n", (uint8_t)reg_cmds, reg_cmds>>8);
                	cc1200_read_register(reg_cmds>>8, &reg_contents);
                	NRF_LOG_INFO("Register %X contains %X\n", reg_cmds,reg_contents);
                }
                else if(m_register_cmd[0] == 'a') // read all registers and dump
                {
                	cc1200_burst_read_register(0,packed_msg,0x2F); // regular registers
                	NRF_LOG_INFO("Standard Register Space\n");
                	for(int i = 0; i < 0x2F; i++)
                		NRF_LOG_INFO("Register 0x%02x : 0x%02x\n", i, packed_msg[i]);
                	cc1200_burst_read_register(0x2F00,packed_msg,0x95); // extended registers, we don't care about many of the last registers, so ignore;
                	NRF_LOG_FLUSH();
                	NRF_LOG_INFO("Extended Register Space\n");
                	for(int i = 0; i < 0x95; i++)
                	{
                		NRF_LOG_INFO("Register 0x%02x : 0x%02x\n", i, packed_msg[i]);
                		if (i%0x50 == 0)  // don't want to overflow the log buffer..
                			NRF_LOG_FLUSH();
                	}
                	cc1200_get_status(&status);
                	NRF_LOG_INFO("And status is 0x%02x\n", status);
                }
                break;
            
            case APP_CMD_START_MEASUREMENT:
                m_new_command_received = APP_CMD_NOCOMMAND;
                cc1200_write_register(CC1200_MDMCFG1, 0x06); // disable fifo and turn on fixed DVGA gain
                cc1200_write_register(CC1200_PKT_CFG2, 0x05); // change packet mode and format
                cc1200_write_register(CC1200_MDMCFG2, 0x03); // enable custom freq mod
                cc1200_write_register(CC1200_SERIAL_STATUS, 0x08); // enable sychroninize for io pins
    			cc1200_cmd_strobe(CC1200_SRX);
    			switch(m_stream_mode_active)
    			{
    			case MEASUREMENT_OPTION_MAGNITUDE:

    				cc1200_write_register(CC1200_IOCFG2, 0x28); // configure GPIO for magnitude/filter on sitara
    				cc1200_ppi_spi_enable(CC1200_MAGN2,(5+2), BLE_NUS_MAX_DATA_LEN/3, GPIO2_INT_PIN);
    				BLE_NUS_tx_num_bytes = BLE_NUS_MAX_DATA_LEN;
    				// TODO: probably need to changy PHY to 2Mbps since this option won't really work without it
    				NRF_LOG_INFO("Magnitude/phase stream mode enabled\r\n");
    				break;
    			case MEASUREMENT_OPTION_RSSI_DIRECT:
    				cc1200_write_register(CC1200_IOCFG2, 0x0E); // configure GPIO for RSSI_update instead of magn
    				cc1200_ppi_spi_enable(CC1200_RSSI1,(1+2), BLE_NUS_MAX_DATA_LEN, GPIO2_INT_PIN); //RSSI register
    				BLE_NUS_tx_num_bytes = BLE_NUS_MAX_DATA_LEN;
    				NRF_LOG_INFO("RSSI direct stream mode enabled\r\n");
    				break;
    			case MEASUREMENT_OPTION_RSSI_CALCULATED:
    				// TODO: configure for calculated RSSI mode
                    cc1200_write_register(CC1200_IOCFG2, 0x28); // configure GPIO for magnitude/filter on sitara
                    cc1200_ppi_spi_enable(CC1200_MAGN2,5,200,GPIO2_INT_PIN); //RSSI register
    				NRF_LOG_INFO("RSSI calculated stream mode enabled\r\n");

    				break;
    			case MEASUREMENT_OPTION_PHASE:
    				// this samples at faster rate, but consequently can only support 4 captures
    				cc1200_write_register(CC1200_IOCFG3, 0x28); // configure GPIO for magnitude/filter on sitara
    				cc1200_ppi_spi_enable(CC1200_ANG1,(2+2), BLE_NUS_MAX_DATA_LEN/2, GPIO2_INT_PIN); // 16 LSb of MAGN and all ANG bytes, the faster sampling rate on spi can only handle 4 bytes max
    				BLE_NUS_tx_num_bytes = BLE_NUS_MAX_DATA_LEN;
    				// TODO: probably need to changy PHY to 2Mbps since this option won't really work without it
    				NRF_LOG_INFO("Phase stream mode enabled\r\n");
    				break;
    			default:
    				NRF_LOG_ERROR("Stream mode option was not set\n")
    			}
            	packet_count = 0;
                break;
                
            case APP_CMD_STOP:
				m_new_command_received = APP_CMD_NOCOMMAND;
            	if(m_receive_mode_active)
            	{
                    m_receive_mode_active = 0;
                    NRF_LOG_INFO("Receive mode disabled.\n");
            	}
            	else
            	{
					m_stream_mode_active = 0;
					cc1200_ppi_spi_disable();
					NRF_LOG_INFO("Stream mode disabled\r\n");
					NRF_LOG_INFO("Packets: %d, of size %d; PPI was paused %d times.\n",packet_count,BLE_NUS_tx_num_bytes, cc1200_get_PPI_paused_count());
                }
                break;
                
            case APP_CMD_CHANGE_FREQ:
                m_new_command_received = APP_CMD_NOCOMMAND;
                tune_frequency();// ISM freq band checking done in function call
                break;
                
            case APP_CMD_RECEIVE:
                m_new_command_received = APP_CMD_NOCOMMAND;
                m_receive_mode_active = 1;
				//cc1200_write_register(CC1200_IOCFG2, 0x01); // configure GPIO for RXFIFO indicator
				//cc1200_write_register(CC1200_FIFO_CFG, 0x0D); // trigger on 13 (0x0D) bytes in RXFIFO
                break;

            case APP_CMD_TRANSMIT:
                m_new_command_received = APP_CMD_NOCOMMAND;
                if (m_transmit_mode == TRANSMIT_MSG)
                {
					cc1200_cmd_strobe(CC1200_SFTX);
					nrf_delay_ms(20);
					cc1200_write_txfifo(m_RF_txbytes,100); // should be the lesser of length or (length tx fifo buffer)
					nrf_delay_ms(20);
					cc1200_cmd_strobe(CC1200_STX); // should do status an error checking too..
					nrf_delay_ms(20);
					cc1200_cmd_strobe(CC1200_SFTX);
                }
                else if(m_transmit_mode == TRANSMIT_CONTINOUS_TONE)
                {
                    NRF_LOG_INFO("Continuous Transmit mode Enabled\r\n");
                	// cc1200_write_register(CC1200_IOCFG0, 0x33); // configure GPIO hardwired to 0 for transmit serial input
                    cc1200_write_register(CC1200_MDMCFG1, 0x06); // disable fifo and turn on fixed DVGA gain
                    cc1200_write_register(CC1200_PKT_CFG2, 0x05); // change packet mode and format
                    cc1200_write_register(CC1200_MDMCFG2, 0x01);//3); // enable custom freq mod, no upsampling
                    cc1200_write_register(CC1200_SERIAL_STATUS, 0x08); // enable sychroninize for io pins

					cc1200_cmd_strobe(CC1200_STX); // should do status an error checking too..
                    // while(ble_spi_tx){}     
                }
                break;

            case APP_CMD_CHANGE_PHY:   
                m_new_command_received = APP_CMD_NOCOMMAND;
            
                NRF_LOG_INFO("Attempting to change phy.\r\n");
                gap_phys_settings.tx_phys = (m_new_phy == 0 ? BLE_GAP_PHY_1MBPS : BLE_GAP_PHY_2MBPS);  
                gap_phys_settings.rx_phys = (m_new_phy == 0 ? BLE_GAP_PHY_1MBPS : BLE_GAP_PHY_2MBPS);  
#ifdef S140            
                sd_ble_gap_phy_request(m_nus.conn_handle, &gap_phys_settings);
#else            
                sd_ble_gap_phy_update(m_nus.conn_handle, &gap_phys_settings);
#endif
                break;
            
            /*case APP_CMD_SEND_BLE_PARAMS:
                m_new_command_received = APP_CMD_NOCOMMAND;
                ble_nus_ble_params_info_send(&m_nus, &m_ble_params_info);
                break;*/
            
            default:
                break;
        }
        
        if(m_stream_mode_active) // processing streaming data
        {
        	/* packing sample bytes! */
        	/* starting with sample_buf bytes 0-4: [XXXXXXXM16] [M15...M8] [M7...M0] [XXXXXXA9A8] [A7...A0] where M = mag, A = angle bit
        	 * result becomes packed_msg bytes 0-2: [M15...M8] [M7...M3A9A8M16] [A7...A0] kind of messy, but should be quick
        	 */

        	do{
        		sample_buf = (uint8_t *)cc1200_get_rx_buffer();
        	}while(sample_buf == 0 && m_stream_mode_active); // otherwise it could get stuck here if PPI has a problem


			switch(m_stream_mode_active)
			{
			case MEASUREMENT_OPTION_MAGNITUDE:
				//mean_sample = 0;
				for (int i = 0; i < BLE_NUS_MAX_DATA_LEN/3; i++) // still need to fix this one
				{
					packed_msg[i*3] = sample_buf[7*i+3]; // byte 2 of 5 becomes byte 1 of 3 (mag)
					packed_msg[i*3+1] = (sample_buf[7*i+4] & 0xF8) | (sample_buf[7*i+2] & 1) | ((sample_buf[7*i+5] & 0x03) << 1); // cram everything else into byte 2
					packed_msg[i*3+2] = sample_buf[7*i+6]; // byte 5 of 5 becomes byte 3 of 3 (ang)
					//mean_sample += ((sample_buf[7*i+2] & 1) << 16) + (sample_buf[7*i+3] << 8) + sample_buf[7*i+4];
				}
        		//mean_sample = mean_sample*3/BLE_NUS_MAX_DATA_LEN; // not useful if AGC is running all over the place..
				break;
			case MEASUREMENT_OPTION_RSSI_DIRECT:
				mean_sample = 0;
        		for (int i = 0; i < BLE_NUS_MAX_DATA_LEN; i++)
        		{
        			packed_msg[i] = sample_buf[i*3+2];
        			mean_sample += packed_msg[i];
        		}
        		mean_sample = (mean_sample/BLE_NUS_MAX_DATA_LEN) << 3; // because we are only taking 8 high bits of 11 total..
        		break;
			case MEASUREMENT_OPTION_RSSI_CALCULATED:
				// TODO: Calculate and process RSSI samples
                for(int j=0;j<=100;j+=100)
	            {
                    for(int i=0;i<100;i++){
	                    magnitude = ((sample_buf[(i*5)+2+j]<<16)&0xFF0000) + ((sample_buf[(i*5)+3+j]<<8)&0x00FF00)
	                                 + (sample_buf[(i*5)+4+j]&0x0000FF);
	                    total_mag +=  pow(magnitude,2);
	                }
	                rss_value = ((10*log10(total_mag/100))+30);
                    if(rss_value < 0){rss_value *= -1;}
	                // NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(rss_value));
                    packed_msg[j/50] = rss_value;
                    packed_msg[(j/50)+1] = (rss_value*100) - (packed_msg[j/50]*100);
                    // NRF_LOG_INFO("value is %d %d \r\n",packed_msg[j/50],j/50+1);
                    total_mag = 0;
                }
                    // NRF_LOG_HEXDUMP_INFO(packed_msg,2);
                BLE_NUS_tx_num_bytes = 4;
	                //NRF_LOG_FLUSH();
				break;
			case MEASUREMENT_OPTION_PHASE:
				sum_angle = 0;
				mean_sample = 0;
				packed_msg[0] = sample_buf[2];
				packed_msg[1] = sample_buf[3];
				previous = (packed_msg[0] << 8) + packed_msg[1]; // set the initial delta so it's not way offq
				for (int i = 1; i < BLE_NUS_MAX_DATA_LEN/2; i++) // only packing the angle measurements
				{
					packed_msg[i*2] = sample_buf[i*4+2]; // should be ANG1 register sample
					packed_msg[i*2+1] = sample_buf[i*4+3]; // should be ANG0 register
					tmp = (packed_msg[i*2] << 8) + packed_msg[i*2+1];
					delta = tmp - previous;
					previous = tmp;
					if (delta < -512)
						delta += 1024;
					else if (delta > 512)
						delta -= 1024;
					sum_angle += delta;
				}
				mean_sample = (sum_angle);///1024);
				break;
			default:
				break;
			}

#ifdef DEBUG
//         	//packed_msg[0] = sample_buf[1001] + '0'; // used to put a counter in each packet
 //DEBUG
			packet_number = sample_buf[1001];
			capture_duration = (sample_buf[1002] << 24) + (sample_buf[1003] << 16) + (sample_buf[1004] << 8) + sample_buf[1005];
			capture_duration = (capture_duration *625)/10; // convert ticks to nanoseconds
        	// done processing samples, so don't need access to sample_buf now, change status
#endif
        	cc1200_rx_buffer_processed();

			do{
				err_code = ble_nus_string_send(&m_nus,packed_msg,BLE_NUS_tx_num_bytes);
			}while(err_code == NRF_ERROR_RESOURCES);
			//NRF_LOG_INFO("packet number %d | capture duration %u (ns) mean_sample: %d\n", packet_number, capture_duration, mean_sample);
			//NRF_LOG_HEXDUMP_INFO(packed_msg,BLE_NUS_tx_num_bytes);
			packet_number = packet_number;
        	if (err_code== NRF_SUCCESS)
        	{
        		//NRF_LOG_HEXDUMP_DEBUG(packed_msg,BLE_NUS_MAX_DATA_LEN);
        		packet_count++;
        	}
        	else
        		NRF_LOG_INFO("We have error code %d\n",err_code);
        }



        if(m_receive_mode_active)
        {

    			//cc1200_read_register(CC1200_MARC_STATUS1, &status);
    			//NRF_LOG_INFO(" status is %02x\r\n", status)
    			nrf_delay_ms(20); // this needs to be long enough to allow rxfifo to fill

				cc1200_read_register(CC1200_NUM_RXBYTES, &rxbytes);
				if (rxbytes > 0) // transmitter currently repeatedly sends 13 bytes, so this should get all of them chars.
				{
					cc1200_read_register(CC1200_MARCSTATE, &status);
					if ((status & 0x1F) == CC1200_MARC_STATE_RX_FIFO_ERR)
					{
						NRF_LOG_INFO("status was %d, clearing RX FIFO\n",status);
						cc1200_cmd_strobe(CC1200_SFRX);
					}
					else
					{
						 NRF_LOG_INFO("In else");
						int rx_fifo_bytes = rxbytes - 2;
						cc1200_read_rxfifo(packed_msg, rx_fifo_bytes);

						NRF_LOG_INFO("MSG Received! DATA: ");
						if(packed_msg != 0)
						{
							NRF_LOG_HEXDUMP_INFO(packed_msg, rx_fifo_bytes+1);
							ble_nus_string_send(&m_nus,packed_msg,rx_fifo_bytes+1);
						}

						//int i;
						//for (i = 0; i < rx_fifo_bytes; i++)

						NRF_LOG_INFO(" bytes: %d\r\n", rx_fifo_bytes);

						cc1200_cmd_strobe(CC1200_SFRX);
					}
				}
				else
					;//NRF_LOG_INFO("no bytes in RX FIFO\n");

    			cc1200_cmd_strobe(CC1200_SRX);
        }

        
        if(m_new_command_received == APP_CMD_NOCOMMAND)
        {
                NRF_LOG_FLUSH();
            	power_manage(); // will wake up on event (including BLE TX complete)
        }
    }
}

