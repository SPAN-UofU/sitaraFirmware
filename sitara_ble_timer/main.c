/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
/* Defines and register set */
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
#include "cc1200-802154g-434mhz-2gfsk-50kbps-cw.h"
#include <math.h>
#include "cc1200.h"
#include "nrf_drv_spi.h"
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
#include "ble_nus.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CONN_CFG_TAG                    1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(300, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static nrf_ble_gatt_t                   m_gatt;                                     /**< GATT module instance. */
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static uint16_t  m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;  /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


#define ARR_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MAGN_VALID_PIN NRF_GPIO_PIN_MAP(1,15)
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define FREQ_DIVIDER 1000
#define FREQ_MULTIPLIER 6826.67
/*
 * Divider + multiplier for calculation of FREQ registers
 * f * 2^16 * 4 / 40000 = f * 2^12 / 625 (no overflow up to frequencies of
 * 1048.576 MHz using uint32_t)
 */
/*---------------------------------------------------------------------------*/
/* RF configuration */
/*---------------------------------------------------------------------------*/
/* Import the rf configuration set by CC1200_RF_CFG */

extern const cc1200_rf_cfg_t CC1200_RF_CFG;
#define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps_cw

const nrf_drv_timer_t TIMER_BLE = NRF_DRV_TIMER_INSTANCE(1);
uint32_t time_ms = 180000; //Time(in miliseconds) between consecutive compare events.
//uint32_t time_ms = 50000;

static volatile bool ble_spi_rx = false;
static volatile bool ble_spi_tx = false;
static volatile bool timer1_flag = false;
static volatile bool chan_updated = false;
static volatile bool chan_change = false;
uint8_t spimsg[10] = {0};
uint8_t sample = false;
uint32_t total_mag[8] = {0};
uint8_t final_msg[20] = {};
uint8_t cmd_ack[] = {'c','m','d',' ','a','c','k'};
int channel_number = 0;
static int frequency_calc(int channel){
    uint32_t frequency;
    frequency = CC1200_RF_CFG.chan_center_freq0 + (channel * CC1200_RF_CFG.chan_spacing);
    frequency *= FREQ_MULTIPLIER;
    frequency /= FREQ_DIVIDER;
    NRF_LOG_INFO("frequency is %d\r\n", frequency)
    return frequency; 
}

static void set_rf_channel(int channel){
    uint32_t freq;
    NRF_LOG_INFO("Channel is %d\r\n",channel);
    NRF_LOG_FLUSH();
    if(channel < CC1200_RF_CFG.min_channel ||channel > CC1200_RF_CFG.max_channel) {
        chan_updated = false; /* Invalid channel */        
    }
    else{
        NRF_LOG_INFO("in freq update\r\n");
        NRF_LOG_FLUSH();
        freq = frequency_calc(channel - CC1200_RF_CFG.min_channel);
        cc1200_write_register(CC1200_FREQ0,((uint8_t *)&freq)[0]);
        cc1200_write_register(CC1200_FREQ1,((uint8_t *)&freq)[1]);
        cc1200_write_register(CC1200_FREQ2,((uint8_t *)&freq)[2]);
        chan_updated = true;
    }
}





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


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t err_code;
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
    NRF_LOG_HEXDUMP_INFO(p_data, length);
    if(p_data[0] == '1' && p_data[1] == '1')
    {
        if(p_data[2] == '1'){
            NRF_LOG_HEXDUMP_INFO(p_data, length);
            p_data[2] = '0';
            chan_change = true;
            channel_number = (1000 * (p_data[3]-'0'))+(100*(p_data[4]-'0')) + (10*(p_data[5]-'0')) + (p_data[6]-'0');
            memset(p_data,0,ARR_SIZE(p_data));
        }
        ble_spi_rx = true;
        ble_spi_tx = false;
        timer1_flag = false;
    }

    else if(p_data[0] == '0' && p_data[1] == '1')
    {
        if(p_data[2] == '1'){
            NRF_LOG_HEXDUMP_INFO(p_data, length);
            p_data[2] = '0';
            chan_change = true;
            //NRF_LOG_INFO("is %d\r\n",(1000 * (p_data[3]-'0'))+(100*(p_data[4]-'0'))+(10*(p_data[5]-'0'))+(p_data[6]-'0'));
            channel_number = (1000 * (p_data[3]-'0'))+(100*(p_data[4]-'0')) + (10*(p_data[5]-'0')) + (p_data[6]-'0');
            memset(p_data,0,ARR_SIZE(p_data));

            //NRF_LOG_INFO("channel_number in handler is %d\r\n",channel_number);         
        }
        ble_spi_rx = false;
        ble_spi_tx = true;
        timer1_flag = false;
    }
    else if(p_data[0] == '1' && p_data[1] == '0'){
        if(ble_spi_tx){         
            ble_spi_tx = false;
            timer1_flag = true;
            ble_spi_tx = true;}
        if(ble_spi_rx){
            ble_spi_rx = false;
            timer1_flag = true;
            ble_spi_rx = true;
        }
    }
    else if(p_data[1] == '0' && p_data[0] == '0')
    {
        ble_spi_rx = false;
        ble_spi_tx = false;
        timer1_flag = false;
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
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            NRF_LOG_INFO("Connected\r\n");
            NRF_LOG_FLUSH();
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            NRF_LOG_INFO("Disconnected\r\n");
            NRF_LOG_FLUSH();

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

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

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
        NRF_LOG_INFO("Data len is set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        NRF_LOG_FLUSH();

    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
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




/**@brief Function for initializing the Advertising functionality.
 */
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
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

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


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
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

void magn_vadid_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Disable 
    nrf_drv_gpiote_in_event_disable(MAGN_VALID_PIN);
    sample = true;
    // Clear
    nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_IN_0);
    // Enable
    nrf_drv_gpiote_in_event_enable(MAGN_VALID_PIN, true);
}



static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t magn_valid_pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    magn_valid_pin_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(MAGN_VALID_PIN, &magn_valid_pin_config, magn_vadid_handler);
    APP_ERROR_CHECK(err_code);
}

void set_timer(uint32_t timer_ms){
    nrf_drv_timer_disable(&TIMER_BLE);
    uint32_t time_ticks;
    //NRF_LOG_INFO("timer set\r\n");
    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_BLE, timer_ms);
    nrf_drv_timer_extended_compare(
         &TIMER_BLE, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_BLE);
}

void timer_ble_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            ble_spi_rx = false;
            ble_spi_tx = false;
            //NRF_LOG_INFO("time up\r\n");
            nrf_drv_timer_disable(&TIMER_BLE);
            break;

        default:
            //Do nothing.
            break;
    }
}
/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
    cc1200_init(); // Initialize cc1200
    uint8_t frequency[3] = {0};

    gpio_init();
    bsp_board_leds_init();

    // Initialize.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);    
    log_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);
    cc1200_burst_read_register(CC1200_FREQ2,frequency,3);
    NRF_LOG_INFO("initial frequency is %d\r\n",(frequency[0]*1000000) + (frequency[1]*1000)+(frequency[2]));
    NRF_LOG_FLUSH();
    nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_IN_0); // Clear
    nrf_drv_gpiote_in_event_enable(MAGN_VALID_PIN, true); // Enable interrupt
 //Configure TIMER_BLE for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_BLE, &timer_cfg, timer_ble_event_handler);
    APP_ERROR_CHECK(err_code);
    cc1200_cmd_strobe(CC1200_SRX);
    int i = 0;
    int average_mag = 0;
    // Enter main loop.
    for (;;)
    {
        if(ble_spi_rx)
        {

            cc1200_cmd_strobe(CC1200_SRX);
            ble_nus_string_send(&m_nus,cmd_ack,ARR_SIZE(cmd_ack));
            set_timer(time_ms);
        }
        while(ble_spi_rx)
        {
            if(timer1_flag){
                set_timer(time_ms);
                timer1_flag =false;
                //NRF_LOG_INFO("refresh\r\n");
            }
            if(chan_change){
                chan_change = false;
                set_rf_channel(channel_number);
            }      
            if(sample)
            {
                memset(spimsg,0,sizeof(spimsg));
                sample = false;
                cc1200_burst_read_register(CC1200_MAGN2, spimsg, 5);
                NRF_LOG_HEXDUMP_INFO(spimsg, 5);
                NRF_LOG_FLUSH();
                //nrf_delay_ms(7);
                total_mag[i] = ((spimsg[0]<<16)&0xFF0000) + ((spimsg[1]<<8)&0x00FF00) + (spimsg[2]&0x0000FF);
                final_msg[i+4] = spimsg[2];
                final_msg[i+12] = spimsg[4];
                average_mag +=  total_mag[i];
                i++;
                if(i == 8)
                {
                    i = 0;
                    average_mag = average_mag/ARR_SIZE(total_mag);
                    for(int k=0; k<8; k++)
                    {
                        if(CHECK_BIT(average_mag+(total_mag[k]&0x000000FF),17))
                        {
                            average_mag |= 1<<(k+18);
                            //NRF_LOG_INFO("check\r\n");
                        }                   
                    }
                    //NRF_LOG_INFO("avg %x\r\n",average_mag);
                    //NRF_LOG_FLUSH();
                    final_msg[0] = (average_mag >> 24) & 0xff;  // overflow 7 bits(lower 7), reserved
                    final_msg[1] = (average_mag >> 16) & 0xff; // average magnitude 1 bit(bit 0), overflow 7 bits
                    final_msg[2] = (average_mag >> 8) & 0xff; // average magnitude
                    final_msg[3] = (average_mag) & 0xff; // average magnitude
                    //NRF_LOG_HEXDUMP_INFO(final_msg,20);
                    //NRF_LOG_FLUSH();
                    for(int l = 0; l<4;l++){
                    do
                    {
                        err_code = ble_nus_string_send(&m_nus,final_msg,20);
                      //  if ( (err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY) )
                        //{
                          //  APP_ERROR_CHECK(err_code);
                    }
                    while (err_code == NRF_ERROR_BUSY);}                             
                    average_mag = 0;
                    memset(total_mag,0,sizeof(total_mag));
                    memset(final_msg,0, sizeof(final_msg));
                }
            }
        }
        if(ble_spi_tx)
        {
            NRF_LOG_INFO("TX Mode\r\n");
            NRF_LOG_FLUSH();

            cc1200_cmd_strobe(CC1200_STX);
            ble_nus_string_send(&m_nus,cmd_ack,ARR_SIZE(cmd_ack));
            set_timer(time_ms);            
            i = 0;
            average_mag = 0;
            memset(spimsg,0,sizeof(spimsg));
            memset(total_mag,0,sizeof(total_mag));
            memset(final_msg,0, sizeof(final_msg));

        }
        while(ble_spi_tx){                       
            if(timer1_flag){
                set_timer(time_ms);
                timer1_flag =false;
                //NRF_LOG_INFO("refresh\r\n");
            }
            if(chan_change){
                chan_change = false;
                NRF_LOG_INFO("update channel\r\n");
                NRF_LOG_INFO("Channel number %d\r\n",channel_number);                
                set_rf_channel(channel_number);
            }
        }

        if(!ble_spi_rx && !ble_spi_tx)
        {
            NRF_LOG_INFO("power saving mode\r\n");
            NRF_LOG_FLUSH();
            nrf_drv_timer_disable(&TIMER_BLE);
            cc1200_cmd_strobe(CC1200_SFTX);
            cc1200_cmd_strobe(CC1200_SFRX);
            i = 0;
            average_mag = 0;
            memset(spimsg,0,sizeof(spimsg));
            memset(total_mag,0,sizeof(total_mag));
            memset(final_msg,0, sizeof(final_msg));
        }
        while(!ble_spi_rx && !ble_spi_tx)
        {
            power_manage();
        }
    }
}
