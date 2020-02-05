/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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

/**
 * @brief Multiperipheral Sample Application main file.
 *
 * This file contains the source code for a sample server application with multiple peripheral connections using the LED Button service.
 */

// #define NRFX_SAADC_ENABLED 1

#include "app_button.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_lbs.h"
#include "ble_nus.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrfx_saadc.h"
#include "nrf_drv_saadc.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <stdint.h>
#include <string.h>

#include "app_uart.h"

//Bootloader section
#define BOOTLOADER_DFU_START 0xB1
//
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(200, UNIT_0_625_MS)
#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */
#define LINK_TOTAL NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                       NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define ADVERTISING_LED BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED BSP_BOARD_LED_1                           /**< Is on when device has connected. */
#define LEDBUTTON_LED BSP_BOARD_LED_2                           /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON BSP_BUTTON_0                           /**< Button that will trigger the notification event with the LED Button Service */
#define APP_ADV_INTERVAL 64                                     /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS)      /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)      /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY 0                                         /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)        /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)     /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                          /**< Number of attempts before giving up the connection parameter negotiation. */
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)              /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

// BEACON
#define APP_BEACON_INFO_LENGTH 0x17   /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH 0x15      /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE 0x02          /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI 0xC3        /**< The Beacon's measured RSSI at 1 meter distance in dBm. */


//#define APP_COMPANY_IDENTIFIER 0x0059 /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_COMPANY_IDENTIFIER 0x004C /**< Company identifier for Apple. as per www.bluetooth.org. */


#define APP_MAJOR_VALUE 0x00, 0x00    /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE 0x00, 0x00    /**< Minor value used to identify Beacons. */

#define APP_BEACON_UUID 0x01, 0x12, 0x23, 0x34, \
                        0x45, 0x56, 0x67, 0x78, \
                        0x89, 0x9a, 0xab, 0xbc, \
                        0xcd, 0xde, 0xef, 0xf0 /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO 3

static uint8_t m_adv_handle = 26;
//static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */
static uint8_t m2_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m2_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */
static ble_gap_adv_params_t beac_adv_params;

ble_nus_init_t nus_init;

const uint8_t *DEVICE_NAME = "NORDIC_USART"; /**< Name of device. Will be included in the advertising data. */
const uint8_t *BEACON_NAME = "iBeacon";
volatile uint16_t ADC[3]= {0,0,0};
volatile uint16_t Temp = 1000,
                  Pres = 2000,
                  Voltage =3000;

/**@brief Struct that contains pointers to the encoded advertising data. */
//normal advert
int adv_flag = 0;
int not_connected = 1;
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

static ble_gap_adv_data_t m2_adv_data =
{
    .adv_data =
    {
        .p_data = m2_enc_advdata,
        .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len = 0

    }
};

BLE_LBS_DEF(m_lbs);                                    /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                              /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< Context for the Queued Write module.*/
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);     /**< NRF UART Service instance. */
BLE_ADVERTISING_DEF(m_advertising);

static void nus_data_handler(ble_nus_evt_t *p_evt) {

    if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        //If recieve 'd' then programm run in bootloader mode
        if (p_evt->params.rx_data.length==1) {
            int rx_char =p_evt->params.rx_data.p_data[0];
            if(rx_char=='d')
            {
                NRF_POWER->GPREGRET = BOOTLOADER_DFU_START;
                NVIC_SystemReset();
            }
        }
        //end of bootloader part

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++) {
            do {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY)) {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r') {
            while (app_uart_put('\n') == NRF_ERROR_BUSY)
                ;
        }
    }
}

//

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void) {
    bsp_board_init(BSP_INIT_LEDS);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void) {
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void) {
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for initializing the Advertising functionality.BLE_ADV_MODE_FAST BLE_ADV_MODE_SLOW
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,    // Manufacturer specific information. Specifies the device type in this
                        // implementation.
    APP_ADV_DATA_LENGTH,// Manufacturer specific information. Specifies the length of the
                        // manufacturer specific data in this implementation.
    APP_BEACON_UUID,    // 128 bit UUID value.
    APP_MAJOR_VALUE,    // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,    // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI   // Manufacturer specific information. The Beacon's measured TX power in
                        // this implementation.
};

/**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_connected(const ble_gap_evt_t *const p_gap_evt) {
    ret_code_t err_code;
    uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection with link 0x%x established.", p_gap_evt->conn_handle);
    not_connected = 0;
    // Assign connection handle to available instance of QWR module.
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID) {
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

    // Update LEDs
    bsp_board_led_on(CONNECTED_LED);
    if (periph_link_cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT) {
        bsp_board_led_off(ADVERTISING_LED);
    } else {
        // Continue advertising. More connections can be established because the maximum link count has not been reached.
        advertising_start();
    }
}

/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_disconnected(ble_gap_evt_t const *const p_gap_evt) {
    ret_code_t err_code;
    uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.
    not_connected = 1;
    NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                 p_gap_evt->conn_handle,
                 p_gap_evt->params.disconnected.reason);

    if (periph_link_cnt == 0) {
        bsp_board_led_off(CONNECTED_LED);
        err_code = app_button_disable();
        APP_ERROR_CHECK(err_code);
    }

    if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1)) {
        // Advertising is not running when all connections are taken, and must therefore be started.
        advertising_start();
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        on_connected(&p_ble_evt->evt.gap_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnected(&p_ble_evt->evt.gap_evt);
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                               BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                               NULL,
                                               NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GAP_EVT_ADV_SET_TERMINATED:
        NRF_LOG_DEBUG(" Adv set terminated");
        bsp_board_led_off(ADVERTISING_LED);
        // Change MAC-address and advdata
        adv_flag = (adv_flag + 1) % 2;
        if (adv_flag == 1) {
            static ble_gap_addr_t m_central_addr;
            m_central_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
            m_central_addr.addr[0] = (uint8_t)NRF_FICR->DEVICEADDR[0] - 1;
            m_central_addr.addr[1] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 8);
            m_central_addr.addr[2] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 16);
            m_central_addr.addr[3] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 24);
            m_central_addr.addr[4] = (uint8_t)NRF_FICR->DEVICEADDR[1];
            m_central_addr.addr[5] = (uint8_t)((NRF_FICR->DEVICEADDR[1] >> 8) | 0xC0); // 2MSB must be set 11
            sd_ble_gap_addr_set(&m_central_addr);
            sd_ble_gap_adv_stop(&m_advertising.adv_handle); 
            m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;  
            ble_advdata_encode(&m_advertising.adv_data, &m_advertising.enc_advdata, &m_advertising.adv_data.adv_data.len);
            ble_advertising_advdata_update(&m_advertising, &m2_adv_data, true);
        } else {
            if(not_connected==1){
              static ble_gap_addr_t m_central_addr;
              m_central_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
              m_central_addr.addr[0] = (uint8_t)NRF_FICR->DEVICEADDR[0];
              m_central_addr.addr[1] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 8);
              m_central_addr.addr[2] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 16);
              m_central_addr.addr[3] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 24);
              m_central_addr.addr[4] = (uint8_t)NRF_FICR->DEVICEADDR[1];
              m_central_addr.addr[5] = (uint8_t)((NRF_FICR->DEVICEADDR[1] >> 8) | 0xC0); // 2MSB must be set 11
              sd_ble_gap_addr_set(&m_central_addr);
              sd_ble_gap_adv_stop( m_adv_handle);
              m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
              ble_advertising_advdata_update(&m_advertising, &m_adv_data, true);
           }
        }
        advertising_start();
        break;
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}

static void advertising_init(void) {
    ret_code_t err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_advdata_t beac_advdata;
    ble_advdata_t beac_srdata;
    ble_advertising_init_t init;

    //Beacon advertise data
    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
    manuf_specific_data.data.size = APP_BEACON_INFO_LENGTH;
   
    memset(&beac_srdata, 0, sizeof(beac_srdata));
    beac_srdata.name_type = BLE_ADVDATA_FULL_NAME;

    memset(&beac_advdata, 0, sizeof(beac_advdata));
    beac_advdata.name_type = BLE_ADVDATA_NO_NAME;
    beac_advdata.short_name_len = 7;
    beac_advdata.include_appearance = false;
    beac_advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    beac_advdata.p_manuf_specific_data = &manuf_specific_data;
    beac_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
    beac_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    beac_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    beac_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    beac_adv_params.duration        = 100;       

    //NRF UART SERVICE advertise data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    ble_uuid_t adv_uuids[] = {
        //{LBS_UUID_SERVICE, m_lbs.uuid_type},
        {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
    };
    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids = adv_uuids;

    //Change device name
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, DEVICE_NAME, strlen(DEVICE_NAME));

    //Encode advertisement data for Beacon
    //err_code = sd_ble_gap_device_name_set(&sec_mode, BEACON_NAME, strlen(BEACON_NAME));
    err_code = ble_advdata_encode(&beac_advdata, m2_adv_data.adv_data.p_data, &m2_adv_data.adv_data.len);
    //APP_ERROR_CHECK(err_code);
    //err_code = ble_advdata_encode(&beac_srdata, m2_adv_data.scan_rsp_data.p_data, &m2_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);
    //Encode advertisement data for NRF UART service
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);
    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);


    
    //Setting advertisement parameter initialisation data
    memset(&init, 0, sizeof(init));
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //init.advdata.p_manuf_specific_data = &manuf_specific_data;
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = 500;
    init.evt_handler = &ble_evt_handler; //&on_adv_evt;

    m_advertising.adv_params.p_peer_addr = NULL;
    m_advertising.adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    m_advertising.adv_params.interval = APP_ADV_INTERVAL;
    m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    m_advertising.adv_params.duration = 50;
    m_advertising.adv_params.primary_phy = BLE_GAP_PHY_1MBPS;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t *p_lbs, uint8_t led_state) {
    if (led_state) {
        bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON from link 0x%x!", conn_handle);
    } else {
        bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF from link 0x%x!", conn_handle);
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void) {
    ret_code_t err_code;
    ble_lbs_init_t init;
    //    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module instances.
    qwr_init.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < LINK_TOTAL; i++) {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    ble_conn_state_init();
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = true;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for writing to the LED characteristic of all connected clients.
 *
 * @details Based on if the button is pressed or released, this function writes a high or low
 *          LED status to the server.
 *
 * @param[in] button_action The button action (press/release).
 *            Determines if the LEDs of the servers will be ON or OFF.
 *
 * @return If successful NRF_SUCCESS is returned. Otherwise, the error code from @ref ble_lbs_led_status_send.
 */
static uint32_t led_status_send_to_all(uint8_t button_action) {
    ret_code_t err_code;
    ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();

    for (uint8_t i = 0; i < conn_handles.len; i++) {
        err_code = ble_lbs_on_button_change(conn_handles.conn_handles[i], &m_lbs, button_action);

        if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("Sent button change 0x%x on connection handle 0x%x.", button_action, conn_handles.conn_handles[i]);
        }
    }
    return NRF_SUCCESS;
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action) {
    ret_code_t err_code;

    switch (pin_no) {
    case LEDBUTTON_BUTTON:
        err_code = led_status_send_to_all(button_action);
        if (err_code == NRF_SUCCESS) {
            NRF_LOG_INFO("Sent button state change to all connected centrals.");
        }
        break;

    default:
        APP_ERROR_HANDLER(pin_no);
        break;
    }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void) {
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the logging module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {


    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }

}



static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;

void uart_event_handle(void)
{
    static uint8_t data_array[]= {0,1,2,3,4,5};
    uint16_t length = sizeof(data_array);
    ble_nus_data_send(&m_nus, 6, &length, m_conn_handle);
}

//ADC Part set timer and events
int Get_Internal_TEMP(void){
    NRF_TEMP->TASKS_START =1;
    while(NRF_TEMP->EVENTS_DATARDY==0);
    NRF_TEMP->TASKS_STOP=1;
    NRF_TEMP->EVENTS_DATARDY=0;
    return (NRF_TEMP->TEMP *0.25);
}
APP_TIMER_DEF(my_IRQ);
uint32_t ADC_Result2 (uint8_t CH);

static void my_IRQ_handler (void * p_context)
{
    static int click=0;
    static uint8_t tik=0;

    //led_status_send_to_all(tik++);
    //ble_lbs_on_button_change(0, &m_lbs, tik++);
    //uart_event_handle();
    tik++;

    ble_gatts_hvx_params_t     hvx_params;
    uint16_t len = sizeof(tik);
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = 16;
    hvx_params.p_data = &tik;
    hvx_params.p_len  = &len;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    sd_ble_gatts_hvx(0, &hvx_params);

    //  ble_gatts_hvx_params_t     hvx_params;
    //  uint16_t len = sizeof(tik);

    char text1[] = "T=";
    char text2[] = "W=";
    char text3[] = "U=";

    //nrfx_saadc_sample_convert(NRF_SAADC_INPUT_AIN1,&ADC);
    Temp = 0;
    Pres =0;
    Voltage =0;

    ADC_Result (&Pres,0);   // read pres A0 P0.02
    ADC_Result (&Temp,1);   // read temp A1 P0.03
    ADC_Result (&Voltage,2);// read vbat A3 P0.04

    //Voltage = ADC_Result2 (2);
    //nrf_drv_saadc_buffer_convert(&Temp,1 );
    Temp /= 1.1477;// convert to mv
    Pres /= 1.1477;// convert to mv
    //Voltage  /= 1.1477;// convert to mv
    // get internal temp sensor value
//    sd_temp_get(&Temp);
//    Temp*=25;
    char str[30];//20 ok
    // data with internal temp sensor string
//    sprintf (str, "%s%d,%02d %s%d %s%d      ", text1, (Temp%10000-Temp%100)/100,Temp%100, text2, Pres, text3, Voltage);
    // data with tmp36 temp sensor string
    Temp = Temp - 500;
    sprintf (str, "%s%d,%2d %s%d %s%d      ", text1, Temp/10,Temp%10, text2, Pres, text3, Voltage);
    len = 20;//sizeof(str);

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = 24;
    hvx_params.p_data = &str;
    hvx_params.p_len  = &len;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    sd_ble_gatts_hvx(0, &hvx_params);

    if (click==0)
    {
        bsp_board_led_on(BSP_BOARD_LED_1);
        app_timer_stop(my_IRQ);
        app_timer_start(my_IRQ, APP_TIMER_TICKS(750),NULL);
        click=1;
        return;
    }
    if (click==1)
    {
        bsp_board_led_off(BSP_BOARD_LED_1);
        app_timer_stop(my_IRQ);
        app_timer_start(my_IRQ, APP_TIMER_TICKS(750),NULL);
        click=0;
    }
}


uint16_t CH_1;

void saadc_callback(void)
{
    //   nrf_drv_saadc_buffer_convert(&CH_1,1 );
}


void ADC_init(void) {
    NRF_SAADC-> ENABLE = 1;
    NRF_SAADC-> CH [0].PSELP = 2;
    NRF_SAADC-> CH [0].PSELN = 0;
    NRF_SAADC-> CH [0].CONFIG = 0x00020000;
    /*
    NRF_SAADC-> CH [1].PSELP = 2;
    NRF_SAADC-> CH [1].PSELN = 0;
    NRF_SAADC-> CH [1].CONFIG = 0x00020000;
    
    NRF_SAADC-> CH [2].PSELP = 3;
    NRF_SAADC-> CH [2].PSELN = 0;
    NRF_SAADC-> CH [2].CONFIG = 0x00020000;
    */
    NRF_SAADC-> RESOLUTION = 2;
    /*
    NRF_SAADC-> RESULT.MAXCNT=1;//
    NRF_SAADC-> RESULT.PTR;// Pointer
    NRF_SAADC-> STATUS;// R 1-BUSY 0-Ready
    NRF_SAADC->TASKS_START = 1;
    while(NRF_SAADC-> STATUS==1);
    NRF_SAADC->TASKS_STOP=1;
     */

    //nrf_drv_saadc_init(NULL, saadc_callback);
}

uint32_t ADC_Result2 (uint8_t CH)
{
    //NRF_SAADC->TASKS_SAMPLE = 1;
    //while(NRF_SAADC-> EVENTS_DONE==0);
    //NRF_SAADC->TASKS_STOP=1;
    uint32_t i = ((uint32_t*)(NRF_SAADC-> RESULT.PTR))[CH];// Pointer
    return ((uint32_t*)NRF_SAADC-> RESULT.PTR)[CH];;
}

void ADC_Result (uint32_t pointer, uint8_t CH)
{
    NRF_SAADC-> ENABLE = 1;

    NRF_SAADC-> CH [0].PSELP = CH+1;
    NRF_SAADC-> CH [0].PSELN = 0;
    NRF_SAADC-> CH [0].CONFIG = 0x00020000;

    NRF_SAADC-> RESOLUTION = 2;

    NRF_SAADC-> RESULT.PTR = pointer;// Pointer
    NRF_SAADC-> RESULT.MAXCNT=1;

    NRF_SAADC->TASKS_SAMPLE = 1;
    NRF_SAADC->TASKS_START = 1;
    while(NRF_SAADC-> EVENTS_RESULTDONE==0);

    NRF_SAADC->TASKS_STOP=1;
    NRF_SAADC-> EVENTS_RESULTDONE =0;
}

void init_saadc()
{
    ret_code_t err_code;
    /*Config the channel*/
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    //channel_config.acq_time = NRF_SAADC_ACQTIME_20US;

    err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    // err_code = nrf_drv_saadc_buffer_convert(m_buffer,SAMPLES_IN_BUFFER);
    // APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void) {
    // Initialize.
    log_init();
    timers_init();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    advertising_start();
    //init_saadc();
    //ADC_init();
    //saadc_init();
    //saadc_sampling_event_init();
    //saadc_sampling_event_enable();
    app_timer_create(&my_IRQ, APP_TIMER_MODE_REPEATED,my_IRQ_handler);
    app_timer_start(my_IRQ, APP_TIMER_TICKS(50),NULL);
    // Start execution.
    NRF_LOG_INFO("Beacon and NRF UART Service Started");
    // Enter main loop.    
    ret_code_t err_code;
    for (;;) {        
        idle_state_handle();
    }
}

