/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "queue.h"
#include "Ringbuffer.h"
#include "i2c.h"
#include "lm75.h"
#include "bh1750.h"
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "nrf_timer.h"
#include "bsp.h"
#include "dht11.h"
#include "adc.h"
#include "exti.h"

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  0                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

/***************************************** ThanhBH Timer ***********************************/
#include "nrf_drv_clock.h"

#define ZONE 1

APP_TIMER_DEF(timer_send_form_central_to_peripheral_id);

#if (ZONE == 3)
APP_TIMER_DEF(timer_handler_PIR_id);
APP_TIMER_DEF(timer_handler_MQ7_id);
#endif

#define MAX_MEM_CONN_HANDLER			50
#define SIZE_OF_QUEUE   								50			//ThanhBH Add

#define FRAME_SOF 			0xB1
#define FRAME_ACK 			0x06
#define FRAME_NACK 			0x15
#define CXOR_INIT_VAL 		0xFF

#define LENG_BUFF_SENSOR		7
#define LENG_BUFF_LED				6
#define LENG_PAYLOAD_SENSOR			9
#define LENG_PAYLOAD_LED				8
#define PIN_PIR						26
#define PIN_MQ7						31
#define DIRECTION_OF_PIR						25
#define DIRECTION_OF_MQ7						27

#if (ZONE == 1)

#define CMD_ID_LM75			0x00
#define CMD_ID_BH1750		0x01
#define CMD_ID_LED_1		0x02
#define CMD_ID_LED_2		0x03
#define CMD_ID_LED_3		0x04
#define CMD_ID_LED_4		0x05
#define CMD_ID_LED_5		0x06
#define CMD_ID_LED_6		0x07
#define CMD_ID_LED_7		0x08
#define CMD_ID_LED_8		0x09

void saadc_callback_handler(nrf_drv_saadc_evt_t const *p_event)
{
	
}

#elif (ZONE == 2)

#define CMD_ID_TEMP_DHT11			0x0A
#define CMD_ID_HUMI_DHT11			0x0B
#define CMD_ID_LM35						0x0C
#define CMD_ID_LED_1					0x0D
#define CMD_ID_LED_2					0x0E
#define CMD_ID_LED_3					0x0F
#define CMD_ID_LED_4					0x10
#define CMD_ID_LED_5					0x11
#define CMD_ID_LED_6					0x12
#define CMD_ID_LED_7					0x13
#define CMD_ID_LED_8					0x14

void saadc_callback_handler(nrf_drv_saadc_evt_t const *p_event)
{
	
}

#elif (ZONE == 3)

#define CMD_ID_PIR			0x15
#define CMD_ID_MQ7			0x16
#define CMD_ID_LED_1		0x17
#define CMD_ID_LED_2		0x18
#define CMD_ID_LED_3		0x19
#define CMD_ID_LED_4		0x1A
#define CMD_ID_LED_5		0x1B
#define CMD_ID_LED_6		0x1C
#define CMD_ID_LED_7		0x1D
#define CMD_ID_LED_8		0x1E

void saadc_callback_handler(nrf_drv_saadc_evt_t const *p_event)
{
	
}

#endif

/*******************************************************************************************/

static RingBuff ringbuff;
BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/************************************************* ThanhBH Add ****************************************************/

ble_nus_c_t *p_ble_nus_c_mem = NULL;
typedef struct mem_connect_handler mem_connect_handler_t;
uint8_t arr_mem_data[SIZE_OF_QUEUE] = {0};
uint8_t byRxBuffer2[SIZE_OF_QUEUE] = {0};
struct mem_connect_handler{
	uint16_t handler_connect;
	uint8_t status;
	ble_nus_c_evt_t const *ble_nus_evt_mem;
	ble_nus_c_t *ble_nus_mem;
};

typedef struct {
	uint8_t cmdid;
	uint8_t type;
}frm_common_t;

mem_connect_handler_t mem_connect[MAX_MEM_CONN_HANDLER];
static void timer_send_form_central_to_peripheral_handler(void * p_context);

#if (ZONE == 3)
static void timer_handler_PIR_handler(void * p_context);
static void timer_handler_MQ7_handler(void * p_context);
#endif

typedef enum{
	LED_NUM_1 = 1,
	LED_NUM_2,
	LED_NUM_3,
	LED_NUM_4,
	LED_NUM_5,
	LED_NUM_6,
	LED_NUM_7,
	LED_NUM_8
}name_led_t;

typedef enum{
	ZONE_NUM_1 = 1,
	ZONE_NUM_2,
	ZONE_NUM_3
}name_zone_t;

void convert_dec_to_hex(uint16_t value, uint8_t *data_1, uint8_t *data_2)
{
	char arr_temp[10] = "", arr_temp_1[3] = "",arr_temp_2[3] = "";
	unsigned int data_raw_1 = 0, data_raw_2 = 0;
	sprintf(arr_temp,"%x",value);
	arr_temp_1[0] = arr_temp[0];
	arr_temp_1[1] = arr_temp[1];
	arr_temp_1[2] = '\0';
	arr_temp_2[0] = arr_temp[2];
	arr_temp_2[1] = arr_temp[3];
	arr_temp_2[2] = '\0';
	sscanf(arr_temp_1,"%x",&data_raw_1);
	sscanf(arr_temp_2,"%x",&data_raw_2);
	*data_1 = (uint8_t)data_raw_1;
	*data_2 = (uint8_t)data_raw_2;
}

uint8_t table_convert_to_command_id(uint8_t number_led, uint8_t number_zone)
{
	name_led_t name_led;
	name_zone_t name_zone;
	if(number_led == 1){name_led = LED_NUM_1;}
	else if(number_led == 2){name_led = LED_NUM_2;}
	else if(number_led == 3){name_led = LED_NUM_3;}
	else if(number_led == 4){name_led = LED_NUM_4;}
	else if(number_led == 5){name_led = LED_NUM_5;}
	else if(number_led == 6){name_led = LED_NUM_6;}
	else if(number_led == 7){name_led = LED_NUM_7;}
	else if(number_led == 8){name_led = LED_NUM_8;}
	
	if(number_zone == 1){name_zone = ZONE_NUM_1;}
	else if(number_zone == 2){name_zone = ZONE_NUM_2;}
	else if(number_zone == 3){name_zone = ZONE_NUM_3;}
	
	switch(name_zone)
	{
		case ZONE_NUM_1:
		{
			if(name_led == LED_NUM_1) {return 0x02;}
			else if(name_led == LED_NUM_2) {return 0x03;}
			else if(name_led == LED_NUM_3) {return 0x04;}
			else if(name_led == LED_NUM_4) {return 0x05;}
			else if(name_led == LED_NUM_5) {return 0x06;}
			else if(name_led == LED_NUM_6) {return 0x07;}
			else if(name_led == LED_NUM_7) {return 0x08;}
			else if(name_led == LED_NUM_8) {return 0x09;}
			break;
		}
		case ZONE_NUM_2:
		{
			if(name_led == LED_NUM_1) {return 0x0D;}
			else if(name_led == LED_NUM_2) {return 0x0E;}
			else if(name_led == LED_NUM_3) {return 0x0F;}
			else if(name_led == LED_NUM_4) {return 0x10;}
			else if(name_led == LED_NUM_5) {return 0x11;}
			else if(name_led == LED_NUM_6) {return 0x12;}
			else if(name_led == LED_NUM_7) {return 0x13;}
			else if(name_led == LED_NUM_8) {return 0x14;}
			break;
		}
		case ZONE_NUM_3:
		{
			if(name_led == LED_NUM_1) {return 0x17;}
			else if(name_led == LED_NUM_2) {return 0x18;}
			else if(name_led == LED_NUM_3) {return 0x19;}
			else if(name_led == LED_NUM_4) {return 0x1A;}
			else if(name_led == LED_NUM_5) {return 0x1B;}
			else if(name_led == LED_NUM_6) {return 0x1C;}
			else if(name_led == LED_NUM_7) {return 0x1D;}
			else if(name_led == LED_NUM_8) {return 0x1E;}
			break;
		}
	}
	return 0xFF;
}

void add_handler(uint16_t handler_ble_connect, ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
	for(uint16_t i = 0; i < MAX_MEM_CONN_HANDLER; i++)
	{
		if(mem_connect[i].status == 0)
		{
			mem_connect[i].handler_connect = handler_ble_connect;
			mem_connect[i].status = 1;
			mem_connect[i].ble_nus_evt_mem = p_ble_nus_evt;
			mem_connect[i].ble_nus_mem = p_ble_nus_c;
			break;
		}
	}
}

void remove_handler(uint16_t handler_ble_connect)
{
	for(uint16_t i = 0; i < MAX_MEM_CONN_HANDLER; i++)
	{
		if(mem_connect[i].handler_connect == handler_ble_connect && mem_connect[i].status == 1)
		{
			mem_connect[i].handler_connect = 0;
			mem_connect[i].status = 0;
			mem_connect[i].ble_nus_evt_mem = 0;
			mem_connect[i].ble_nus_mem = 0;
			memset(&mem_connect[i], 0, sizeof(mem_connect[i]));
			break;
		}
	}
}

void reset_handler(void)
{
	for(uint16_t i = 0; i < MAX_MEM_CONN_HANDLER; i++)
	{
		mem_connect[i].handler_connect = 0;
		mem_connect[i].status = 0;
		mem_connect[i].ble_nus_evt_mem = 0;
		mem_connect[i].ble_nus_mem = 0;
	}
	memset(mem_connect, 0, sizeof(mem_connect));
}

/******************************************************************************************************************/

/********************************************* ThanhBH Timer ***************************************/

static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void create_timers(void)
{
    ret_code_t err_code;
	
		err_code = app_timer_create(&timer_send_form_central_to_peripheral_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_send_form_central_to_peripheral_handler);
    APP_ERROR_CHECK(err_code);
#if (ZONE == 3)
		err_code = app_timer_create(&timer_handler_PIR_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_handler_PIR_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&timer_handler_MQ7_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_handler_MQ7_handler);
    APP_ERROR_CHECK(err_code);
#endif
}

/***************************************************************************************************/

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);
	
		err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, "Peripheral_Device");
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, (NRF_BLE_SCAN_UUID_FILTER | NRF_BLE_SCAN_NAME_FILTER), false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);
		NRF_LOG_INFO("Receiving data.!");
    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
						ring_buff_push(&ringbuff, p_data[i]);				//ThanhBH Add
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to the peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= (m_ble_nus_max_data_len)))
            {
//                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
//								NRF_LOG_INFO("Data_UART: %s",data_array);
                for(uint32_t i = 0; i < MAX_MEM_CONN_HANDLER; i++)
								{
									if(mem_connect[i].status == 1)
									{
										ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_array, index);
										NRF_LOG_INFO("Entry to Send %d",i);
									}
								}

                index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
//            NRF_LOG_ERROR("Communication error occurred while handling UART.");
//            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
//            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
//            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
						NRF_LOG_INFO("Discovery complete. With handler: %d",p_ble_nus_evt->conn_handle);
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
						add_handler(p_ble_nus_evt->conn_handle,p_ble_nus_c,p_ble_nus_evt);
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
						p_ble_nus_c_mem = p_ble_nus_c;
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. With handler: %d",p_ble_nus_evt->conn_handle);
						remove_handler(p_ble_nus_evt->conn_handle);
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

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
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
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


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
		uint32_t err_code = NRF_SUCCESS;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/********************************************* ThanhBH Add **************************************/

typedef enum{
	LM75,
	BH1750,
	TEMP_DHT11,
	HUMI_DHT11,
	LM35,
	PIR,
	MQ7,
	LED
}type_device_t;

typedef enum{
	GET,
	SET
}action_t;

enum{
	UART_STATE_IDLE,
	RX_STATE_START_BYTE,
	RX_STATE_DATA_BYTES,
	UART_STATE_ACK_RECEIVED,
	UART_STATE_NACK_RECEIVED,
	UART_STATE_ERROR,
	RX_STATE_CXOR_BYTE,
	UART_STATE_DATA_RECEIVED,
	UART_STATE_RX_TIMEOUT
};

static uint8_t byRxBufState = RX_STATE_START_BYTE;
static uint8_t byIndexRxBuf = 0;
static uint8_t byCheckXorRxBuf = 0;

void calcu_data_to_send(type_device_t device, action_t action, uint8_t command_id, uint8_t option, uint8_t sequen,uint8_t data_to_send[], uint8_t leng_of_data, uint8_t data[])
{
	uint8_t result_xor = 0;
	if(device == LED)
	{
		if(action == SET)
		{
			result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
			for(uint32_t i = 0; i < leng_of_data; i++)
			{
				result_xor = result_xor ^ data_to_send[i];
			}
			data[0] = FRAME_SOF; data[1] = LENG_BUFF_LED; data[2] = option; data[3] = command_id; data[4] = 0x01;
			data[5] = data_to_send[0]; data[6] = sequen; data[7] = result_xor;
		}
		else if(action == GET)
		{
			result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x02 ^ sequen);
			for(uint32_t i = 0; i < leng_of_data; i++)
			{
				result_xor = result_xor ^ data_to_send[i];
			}
			data[0] = FRAME_SOF; data[1] = LENG_BUFF_LED; data[2] = option; data[3] = command_id; data[4] = 0x02;
			data[5] = data_to_send[0]; data[6] = sequen; data[7] = result_xor;
		}
	}
	else if(device == LM75)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == BH1750)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
		NRF_LOG_INFO("\nData BH1750 Handler: %x   %x %x   %x\n",data[5], data_to_send[0], data[6], data_to_send[1]);
	}
	else if(device == TEMP_DHT11)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == HUMI_DHT11)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == LM35)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == PIR)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
	else if(device == MQ7)
	{
		result_xor = (CXOR_INIT_VAL ^ option ^ command_id ^ 0x01 ^ sequen);
		for(uint32_t i = 0; i < leng_of_data; i++)
		{
			result_xor = result_xor ^ data_to_send[i];
		}
		data[0] = FRAME_SOF; data[1] = LENG_BUFF_SENSOR; data[2] = option; data[3] = command_id; data[4] = 0x01;
		data[5] = data_to_send[0]; data[6] = data_to_send[1]; data[7] = sequen; data[8] = result_xor;
	}
}

uint8_t PollRxBuff(void)
{
	uint8_t byRxData;
	uint8_t byUartState = (uint8_t)UART_STATE_IDLE;
	while((ring_buff_available(&ringbuff) != 0) && (byUartState == UART_STATE_IDLE))
	{
		ring_buff_pop(&ringbuff, &byRxData);
		NRF_LOG_INFO("Data recive PollRxBuff: %d",byRxData);
		switch(byRxBufState)
		{
			case RX_STATE_START_BYTE:
			{
				if(byRxData == FRAME_SOF)
				{
					byIndexRxBuf = 0;
					byCheckXorRxBuf = CXOR_INIT_VAL;
					byRxBufState = RX_STATE_DATA_BYTES;
				}
				else if(byRxData == FRAME_ACK)
				{
					byUartState = UART_STATE_ACK_RECEIVED;
				}
				else if(byRxData == FRAME_NACK)
				{
					byUartState = UART_STATE_NACK_RECEIVED;
				}
				else
				{
					byUartState = UART_STATE_ERROR;
				}
				break;
			}
			case RX_STATE_DATA_BYTES:
			{
				if(byIndexRxBuf < 254)
				{
					byRxBuffer2[byIndexRxBuf] = byRxData;
					if(byIndexRxBuf > 0)
					{
						byCheckXorRxBuf ^= byRxData;
					}
					if(++byIndexRxBuf == *byRxBuffer2)
					{
						byRxBufState = RX_STATE_CXOR_BYTE;
					}
				}
				else
				{
					byRxBufState = RX_STATE_START_BYTE;
					byUartState = UART_STATE_ERROR;
				}
				break;
			}
			case RX_STATE_CXOR_BYTE:
			{
				if(byRxData == byCheckXorRxBuf)
				{
					byUartState = UART_STATE_DATA_RECEIVED;
				}
				else
				{
					byUartState = UART_STATE_ERROR;
				}
			}
			default:
				byRxBufState = RX_STATE_START_BYTE;
				break;
		}
	}
	return byUartState;
}

void UartCommandProcess(void *arg)
{
	uint32_t data_sensor = 0;
	frm_common_t *pCmd = (frm_common_t *)arg;
	
	uint8_t data_send[10] = {0};
	uint8_t data_led[2] = {byRxBuffer2[4],0};
#if (ZONE == 1)
	
	if(byRxBuffer2[1] != 1)
	{
		return;
	}

#elif (ZONE == 2)
	
	if(byRxBuffer2[1] != 2)
	{
		return;
	}
	
#elif (ZONE == 3)
	
	if(byRxBuffer2[1] != 3)
	{
		return;
	}
	
#endif
	switch(pCmd->cmdid){
		
#if (ZONE == 1)
		case CMD_ID_LM75:
		{
			data_sensor = ((byRxBuffer2[4] * 100) + byRxBuffer2[5]);
			printf("\nLM75 Need handler !\n");
			NRF_LOG_INFO("LM75 Need handler ! Data: %d",data_sensor);
			break;
		}
		case CMD_ID_BH1750:
		{
			data_sensor = ((byRxBuffer2[4] * 100) + byRxBuffer2[5]);
			printf("\nBH1750 Need handler !\n");
			NRF_LOG_INFO("BH1750 Need handler ! Data: %d",data_sensor);
			break;
		}
		case CMD_ID_LED_1:
		{
			printf("\nLED_1 Need handler !\n");
			NRF_LOG_INFO("LED_1 Need handler !");
			nrf_gpio_pin_write(2, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_1, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_2:
		{
			printf("\nLED_2 Need handler !\n");
			NRF_LOG_INFO("LED_2 Need handler !");
			nrf_gpio_pin_write(3, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_2, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_3:
		{
			printf("\nLED_3 Need handler !\n");
			NRF_LOG_INFO("LED_3 Need handler !");
			nrf_gpio_pin_write(4, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_3, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_4:
		{
			printf("\nLED_4 Need handler !\n");
			NRF_LOG_INFO("LED_4 Need handler !");
			nrf_gpio_pin_write(11, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_4, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_5:
		{
			printf("\nLED_5 Need handler !\n");
			NRF_LOG_INFO("LED_5 Need handler !");
			nrf_gpio_pin_write(12, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_5, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_6:
		{
			printf("\nLED_6 Need handler !\n");
			NRF_LOG_INFO("LED_6 Need handler !");
			nrf_gpio_pin_write(15, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_6, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_7:
		{
			printf("\nLED_7 Need handler !\n");
			NRF_LOG_INFO("LED_7 Need handler !");
			nrf_gpio_pin_write(16, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_7, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_8:
		{
			printf("\nLED_8 Need handler !\n");
			NRF_LOG_INFO("LED_8 Need handler !");
			nrf_gpio_pin_write(19, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_8, ZONE_NUM_1), ZONE_NUM_1, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		
#elif (ZONE == 2)
		
		case CMD_ID_TEMP_DHT11:
		{
			printf("\nTEMP_DHT11 Need handler !\n");
			NRF_LOG_INFO("TEMP_DHT11 Need handler !");
			break;
		}
		case CMD_ID_HUMI_DHT11:
		{
			printf("\nHUMI_DHT11 Need handler !\n");
			NRF_LOG_INFO("HUMI_DHT11 Need handler !");
			break;
		}
		case CMD_ID_LM35:
		{
			printf("\nLM35 Need handler !\n");
			NRF_LOG_INFO("LM35 Need handler !");
			break;
		}
		case CMD_ID_LED_1:
		{
			printf("\nLED_1 Need handler !\n");
			NRF_LOG_INFO("LED_1 Need handler !");
			nrf_gpio_pin_write(2, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_1, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_2:
		{
			printf("\nLED_2 Need handler !\n");
			NRF_LOG_INFO("LED_2 Need handler !");
			nrf_gpio_pin_write(3, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_2, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_3:
		{
			printf("\nLED_3 Need handler !\n");
			NRF_LOG_INFO("LED_3 Need handler !");
			nrf_gpio_pin_write(4, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_3, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_4:
		{
			printf("\nLED_4 Need handler !\n");
			NRF_LOG_INFO("LED_4 Need handler !");
			nrf_gpio_pin_write(11, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_4, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_5:
		{
			printf("\nLED_5 Need handler !\n");
			NRF_LOG_INFO("LED_5 Need handler !");
			nrf_gpio_pin_write(12, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_5, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_6:
		{
			printf("\nLED_6 Need handler !\n");
			NRF_LOG_INFO("LED_6 Need handler !");
			nrf_gpio_pin_write(15, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_6, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_7:
		{
			printf("\nLED_7 Need handler !\n");
			NRF_LOG_INFO("LED_7 Need handler !");
			nrf_gpio_pin_write(16, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_7, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_8:
		{
			printf("\nLED_8 Need handler !\n");
			NRF_LOG_INFO("LED_8 Need handler !");
			nrf_gpio_pin_write(19, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_8, ZONE_NUM_2), ZONE_NUM_2, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		
#elif (ZONE == 3)

		case CMD_ID_PIR:
		{
			printf("\nPIR Need handler !\n");
			NRF_LOG_INFO("PIR Need handler !");
			break;
		}
		case CMD_ID_MQ7:
		{
			printf("\nMQ8 Need handler !\n");
			NRF_LOG_INFO("MQ8 Need handler !");
			break;
		}
		case CMD_ID_LED_1:
		{
			printf("\nLED_1 Need handler !\n");
			NRF_LOG_INFO("LED_1 Need handler !");
			nrf_gpio_pin_write(2, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_1, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_2:
		{
			printf("\nLED_2 Need handler !\n");
			NRF_LOG_INFO("LED_2 Need handler !");
			nrf_gpio_pin_write(3, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_2, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_3:
		{
			printf("\nLED_3 Need handler !\n");
			NRF_LOG_INFO("LED_3 Need handler !");
			nrf_gpio_pin_write(4, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_3, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_4:
		{
			printf("\nLED_4 Need handler !\n");
			NRF_LOG_INFO("LED_4 Need handler !");
			nrf_gpio_pin_write(11, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_4, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_5:
		{
			printf("\nLED_5 Need handler !\n");
			NRF_LOG_INFO("LED_5 Need handler !");
			nrf_gpio_pin_write(12, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_5, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_6:
		{
			printf("\nLED_6 Need handler !\n");
			NRF_LOG_INFO("LED_6 Need handler !");
			nrf_gpio_pin_write(15, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_6, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_7:
		{
			printf("\nLED_7 Need handler !\n");
			NRF_LOG_INFO("LED_7 Need handler !");
			nrf_gpio_pin_write(16, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_7, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		case CMD_ID_LED_8:
		{
			printf("\nLED_8 Need handler !\n");
			NRF_LOG_INFO("LED_8 Need handler !");
			nrf_gpio_pin_write(19, byRxBuffer2[4]);
			calcu_data_to_send(LED, SET, table_convert_to_command_id(LED_NUM_8, ZONE_NUM_3), ZONE_NUM_3, 0,data_led, 1, data_send);
			ble_nus_c_string_send(p_ble_nus_c_mem, data_send, LENG_PAYLOAD_LED);
			break;
		}
		
#endif
		
		default:
			printf("\nData Error !!!\n");
			NRF_LOG_INFO("Data Error !!!");
			break;
	}
}

void processDataReceiver(void)
{
	uint8_t stateRx;
	stateRx = PollRxBuff();
	if(stateRx != UART_STATE_IDLE)
	{
		switch(stateRx)
		{
			case UART_STATE_ACK_RECEIVED:
			{
				printf("UART_STATE_ACK_RECEIVED\r\n");
				break;
			}
			case UART_STATE_NACK_RECEIVED:
			{
				printf("UART_STATE_NACK_RECEIVED\r\n");
				break;
			}
			case UART_STATE_DATA_RECEIVED:
			{
				printf("UART_STATE_DATA_RECEIVED\r\n");
				UartCommandProcess(&byRxBuffer2[2]);
				break;
			}
			case UART_STATE_ERROR:
			case UART_STATE_RX_TIMEOUT:
			{
				printf("UART_STATE_RX_TIMEOUT\r\n");
				break;
			}
			default:
				break;
		}
	}
}

/************************************************************************************************/

#define CENTRAL_3

#ifdef CENTRAL_1
static void timer_send_form_central_to_peripheral_handler(void * p_context)				//ThanhBH Add
{
	uint8_t data_send[10] = {0};
	uint8_t data_lm75[] = {0x00, 0x76};
	uint8_t data_bh1750[] = {0xA9, 0xF6};
	calcu_data_to_send(BH1750, SET, 0x01, 0, 0,data_bh1750, 2, data_send);
	for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
	{
		if(mem_connect[i].status == 1)
		{
			ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
		}
	}
}
#endif

#ifdef CENTRAL_2

static void timer_send_form_central_to_peripheral_handler(void * p_context)				//ThanhBH Add
{
	for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
	{
		if(mem_connect[i].status == 1)
		{
			ble_nus_c_string_send(mem_connect[i].ble_nus_mem, (uint8_t *)"Data_form_cetral_2\r\n", strlen("Data_form_cetral_2\r\n"));
		}
	}
}

#endif

#if (ZONE == 1)

static void timer_send_form_central_to_peripheral_handler(void * p_context)				//ThanhBH Add
{
	uint8_t data_raw_lux[2] = {0}, data_raw_temp[2] = {0};
	uint8_t data_send[10] = {0};
	uint16_t data_lux = 0, data_temp = 0;
	data_lux = (uint16_t)BH1750_read_lux();
	data_temp = (uint16_t)LM75_read_temp();
	convert_dec_to_hex(data_lux, &data_raw_lux[0], &data_raw_lux[1]);
	NRF_LOG_INFO("Data lux 1: %x  ---  Data lux 2: %x",data_raw_lux[0],data_raw_lux[1]);
	calcu_data_to_send(BH1750, SET, CMD_ID_BH1750, ZONE_NUM_1, 0,(uint8_t *)data_raw_lux, 2, data_send);
	NRF_LOG_INFO("Data lux send: %x %x %x %x %x %x",data_send[4],data_send[5],data_send[6],data_send[7],data_send[8],data_send[9]);
	for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
	{
		if(mem_connect[i].status == 1)
		{
			ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
		}
	}
	
	memset(data_send, 0, sizeof(data_send));
	
	convert_dec_to_hex(data_temp, &data_raw_temp[0], &data_raw_temp[1]);
	NRF_LOG_INFO("Data temp 1: %x  ---  Data temp 2: %x",data_raw_temp[0],data_raw_temp[1]);
	calcu_data_to_send(LM75, SET, CMD_ID_LM75, ZONE_NUM_1, 0,(uint8_t *)data_raw_temp, 2, data_send);
	for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
	{
		if(mem_connect[i].status == 1)
		{
			ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
		}
	}
}

#elif (ZONE == 2)

static void timer_send_form_central_to_peripheral_handler(void * p_context)				//ThanhBH Add
{
	uint8_t temp = 0, humi = 0, data_send[10] = {0};
	uint8_t data_raw_temp[2] = {0}, data_raw_humi[2] = {0}, data_raw_lm35[2] = {0};
	static uint32_t total_temp_lm35_value = 0;
	static uint8_t count = 0;
	count++;
	total_temp_lm35_value += (uint16_t)((read_adc(AN6) * 2.7 * 100)/512);
	if(count == 20)
	{
		read_5_byte_data(&temp,&humi);
		NRF_LOG_INFO("Data Temp_DHT_11: %d  ---   Humi_DHT_11: %d", temp, humi);
		convert_dec_to_hex(temp, &data_raw_temp[0], &data_raw_temp[1]);
		calcu_data_to_send(HUMI_DHT11, SET, CMD_ID_HUMI_DHT11, ZONE_NUM_2, 0,(uint8_t *)data_raw_temp, 2, data_send);
		for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
		{
			if(mem_connect[i].status == 1)
			{
				ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
			}
		}
		
		convert_dec_to_hex(humi, &data_raw_humi[0], &data_raw_humi[1]);
		memset(data_send, 0, sizeof(data_send));
		calcu_data_to_send(TEMP_DHT11, SET, CMD_ID_TEMP_DHT11, ZONE_NUM_2, 0,(uint8_t *)data_raw_humi, 2, data_send);
		for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
		{
			if(mem_connect[i].status == 1)
			{
				ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
			}
		}
		
		NRF_LOG_INFO("Data temp_lm35: %d",(total_temp_lm35_value / count));
		memset(data_send, 0, sizeof(data_send));
		convert_dec_to_hex((total_temp_lm35_value / count), &data_raw_lm35[0], &data_raw_lm35[1]);
		calcu_data_to_send(LM35, SET, CMD_ID_LM35, ZONE_NUM_2, 0,(uint8_t *)data_raw_lm35, 2, data_send);
		for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
		{
			if(mem_connect[i].status == 1)
			{
				ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
			}
		}
		
		count = 0;
		total_temp_lm35_value = 0;
		return;
	}
}

#elif (ZONE == 4)

static void timer_send_form_central_to_peripheral_handler(void * p_context)				//ThanhBH Add
{
	
}

#endif

void init_led(void)
{
	nrf_gpio_pin_dir_set(2, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(3, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(4, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(11, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(12, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(15, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(16, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(19, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	
#if (ZONE == 3)
	nrf_gpio_pin_dir_set(DIRECTION_OF_PIR, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_dir_set(DIRECTION_OF_MQ7, NRF_GPIO_PIN_DIR_OUTPUT);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
//	nrf_gpio_cfg_input(PIN_MQ7, NRF_GPIO_PIN_NOPULL);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
//	nrf_gpio_cfg_input(PIN_PIR, NRF_GPIO_PIN_NOPULL);			//direction: NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_DIR_OUTPUT
	nrf_gpio_pin_write(DIRECTION_OF_PIR, 0);
	nrf_gpio_pin_write(DIRECTION_OF_MQ7, 0);
#endif	
	
	nrf_gpio_pin_write(2, 0);
	nrf_gpio_pin_write(3, 0);
	nrf_gpio_pin_write(4, 0);
	nrf_gpio_pin_write(11, 0);
	nrf_gpio_pin_write(12, 0);
	nrf_gpio_pin_write(15, 0);
	nrf_gpio_pin_write(16, 0);
	nrf_gpio_pin_write(19, 0);
	
}

void init_bh1750_lm75(void)
{
	i2c_init(22,23,super_high);
}

void init_dht11(void)
{
	DHT11_Init();
}

void init_lm35(void)
{
//	adc_ref_input_init(AN6,VDD,saadc_callback_handler);
	adc_ref_grou_init(AN6,saadc_callback_handler);
}

#if (ZONE == 3)

typedef enum{
	PIR_UNMOTION,
	PIR_MOTION,
	PIR_STATE_DEBOUNCE,
	PIR_STATE_WAIT_5S,
	PIR_STATE_WAIT_30S
}state_of_PIR_sensor;

typedef enum{
	MQ7_UNMOTION,
	MQ7_MOTION,
	MQ7_STATE_DEBOUNCE,
	MQ7_STATE_WAIT_5S,
	MQ7_STATE_WAIT_30S
}state_of_MQ7_sensor;

state_of_PIR_sensor pir_state;
state_of_MQ7_sensor mq7_state;

static void timer_send_form_central_to_peripheral_handler(void * p_context)				//ThanhBH Add
{
	
}

void function_handler_exti_PIR(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action)
{
	ret_code_t err_code;
	disable_exti(PIN_PIR);
	if(nrf_gpio_pin_read(PIN_PIR) == 1)
	{
		pir_state = PIR_STATE_DEBOUNCE;
		disable_exti(PIN_PIR);
		err_code = app_timer_stop(timer_handler_PIR_id);	//ThanhBH Add timer handler PIR sensor
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
		err_code = app_timer_start(timer_handler_PIR_id, APP_TIMER_TICKS(100), NULL);	//ThanhBH Add timer handler PIR sensor
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
	}
	else
	{
		enable_exti(PIN_PIR);
	}
}

void function_handler_exti_MQ7(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action)
{
	ret_code_t err_code;
	disable_exti(PIN_MQ7);
	if(nrf_gpio_pin_read(PIN_MQ7) == 0)
	{
		NRF_LOG_INFO("MQ7_Event_Interrupt");
		mq7_state = MQ7_STATE_DEBOUNCE;
		disable_exti(PIN_MQ7);
		err_code = app_timer_stop(timer_handler_MQ7_id);	//ThanhBH Add timer handler MQ7 sensor
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
		err_code = app_timer_start(timer_handler_MQ7_id, APP_TIMER_TICKS(100), NULL);	//ThanhBH Add timer handler MQ7 sensor
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
	}
	else
	{
		enable_exti(PIN_MQ7);
	}
}

static void timer_handler_PIR_handler(void * p_context)
{
	uint8_t data_pir[] = {0x00, 0x00};
	uint8_t data_send[10] = {0};
	ret_code_t err_code;
	err_code = app_timer_stop(timer_handler_PIR_id);	//ThanhBH Add timer handler PIR sensor
  APP_ERROR_CHECK(err_code);	//ThanhBH Add
	switch(pir_state)
	{
		case PIR_STATE_DEBOUNCE:
		{
			if(nrf_gpio_pin_read(PIN_PIR) == 1)
			{
				NRF_LOG_INFO("PIR_STATE_DEBOUNCE_TRUE.........................");
				NRF_LOG_INFO("PIR_DETECH_MOTION");
				nrf_gpio_pin_write(DIRECTION_OF_PIR, 1);
				data_pir[0] = 0x01;
				//Gui Thong bao trang thai het chuyen dong cua cam bien PIR cho peripheral
				calcu_data_to_send(PIR, SET, CMD_ID_PIR, ZONE_NUM_3, 0,data_pir, 2, data_send);
				for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
				{
					if(mem_connect[i].status == 1)
					{
						ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
					}
				}
				if(timer_handler_PIR_id->active == 1)
				{
					err_code = app_timer_stop(timer_handler_PIR_id);	//ThanhBH Add timer handler PIR sensor
					APP_ERROR_CHECK(err_code);	//ThanhBH Add
				}
				err_code = app_timer_start(timer_handler_PIR_id, APP_TIMER_TICKS(10000), NULL);	//ThanhBH Add timer 10s
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			else
			{
				NRF_LOG_INFO("PIR_STATE_DEBOUNCE_FAILED.........................");
				pir_state = PIR_STATE_WAIT_5S;
				nrf_gpio_pin_write(DIRECTION_OF_PIR, 0);
				if(timer_handler_PIR_id->active == 1)
				{
					err_code = app_timer_stop(timer_handler_PIR_id);	//ThanhBH Add timer handler PIR sensor
					APP_ERROR_CHECK(err_code);	//ThanhBH Add
				}
				err_code = app_timer_start(timer_handler_PIR_id, APP_TIMER_TICKS(100), NULL);	//ThanhBH Add timer 100ms
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			break;
		}
		case PIR_STATE_WAIT_5S:
		{
			NRF_LOG_INFO("TEST_LOG_PIR_STATE_WAIT_5S..........................");
			pir_state = PIR_STATE_WAIT_30S;		//Gan trang thai cho su kien tiep theo
			enable_exti(PIN_PIR);
			if(timer_handler_PIR_id->active == 1)
			{
				err_code = app_timer_stop(timer_handler_PIR_id);	//ThanhBH Add timer handler PIR sensor
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			err_code = app_timer_start(timer_handler_PIR_id, APP_TIMER_TICKS(30000), NULL);	//ThanhBH Add timer 30s
			APP_ERROR_CHECK(err_code);	//ThanhBH Add
			break;
		}
		case PIR_STATE_WAIT_30S:
		{
			NRF_LOG_INFO("TEST_LOG_PIR_STATE_WAIT_30S.........................");
			NRF_LOG_INFO("PIR_DETECH_UNMOTION");
			if(timer_handler_PIR_id->active == 1)
			{
				err_code = app_timer_stop(timer_handler_PIR_id);	//ThanhBH Add timer handler PIR sensor
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			//Gui Thong bao trang thai het chuyen dong cua cam bien PIR cho peripheral
			data_pir[0] = 0x00;
			calcu_data_to_send(PIR, SET, CMD_ID_PIR, ZONE_NUM_3, 0,data_pir, 2, data_send);
			for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
			{
				if(mem_connect[i].status == 1)
				{
					ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
				}
			}
			break;
		}
		default:
			break;
	}
}

static void timer_handler_MQ7_handler(void * p_context)
{
	uint8_t data_mq7[] = {0x00, 0x00};
	uint8_t data_send[10] = {0};
	ret_code_t err_code;
	err_code = app_timer_stop(timer_handler_MQ7_id);	//ThanhBH Add timer handler MQ7 sensor
  APP_ERROR_CHECK(err_code);	//ThanhBH Add
	switch(mq7_state)
	{
		case MQ7_STATE_DEBOUNCE:
		{
			if(nrf_gpio_pin_read(PIN_MQ7) == 0)
			{
				NRF_LOG_INFO("MQ7_STATE_DEBOUNCE_TRUE.........................");
				NRF_LOG_INFO("MQ7_DETECH_MOTION");
				nrf_gpio_pin_write(DIRECTION_OF_MQ7, 1);
				data_mq7[0] = 0x01;
				//Gui Thong bao trang thai co khi doc cho peripheral
				calcu_data_to_send(MQ7, SET, CMD_ID_MQ7, ZONE_NUM_3, 0,data_mq7, 2, data_send);
				for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
				{
					if(mem_connect[i].status == 1)
					{
						ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
					}
				}
				if(timer_handler_MQ7_id->active == 1)
				{
					err_code = app_timer_stop(timer_handler_MQ7_id);	//ThanhBH Add timer handler MQ7 sensor
					APP_ERROR_CHECK(err_code);	//ThanhBH Add
				}
				err_code = app_timer_start(timer_handler_MQ7_id, APP_TIMER_TICKS(10000), NULL);	//ThanhBH Add timer 10s
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			else
			{
				NRF_LOG_INFO("MQ7_STATE_DEBOUNCE_FAILED.........................");
				mq7_state = MQ7_STATE_WAIT_5S;
				nrf_gpio_pin_write(DIRECTION_OF_MQ7, 0);
				if(timer_handler_MQ7_id->active == 1)
				{
					err_code = app_timer_stop(timer_handler_MQ7_id);	//ThanhBH Add timer handler MQ7 sensor
					APP_ERROR_CHECK(err_code);	//ThanhBH Add
				}
				err_code = app_timer_start(timer_handler_MQ7_id, APP_TIMER_TICKS(100), NULL);	//ThanhBH Add timer 100ms
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			break;
		}
		case MQ7_STATE_WAIT_5S:
		{
			NRF_LOG_INFO("TEST_LOG_MQ7_STATE_WAIT_5S..........................");
			mq7_state = MQ7_STATE_WAIT_30S;		//Gan trang thai cho su kien tiep theo
			enable_exti(PIN_MQ7);
			if(timer_handler_MQ7_id->active == 1)
			{
				err_code = app_timer_stop(timer_handler_MQ7_id);	//ThanhBH Add timer handler MQ7 sensor
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			err_code = app_timer_start(timer_handler_MQ7_id, APP_TIMER_TICKS(30000), NULL);	//ThanhBH Add timer 30s
			APP_ERROR_CHECK(err_code);	//ThanhBH Add
			break;
		}
		case MQ7_STATE_WAIT_30S:
		{
			NRF_LOG_INFO("TEST_LOG_MQ7_STATE_WAIT_30S.........................");
			NRF_LOG_INFO("MQ7_DETECH_UNMOTION");
			if(timer_handler_MQ7_id->active == 1)
			{
				err_code = app_timer_stop(timer_handler_MQ7_id);	//ThanhBH Add timer handler MQ7 sensor
				APP_ERROR_CHECK(err_code);	//ThanhBH Add
			}
			//Gui Thong bao trang thai het khi doc cho peripheral
			data_mq7[0] = 0x00;
			calcu_data_to_send(MQ7, SET, CMD_ID_MQ7, ZONE_NUM_3, 0,data_mq7, 2, data_send);
			for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
			{
				if(mem_connect[i].status == 1)
				{
					ble_nus_c_string_send(mem_connect[i].ble_nus_mem, data_send, LENG_PAYLOAD_SENSOR);
				}
			}
			break;
		}
		default:
			break;
	}
}

#endif

void init_interrupt_external(void)
{

#if (ZONE == 3)
	init_exti(PIN_PIR,lo_to_hi,no_pull, function_handler_exti_PIR);
	enable_exti(PIN_PIR);
	init_exti(PIN_MQ7,hi_to_lo,no_pull, function_handler_exti_MQ7);
	enable_exti(PIN_MQ7);
#endif	

}

void init_peripheral(void)
{
	init_led();
#if (ZONE == 1)
	
	init_bh1750_lm75();

#elif (ZONE == 2)

	init_lm35();
	init_dht11();

#elif (ZONE == 3)	
	
	init_interrupt_external();
	
#endif
}

int main(void)
{
		ret_code_t err_code;
    // Initialize.
		ring_buff_init(&ringbuff, arr_mem_data, SIZE_OF_QUEUE);
		init_peripheral();
		reset_handler();
		lfclk_request();	//ThanhBH Add
    log_init();
    timer_init();
		create_timers();	//ThanhBH Add
    uart_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    nus_c_init();
    scan_init();
    // Start execution.
    printf("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();
		
#if (ZONE == 1)
		
		err_code = app_timer_start(timer_send_form_central_to_peripheral_id, APP_TIMER_TICKS(1000), NULL);	//ThanhBH Add timer 100ms
    APP_ERROR_CHECK(err_code);	//ThanhBH Add

#elif (ZONE == 2)

		err_code = app_timer_start(timer_send_form_central_to_peripheral_id, APP_TIMER_TICKS(100), NULL);	//ThanhBH Add timer 100ms
		APP_ERROR_CHECK(err_code);	//ThanhBH Add

#elif (ZONE == 3)
		err_code = app_timer_stop(timer_handler_PIR_id);	//ThanhBH Add timer handler PIR sensor
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
		err_code = app_timer_stop(timer_handler_MQ7_id);	//ThanhBH Add timer handler MQ7 sensor
    APP_ERROR_CHECK(err_code);	//ThanhBH Add
#endif
    // Enter main loop.
    for (;;)
    {
				processDataReceiver();		//ThanhBH Add
        idle_state_handle();
    }
}
