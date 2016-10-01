/* MIT License

Copyright (c) 2016 Marco Tozzini - Soliditec Solutions

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "boards.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_twi.h"
#include "ble_idry.h"
#include "SEGGER_RTT.h"
#include "nrf_delay.h"


/*  TWI stuff   */
/*Pins to connect shield. */
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 29

/*Common addresses definition for temp sensor. */
/* Change this one if your sensor is mapped to a different I2C address */
#define MCP9808_ADDR        (0x3eU >> 1)

#define MCP9808_RA_CONFIG 0x01
#define MCP9808_RA_TEMP   0x05


#define TWI_WAIT_MCP9808                1
#define TWI_MCP9808                     2
#define TWI_WAIT_HTDU21D_RH             3
#define TWI_HTDU21D_RH                  4
#define TWI_WAIT_HTDU21D_TEMP           5
#define TWI_HTDU21D_TEMP                6
#define TWI_SEQ_COMPLETED               7
#define TWI_IDLE                        8
#define TWI_ERROR                       9

/* TWI Variables  */
static volatile bool m_xfer_done = true;
static uint8_t twi_case_handler = TWI_ERROR;




/*  Nordic stuff   */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define DEVICE_NAME                     "Soliditec"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


// define timer stuff
APP_TIMER_DEF(m_app_timer_id);
#define TIMER_INTERVAL					APP_TIMER_TICKS(2000,APP_TIMER_PRESCALER)

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */


// marco stuff


struct idry_data_s 
{
uint16_t							temperature;
};

static struct idry_data_s idry_data_str = {
  .temperature = 0,
};




/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done;
/* TWI instance. */
static const nrf_drv_twi_t m_twi_sensors = NRF_DRV_TWI_INSTANCE(0);

 static ble_idry_t m_idry;
//**** I2C stuff

static ble_uuid_t                       m_adv_uuids[] = {
//   {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
  {IDRY_UUID_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN},
};  /**< Universally unique service identifier. */


// internal function declaration
static void process_data (void);




static void twi_main_handler(void) {
    static ret_code_t twi_err_code;
    static uint8_t twi_main = TWI_WAIT_MCP9808;
    static uint8_t reg[1];

    switch (twi_main) {
      
          case TWI_WAIT_MCP9808:
            if (m_xfer_done == true) {
              twi_main = TWI_MCP9808;
            }
          break;
          case TWI_MCP9808:
            twi_case_handler = TWI_MCP9808;  // set case for twi_handler
            m_xfer_done = false;

            reg[0] = MCP9808_RA_TEMP;
            twi_err_code = nrf_drv_twi_tx(&m_twi_sensors, MCP9808_ADDR, reg, sizeof(reg), false);  
            APP_ERROR_CHECK(twi_err_code);

            twi_main = TWI_SEQ_COMPLETED;
          break;

          case TWI_SEQ_COMPLETED:
            if (m_xfer_done == true) {
              twi_main = TWI_IDLE;
              process_data();
            }
          break;

          case TWI_IDLE:
              // add a conditional jump to start only when some time has expired
// SEGGER_RTT_WriteString(0,"SENSOR READ ------------------------- IDLE \n");
              nrf_delay_ms(450);
              twi_main = TWI_WAIT_MCP9808;
          break;
          
          default:
            twi_case_handler = TWI_WAIT_MCP9808;
          break;
      }
}




/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    static uint8_t data[5];
    uint32_t temp;
    static char stringa[50];
    
    switch (twi_case_handler) {

      // TWI events sequence is initiated at timer timout with a nrf_drv_twi_tx() call and complete with the following sequence 
      case TWI_MCP9808:
        switch(p_event->type)
        {
sprintf(stringa,"twi_case_handler->switch: %d\n ", (int)p_event->type);
SEGGER_RTT_WriteString(0, stringa);


          case NRF_DRV_TWI_TX_DONE:

            // Read 2 bytes from the specified address. 
            err_code = nrf_drv_twi_rx(&m_twi_sensors, MCP9808_ADDR, data, sizeof(uint8_t) * 2, false);
            APP_ERROR_CHECK(err_code);
          break;
  
          case NRF_DRV_TWI_RX_DONE:
            // Mask out flags and clean up sign bit
            temp = data[0] << 8 | data[1];
            temp &= 0x1FFF;                // Clear flag bits
            if ((temp & 0x1000) == 0x1000) //TA < 0°C
            {
              temp |= 0xF000;            // Pad sign bit
            }
            idry_data_str.temperature = temp * 625 / 100; // idry_data_str.temperature Celcius deg x 10 - real formula = temp * 6.25 / 100

temp = idry_data_str.temperature / 100;
sprintf(stringa,"idry_data_str.temperature: %d.%02dC(/100)\n ", (int)temp, (int) (idry_data_str.temperature - (temp * 100)));
// SEGGER_RTT_WriteString(0,"SENSOR 1 >>>>>>>>>>>> ");
SEGGER_RTT_WriteString(0, stringa);
              

              m_xfer_done = true;  // tell main app sensor read has been complete
          break;

          default:
sprintf(stringa,"twi_case_handler: %d\n ", (int)twi_case_handler);
SEGGER_RTT_WriteString(0,stringa);
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
          break;        
        }   
      break;

      default:
        APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
      break;

    }
}

/**
 * @brief TWI UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_port_config = {
       .scl                = I2C_SCL_PIN,
       .sda                = I2C_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_sensors, &twi_port_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_sensors);
}


// end I2C ******/




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
    SEGGER_RTT_WriteString(0, "ERROR: ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++!\n");
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
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




/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_idry_init_t idry_init;

		memset(&m_idry, 0, sizeof(m_idry));
	  err_code = ble_idry_init(&m_idry, &idry_init);
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
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
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
uint32_t err_code;

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
//    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
SEGGER_RTT_WriteString(0,"$$$$$$$$$$$$ IMPORTANT: I'm setting sd_ble_gatts_sys_attr_set() $$$$$$$$$$$\n ");
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
  ble_conn_params_on_ble_evt(p_ble_evt);
  ble_idry_on_ble_evt(&m_idry, p_ble_evt);
  on_ble_evt(p_ble_evt);
  ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
SEGGER_RTT_WriteString(0, "ble_stack_init 1\n");
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
SEGGER_RTT_WriteString(0, "ble_stack_init 2\n");

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
SEGGER_RTT_WriteString(0, "ble_stack_init 3\n");
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
SEGGER_RTT_WriteString(0, "ble_stack_init 4\n");
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
SEGGER_RTT_WriteString(0, "ble_stack_init 5\n");
SEGGER_RTT_WriteString(0, "in ble_stack_init before sd_ble_enable()\n");
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
SEGGER_RTT_WriteString(0, "in ble_stack_init before softdevice_ble_evt_handler()\n");
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void process_data () {
	uint32_t err_code;
//	uint8_t len;
	static uint8_t timer_counter = 1;
  uint8_t temp_array[2];
  
  timer_counter++;

  temp_array[0] = idry_data_str.temperature & 0xff;
  temp_array[1] = (idry_data_str.temperature >> 8) & 0xff;

  err_code = ble_idry_on_temperature_change(&m_idry, temp_array);
  switch (err_code) {
    case NRF_SUCCESS:
// SEGGER_RTT_WriteString(0, "BLE idry_data_str.temperature updated SUCCESS!\n");
    break;
    case BLE_ERROR_INVALID_CONN_HANDLE:
SEGGER_RTT_WriteString(0, "BLE Not Connected!\n");
    break;
    case NRF_ERROR_INVALID_STATE:
SEGGER_RTT_WriteString(0, "BLE Invalid State!\n");
    break;
    case BLE_ERROR_GATTS_SYS_ATTR_MISSING:
SEGGER_RTT_WriteString(0, ">>>>>>>>>>>>>>>>>>> This is the error but let's keep going !\n");
    break;
      
    default:
SEGGER_RTT_WriteString(0, "idry_data_str.temperature update ERROR!\n");
      APP_ERROR_CHECK(err_code);
    break;
  }

	return;
}


static void timer_timeout_handler (void * pcontext) {
//  ret_code_t err_code;
//  uint8_t reg[1] = {MCP9808_RA_TEMP};

		// first write to I2C (TWI) interface kick start the full reading sequence for all sensors (check twi_handler function)
/*
  if (m_xfer_done == true) {
    // TODO : a good error control when reading sensors - if sensors cannot be read -> TWI cycle need a reset
    m_xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi_sensors, MCP9808_ADDR, reg, sizeof(reg), false);  
    APP_ERROR_CHECK(err_code);
  }
*/

//   	process_data();

	return;
}

static void timers_init(void)
{
    uint32_t err_code;


    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    
    // Create timer.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code);

}

static void application_timers_start(void)
{
    uint32_t err_code;


    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
   
}



/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
//    bool erase_bonds;

    
  SEGGER_RTT_WriteString(0, "BOOT ---------------------------------- !\n");

    timers_init();
    SEGGER_RTT_WriteString(0, "timers_init done!\n");
    twi_init();
    SEGGER_RTT_WriteString(0, "twi_init done!\n");

    ble_stack_init();
    SEGGER_RTT_WriteString(0, "ble_stack_init done!\n");
    gap_params_init();
    SEGGER_RTT_WriteString(0, "gap_param_init done!\n");
    services_init();
    SEGGER_RTT_WriteString(0, "services_init done!\n");
    advertising_init();
    SEGGER_RTT_WriteString(0, "advertising_init done!\n");
    conn_params_init();
    SEGGER_RTT_WriteString(0, "conn_param_init done!\n");
    
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    SEGGER_RTT_WriteString(0, "ble_advertising_start done!\n");


    application_timers_start();
    SEGGER_RTT_WriteString(0, "timer_start done!\n");
    
    SEGGER_RTT_WriteString(0, "...................... Before Main Loop!\n");

    
   
    // Enter main loop.
    for (;;)
    {
      twi_main_handler();
  //        power_manage();
    }
}


/** 
 * @}
 */
