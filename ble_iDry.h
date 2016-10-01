/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_srv_idry iDry Service
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief iDry Service module.
 *
 * @details This module implements a custom iDry Service with a sensors and fan Characteristics.
 *          During initialization, the module adds the iDry Service and Characteristics
 *          to the BLE stack database.
 *
 *          The application must supply an event handler for receiving iDry Service
 *          events. Using this handler, the service notifies the application when the
 *          FAN value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of the Sensors Characteristic to connected peers.
 *
 * @note The application must propagate BLE stack events to the iDry Service
 *       module by calling ble_idry_on_ble_evt() from the @ref softdevice_handler callback.
*/

#ifndef BLE_IDRY_H__
#define BLE_IDRY_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define IDRY_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}

                             
/*
#define IDRY_UUID_BASE        {0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x99, 0x88, 0x77, \
                              0x66, 0x55, 0x44, 0x33, 0x00, 0x00, 0x22, 0x22}
*/
#define IDRY_UUID_SERVICE                   0x1546
#define IDRY_UUID_FAN_CHAR                  0x1547
#define IDRY_UUID_TEMPERATURE_CHAR          0x1548

// Forward declaration of the ble_idry_t type. 
typedef struct ble_idry_s ble_idry_t;

typedef void (*ble_idry_fan_write_handler_t) (ble_idry_t * p_idry, uint8_t new_state);

/** @brief iDry Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
//    ble_idry_fan_write_handler_t fan_write_handler; /**< Event handler to be calfan when the FAN Characteristic is written. */
} ble_idry_init_t;

/**@brief iDry Service structure. This structure contains various status information for the service. */
struct ble_idry_s
{
    uint16_t                    service_handle;      /**< Handle of iDry Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    temperature_char_handles; /**< Handles related to the Button Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the iDry Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    uint8_t                     temperature_last[2]; /** remember last temp value **/
};

/**@brief Function for initializing the iDry Service.
 *
 * @param[out] p_idry      iDry Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_idry_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_idry_init(ble_idry_t * p_idry, const ble_idry_init_t * p_idry_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the iDry Service.
 *
 * @param[in] p_idry      iDry Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_idry_on_ble_evt(ble_idry_t * p_idry, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 *
 * @param[in] p_idry      iDry Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_idry_on_temperature_change(ble_idry_t * p_idry, uint8_t* temperature);

#endif // BLE_IDRY_H__

/** @} */
