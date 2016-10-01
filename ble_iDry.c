/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#include <string.h>
#include "ble_idry.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "SEGGER_RTT.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_idry      IDRY Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_idry_t * p_idry, ble_evt_t * p_ble_evt)
{
    p_idry->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_idry      IDRY Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_idry_t * p_idry, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_idry->conn_handle = BLE_CONN_HANDLE_INVALID;
}



void ble_idry_on_ble_evt(ble_idry_t * p_idry, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_idry, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_idry, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Temperature  Characteristic.
 *
 * @param[in] p_idry      IDRY Service structure.
 * @param[in] p_idry_init IDRY Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t temperature_char_add(ble_idry_t * p_idry, const ble_idry_init_t * p_idry_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_idry->uuid_type;
    ble_uuid.uuid = IDRY_UUID_TEMPERATURE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t)*2;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t)*2;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_idry->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_idry->temperature_char_handles);
}




uint32_t ble_idry_init(ble_idry_t * p_idry, const ble_idry_init_t * p_idry_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_idry->conn_handle       = BLE_CONN_HANDLE_INVALID;

    // Add service.
    ble_uuid128_t base_uuid = {IDRY_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_idry->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_idry->uuid_type;
    ble_uuid.uuid = IDRY_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_idry->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add characteristics.
    err_code = temperature_char_add(p_idry, p_idry_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_idry_on_temperature_change(ble_idry_t * p_idry, uint8_t* temperature)
{
  if (p_idry == NULL)
  {
      return NRF_ERROR_NULL;
  }
    
  uint32_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  if ((temperature[0] != p_idry->temperature_last[0]) || (temperature[1] != p_idry->temperature_last[1]))
  {
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t)*2;
    gatts_value.offset  = 0;
    gatts_value.p_value = temperature;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_idry->conn_handle,
                                      p_idry->temperature_char_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
      // Save new temperature value.
      p_idry->temperature_last[0] = temperature[0];
      p_idry->temperature_last[1] = temperature[1];
// SEGGER_RTT_WriteString(0,"============================================== temperature update success ");
    }
    else
    {
      return err_code;
    }

    // Send value if connected and notifying.
//        if ((p_idry->conn_handle != BLE_CONN_HANDLE_INVALID) && p_idry->is_notification_supported)
    if ((p_idry->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
      ble_gatts_hvx_params_t hvx_params;

      memset(&hvx_params, 0, sizeof(hvx_params));

      hvx_params.handle = p_idry->temperature_char_handles.value_handle;
      hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
      hvx_params.offset = gatts_value.offset;
      hvx_params.p_len  = &gatts_value.len;
      hvx_params.p_data = gatts_value.p_value;

      err_code = sd_ble_gatts_hvx(p_idry->conn_handle, &hvx_params);
    }
    else
    {
      err_code = NRF_ERROR_INVALID_STATE;
    }
  }

  return err_code;
}

