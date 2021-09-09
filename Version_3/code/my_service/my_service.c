/***						***
 *** Saar CODE: ***
 ***						***/


#include <stdint.h>
#include <string.h>
#include "my_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

// A function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_my_service_on_ble_evt(ble_os_t * p_my_service, ble_evt_t * p_ble_evt)
{
    // Step 3.D Implement switch case handling BLE events related to our service. 
		switch (p_ble_evt->header.evt_id)
		{
				case BLE_GAP_EVT_CONNECTED:
						p_my_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						break;
				case BLE_GAP_EVT_DISCONNECTED:
						p_my_service->conn_handle = BLE_CONN_HANDLE_INVALID;
						break;
				default:
						// No implementation needed.
						break;
		}				
}


/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
uint32_t my_chars_add(ble_os_t * p_our_service, uint8_t characteristic_length, uint8_t read_permission, uint8_t write_permission, uint8_t notify_permission)
{
    // Step 2.A, Add a custom characteristic UUID
    uint32_t       err_code;
		ble_uuid_t     char_uuid;
		ble_uuid128_t  base_uuid = BLE_UUID_MY_BASE_UUID;
		char_uuid.uuid      		 = BLE_UUID_MY_CHARACTERISTC_UUID + (char_uuid_counter++);
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	
    // Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read		= read_permission;
		char_md.char_props.write	= write_permission;

		if( notify_permission ) {
				// Step 3.A, Configuring Client Characteristic Configuration Descriptor(CCCD) metadata 
				// and add to char_md structure
				ble_gatts_attr_md_t cccd_md;
				memset(&cccd_md, 0, sizeof(cccd_md));
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
				cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
				char_md.p_cccd_md           = &cccd_md;
				char_md.char_props.notify   = 1;   
		}
		
    // Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));  
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    
    // Step 2.G, Set read/write security levels to our characteristic
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.write_perm);

		
    // Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;

    // Step 2.H, Set characteristic length in number of bytes
		attr_char_value.max_len	 = 20;
		attr_char_value.init_len = characteristic_length;
		uint8_t value[10]        = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
		attr_char_value.p_value  = value;

    // Step 2.E, Add our new characteristic to the service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
																							 &char_md,
																							 &attr_char_value,
																							 &p_our_service->char_handles[char_uuid_counter - 1]);
		APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


void my_service_init(ble_os_t * p_my_service)
{	
		uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid 		= BLE_UUID_MY_BASE_UUID;
    service_uuid.uuid 							= BLE_UUID_MY_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    

    // Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
		p_my_service->conn_handle = BLE_CONN_HANDLE_INVALID;	
	
    // Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_my_service->service_handle);
    APP_ERROR_CHECK(err_code);

    // SEGGER_RTT_WriteString(0, "Initializing my_service.\n");
    // SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid);
    // SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type);
    // SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_my_service->service_handle);
	
		uint8_t	characteristic_length = 5;
		uint8_t read_permission 			= 0;
		uint8_t write_permission 			= 1;
		uint8_t notify_permission 		= 0;
		err_code = my_chars_add(p_my_service, 
														characteristic_length, 
														read_permission, 
														write_permission, 
														notify_permission);// CHAR 1
    APP_ERROR_CHECK(err_code);

		characteristic_length = 5;
		read_permission 			= 0;
		write_permission 			= 0;
		notify_permission 		= 1;
		err_code = my_chars_add(p_my_service, 
														characteristic_length, 
														read_permission, 
														write_permission, 
														notify_permission);// CHAR 2
    APP_ERROR_CHECK(err_code);

}

// Function to be called when updating characteristic value
void my_characteristic_update(ble_os_t *p_my_service, uint8_t length, uint8_t *value, int32_t index_of_characeristic)
{
    // OUR_JOB: Step 3.E, Update characteristic value
		if (p_my_service->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				uint16_t               len = length;
				ble_gatts_hvx_params_t hvx_params;
				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_my_service->char_handles[index_of_characeristic].value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = 0;
				hvx_params.p_len  = &len;
				hvx_params.p_data = (uint8_t*)value;  

				sd_ble_gatts_hvx(p_my_service->conn_handle, &hvx_params);
		}
}



