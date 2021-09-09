/***						***
 *** Saar CODE: ***
 ***						***/

#ifndef MY_SERVICE_H__
#define MY_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_MY_BASE_UUID              {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_MY_SERVICE                0x1802 // Just a random, but recognizable value
#define BLE_UUID_MY_CHARACTERISTC_UUID		 0xFFF1 // Just a random, but recognizable value

static int char_uuid_counter = 0;

/**
 * @brief This structure contains various status information for our service. 
 * It only holds one entry now, but will be populated with more items as we go.
 * The name is based on the naming convention used in Nordic's SDKs. 
 * 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
 * ‘os’ is short for Our Service). 
 */
typedef struct
{
    uint16_t    service_handle;     /**< Handle of Our Service (as provided by the BLE stack). */
		uint16_t    conn_handle; 
    // Step 2.D, Add handles for our characteristic
    ble_gatts_char_handles_t    char_handles[2];
}ble_os_t;


// A function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_my_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t * p_ble_evt);


/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
uint32_t my_chars_add(ble_os_t * p_our_service, uint8_t characteristic_length, uint8_t read_permission, uint8_t write_permission, uint8_t notify_permission);



/**@brief Function for initializing my new service.
 *
 * @param[in]   p_my_service       Pointer to Our Service structure.
 */
void my_service_init(ble_os_t * p_my_service);

// Function to be called when updating characteristic value
void my_characteristic_update(ble_os_t *p_my_service, uint8_t length, uint8_t *value, int32_t index_of_characeristic);

#endif  /* _ MY_SERVICE_H__ */

