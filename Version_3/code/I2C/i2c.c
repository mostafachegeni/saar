/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Thomas R.
  * @version V1.0
  * @date    15/11/15
  * @brief   i2c Driver
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_soc.h"
#include "app_trace.h"
#include "i2c.h"

#if defined HELENA_DEBUG_RTT
#include "SEGGER_RTT.h"
#endif

/* External variables --------------------------------------------------------*/

// SAAR CODE:
// MPU
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

// SAAR CODE:
// MPU
#define MPU_TWI_TIMEOUT 			10000 

/* Private variables ---------------------------------------------------------*/
const nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(1);

const nrf_drv_twi_config_t twi_config =
{
    .scl                    = SCL_pin,
    .sda                    = SDA_pin,
    .frequency              = NRF_TWI_FREQ_400K,
    .interrupt_priority     = APP_IRQ_PRIORITY_HIGHEST // "NRF_APP_PRIORITY_LOW" replaced by "TWI1_CONFIG_IRQ_PRIORITY" 
};

/* Private function prototypes -----------------------------------------------*/

// SAAR CODE:
// MPU
static void nrf_drv_mpu_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}

/* Private functions ---------------------------------------------------------*/
void i2c_Recover()
{
    NRF_TWI1->EVENTS_ERROR = 0;
    NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    NRF_TWI1->POWER        = 0;
    nrf_delay_us(5);
    NRF_TWI1->POWER        = 1;
    NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
    //app_trace_log("[I2C]: i2c recovered\r\n");
    (void)i2c_Init();
}

/* Public functions ----------------------------------------------------------*/
uint32_t i2c_Init()
{
    uint32_t err_code;
    
		// SAAR CODE:
		// MPU
		//err_code = nrf_drv_twi_init(&twi_instance, &twi_config, NULL);
    err_code = nrf_drv_twi_init(&twi_instance, &twi_config, nrf_drv_mpu_twi_event_handler, NULL);
	
    //app_trace_log("[I2C]: i2c bus initialized\r\n");
    if (err_code == NRF_SUCCESS)
    {
        nrf_drv_twi_enable(&twi_instance);
        nrf_delay_ms(100);
    }
    return err_code;
}

void i2c_Deinit()
{
    nrf_drv_twi_uninit(&twi_instance);
}

uint32_t i2c_read(uint8_t device_address, uint8_t reg, uint8_t length, uint8_t * p_data)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&twi_instance, device_address, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&twi_instance, device_address, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = MPU_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    return err_code;
}


/*
uint32_t i2c_read(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data)
{
    uint32_t err_code;
    uint8_t retry_cnt = 2;                  //< retry counter, in case of acknowledge error, communication is tryed again (MPU sometimes doesn't acknowledge) /

    do
    {                                       //< send register address to read from /
        err_code = nrf_drv_twi_tx(&twi_instance, device_address, &register_address, 1, true);
        if (err_code != NRF_SUCCESS)        //< if transmission failed, check why /
        {
            if (NRF_TWI1->ERRORSRC)         //< in case of acknowledge error /
            {
                NRF_TWI1->ERRORSRC = 0;     //< delete error source /
                if (--retry_cnt == 0)
                    return err_code;        //< return error, if error occured twice /
                else
                    continue;               //< otherwise try again /
            }
            else                            //< in case of timeout error, PAN56 might be the reason, so recover bus and start over /
            {
                i2c_Recover();
                continue;
            }
        }
    }   while(0);

    retry_cnt = 2;
    do
    {                                       //< read data /
        err_code = nrf_drv_twi_rx(&twi_instance, device_address, data, length);
        if (err_code != NRF_SUCCESS)        //< if transmission failed, check why /
        {
            if (NRF_TWI1->ERRORSRC)         //< in case of acknowledge error /
            {
                NRF_TWI1->ERRORSRC = 0;     //< delete error source /
                if (--retry_cnt == 0)
                    return err_code;        //< return error, if error occured twice /
                else
                    continue;               //< otherwise try again /
            }
            else                            //< in case of timeout error, PAN56 might be the reason, so recover bus and start over /
            {
                i2c_Recover();
                continue;
            }
        }
    }   while(0);

		// SAAR CODE:
		// MPU
		uint32_t timeout = MPU_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;
			
    return err_code;
}
*/

/*
uint32_t i2c_write(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data)
{
    uint8_t buffer[I2CBUFFERSIZE], retry_cnt = 2;
    uint32_t err_code;

    if (length > I2CBUFFERSIZE-1)
        return NRF_ERROR_INVALID_LENGTH;
    buffer[0] = register_address;
    memcpy(&buffer[1], data, length);
    do
    {                                       //< send data /
        err_code = nrf_drv_twi_tx(&twi_instance, device_address, buffer, length+1, false);
						
        if (err_code != NRF_SUCCESS)        //< if transmission failed, check why /
        {
            if (NRF_TWI1->ERRORSRC)         //< in case of acknowledge error /
            {
                NRF_TWI1->ERRORSRC = 0;     //< delete error source /
                if (--retry_cnt == 0)
                    return err_code;        //< return error, if error occured twice /
                else
                    continue;               //< otherwise try again /
            }
            else                            //< in case of timeout error, PAN56 might be the reason, so recover bus and start over /
            {
                i2c_Recover();
                continue;
            }
        }
    }   while(0);

		// SAAR CODE:
		// MPU
		uint32_t timeout = MPU_TWI_TIMEOUT;
    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    return err_code;
}
*/

// The TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
// Hence we need to merge the MPU register address with the buffer and then transmit all as one transmission
static void merge_register_and_data(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint32_t length)
{
    new_buffer[0] = reg;
    memcpy((new_buffer + 1), p_data, length);
}

#define MPU_TWI_BUFFER_SIZE     	14 // 14 byte buffers will suffice to read acceleromter, gyroscope and temperature data in one transmission.
uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

uint32_t i2c_write(uint8_t device_address, uint8_t reg, uint8_t length, uint8_t * p_data)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    // Merging MPU register address and p_data into one buffer.
    merge_register_and_data(twi_tx_buffer, reg, p_data, length);

    // Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = device_address;
    xfer_desc.type = NRF_DRV_TWI_XFER_TX;
    xfer_desc.primary_length = length + 1;
    xfer_desc.p_primary_buf = twi_tx_buffer;

    // Transferring
    err_code = nrf_drv_twi_xfer(&twi_instance, &xfer_desc, 0);

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    return err_code;
}


/**END OF FILE*****************************************************************/



