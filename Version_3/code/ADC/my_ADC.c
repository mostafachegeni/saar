/*
****************************************************************************
* Copyright (C) 2017 SAAR
*
* bme280_support.c
* Date: 2017/09/03
* Revision: 1.0.0 $
*
* Usage: ADC Driver support file for Nrf51822
*
****************************************************************************/
// SAAR CODE:
// ADC

#include "my_ADC.h"
#include "i2c.h"

nrf_adc_value_t		adc_buffer[ADC_BUFFER_SIZE]; //ADC buffer
int16_t 					adc_1, adc_2, adc_3, adc_4, adc_5, adc_6, adc_7;
static uint8_t    number_of_adc_channels = 0;

/**
 * @brief ADC initialization.
 */
void adc_config(void)
{
    ret_code_t ret_code;

		//Initialize ADC
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);
	
    //Configure and enable ADC channel 0
    static nrf_drv_adc_channel_t m_channel_0_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2); 
    m_channel_0_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_0_config);
		number_of_adc_channels++;
	
    //Configure and enable ADC channel 1
    static nrf_drv_adc_channel_t m_channel_1_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_6); 
    m_channel_1_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_1_config);
		number_of_adc_channels++;
	
    //Configure and enable ADC channel 2
    // static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_7);	
    // m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    // nrf_drv_adc_channel_enable(&m_channel_2_config);
		//number_of_adc_channels++;
	
}

/**
 * @brief ADC interrupt handler.
 */
void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        uint32_t i;
			
				/*
        for (i = 0; i < p_event->data.done.size; i++)
        {
						static int counter = 0;
						counter++;
						if(counter == 100){
							LOG_ADC(p_event->data.done.p_buffer[i]);
							counter = 0;
						}
        }
				*/

        for (i = 0; i < p_event->data.done.size; i++)
        {
						switch (i % number_of_adc_channels) {
								case 0:
										adc_2 = p_event->data.done.p_buffer[i];
								break;
								
								case 1:
										adc_6 = p_event->data.done.p_buffer[i];								
								break;
						}
        }

				
				APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE));
    }
}

