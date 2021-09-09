#ifndef MY_ADC_H
#define MY_ADC_H

/* Includes ----------------------------------------------------------------*/
#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_drv_adc.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_log.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

/* Variables ----------------------------------------------------------------*/
#define ADC_BUFFER_SIZE 6                                	// Size of buffer for ADC samples
extern nrf_adc_value_t       adc_buffer[ADC_BUFFER_SIZE]; // ADC buffer
extern int16_t adc_1, adc_2, adc_3, adc_4, adc_5, adc_6, adc_7;

/* Functions ----------------------------------------------------------------*/
void adc_config(void);
void adc_event_handler(nrf_drv_adc_evt_t const * p_event);


#endif /* MY_ADC_H */
