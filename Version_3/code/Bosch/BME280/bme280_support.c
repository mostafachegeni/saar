/*
****************************************************************************
* Copyright (C) 2017 SAAR
*
* bme280_support.c
* Date: 2017/08/25
* Revision: 1.0.0 $
*
* Usage: Sensor Driver support file for BME280
*
****************************************************************************/
// SAAR CODE:
// BME

/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bme280.h"
#include "nrf_delay.h"
#include "bme280_support.h"

struct bme280_dev dev;
int8_t rslt = BME280_OK;

int32_t bme280_open(void)
{
		#ifdef BME280_API
				I2C_routine();
		#endif

		rslt += bme280_init(&dev);	

	
	#ifdef BME280_WORK_IN_FORCED_MODE
			rslt += stream_sensor_data_forced_mode(&dev);
	#else
			rslt += stream_sensor_data_normal_mode(&dev);
	#endif	
		return rslt;
}

// Stream sensor data in "forced mode":
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;

	/* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_16X;
  dev->settings.osr_t = BME280_OVERSAMPLING_2X;
  dev->settings.filter = BME280_FILTER_COEFF_16;	

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	rslt  = bme280_set_sensor_settings(settings_sel, dev);
	rslt += bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
	
	return rslt;
}

// Stream sensor data in "normal mode":
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel  = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);			

	return rslt;
}

#ifdef BME280_API

void I2C_routine(void) 
{
		dev.dev_id = BME280_I2C_ADDR_SEC; //BME280_I2C_ADDR_PRIM;
		dev.intf = BME280_I2C_INTF;
		dev.read = user_i2c_read;
		dev.write = user_i2c_write;
		dev.delay_ms = user_delay_ms;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

		rslt = (int8_t) i2c_write(dev_id, reg_addr, (uint8_t)len, reg_data);
    return rslt;
}


int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
		rslt = (int8_t) i2c_read(dev_id, reg_addr, (uint8_t)len, reg_data);
    return rslt;
}


void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
	
  nrf_delay_ms(period);
	
}


#endif /* BME280_API */

