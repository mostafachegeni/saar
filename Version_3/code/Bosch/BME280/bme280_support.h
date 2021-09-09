#ifndef BME_SUPPORT_H
#define BME_SUPPORT_H

/* Includes ----------------------------------------------------------------*/
#include "bme280.h"

/*----------------------------------------------------------------------------
  * The following functions are used for reading and writing of
  * sensor data using I2C or SPI communication
----------------------------------------------------------------------------*/

extern struct bme280_dev dev;
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);

#ifdef BME280_API

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
void 	 I2C_routine(void);

#endif

void user_delay_ms(uint32_t period);
int32_t bme280_open(void);



#endif /* BME_SUPPORT_H */




