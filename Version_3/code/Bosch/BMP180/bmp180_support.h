#ifndef BMP_SUPPORT_H
#define BMP_SUPPORT_H


/* Includes ----------------------------------------------------------------*/
#include "bmp180.h"

/*----------------------------------------------------------------------------
  * The following functions are used for reading and writing of
  * sensor data using I2C or SPI communication
----------------------------------------------------------------------------*/
#ifdef BMP180_API
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: I2C init routine
*/
s8 I2C_routine(void);
#endif
/********************End of I2C function declarations***********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP180_delay_msek(u32 msek);


/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
 
// SAAR CODE:
// BMP
//s32 bmp180_data_readout(void);
s32 bmp180_open(void);



#endif /* BMP_SUPPORT_H */


