/**
  ******************************************************************************
  * @file    i2c.h
  * @author  Thomas R.
  * @version V1.0
  * @date    15/11/15
  * @brief   header file for i2c driver
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_soc.h"
#include "app_trace.h"
#include "SEGGER_RTT.h"

/* Exported types ------------------------------------------------------------*/
// PINs:
		// PINs:
		#if defined PCB4
				#define	SCL_pin		28
				#define	SDA_pin		29
		#elif defined PCB6_v5		
				#define	SCL_pin		19
				#define	SDA_pin		18
		#elif defined PCB7		
				#define	SCL_pin		29
				#define	SDA_pin		28
		#endif

// SAAR CODE:
// MPU
static inline void LOG(char* TAG, int line){
				char buf[100];
				sprintf(buf, "%s %d\n", TAG, line);	
				SEGGER_RTT_WriteString(0, buf);	
}

// SAAR CODE:
// MPU
static inline void LOG_YPR(int yaw, int roll, int pitch){
				char buf[100];
				char* Yaw = "YAW";
				char* Roll = "Roll";
				char* Pitch = "Pitch";
					
				sprintf(buf, "%s %d | %s %d | %s %d | \n", Yaw, yaw, Roll, roll, Pitch, pitch);	
				SEGGER_RTT_WriteString(0, buf);	
}

// SAAR CODE:
// MPU
static inline void LOG_QUAT(float w, float x, float y, float z){
				char buf[100];
				char* W = "W";
				char* X = "X";
				char* Y = "Y";
				char* Z = "Z";
					
				sprintf(buf, "%s %10f | %s %10f | %s %10f | %s %10f \n", W, w, X, x, Y, y, Z, z);	
				SEGGER_RTT_WriteString(0, buf);	
}

// SAAR CODE:
// PID
static inline void LOG_PID_ABC(float a, float b, float c){
				char buf[100];
				char* A = "A";
				char* B = "B";
				char* C = "C";
	
				sprintf(buf, "%s %10f | %s %10f | %s %10f \n", A, a, B, b, C, c);	
				SEGGER_RTT_WriteString(0, buf);	
}

static inline void LOG_Errors(int32_t e0, int32_t e1, int32_t e2, int32_t c_v){
				char buf[100];
				char* Error_0 = "Error_0";
				char* Error_1 = "Error_1";
				char* Error_2 = "Error_2";
				char* Control_Variable = "Control_Variable";
	
				sprintf(buf, "%s %10d | %s %10d | %s %10d | %s %10d \n", Error_0, e0, Error_1, e1, Error_2, e2, Control_Variable, c_v);	
				SEGGER_RTT_WriteString(0, buf);	
}

// SAAR CODE:
// BMP
#ifdef BMP180_API
static inline void LOG_BMP_180_temperature_pressure(uint16_t temperature, uint32_t pressure){
				char buf[100];
				char* Temp 	= "Temperature =";
				char* Press = "Pressure =";
					
				sprintf(buf, "BMP180:  %s %4d | %s %6d \n", Temp, temperature, Press, pressure);	
				SEGGER_RTT_WriteString(0, buf);	
}
#endif

// SAAR CODE:
// BME
#ifdef BME280_API 
static inline void LOG_BME_280_temperature_pressure_humidity(uint32_t temperature, uint32_t pressure, uint32_t humidity){
				char buf[100];
				char* Temp 	= "Temperature =";
				char* Press = "Pressure =";
				char* Hum		= "Humidity =";
					
				sprintf(buf, "BME280:  %s %4d | %s %6d | %s %d \n", Temp, temperature, Press, pressure, Hum, humidity);	
				SEGGER_RTT_WriteString(0, buf);	
}
#endif

// SAAR CODE:
// ADC
#ifdef ADC 
static inline void LOG_ADC(int16_t adc_value){
				char buf[100];
				char* ADC_1 	= "ADC_1 =";
					
				sprintf(buf, "ADC:  %s %4d \n", ADC_1, adc_value);	
				SEGGER_RTT_WriteString(0, buf);	
}
#endif

/* Exported constants --------------------------------------------------------*/
#define I2CERRORCOUNT_TO_RECOVER    5
#define I2CBUFFERSIZE               20

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
uint32_t i2c_Init(void);
void i2c_Deinit(void);
uint32_t i2c_read(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data);
uint32_t i2c_write(uint8_t device_address, uint8_t register_address, uint8_t length, uint8_t *data);

#endif /* I2C_H_INCLUDED */

/**END OF FILE*****************************************************************/
