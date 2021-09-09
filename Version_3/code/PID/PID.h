#ifndef PID_H
#define PID_H

/* Includes ----------------------------------------------------------------*/
#include <stdint.h>

/* Variables ----------------------------------------------------------------*/

/* Functions ----------------------------------------------------------------*/
void coeff_PID_2_ABC (	float Kp, 
												float Ki, 
												float Kd, 
												float Ts, 
												float* A, 
												float* B, 
												float* C
											);

void PID_Controller_Function (	int32_t desired_value, 
																int32_t measured_output, 
																float A, 
																float B, 
																float C, 
																int32_t* control_variable,
																int32_t  control_variable_max,
																int16_t* error_0,
																int16_t* error_1,
																int16_t* error_2																
															);



#endif /* PID_H */

