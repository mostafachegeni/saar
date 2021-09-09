/*
****************************************************************************
* Copyright (C) 2017 SAAR
*
* Date: 2017/10/04
* Revision: 1.0.0 $
*
* Usage: PID Controller
*
****************************************************************************/
// SAAR CODE:
// PID

#include "PID.h"
//#include "i2c.h"


void coeff_PID_2_ABC (float Kp, float Ki, float Kd, float Ts, float* A, float* B, float* C) 
{
	/*	
		// Z-Transform of "PID" Controller
		*A =  Kp 	 + Ki*(Ts/2.f) + Kd/Ts 			; //  Kp 		+ Ki/16 + Kd*8  ; //  
		*B = -Kp 	 + Ki*(Ts/2.f) - Kd*(2.f/Ts); // -Kp 		+ Ki/16 - Kd*16 ; // 
		*C =  										 Kd/Ts			;	//  Kd*8									; //  	
	*/	
	
		
		// Z-Transform of "PID" Controller 3
		*A =  Kp	; 
		*B = 	Ki	; 
		*C = 	Kd	;	

	
	/*
		// Z-Transform of "P" Controller
		*A =  Kp ;
		*B =   0 ; 
		*C =   0 ;  	
	*/
}

static int32_t	y_0 						= 0,	y_1 						= 0,	y_2 						= 0;
static int32_t	desired_value_0 = 0, 	desired_value_1 = 0, 	desired_value_2 = 0;

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
															)
{
		y_2 = y_1;
		y_1 = y_0;
		y_0 = measured_output;
							
		desired_value_2 = desired_value_1;
		desired_value_1 = desired_value_0;
		desired_value_0 = desired_value;

		int32_t error_degree_k_0 = 		desired_value_0 		- 		y_0 ;
		int32_t error_degree_k_1 = 		desired_value_1 		- 		y_1 ;
		int32_t error_degree_k_2 = 		desired_value_2 		- 		y_2 ;

		/*
		// "PID" Controller 1
		*control_variable = *control_variable + 
												(int32_t) ( A * ( (float)error_degree_k_0 ) ) + 
												(int32_t) ( B * ( (float)error_degree_k_1 ) ) + 
												(int32_t) ( C * ( (float)error_degree_k_2 ) );
		*/
		
		/*
		// "PID" Controller 2
		*control_variable = *control_variable  + 
												(int32_t) (
																	 (
																		(int64_t)			( ( (int64_t) (A*10.f) ) * ((int64_t) error_degree_k_0) ) + 
																		(int64_t)			(	( (int64_t) (B*10.f) ) * ((int64_t) error_degree_k_1) ) + 
																		(int64_t)			(	( (int64_t) (C*10.f) ) * ((int64_t) error_degree_k_2) )
																	 ) / 10
												);
		*/

		
		// "PID" Controller 3
		static float	Integrated_Error 			=  0;
		static float	Integrated_Error_max 	= 20;
		Integrated_Error	+= B * ( (float)(error_degree_k_0 + error_degree_k_1) ) * (0.050f / 2);
		if			( Integrated_Error >  Integrated_Error_max	)		Integrated_Error =  Integrated_Error_max;
		else	if(	Integrated_Error < -Integrated_Error_max	)		Integrated_Error = -Integrated_Error_max;
		
		*control_variable = (int32_t) (
																	 (
																		( A ) * ((float) error_degree_k_0 ) + 
																		Integrated_Error + 
																		( C ) * ( ((float)(error_degree_k_0 - error_degree_k_1) ) / 0.050f )
																	 ) / 1.f
												);
		
																		
		/*
		// "P" Controller 1
		*control_variable =	(error_degree_k_0 > 0)? ( (int32_t) ( A * ( (float)error_degree_k_0 ) ) ) :
																								( (int32_t) ( A * ( (float)error_degree_k_0 ) ) ) ;
		*/

		/*
		// "P" Controller 2
		*control_variable =	(error_degree_k_0 > 0)? (  5 ) :
																								( -5 ) ;
		*/
		
		if			( *control_variable >  control_variable_max	)		*control_variable =  control_variable_max;
		else	if(	*control_variable < -control_variable_max	)		*control_variable = -control_variable_max;
		
		//LOG_PID_ABC(A, B, C);	
		//LOG_Errors(error_degree_k_0, error_degree_k_1, error_degree_k_2, *control_variable);										

		*error_0 = (int16_t) error_degree_k_0;
		*error_1 = (int16_t) error_degree_k_1;
		*error_2 = (int16_t) error_degree_k_2;

}
	


