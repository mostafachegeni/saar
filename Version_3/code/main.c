/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "nrf_gpio.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_delay.h"
#include "my_service.h"
#include "SEGGER_RTT.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "Nordic_Template"                          /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 100                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       0 //180                                    /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(8, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.025 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(12, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.040 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// SAAR CODE:
// my_service
// FROM_SERVICE_TUTORIAL: Declare a service structure for our application
ble_os_t my_service;

// ************************************
// SAAR CODE:
// I2C
#include "i2c.h"		
		// PINs:
		// Defined in "i2c.h"
		
		
// ***********************************
// SAAR CODE:
// PID
#ifdef PID
		#include "PID.h"

		// test pid
		int16_t 		global_error_0 = 0;
		int16_t 		global_error_1 = 0;
		int16_t 		global_error_2 = 0;
				
		int8_t 			global_control_value = 0;
			
		uint8_t 		global_yaw_0 				 = 0;
		uint8_t 		global_yaw_1 				 = 0;
		int8_t 			global_num_of_turns  = 0;

#endif

// ***********************************
// SAAR CODE:
// MPU
#ifdef MPU
		#include "mpu.h"		
		bool ready_flag_YPR = false;
		APP_TIMER_DEF(m_MPU_timer_id); 
		#define TIMER_INTERVAL_MPU	APP_TIMER_TICKS(40, APP_TIMER_PRESCALER) // Defines the interval between consecutive app timer interrupts in milliseconds

		static void timer_timeout_handler_MPU(void * p_context)
		{
				int ret;

				unsigned int c 		 = 1; //cumulative number of successful MPU/DMP reads
				unsigned int np 	 = 0; //cumulative number of MPU/DMP reads that brought no packet back
				unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
				unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

				ret = mympu_update();
				// LOG("mympu_update returns: ", ret);
				switch (ret) {
						case 0: c++; 			break;
						case 1: np++; 		break;
						case 2: err_o++; 	break;
						case 3: err_c++; 	break; 
						default: 					break;
				}
				

				//LOG("MPU_ret =", ret);
				if(ret != 0 && ret != 1 && ret != 2 && ret != 3) {
						LOG("****************************************************", ret);
						LOG("                    MPU_ret ="												, ret);
						LOG("****************************************************", ret);
						(void)mympu_open(200);										
				}
				
				if (!(c%2)) {					
						ready_flag_YPR = true;
				}
		}				
#endif

// ***********************************
// SAAR CODE:
// PWM
#ifdef PWM
		// PINs:
		#if defined PCB4
				#define	motor_1_pwm_pin_Right		15
				#define	motor_1_pwm_pin_Left		14
				#define	motor_2_pwm_pin_Right		13
				#define	motor_2_pwm_pin_Left		12
				#define	motor_3_pwm_pin_Right		 9
				#define	motor_3_pwm_pin_Left		11
				#define	motor_4_pwm_pin_Right		10
				#define	motor_4_pwm_pin_Left		 8
		#elif defined PCB6_v5
				#define	motor_1_pwm_pin_Right		12
				#define	motor_1_pwm_pin_Left		11
				#define	motor_2_pwm_pin_Right		25
				#define	motor_2_pwm_pin_Left		24
				#define	motor_3_pwm_pin_Right		23
				#define	motor_3_pwm_pin_Left		22
				#define	motor_4_pwm_pin_Right		21
				#define	motor_4_pwm_pin_Left		20
		#elif defined PCB7
				#define	motor_1_pwm_pin_Right		14
				#define	motor_1_pwm_pin_Left		15
//				#define	motor_1_pwm_pin_Right		15
//				#define	motor_1_pwm_pin_Left		14
				#define	motor_2_pwm_pin_Right		13
				#define	motor_2_pwm_pin_Left		12
//				#define	motor_2_pwm_pin_Right		12
//				#define	motor_2_pwm_pin_Left		13
				#define	motor_3_pwm_pin_Right		11
				#define	motor_3_pwm_pin_Left		10
				#define	motor_4_pwm_pin_Right		 9
				#define	motor_4_pwm_pin_Left		 8				
		#endif
		
		#include "app_pwm.h"
		static volatile bool ready_flag_PWM1;       // A flag indicating PWM status.
		static volatile bool ready_flag_PWM2;       // A flag indicating PWM status.
		APP_TIMER_DEF(m_PWM_timer_id); 
		
		// Measured Period = 50 ms
		#define TIMER_INTERVAL_PWM	APP_TIMER_TICKS(50, APP_TIMER_PRESCALER) // Defines the interval between consecutive app timer interrupts in milliseconds
		
		APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.
		APP_PWM_INSTANCE(PWM2,2);                   // Create the instance "PWM2" using TIMER2.

		void pwm1_ready_callback(uint32_t pwm_id)   // PWM callback function
		{
				ready_flag_PWM1 = true;
		}

		void pwm2_ready_callback(uint32_t pwm_id)   // PWM callback function
		{
				ready_flag_PWM2 = true;
		}		
				
		static	uint8_t	motor_1_pwm_pin = motor_1_pwm_pin_Right, 
										motor_2_pwm_pin = motor_2_pwm_pin_Right, 
										motor_3_pwm_pin = motor_3_pwm_pin_Right, 
										motor_4_pwm_pin = motor_4_pwm_pin_Right;		
		
		static void timer_timeout_handler_PWM(void * p_context) {
				#define Right true
				#define Left  false
				
				uint8_t				char_1[10];			
				uint8_t 			motor_1_speed 			= 0, 			motor_2_speed 			= 0, 			motor_3_speed 			= 0, 			motor_4_speed = 0;
				static	bool	motor_1_dir_current = Right,	motor_2_dir_current = Right,	motor_3_dir_current = Right,	motor_4_dir_current = Right;
				bool 					motor_1_dir_last 		= Right,	motor_2_dir_last 		= Right,	motor_3_dir_last 		= Right,	motor_4_dir_last = Right;

				//Get Characteristic Value:
				ble_gatts_value_t gatts_value;
				gatts_value.len 			= 10; 						// number_of_bytes_you_want_to_read
				gatts_value.offset  	= 0;							// location_for_my_value
				gatts_value.p_value 	= char_1;
				sd_ble_gatts_value_get(	(&my_service) -> conn_handle, 									//uint16_t conn_handle
																(&my_service) -> char_handles[0].value_handle, 	//uint16_t handle
																(&gatts_value) 																	//ble_gatts_value_t *p_value
															);

				// SAAR CODE:
				// PID
				#ifdef PID
						#define	PID_State_Initial	0	// Initial State
						#define	PID_State_Move		1	// Move_the_ballon_by_PID State
						#define	PID_State_Stop		2	// Stop_the_ballon_after_moving State
						#define	PID_State_Wait		3	// Wait_for_Move_command State

		 static	uint8_t	PID_State										= PID_State_Initial;
		 static int32_t control_variable						= 0;
						int8_t	mean_motors_speed						= *( (int8_t*)(&char_1[5]) );
						int32_t control_variable_max				= char_1[8]; // 50;
						float 	Coeff_integrated_Error_max 	= ( (float)char_1[9] ) / 100.f;
						int32_t	desired_value								= (*( (int8_t*)(&char_1[3]) ))*360 + (int32_t) ( ((float)char_1[4]) * (359.f/255.f) );
						float		Kp, 	Ki, 	Kd;
		 static float 	Ts = 50.f/1000.f;
						float 	A, 	 B, 	 C;
						
						Kp = (char_1[0] <= 100)? ( ( (float) char_1[0] ) / ( 100.f ) ) : ( ( (float) (100 - char_1[0]) ) / ( 100.f ) );//char_1[0]; 
						Ki = (char_1[1] <= 100)? ( ( (float) char_1[1] ) / ( 100.f ) ) : ( ( (float) (100 - char_1[1]) ) / ( 100.f ) );//char_1[1]; 
						Kd = (char_1[2] <= 100)? ( ( (float) char_1[2] ) / ( 100.f ) ) : ( ( (float) (100 - char_1[2]) ) / ( 100.f ) );//char_1[2]; 
						coeff_PID_2_ABC ( (float) Kp, (float) Ki, (float) Kd,  Ts, &A, &B, &C);

						#ifdef MPU
/*						
								static int8_t  num_of_turns					= 	0;
								static int32_t measured_output 			=	180; // is set to 180 in ordet not to change "num_of_turns" in first loop.
								static int32_t measured_output_last = 180; // is set to 180 in ordet not to change "num_of_turns" in first loop.
								measured_output_last 	= measured_output;
								measured_output				= ( (int32_t) (mympu.ypr[0] + 180.f) ) + num_of_turns * 360; //[0]:yaw , [1]:roll, [2]:pitch

								if( ( measured_output - (num_of_turns * 360) <  60 ) && ( measured_output_last - (num_of_turns * 360) > 300) )	{
											num_of_turns++;
											measured_output = measured_output + 360;
								}	else {
												if( ( measured_output - (num_of_turns * 360) > 300 ) && ( measured_output_last - (num_of_turns * 360) <  60) )	{
														num_of_turns--;
														measured_output = measured_output - 360;												
												}
								}

								
								if( abs(measured_output -  measured_output_last) >  180 ) {
//										measured_output = ( measured_output_last + measured_output ) / 2; 
										measured_output = measured_output_last; 
										
								}
								

								global_yaw_1 				= (uint8_t) ( ( 					 						  measured_output 									 									 			 & 0x0000FF00 ) >> 8 );
								global_yaw_0 				= (uint8_t) ( (  ((int32_t) (  ((float)(measured_output - num_of_turns * 360))*( 255.f/360.f )  )) & 0x000000FF ) >> 0 );
								global_num_of_turns = num_of_turns;
								
								// LOG("measured_output =", measured_output);
*/
								

// Float measured_output

								static int8_t  num_of_turns					= 	0;
								static float measured_output 			=	180; // is set to 180 in ordet not to change "num_of_turns" in first loop.
								static float measured_output_last = 180; // is set to 180 in ordet not to change "num_of_turns" in first loop.
								measured_output_last 	= measured_output;
								measured_output				= ( (mympu.ypr[0] + 180.f) ) + ( (float) (num_of_turns * 360) ); //[0]:yaw , [1]:roll, [2]:pitch

								if( ( measured_output - (num_of_turns * 360) <  60 ) && ( measured_output_last - (num_of_turns * 360) > 300) )	{
											num_of_turns++;
											measured_output = measured_output + 360.f;
								}	else {
												if( ( measured_output - (num_of_turns * 360) > 300 ) && ( measured_output_last - (num_of_turns * 360) <  60) )	{
														num_of_turns--;
														measured_output = measured_output - 360.f;												
												}
								}

								
								if( abs(measured_output -  measured_output_last) >  180 ) {
//										measured_output = ( measured_output_last + measured_output ) / 2; 
										measured_output = measured_output_last; 
										
								}
								

								global_yaw_1 				= (uint8_t) ( ( 					 			(int32_t) measured_output 								 									 			 			 & 0x0000FF00 ) >> 8 );
								global_yaw_0 				= (uint8_t) ( (  ((int32_t) (  ( (measured_output - (float)(num_of_turns * 360.f)) )*( 255.f/360.f )  )) & 0x000000FF ) >> 0 );
								global_num_of_turns = num_of_turns;
								
								// LOG("measured_output =", measured_output);
								
						#endif
								
						//LOG_PID_ABC(A, B, C);		

/*
		 static bool 		flag_turn_motors_off					= false;	
		 static bool 		flag_1st_itteration_to_stop 	= true;
		 static int32_t	temp_desired_value						= 0;
						bool		flag_go_to_move_state					= char_1[7] & 0x01;

						switch (PID_State) {
								case PID_State_Initial:
											flag_turn_motors_off = true;
											if(flag_go_to_move_state)	
													PID_State = PID_State_Move;			 
								break;
								
								case PID_State_Move:
											flag_1st_itteration_to_stop = true;																		
											flag_turn_motors_off 				= false;
											if(!flag_go_to_move_state)
													PID_State = PID_State_Stop;			 
								break;
								
								case PID_State_Stop:
											if(!flag_go_to_move_state) {
														if(flag_1st_itteration_to_stop) {
																temp_desired_value					= measured_output;
																flag_1st_itteration_to_stop	= false;
														}
														desired_value	= temp_desired_value;
														static uint8_t	counter = 0;
														counter++;
														if(counter > 1 * 20) {
																counter 							= 0;
																flag_turn_motors_off 	= true;															
																PID_State							= PID_State_Wait;
														}
											} else {
																PID_State							= PID_State_Wait;
											}
								break;	

								case PID_State_Wait:
											num_of_turns				 	= 0;
											measured_output 		 	= 180;
											measured_output_last 	= 180;								
											if(flag_go_to_move_state)	
													PID_State = PID_State_Move;
								break;	
											
								default:
								break;
						}
	
						PID_Controller_Function (	desired_value, 
																			measured_output, 
																			A, 
																			B, 
																			C, 
																			&control_variable,
																			control_variable_max,
																			&global_error_0,
																			&global_error_1,
																			&global_error_2
																		);
*/
						

// Float measured_output:
		 static bool 		flag_turn_motors_off								= false;	
		 static bool 		flag_1st_itteration_to_stop 				= true;
		 static bool 		flag_reset_Integrated_Error_in_PID	= true;
		 static int32_t	temp_desired_value									= 0;
						bool		flag_go_to_move_state								= char_1[7] & 0x01;

						switch (PID_State) {
								case PID_State_Initial:
											flag_turn_motors_off 								= true;
											flag_reset_Integrated_Error_in_PID	= true;
											if(flag_go_to_move_state)
													PID_State = PID_State_Move;
								break;
								
								case PID_State_Move:
											flag_1st_itteration_to_stop 			 = true;																		
											flag_turn_motors_off 							 = false;
											flag_reset_Integrated_Error_in_PID = false;
											if(!flag_go_to_move_state)
													PID_State = PID_State_Stop;
								break;
								
								case PID_State_Stop:
											if(!flag_go_to_move_state) {
														if(flag_1st_itteration_to_stop) {
																temp_desired_value					= (int32_t) measured_output;
																flag_1st_itteration_to_stop	= false;
														}
														desired_value	= temp_desired_value;
														static uint8_t	counter = 0;
														counter++;
														if(counter > 1 * 20) {
																counter 													 = 0;
																flag_turn_motors_off 							 = true;															
																flag_reset_Integrated_Error_in_PID = true;
																PID_State													 = PID_State_Wait;
																
														}
											} else {
																PID_State							= PID_State_Wait;
											}
								break;	

								case PID_State_Wait:
											num_of_turns				 	= 0;
											measured_output 		 	= 180;
											measured_output_last 	= 180;								
											if(flag_go_to_move_state)	
													PID_State = PID_State_Move;
								break;	
											
								default:
								break;
						}
								
						
						PID_Controller_Function (	desired_value, 
																			measured_output, 
																			A, 
																			B, 
																			C, 
																			&control_variable,
																			control_variable_max,
																			mympu.gyro,
																			(int) flag_reset_Integrated_Error_in_PID,
																			Coeff_integrated_Error_max,	
																			&global_error_0,
																			&global_error_1,
																			&global_error_2
																		);

						
						// LOG("control_variable =", control_variable);

						// test pid
						/*
						control_variable	= (Kp							)?	(control_variable	):
																(char_1[5] < 100)? 	(char_1[5]				):
																										(100 - char_1[5]	);
						*/
						
						global_control_value = (int8_t) ( control_variable );
						

						if(control_variable > 0) {
//								motor_1_speed = (uint8_t) (50 + abs(control_variable)); //char_1[0]; //char_1[0]/2;
//								motor_2_speed = (uint8_t) (50 - abs(control_variable)); //char_1[1]; //char_1[1]/2;

//								motor_1_speed = (uint8_t) (0 + abs(control_variable)); //char_1[0]; //char_1[0]/2;
//								motor_2_speed = 0;

//								motor_1_speed = (uint8_t) ( abs(char_1[5] + abs(control_variable)) ); //char_1[0]; //char_1[0]/2;
//								motor_2_speed = (uint8_t) ( abs(char_1[5] - abs(control_variable)) ); //char_1[1]; //char_1[1]/2;

//								motor_1_speed = (uint8_t) ( abs(char_1[5] + abs(control_variable)) ); 
//								motor_2_speed = (abs(control_variable) < char_1[5])?	((uint8_t) ( abs(char_1[5] - abs(control_variable)) )) : 0; 

								motor_1_speed = (uint8_t) ( abs(mean_motors_speed + 1*abs(control_variable)) );
								motor_2_speed = (uint8_t) ( abs(mean_motors_speed - 2*abs(control_variable)) );
								if(motor_1_speed > 100)	motor_1_speed = 100;
								if(motor_2_speed > 100)	motor_2_speed = 100;
							
//								motor_1_speed = (uint8_t) (char_1[5] + abs(control_variable)); //char_1[0]; //char_1[0]/2;
//								motor_2_speed = (uint8_t) (char_1[5] + abs(control_variable)); //char_1[1]; //char_1[1]/2;

								motor_1_dir_last			= motor_1_dir_current;
								motor_2_dir_last			= motor_2_dir_current;

//								motor_1_dir_current		= Right;
//								motor_2_dir_current		= (char_1[5] < abs(control_variable))?	Left : Right;

								motor_1_dir_current		= ( (mean_motors_speed + 1*abs(control_variable)) < 0 )?	Left : Right;
								motor_2_dir_current		= ( (mean_motors_speed - 2*abs(control_variable)) < 0 )?	Left : Right;

//								motor_1_dir_current		= Right;
//								motor_2_dir_current		= Left;

//								motor_1_dir_current		= Right;
//								motor_2_dir_current		= Right;

						} else {
//								motor_1_speed = (uint8_t) (50 - abs(control_variable)); //char_1[0]; //char_1[0]/2;
//								motor_2_speed = (uint8_t) (50 + abs(control_variable)); //char_1[1]; //char_1[1]/2;

//								motor_1_speed = 0;
//								motor_2_speed = (uint8_t) (0 + abs(control_variable)); //char_1[1]; //char_1[1]/2;

//								motor_1_speed = (uint8_t) ( abs(char_1[5] - abs(control_variable)) ); //char_1[0]; //char_1[0]/2;
//								motor_2_speed = (uint8_t) ( abs(char_1[5] + abs(control_variable)) ); //char_1[1]; //char_1[1]/2;

//								motor_1_speed = (abs(control_variable) < char_1[5])?	((uint8_t) ( abs(char_1[5] - abs(control_variable)) )) : 0;
//								motor_2_speed = (uint8_t) ( abs(char_1[5] + abs(control_variable)) );

								motor_1_speed = (uint8_t) ( abs(mean_motors_speed - 2*abs(control_variable)) );
								motor_2_speed = (uint8_t) ( abs(mean_motors_speed + 1*abs(control_variable)) );
								if(motor_1_speed > 100)	motor_1_speed = 100;
								if(motor_2_speed > 100)	motor_2_speed = 100;
									 
//								motor_1_speed = (uint8_t) (char_1[5] + abs(control_variable)); //char_1[0]; //char_1[0]/2;
//								motor_2_speed = (uint8_t) (char_1[5] + abs(control_variable)); //char_1[1]; //char_1[1]/2;

								motor_1_dir_last			= motor_1_dir_current;
								motor_2_dir_last			= motor_2_dir_current;

//								motor_1_dir_current		= (char_1[5] < abs(control_variable))?	Left : Right;
//								motor_2_dir_current		= Right;							
							
//								motor_1_dir_current		= Left;
//								motor_2_dir_current		= Right;							
							
//								motor_1_dir_current		= Right;
//								motor_2_dir_current		= Right;

								motor_1_dir_current		= ( (mean_motors_speed - 2*abs(control_variable)) < 0 )?	Left : Right;
								motor_2_dir_current		= ( (mean_motors_speed + 1*abs(control_variable)) < 0 )?	Left : Right;

						}

						motor_3_speed 			= (uint8_t) ( (char_1[6] < 100)? abs(char_1[6]) : abs(100 - char_1[6]) ); // (if "<100" then "+" ) (if ">100" then "-" )
						motor_3_dir_last		= motor_3_dir_current;
						motor_3_dir_current	= (char_1[6] < 100)? Right : Left;

						/*
						motor_4_speed 			= (uint8_t) ( (char_1[6] < 100)? abs(char_1[6]) : abs(100 - char_1[6]) );
						motor_4_dir_last		= motor_4_dir_current;
						motor_4_dir_current	= (char_1[6] < 100)? Right : Left;
						*/
						
						if(flag_turn_motors_off) {
								motor_1_speed = 0;
								motor_2_speed = 0;
								//motor_3_speed = 0;
								//motor_4_speed = 0;
						}
						
				#else
						motor_1_speed = char_1[0];
						motor_2_speed = char_1[1];
						motor_3_speed = char_1[2];
						motor_4_speed = char_1[3];
					
						motor_1_dir_last			= motor_1_dir_current;
						motor_2_dir_last			= motor_2_dir_current;
						motor_3_dir_last			= motor_3_dir_current;
						motor_4_dir_last			= motor_4_dir_current;
						
						motor_1_dir_current		= (char_1[4] & 0x01)? Right : Left;
						motor_2_dir_current		= (char_1[4] & 0x02)? Right : Left;
						motor_3_dir_current		= (char_1[4] & 0x04)? Right : Left;
						motor_4_dir_current		= (char_1[4] & 0x08)? Right : Left;
				#endif
				
				
				
				// ------------------------------------- PWM 1: --------------------------------- //
				ready_flag_PWM1 = false;
				//Set the duty cycle - keep trying until PWM is ready...
				app_pwm_channel_duty_set(&PWM1, 0, motor_1_speed);
				app_pwm_channel_duty_set(&PWM1, 1, motor_2_speed);
				//while (app_pwm_channel_duty_set(&PWM1, 0, motor_1_speed) == NRF_ERROR_BUSY);
				//while (app_pwm_channel_duty_set(&PWM1, 1, motor_2_speed) == NRF_ERROR_BUSY);
			
				//... or wait for callback. 
				//while(!ready_flag_PWM1);
				//APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, value));
				// ------------------------------------------------------------------------------ //
				
				// ------------------------------------- PWM 2: --------------------------------- //					
				ready_flag_PWM2 = false;
				//Set the duty cycle - keep trying until PWM is ready... 
				app_pwm_channel_duty_set(&PWM2, 0, motor_3_speed);
				app_pwm_channel_duty_set(&PWM2, 1, motor_4_speed);
				//while (app_pwm_channel_duty_set(&PWM2, 0, motor_3_speed) == NRF_ERROR_BUSY);
				//while (app_pwm_channel_duty_set(&PWM2, 1, motor_4_speed) == NRF_ERROR_BUSY);
					
				//... or wait for callback. 
				//while(!ready_flag_PWM2);					
				//APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM2, 1, value));
				// ------------------------------------------------------------------------------ //
				
				// For Test
				// buzzer
				/*
				static uint32_t counter	= 0;
				static uint32_t period	= 1;
				
				counter++;
				if(counter > 30000)	counter = 0;

				//if(counter % 2 == 0) {
				if(period != (char_1[2]*100) ) {
						//period += 10;
						period = char_1[2] * 100;
						//if(period > 1000)	
						//		period	= 1;
						
						app_pwm_disable(&PWM1);
						APP_ERROR_CHECK(app_pwm_uninit(&PWM1));
						
						//if(counter % 2 == 0)	{
						//			if(motor_1_dir_current == Right) 		motor_1_dir_current = Left;
						//			else																motor_1_dir_current = Right;	
						//			if(motor_2_dir_current == Right) 		motor_2_dir_current = Left;
						//			else																motor_2_dir_current = Right;
						//}
						//if( motor_1_dir_current == Right)		{	motor_1_pwm_pin = motor_1_pwm_pin_Right;	motor_1_pwm_pin_last = motor_1_pwm_pin_Left;}
						//else																{	motor_1_pwm_pin = motor_1_pwm_pin_Left;		motor_1_pwm_pin_last = motor_1_pwm_pin_Right;}
						//if( motor_2_dir_current == Right)		{	motor_2_pwm_pin = motor_2_pwm_pin_Right;	motor_2_pwm_pin_last = motor_2_pwm_pin_Left;}
						//else																{	motor_2_pwm_pin = motor_2_pwm_pin_Left;		motor_2_pwm_pin_last = motor_2_pwm_pin_Right;}
						
						app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(period, motor_1_pwm_pin, motor_2_pwm_pin);
						pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
						pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
						APP_ERROR_CHECK(app_pwm_init(&PWM1, &pwm1_cfg, pwm1_ready_callback));
						app_pwm_enable(&PWM1);

						app_pwm_disable(&PWM2);
						APP_ERROR_CHECK(app_pwm_uninit(&PWM2));
						
						// if(counter % 2 == 0)	{
						// 			if(motor_3_dir_current == Right) 		{motor_3_dir_current = Left;	}
						// 			else																{motor_3_dir_current = Right;	}	
						// 			if(motor_4_dir_current == Right) 		{motor_4_dir_current = Left;	}
						// 			else																{motor_4_dir_current = Right;	}
						
						// if( motor_3_dir_current == Right)		{	motor_3_pwm_pin = motor_3_pwm_pin_Right;	motor_3_pwm_pin_last = motor_3_pwm_pin_Left;}
						// else																{	motor_3_pwm_pin = motor_3_pwm_pin_Left;		motor_3_pwm_pin_last = motor_3_pwm_pin_Right;}
						// if( motor_4_dir_current == Right)		{	motor_4_pwm_pin = motor_4_pwm_pin_Right;	motor_4_pwm_pin_last = motor_4_pwm_pin_Left;}
						// else																{	motor_4_pwm_pin = motor_4_pwm_pin_Left;		motor_4_pwm_pin_last = motor_4_pwm_pin_Right;}
						// }
						
						app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_2CH(period, motor_3_pwm_pin, motor_4_pwm_pin);												
						pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
						pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
						APP_ERROR_CHECK(app_pwm_init(&PWM2, &pwm2_cfg, pwm2_ready_callback));
						app_pwm_enable(&PWM2);
						
				}				
				*/
				
				if(	motor_1_dir_last != motor_1_dir_current) {
								LOG("PWM: motor_1 changed direction - Last =", motor_1_dir_last);
								LOG("PWM: motor_1 changed direction - Curr =", motor_1_dir_current);
								app_pwm_disable(&PWM1);
								APP_ERROR_CHECK(app_pwm_uninit(&PWM1));
					
								if( motor_1_dir_current == Right)		motor_1_pwm_pin = motor_1_pwm_pin_Right;
								else																motor_1_pwm_pin = motor_1_pwm_pin_Left;
								
								app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_1_pwm_pin, motor_2_pwm_pin);
								pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
								pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

								APP_ERROR_CHECK(app_pwm_init(&PWM1, &pwm1_cfg, pwm1_ready_callback));
								app_pwm_enable(&PWM1);																
				}

				if(	motor_2_dir_last != motor_2_dir_current) {
								app_pwm_disable(&PWM1);
								APP_ERROR_CHECK(app_pwm_uninit(&PWM1));
					
								if( motor_2_dir_current == Right)		motor_2_pwm_pin = motor_2_pwm_pin_Right;
								else																motor_2_pwm_pin = motor_2_pwm_pin_Left;
								
								app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_1_pwm_pin, motor_2_pwm_pin);
								pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
								pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

								APP_ERROR_CHECK(app_pwm_init(&PWM1, &pwm1_cfg, pwm1_ready_callback));
								app_pwm_enable(&PWM1);																
				}

				if(	motor_3_dir_last != motor_3_dir_current) {
								app_pwm_disable(&PWM2);
								APP_ERROR_CHECK(app_pwm_uninit(&PWM2));
					
								if( motor_3_dir_current == Right)		motor_3_pwm_pin = motor_3_pwm_pin_Right;
								else																motor_3_pwm_pin = motor_3_pwm_pin_Left;
								
								app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_3_pwm_pin, motor_4_pwm_pin);
								pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
								pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
					
								APP_ERROR_CHECK(app_pwm_init(&PWM2, &pwm2_cfg, pwm2_ready_callback));
								app_pwm_enable(&PWM2);																
				}

				if(	motor_4_dir_last != motor_4_dir_current) {
								app_pwm_disable(&PWM2);
								APP_ERROR_CHECK(app_pwm_uninit(&PWM2));
					
								if( motor_4_dir_current == Right)		motor_4_pwm_pin = motor_4_pwm_pin_Right;
								else																motor_4_pwm_pin = motor_4_pwm_pin_Left;
								
								app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_3_pwm_pin, motor_4_pwm_pin);
								pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
								pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
					
								APP_ERROR_CHECK(app_pwm_init(&PWM2, &pwm2_cfg, pwm2_ready_callback));
								app_pwm_enable(&PWM2);																
				}
				
		}
#endif

		
// ***********************************
// SAAR CODE:
// BMP
#ifdef BMP180_API  
		#include "bmp180_support.h"
		APP_TIMER_DEF(m_BMP180_timer_id); 
		#define TIMER_INTERVAL_BMP180	APP_TIMER_TICKS(50, APP_TIMER_PRESCALER) // Defines the interval between consecutive app timer interrupts in milliseconds

		static void timer_timeout_handler_BMP180(void * p_context)
		{
						u16 v_uncomp_temp_u16;
						u32 v_uncomp_press_u32;		
						s16 v_true_temp_s16;
						s32 v_true_press_s32;		

						v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();
						v_uncomp_press_u32 = bmp180_get_uncomp_pressure();	
						v_true_temp_s16 = bmp180_get_temperature(v_uncomp_temp_u16);
						v_true_press_s32 = bmp180_get_pressure(v_uncomp_press_u32);
						LOG_BMP_180_temperature_pressure( (uint16_t) v_true_temp_s16, (uint32_t) v_true_press_s32);
		}			
#endif

// ***********************************
// SAAR CODE:
// BME
#ifdef BME280_API  
		#include "bme280_support.h"
		APP_TIMER_DEF(m_BME280_timer_id); 
		#define TIMER_INTERVAL_BME280	APP_TIMER_TICKS(50, APP_TIMER_PRESCALER) // Defines the interval between consecutive app timer interrupts in milliseconds

		static void timer_timeout_handler_BME280(void * p_context)
		{
				struct bme280_data comp_data;

				#ifdef BME280_WORK_IN_FORCED_MODE
						// Forced mode:
						// Wait for the measurement to complete and print data @25Hz 
						int8_t rslt;
						rslt += bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
						dev.delay_ms(40);
						rslt += bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
						LOG_BME_280_temperature_pressure_humidity(comp_data.temperature, comp_data.pressure, comp_data.humidity);
				#else
						// Normal mode:
						// Delay while the sensor completes a measurement 
						int8_t rslt; 
						dev.delay_ms(70);
						rslt += bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);		
						LOG_BME_280_temperature_pressure_humidity(comp_data.temperature, comp_data.pressure, comp_data.humidity);						
				#endif
		}			
#endif

// ***********************************
// SAAR CODE:
// ADC
#ifdef ADC  
		#include "my_ADC.h"
		APP_TIMER_DEF(m_ADC_timer_id); 
		#define TIMER_INTERVAL_ADC	APP_TIMER_TICKS(10, APP_TIMER_PRESCALER) // Defines the interval between consecutive app timer interrupts in milliseconds

		static void timer_timeout_handler_ADC(void * p_context)
		{
			if((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) != ADC_BUSY_BUSY_Busy)
				nrf_drv_adc_sample();
		}
#endif

// ***********************************
// SAAR CODE:
// GPIOs
#ifdef GPIOs
		// PINs:
		#if defined PCB4
				#define Driver_1_Sleep_pin	17 
				#define Driver_2_Sleep_pin	18
				#define LED_power_pin 			 2 
				#define LED_1_pin 					 4 
				#define LED_2_pin 					 3 
				#define LED_3_pin 					19 
				#define LED_4_pin 					20		
		#elif defined	PCB6_v5
				#define Driver_1_Fault_pin	16 
				#define Driver_2_Fault_pin	15
				#define Driver_1_Sleep_pin	13
				#define Driver_2_Sleep_pin	14
				#define LED_power_pin 			17 
				#define LED_1_pin 					29
				#define LED_2_pin 					28
				#define LED_3_pin 					27
				#define LED_4_pin 					26	
		#elif defined PCB7		
				#define Driver_1_Fault_pin	22 
				#define Driver_2_Fault_pin	24
				#define Driver_1_Sleep_pin	21
				#define Driver_2_Sleep_pin	23
				#define LED_power_pin 			25 
				#define LED_1_pin 					17
				#define LED_2_pin 					18
				#define LED_3_pin 					19
				#define LED_4_pin 					16					
		#endif

		APP_TIMER_DEF(m_GPIOs_timer_id); 
		#define TIMER_INTERVAL_GPIOs	APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) // Defines the interval between consecutive app timer interrupts in milliseconds
		
static void timer_timeout_handler_GPIOs(void * p_context)
{
				#ifdef	PCB6_v5
						int  flag = 0;
						uint32_t fault1 = nrf_gpio_pin_read(Driver_1_Fault_pin);
						uint32_t fault2 = nrf_gpio_pin_read(Driver_2_Fault_pin);
						if(fault1) {
								flag = 1;
						}
				#elif defined PCB7		
						int  flag = 0;
						uint32_t fault1 = nrf_gpio_pin_read(Driver_1_Fault_pin);
						uint32_t fault2 = nrf_gpio_pin_read(Driver_2_Fault_pin);
						if(fault1) {
								flag = 1;
						}						
				#endif
	
				nrf_gpio_pin_set(Driver_1_Sleep_pin);	//nrf_gpio_pin_clear(Driver_1_Sleep_pin);
				nrf_gpio_pin_set(Driver_2_Sleep_pin);	//nrf_gpio_pin_clear(Driver_2_Sleep_pin);

				nrf_gpio_pin_set(LED_power_pin);			//nrf_gpio_pin_clear(LED_power_pin);
				nrf_gpio_pin_set(LED_1_pin);					//nrf_gpio_pin_clear(LED_1_pin);
				nrf_gpio_pin_set(LED_2_pin);					//nrf_gpio_pin_clear(LED_2_pin);
				nrf_gpio_pin_set(LED_3_pin);					//nrf_gpio_pin_clear(LED_3_pin);
				nrf_gpio_pin_set(LED_4_pin);					//nrf_gpio_pin_clear(LED_4_pin);
}		
#endif


static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

                                   
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



// SAAR CODE:
// my_service
// Step 3.G, Declare an app_timer id variable and define our timer interval and define a timer interval
APP_TIMER_DEF(m_my_char_timer_id);
#define TIMER_INTERVAL_MY_CHAR     APP_TIMER_TICKS(40, APP_TIMER_PRESCALER) // 100 ms intervals

// SAAR CODE:
// my_service
static void timer_timeout_handler_MY_CHAR(void * p_context)
{
	// Step 3.F, Update temperature and characteristic value.
	
	// test pid
	static uint8_t  	char_2[18];
	
	//Set Characteristic Value:
	/*
	ble_gatts_value_t 		p_value;
	p_value.len 			= 5;
	p_value.offset 		= 0;
	uint8_t value[p_value.len];
	p_value.p_value 	= value;
	sd_ble_gatts_value_set(	(BLE_CONN_HANDLE_INVALID),											// uint16_t conn_handle
													(&my_service) -> char_handles[0].value_handle, 	// uint16_t handle
													(&p_value)																			// ble_gatts_value_t *p_value
												);
	*/	

	//Get Characteristic Value:
	/*
	ble_gatts_value_t gatts_value;
	gatts_value.len 			= 4; 									// number_of_bytes_you_want_to_read
	gatts_value.offset  	= 0;									// location_for_my_value
	gatts_value.p_value 	= (uint8_t*)char_2;	//
	sd_ble_gatts_value_get(	(&my_service) -> conn_handle, 									//uint16_t conn_handle
													(&my_service) -> char_handles[0].value_handle, 	//uint16_t handle
													(&gatts_value) 																	//ble_gatts_value_t *p_value
												);
	*/
	
	// SAAR CODE:
	// MPU
	#ifdef MPU
			if(ready_flag_YPR){
				ready_flag_YPR = false;
				
				int32_t Yaw, Pitch, Roll;
				
				Yaw 	= (int32_t) ( (mympu.ypr[0] + 180.f) * ( 250.f/360.f ) );
				Roll 	= (int32_t) ( (mympu.ypr[1] +  90.f) * ( 250.f/180.f ) );
				Pitch = (int32_t) ( (mympu.ypr[2] + 180.f) * ( 250.f/360.f ) );
				
				//Yaw		= (int32_t) ( mympu.ypr[0] * ( 250.f/360.f ));
				//Roll	= (int32_t) ( mympu.ypr[1] * ( 250.f/360.f ));
				//Pitch	= (int32_t) ( mympu.ypr[2] * ( 250.f/360.f ));
				
				//LOG_YPR(Yaw, Roll, Pitch);
								
				char_2[0] = global_yaw_0; //(uint8_t) ( ( ( Yaw   <<  0 ) & 0x000000FF ) >>  0 );
				char_2[1] = (uint8_t) ( ( ( Roll  <<  8 ) & 0x0000FF00 ) >>  8 );
				char_2[2] = (uint8_t) ( ( ( Pitch << 16 ) & 0x00FF0000 ) >> 16 );
				
				
				char_2[3] = (uint8_t)( ( (*( (uint16_t*)(&global_error_0) )) >> 0 ) & 0xFF); // mympu.accel[0];
				char_2[4] = (uint8_t)( ( (*( (uint16_t*)(&global_error_0) )) >> 8 ) & 0xFF); // mympu.accel[1];
				char_2[5] = (uint8_t)( ( (*( (uint16_t*)(&global_error_1) )) >> 0 ) & 0xFF); // mympu.accel[2];
				char_2[6] = (uint8_t)( ( (*( (uint16_t*)(&global_error_1) )) >> 8 ) & 0xFF);
				char_2[7] = (uint8_t)( ( (*( (uint16_t*)(&global_error_2) )) >> 0 ) & 0xFF);
				char_2[8] = (uint8_t)( ( (*( (uint16_t*)(&global_error_2) )) >> 8 ) & 0xFF);

				int16_t Gyro = (int16_t) mympu.gyro[2];
				char_2[9]	 = (uint8_t)( ( (*( (uint16_t*)(&Gyro) )) >> 0 ) & 0xFF );
				char_2[10] = (uint8_t)( ( (*( (uint16_t*)(&Gyro) )) >> 8 ) & 0xFF );

				// test pid
				char_2[15] = 0x75;
				char_2[16] = (uint8_t)( *( (uint8_t*)(&global_num_of_turns	)) );
				char_2[17] = (uint8_t)( *( (uint8_t*)(&global_control_value )) );
			}
	#endif

	// SAAR CODE:
	// ADC
	#ifdef ADC
				char_2[11] = (uint8_t) ( (adc_2 >> 8) & 0xFF );
				char_2[12] = (uint8_t) ( (adc_2 >> 0) & 0xFF );
				char_2[13] = (uint8_t) ( (adc_6 >> 8) & 0xFF );
				char_2[14] = (uint8_t) ( (adc_6 >> 0) & 0xFF );
	#endif

	// test pid		
	uint8_t 	length = 18;
//	uint8_t 	length = 15;

	uint32_t 	index_of_characeristic = 1;		
	my_characteristic_update(&my_service, length, char_2, index_of_characeristic); //updating characteristic value (char_2)
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

		// SAAR CODE:
		// my_service
    // Create timers.
    // Step 3.H, Initiate our timer
    uint32_t err_code;
		err_code = app_timer_create(&m_my_char_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler_MY_CHAR);
    APP_ERROR_CHECK(err_code);

		// SAAR CODE:
		// MPU
		#ifdef MPU
				err_code = app_timer_create(&m_MPU_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler_MPU);
				APP_ERROR_CHECK(err_code);
		#endif
	
		// SAAR CODE:
		// PWM
		#ifdef PWM 
				err_code = app_timer_create(&m_PWM_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler_PWM);
				APP_ERROR_CHECK(err_code);
		#endif

		// SAAR CODE:
		// BMP
		#ifdef BMP180_API  
				err_code = app_timer_create(&m_BMP180_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler_BMP180);
				APP_ERROR_CHECK(err_code);
		#endif

		// SAAR CODE:
		// BME
		#ifdef BME280_API  
				err_code = app_timer_create(&m_BME280_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler_BME280);
				APP_ERROR_CHECK(err_code);
		#endif

		// SAAR CODE:
		// ADC
		#ifdef ADC 
				err_code = app_timer_create(&m_ADC_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler_ADC);
				APP_ERROR_CHECK(err_code);
		#endif
		
		// SAAR CODE:
		// GPIOs
		#ifdef GPIOs
				err_code = app_timer_create(&m_GPIOs_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler_GPIOs);
				APP_ERROR_CHECK(err_code);		
		#endif
}

// SAAR CODE:
/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
		// SAAR CODE:
		// my_service
		// Step 3.I, Start our timer
    uint32_t err_code;
    err_code = app_timer_start(m_my_char_timer_id, TIMER_INTERVAL_MY_CHAR, NULL);
    APP_ERROR_CHECK(err_code);

		// SAAR CODE:
		// MPU
		#ifdef MPU
				err_code = app_timer_start(m_MPU_timer_id, TIMER_INTERVAL_MPU, NULL);
				APP_ERROR_CHECK(err_code);
				nrf_delay_ms(15000); // If Motors start from begining(Before MPU Stabilization) -> MPU Wraps around continuesly -> So we Delay Motors Starting 
		#endif
	
		// SAAR CODE:
		// PWM
		#ifdef PWM
				//nrf_delay_ms(10000); // If Motors start from begining(Before MPU Stabilization) -> MPU Wraps around continuesly -> So we Delay Motors Starting 
				//err_code = app_timer_start(m_PWM_timer_id, TIMER_INTERVAL_PWM, NULL);
				//APP_ERROR_CHECK(err_code);
		#endif

		// SAAR CODE:
		// BMP
		#ifdef BMP180_API  
				err_code = app_timer_start(m_BMP180_timer_id, TIMER_INTERVAL_BMP180, NULL);
				APP_ERROR_CHECK(err_code);
		#endif

		// SAAR CODE:
		// BME
		#ifdef BME280_API  
				err_code = app_timer_start(m_BME280_timer_id, TIMER_INTERVAL_BME280, NULL);
				APP_ERROR_CHECK(err_code);
		#endif
		
		// SAAR CODE:
		// ADC
		#ifdef ADC
				err_code = app_timer_start(m_ADC_timer_id, TIMER_INTERVAL_ADC, NULL);
				APP_ERROR_CHECK(err_code);
		#endif

		// SAAR CODE:
		// GPIOs
		#ifdef GPIOs
				err_code = app_timer_start(m_GPIOs_timer_id, TIMER_INTERVAL_GPIOs, NULL);
				APP_ERROR_CHECK(err_code);
		#endif

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
																					
    APP_ERROR_CHECK(err_code);

																					
    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

		// SAAR CODE:
		// Tx_POWER																					
		sd_ble_gap_tx_power_set(4);

}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
		// SAAR CODE:
		// my_service
		my_service_init(&my_service);
	
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        APP_ERROR_CHECK( sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE) );
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
	
		static bool flag_first_connection							= 1;
	
		static bool flag_MPU_timer_has_been_stopped 	= 0;
		static bool flag_PWM_timer_has_been_stopped 	= 0;
		static bool flag_BMP_timer_has_been_stopped 	= 0;
		static bool flag_BME_timer_has_been_stopped 	= 0;
		static bool flag_ADC_timer_has_been_stopped 	= 0;
		static bool flag_GPIOs_timer_has_been_stopped = 0;

    switch (p_ble_evt->header.evt_id)
            {
							
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				
						// SAAR CODE
						LOG("When Connect", __LINE__);
				
						// SAAR CODE:
						// Timers of "my_service" and "MPU" can't be Stoped/Started in this function.
						// Other Timers can be Stopped when DisconnectEvent happens and Started when ConnectEvent.
				
						// SAAR CODE:
						// my_service
						//err_code = app_timer_start(m_my_char_timer_id, TIMER_INTERVAL_MY_CHAR, NULL);
						//APP_ERROR_CHECK(err_code);

						// SAAR CODE:
						// MPU
						#ifdef MPU
								if( flag_MPU_timer_has_been_stopped ) {
										//err_code = app_timer_start(m_MPU_timer_id, TIMER_INTERVAL_MPU, NULL);
										//APP_ERROR_CHECK(err_code);
								}
						#endif
						
						// SAAR CODE:
						// PWM
						#ifdef PWM
								if( flag_PWM_timer_has_been_stopped ) {
										app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_1_pwm_pin, motor_2_pwm_pin);
										app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_3_pwm_pin, motor_4_pwm_pin);
										pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
										pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
										pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
										pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

										 //Initialize and enable PWM 1. 
										err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm1_ready_callback);
										APP_ERROR_CHECK(err_code);

										 //Initialize and enable PWM 2. 
										err_code = app_pwm_init(&PWM2, &pwm2_cfg, pwm2_ready_callback);
										APP_ERROR_CHECK(err_code);
									
										app_pwm_enable(&PWM1);
										app_pwm_enable(&PWM2);
									
										err_code = app_timer_start(m_PWM_timer_id, TIMER_INTERVAL_PWM, NULL);
										APP_ERROR_CHECK(err_code);
								}	else {
										if(flag_first_connection) {
												err_code = app_timer_start(m_PWM_timer_id, TIMER_INTERVAL_PWM, NULL);
												APP_ERROR_CHECK(err_code);
										}
								}
						#endif						

						// SAAR CODE:
						// BMP
						#ifdef BMP180_API  
								if( flag_BMP_timer_has_been_stopped ) {
										err_code = app_timer_start(m_BMP180_timer_id, TIMER_INTERVAL_BMP180, NULL);
										APP_ERROR_CHECK(err_code);				
								}
						#endif
								
						// SAAR CODE:
						// BME
						#ifdef BME280_API  
								if( flag_BME_timer_has_been_stopped ) {
										err_code = app_timer_start(m_BME280_timer_id, TIMER_INTERVAL_BME280, NULL);
										APP_ERROR_CHECK(err_code);				
								}
						#endif
								
						// SAAR CODE:
						// ADC
						#ifdef ADC
								if( flag_ADC_timer_has_been_stopped ) {
										err_code = app_timer_start(m_ADC_timer_id, TIMER_INTERVAL_ADC, NULL);
										APP_ERROR_CHECK(err_code);
								}
						#endif
								
						// SAAR CODE:
						// GPIOs
						#ifdef GPIOs
								if( flag_GPIOs_timer_has_been_stopped ) {
										//err_code = app_timer_start(m_GPIOs_timer_id, TIMER_INTERVAL_GPIOs, NULL);
										//APP_ERROR_CHECK(err_code);
								}
						#endif
									
						flag_first_connection = 0;				
						break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				
						// SAAR CODE
						LOG("When Disconnect", __LINE__);
																
						// SAAR CODE:
						// my_service
						// app_timer_stop(m_my_char_timer_id);

						// SAAR CODE:
						// MPU
						#ifdef MPU
								//app_timer_stop(m_MPU_timer_id);
								//flag_MPU_timer_has_been_stopped = 1;
						#endif
				
						// SAAR CODE:
						// PWM
						#ifdef PWM
								app_timer_stop(m_PWM_timer_id);
								flag_PWM_timer_has_been_stopped = 1;
				
								app_pwm_disable(&PWM1);
								APP_ERROR_CHECK(app_pwm_uninit(&PWM1));
								app_pwm_disable(&PWM2);
								APP_ERROR_CHECK(app_pwm_uninit(&PWM2));
						#endif
				
						// SAAR CODE:
						// BMP
						#ifdef BMP180_API  
								app_timer_stop(m_BMP180_timer_id);				
								flag_BMP_timer_has_been_stopped = 1;
						#endif

						// SAAR CODE:
						// BME
						#ifdef BME280_API  
								app_timer_stop(m_BME280_timer_id);				
								flag_BME_timer_has_been_stopped = 1;
						#endif
				
						// SAAR CODE:
						// ADC
						#ifdef ADC
								app_timer_stop(m_ADC_timer_id);
								flag_ADC_timer_has_been_stopped = 1;
						#endif
						
						// SAAR CODE:
						// GPIOs
						#ifdef GPIOs
								//app_timer_stop(m_GPIOs_timer_id);
								//flag_GPIOs_timer_has_been_stopped = 1;
						#endif						
						
						
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
	
    // Step 3.C, Call ble_my_service_on_ble_evt() to do housekeeping of ble connections related to our service and characteristic
		ble_my_service_on_ble_evt(&my_service, p_ble_evt);

}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
			uint32_t err_code;
    
//    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
/*
			nrf_clock_lf_cfg_t clock_lf_cfg = {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
																				 .rc_ctiv       = 0,                                \
																				 .rc_temp_ctiv  = 0,                                \
																				 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM};
*/
	
			nrf_clock_lf_cfg_t clock_lf_cfg = {.source        = NRF_CLOCK_LF_SRC_RC,            
																				 .rc_ctiv       = 1,                                
																				 .rc_temp_ctiv  = 1,                                
																				 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM};

																				 
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
		
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

		// SAAR CODE:
		// my_service
		// Declare a variable holding our service uuid
		ble_uuid_t m_adv_uuids[] = {BLE_UUID_MY_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN};
		// Declare and instantiate the scan response
		ble_advdata_t srdata;
		memset(&srdata, 0, sizeof(srdata));
		// Add the UUID to the scan response packet
		srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
		srdata.uuids_complete.p_uuids = m_adv_uuids;
		
		// SAAR CODE:
		// my_service
		// Include scan response packet in advertising
		//err_code = ble_advertising_init(&advdata, &srdata, &options, on_adv_evt, NULL);
    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
		uint32_t err_code;
    bool erase_bonds;

		// ***********************************
		// SAAR CODE:
		// PWM
		#ifdef PWM
				/* 2-channel PWM, 200Hz, output on DK LED pins. */
				app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_1_pwm_pin, motor_2_pwm_pin);
				app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, motor_3_pwm_pin, motor_4_pwm_pin);
				pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
				pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
				pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
				pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

				 //Initialize and enable PWM 1. 
				err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm1_ready_callback);
				APP_ERROR_CHECK(err_code);

				 //Initialize and enable PWM 2. 
				err_code = app_pwm_init(&PWM2, &pwm2_cfg, pwm2_ready_callback);
				APP_ERROR_CHECK(err_code);

		#endif


    // Initialize.
    timers_init();
    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();		
		
		// ***********************************
		// SAAR CODE:
		// PWM
		#ifdef PWM
				app_pwm_enable(&PWM1);
				app_pwm_enable(&PWM2);
		#endif
		
		// ***********************************
		// SAAR CODE:
		// I2C
		int ret;
		ret = i2c_Init();
		#ifdef SAAR_DEBUG
			LOG("i2c_Init", ret);
		#endif	

		// ***********************************
		// SAAR CODE:
		// MPU
		#ifdef MPU
				(void)mympu_open(200);				
		#endif

		// ***********************************
		// SAAR CODE:
		// BMP
		#ifdef BMP180_API  
				ret = bmp180_open();
				LOG("bmp180_open", ret);
		#endif
		
		// ***********************************
		// SAAR CODE:
		// BME
		#ifdef BME280_API  
				ret = bme280_open();			
				LOG("bme280_open", ret);
		#endif
		
		// ***********************************
		// SAAR CODE:
		// ADC
		#ifdef ADC		
		    adc_config();
				APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE));				
		#endif

		// ***********************************
		// SAAR CODE:
		// GPIOs
		#ifdef GPIOs
				#ifdef PCB6_v5
						nrf_gpio_pin_dir_set(Driver_1_Fault_pin, 	NRF_GPIO_PIN_DIR_INPUT);
						nrf_gpio_pin_dir_set(Driver_2_Fault_pin, 	NRF_GPIO_PIN_DIR_INPUT);
				#elif defined PCB7		
						nrf_gpio_pin_dir_set(Driver_1_Fault_pin, 	NRF_GPIO_PIN_DIR_INPUT);
						nrf_gpio_pin_dir_set(Driver_2_Fault_pin, 	NRF_GPIO_PIN_DIR_INPUT);
				#endif

				nrf_gpio_pin_dir_set(Driver_1_Sleep_pin, 	NRF_GPIO_PIN_DIR_OUTPUT);
				nrf_gpio_pin_dir_set(Driver_2_Sleep_pin, 	NRF_GPIO_PIN_DIR_OUTPUT);
				nrf_gpio_pin_dir_set(LED_power_pin, 			NRF_GPIO_PIN_DIR_OUTPUT);
				nrf_gpio_pin_dir_set(LED_1_pin, 					NRF_GPIO_PIN_DIR_OUTPUT);
				nrf_gpio_pin_dir_set(LED_2_pin, 					NRF_GPIO_PIN_DIR_OUTPUT);
				nrf_gpio_pin_dir_set(LED_3_pin, 					NRF_GPIO_PIN_DIR_OUTPUT);
				nrf_gpio_pin_dir_set(LED_4_pin, 					NRF_GPIO_PIN_DIR_OUTPUT);


				nrf_gpio_pin_set(Driver_1_Sleep_pin);	// nrf_gpio_pin_clear(Driver_1_Sleep_pin);
				nrf_gpio_pin_set(Driver_2_Sleep_pin);	// nrf_gpio_pin_clear(Driver_2_Sleep_pin);

				nrf_gpio_pin_set(LED_power_pin);			// nrf_gpio_pin_clear(LED_power_pin);
				nrf_gpio_pin_set(LED_1_pin);					// nrf_gpio_pin_clear(LED_1_pin);
				nrf_gpio_pin_set(LED_2_pin);					// nrf_gpio_pin_clear(LED_2_pin);
				nrf_gpio_pin_set(LED_3_pin);					// nrf_gpio_pin_clear(LED_3_pin);
				nrf_gpio_pin_set(LED_4_pin);					// nrf_gpio_pin_clear(LED_4_pin);
		#endif

		// ***********************************	
    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
			
        power_manage();

    }
			
}

/**
 * @}
 */
