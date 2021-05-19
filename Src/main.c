/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "MPU6050_Add.h"
#include "I2C_Com.h"
#include "OtherSensors_Add.h"
#include "Sensor_Func.h"
#include "FS_IA6B.h"
#include "PID.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

uint16_t tele_time = 0;
uint32_t time = 0, loop = 0;

__IO uint32_t ConState = 4;
float PI = 3.141593;
uint8_t start = 0, check = 0;
int16_t throttle = 0, takeoff_throttle = 0;
int16_t esc_1 = 0, esc_2 = 0, esc_3 = 0, esc_4 = 0;
uint8_t flight_mode = 0, takeoff_detected = 0;
uint16_t manual_takeoff_speed = 1500;    	 //Enter the manual hover point when auto take-off detection is not desired (between 1400 and 1600).
int16_t motor_idle_speed = 1100;             //Enter the minimum throttle pulse of the motors when they idle (between 1000 and 1200). 1170 for DJI
int16_t manual_throttle = 0;


//MPU6050 & HMC5883L variables
float dt = 0.004f;
RawData_Def myAccelRaw, myGyroRaw, myMagRaw;
ScaledData_Def Manual_gyro_cal_raw_value, Manual_accel_cal_raw_value;
ScaledData_Def myGyroScaled, myAccelScaled, myMagScaled;
float gyro_roll_cal = -28.699028f;
float gyro_pitch_cal = -3.94793987f;
float gyro_yaw_cal = 0.745403409f;
float accel_roll_cal = 0.0f;
float accel_pitch_cal = 0.0f;
float accel_yaw_cal = 0.0f;
float accel_roll_cal_1 = 15.0320644f;
float accel_pitch_cal_1 = -35.7130089f;
float accel_yaw_cal_1 = 4075.18262f;
float Xh_offset = 437.0f, Yh_offset = 19.0f;

int16_t temperature = 0;
float Total_acc = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; //Angles in the complementary filter
float rollangle_acc = 0.0f, pitchangle_acc = 0.0f, rollangle_gyro = 0.0f, pitchangle_gyro = 0.0f;
float roll_level_adjust = 0.0f, pitch_level_adjust = 0.0f;
float Xh = 0.0f, Yh = 0.0f, Mag_x_dampened = 0.0f, Mag_y_dampened = 0.0f, Heading = 0.0f;
float Mag_Pitch = 0.0f, Mag_Roll = 0.0f;
float declinationAngle = -0.65f, compass_gain_fact = 1.22f;
float course_lock_heading = 0.0f, heading_lock_course_deviation = 0.0f;
//***********************************//

//MS5611 Variables
uint8_t TempCount = 0;
uint8_t MSCount = 0;
uint16_t C[7];
uint32_t D1;
uint32_t D2;
int32_t dT, TEMP;
int64_t OFF, SENS;
uint8_t average_temperature_mem_location;
uint32_t raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

float actual_pressure = 0.0f; 
float ground_pressure = 0.0f, altutude_hold_pressure = 0.0f, return_to_home_decrease = 0.0f;
int32_t t = 0, p = 0;
//*****************************//

//Altitude PID variables
uint8_t manual_altitude_change = 0;
float pid_i_mem_altitude = 0.0f, pid_altitude_setpoint = 0.0f, pid_altitude_input = 0.0f, pid_output_altitude = 0.0f;
float pid_error_gain_altitude = 0.0f;
//*********************//

//PID variables
float pid_i_mem_roll = 0.0f, pid_last_roll_d_error = 0.0f;
float pid_i_mem_pitch = 0.0f, pid_last_pitch_d_error = 0.0f;
float pid_i_mem_yaw = 0.0f, pid_last_yaw_d_error = 0.0f;
float pid_roll_setpoint = 0.0f, pid_pitch_setpoint = 0.0f, pid_yaw_setpoint = 0.0f;
float gyro_roll_input = 0.0f, gyro_pitch_input = 0.0f, gyro_yaw_input = 0.0f;
float pid_output_roll = 0.0f, pid_output_pitch = 0.0f, pid_output_yaw = 0.0f;

float pid_error_temp_roll = 0.0f, pid_error_temp_pitch = 0.0f, pid_error_temp_yaw = 0.0f;

float pid_p_gain_roll = 1.65f;              //Gain setting for the pitch and roll P-controller (default = 1.5).
float pid_i_gain_roll = 0.003f;          	//Gain setting for the pitch and roll I-controller (default = 0.0).
float pid_d_gain_roll = 13.0f;              //Gain setting for the pitch and roll D-controller (default = 15.0).
int pid_max_roll = 400;                     //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = 1.65f;  						//Gain setting for the pitch P-controller. 1.5
float pid_i_gain_pitch = 0.003f;  			    //Gain setting for the pitch I-controller. 0.0
float pid_d_gain_pitch = 13.0f;  					  //Gain setting for the pitch D-controller. 15.0
int pid_max_pitch = 400;          				  //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 3.0f;                //Gain setting for the pitch P-controller (default = 4.0) 
float pid_i_gain_yaw = 0.02f;               //Gain setting for the pitch I-controller (default = 0.02)
float pid_d_gain_yaw = 0.0f;                //Gain setting for the pitch D-controller (default = 0.0)
int pid_max_yaw = 400;                      //Maximum output of the PID-controller (+/-).

float pid_p_gain_altitude = 1.3f;           //Gain setting for the altitude P-controller (default = 1.0).
float pid_i_gain_altitude = 0.3f;           //Gain setting for the altitude I-controller (default = 0.3).
float pid_d_gain_altitude = 0.6f;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                 //Maximum output of the PID-controller (+/-).
float pid_error_temp = 0.0f;
//**************************//

int32_t channel_1_start, channel_2_start, channel_3_start, channel_4_start, channel_5_start, channel_6_start;
uint8_t Is_First_Captured_1 = 0; 
uint8_t Is_First_Captured_2 = 0;
uint8_t Is_First_Captured_3 = 0;
uint8_t Is_First_Captured_4 = 0;
uint8_t Is_First_Captured_5 = 0;
uint8_t Is_First_Captured_6 = 0;
int32_t channel_1_end, channel_2_end, channel_3_end, channel_4_end, channel_5_end, channel_6_end;
int32_t channel_1 = 1500, channel_2 = 1500, channel_3 = 1000, channel_4 = 1500, channel_5 = 1000, channel_6 = 1000;
int32_t channel_1_base = 0, channel_2_base = 0, pid_roll_setpoint_base = 0, pid_pitch_setpoint_base = 0;
uint8_t heading_lock = 0;

void return_to_home(void) {
  if (flight_mode == 3) {
		if(actual_pressure == ground_pressure){
			start = 0;
		}
		else{
			pid_altitude_setpoint += 0.035f;
		}
  }
}

float course_deviation(float course_b, float course_c) {
  float base_course_mirrored, actual_course_mirrored;
	float course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}

void start_stop_takeoff(void) {
  if (channel_3 < 1050 && channel_4 < 1300)start = 1;                             //For starting the motors: throttle low and yaw left.
  if (start == 1 && channel_4 > 1450) {                        										//When yaw stick is back in the center position start the motors.
		throttle = motor_idle_speed;                                                  //Set the base throttle to the motor_idle_speed variable.
    pitch = pitchangle_acc;                                                 			//Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    roll = rollangle_acc;                                                   			//Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    ground_pressure = actual_pressure;                                            //Register the pressure at ground level for altitude calculations.
    course_lock_heading = yaw;                                               			//Set the current compass heading as the course lock heading.

		start = 2;                                                                    //Set the start variable to 2 to indicate that the quadcopter is started.
    takeoff_throttle = 0;                           															//Use the manual hover throttle.
    takeoff_detected = 1;                                                         //Set the auto take-off detection to 1, indicated that the quadcopter is flying.
    //Reset the PID controllers for a smooth take-off.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_output_roll = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_output_pitch = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    pid_output_yaw = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1700) {
    start = 0;                                                                     //Set the start variable to 0 to disable the motors.
    takeoff_detected = 0;																											  	 //Reset the auto take-off detection.
		throttle = 0;
		ground_pressure = 0;
	}

  if (takeoff_detected == 0 && start == 2) {                                       //When the quadcopter is started and no take-off is detected.
    if (channel_3 > 1480 && throttle < 1750) throttle++;                           //When the throttle is half way or higher, increase the throttle.
    if (throttle == 1750)check = 2;                                                //If take-off is not detected when the throttle has reached 1700: error = 6.
    if (channel_3 <= 1480) {                                                       //When the throttle is below the center stick position.
      if (throttle > motor_idle_speed)throttle--;                                  //Lower the throttle to the motor_idle_speed variable.
      //Reset the PID controllers for a smooth take-off.
      else {                                                                       //When the throttle is back at idle speed reset the PID controllers.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;
      }
    }
  }
}

//Get system time microsec
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}
//*************************************//
int main(void)
{ 
	HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	
	MPU_ConfigTypeDef myMpuConfig;
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	  
	//Configure Accel and Gyro parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_8g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_44A_42G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
	
	MPU6050_Config(&myMpuConfig);
  MPU6050_Set_SMPRT_DIV(0x04);
	HAL_Delay(10);
	
	//Configure Magnetometer
	HMC5883L_Config();
	
	myGyroRaw.x = 0;
	myGyroRaw.y = 0;
	myGyroRaw.z = 0;
	
	myAccelRaw.x = 0;
	myAccelRaw.y = 0;
	myAccelRaw.z = 0;
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	
	//Calibration
	if (channel_1 > 1700 && channel_2 < 1300)
	{
		HAL_Delay(2000);
		for (int cal_int = 0; cal_int < 2100 ; cal_int ++) 
			{ 				
				if(cal_int > 99)
				{
						if (cal_int % 25 == 0) HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);                //Change the led status every 25 readings to indicate calibration.					
						while(ConState % 8 == 0);
						MPU6050_Get_Gyro_RawData(&myGyroRaw);
						MPU6050_Get_Accel_RawData(&myAccelRaw);					
						//Take 2000 readings for calibration.					
						gyro_pitch_cal += myGyroRaw.y;                                                //Add roll value to gyro_roll_cal.
						gyro_roll_cal  += myGyroRaw.x;                                                //Add pitch value to gyro_pitch_cal.
						gyro_yaw_cal   += myGyroRaw.z;                                                //Add yaw value to gyro_yaw_cal.
					
						accel_pitch_cal_1 += (float)myAccelRaw.y;                                     //Add roll value to gyro_roll_cal.
						accel_roll_cal_1  += (float)myAccelRaw.x;                                     //Add pitch value to gyro_pitch_cal.
						accel_yaw_cal_1   += (float)myAccelRaw.z;                                     //Add yaw value to gyro_yaw_cal.							
						HAL_Delay(3);					
				}
				else
				{
					HAL_Delay(4);
				}
			}
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);                            //Set output PD12 LOW
		//Divide by 2000 to get the average gyro offset.
    gyro_roll_cal  /= 2000;                                                           //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal   /= 2000; 

		accel_roll_cal_1  /= 2000;                                                        //Divide the roll total by 2000.
    accel_pitch_cal_1 /= 2000;                                                        //Divide the pitch total by 2000.
    accel_yaw_cal_1   /= 2000;  					
  }     
	Manual_gyro_cal_raw_value.y = gyro_pitch_cal;                                     //Set the manual pitch calibration variable to the detected value.
  Manual_gyro_cal_raw_value.x = gyro_roll_cal;                                      //Set the manual roll calibration variable to the detected value.
  Manual_gyro_cal_raw_value.z = gyro_yaw_cal;																				//Set the manual yaw calibration variable to the detected value.

  Manual_accel_cal_raw_value.y = accel_pitch_cal_1;
  Manual_accel_cal_raw_value.x = accel_roll_cal_1;
	Manual_accel_cal_raw_value.z = accel_yaw_cal_1;
		
	float Calib_Total = sqrt(Manual_accel_cal_raw_value.y*Manual_accel_cal_raw_value.y + Manual_accel_cal_raw_value.z*Manual_accel_cal_raw_value.z + Manual_accel_cal_raw_value.x*Manual_accel_cal_raw_value.x);
	if(sqrt(Manual_accel_cal_raw_value.y*Manual_accel_cal_raw_value.y) <= Calib_Total)	
		accel_roll_cal = asin(Manual_accel_cal_raw_value.y/Calib_Total)*180.0f/PI;
	if(sqrt(Manual_accel_cal_raw_value.x*Manual_accel_cal_raw_value.x) <= Calib_Total)  
		accel_pitch_cal = asin(Manual_accel_cal_raw_value.x/Calib_Total)*180.0f/PI;
	
	//Get initial yaw
	HMC5883L_Get_Mag_RawData(&myMagRaw);
	Xh = myMagRaw.x - Xh_offset;
	Yh = myMagRaw.y - Yh_offset;
	if(Yh > 0) Heading = 90 - atan(Xh/Yh) * 180.0f/PI;	
	if(Yh < 0) Heading = 270 - atan(Xh/Yh) * 180.0f/PI;
	if(Yh == 0 && Xh < 0) Heading = 180.0f;
	if(Yh == 0 && Xh > 0) Heading = 0.0f;
	Heading += declinationAngle;
	Heading -= 90.0f;		
	if (Heading > 360.0f) Heading -= 360.0f;
	if (Heading < 0.0f) Heading += 360.0f;
	yaw = Heading;
	
	//Init MS5611	
	MS5611_Init();
	for(int i = 0; i<10; i++){
		MS5611WriteByte(0x50 + MS5611_OSR * 2);
		HAL_Delay(9);
		MS5611ReadLong(0x00);
	}
	actual_pressure = 0;                                          //Reset the pressure calculations.
  
	if (motor_idle_speed < 1000)motor_idle_speed = 1000;          //Limit the minimum idle motor speed to 1000us.
  if (motor_idle_speed > 1200)motor_idle_speed = 1200;          //Limit the maximum idle motor speed to 1200us.
	
while (1)
{	
		time = getCurrentMicros();
				
		heading_lock = 0;
		
		if(channel_5 < 1200) flight_mode = 1;                                           //In all other situations the flight mode is 1;
		
		if(channel_5 >= 1200)flight_mode = 2;                                           //If channel 5 is above 1200us the flight mode is 2

		if(channel_6 >= 1500)flight_mode = 3;
		
		return_to_home();
		MPU6050_Get_Gyro_RawData(&myGyroRaw);
		MPU6050_Get_Accel_RawData(&myAccelRaw);
		temperature = MPU6050_Get_Temp();
		HMC5883L_Get_Mag_RawData(&myMagRaw);

		if(MSCount == 0) {
			if(TempCount == 20){
				MS5611WriteByte(0x50 + MS5611_OSR * 2);
			}
			else{
				MS5611WriteByte(0x40 + MS5611_OSR * 2);
			}
			if(TempCount < 20) TempCount++;
			else TempCount = 0;
		}
		
		if(MSCount == 3) {
			if(TempCount == 0){
				raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
				raw_temperature_rotating_memory[average_temperature_mem_location] = MS5611ReadLong(0x00);
				raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
				average_temperature_mem_location++;
				if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
				D2 = raw_average_temperature_total / 5;
			}
			else{
				D1 = MS5611ReadLong(0x00);
				dT = D2 - (C[5] << 8);
				TEMP = 2000 + (((int64_t)dT * C[6])/pow(2, 23));
				OFF = ((int64_t)C[2]*pow(2, 16)) + (((int64_t)C[4] * (int64_t)dT)/pow(2, 7));
				SENS = ((int64_t)C[1]*pow(2, 15)) + (((int64_t)C[3] * (int64_t)dT )/pow(2, 8));

				p = ((((D1 * SENS)/pow(2, 21)) - OFF))/pow(2, 15);
				t = TEMP/100;

				pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
				pressure_rotating_mem[pressure_rotating_mem_location] = p;                                                //Calculate the new change between the actual pressure and the previous measurement.
				pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
				pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
				if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
				actual_pressure_fast = (float)pressure_total_avarage / 20.0f;                                             //Calculate the average pressure of the last 20 pressure readings.

				//To get better results we will use a complementary fillter that can be adjusted by the fast average.
				actual_pressure_slow = actual_pressure_slow * 0.985f + actual_pressure_fast * 0.015f;
				actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
				if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
				if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
				//If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
				if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0f;
				actual_pressure = actual_pressure_slow;
			}
			
			calculate_altitude_pid();
				//MS5611WriteByte(0x50 + MS5611_OSR * 2);
		}
		
//		if(MSCount == 4){
//			raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
//			raw_temperature_rotating_memory[average_temperature_mem_location] = MS5611ReadLong(0x00);
//			raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
//			average_temperature_mem_location++;
//			if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
//			D2 = raw_average_temperature_total / 5;
//			
//			dT = D2 - (C[5] << 8);
//			TEMP = 2000 + (((int64_t)dT * C[6])/pow(2, 23));
//			OFF = ((int64_t)C[2]*pow(2, 16)) + (((int64_t)C[4] * (int64_t)dT)/pow(2, 7));
//			SENS = ((int64_t)C[1]*pow(2, 15)) + (((int64_t)C[3] * (int64_t)dT )/pow(2, 8));

//			p = ((((D1 * SENS)/pow(2, 21)) - OFF))/pow(2, 15);
//			t = TEMP/100;

//			pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
//			pressure_rotating_mem[pressure_rotating_mem_location] = p;                                                //Calculate the new change between the actual pressure and the previous measurement.
//			pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
//			pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
//			if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
//			actual_pressure_fast = (float)pressure_total_avarage / 20.0f;                                             //Calculate the average pressure of the last 20 pressure readings.

//			//To get better results we will use a complementary fillter that can be adjusted by the fast average.
//			actual_pressure_slow = actual_pressure_slow * 0.985f + actual_pressure_fast * 0.015f;
//			actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
//			if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
//			if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
//			//If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
//			if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0f;
//			actual_pressure = actual_pressure_slow;
//			
//			calculate_altitude_pid();
//		}	
	
		myGyroScaled.x = (float)myGyroRaw.x - Manual_gyro_cal_raw_value.x;
		myGyroScaled.y = (float)myGyroRaw.y - Manual_gyro_cal_raw_value.y;
		myGyroScaled.z = (float)myGyroRaw.z - Manual_gyro_cal_raw_value.z;
		
		myGyroScaled.x = myGyroScaled.x/65.536f;
		myGyroScaled.y = myGyroScaled.y/65.536f;
		myGyroScaled.z = myGyroScaled.z/65.536f;
		
		myAccelScaled.x = (float)myAccelRaw.x/4096.0f;
		myAccelScaled.y = (float)myAccelRaw.y/4096.0f;
		myAccelScaled.z = (float)myAccelRaw.z/4096.0f;
				
		myMagScaled.x = (float)myMagRaw.x - Xh_offset;
		myMagScaled.y = (float)myMagRaw.y - Yh_offset;
		myMagScaled.z = (float)myMagRaw.z;
		
		gyro_roll_input  = (gyro_roll_input * 0.8f) + (myGyroScaled.x * 0.2f);    //Gyro pid input is deg/sec.
		gyro_pitch_input = (gyro_pitch_input * 0.8f) + (myGyroScaled.y * 0.2f);	  //Gyro pid input is deg/sec.
		gyro_yaw_input 	 = (gyro_yaw_input * 0.8f) + (myGyroScaled.z * 0.2f);     //Gyro pid input is deg/sec.
	
		//Calculate angles from gyrometer
		roll 	+= myGyroScaled.x*dt;
		pitch += myGyroScaled.y*dt;		
		yaw   += myGyroScaled.z*dt;                                        	 //Calculate the traveled yaw angle and add this to the angle_yaw variable.
		if (yaw < 0) yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
		else if (yaw >= 360) yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

		pitch -= roll  * sin(myGyroScaled.z*dt * PI/180.0f);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
		roll 	+= pitch * sin(myGyroScaled.z*dt * PI/180.0f);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
		
		//Calculate angles from accelerometer		
		Total_acc = sqrt(myAccelScaled.y*myAccelScaled.y + myAccelScaled.z*myAccelScaled.z + myAccelScaled.x*myAccelScaled.x);				
		if(sqrt(myAccelScaled.y*myAccelScaled.y) <= Total_acc)	
			rollangle_acc = asin(myAccelScaled.y/Total_acc)*180.0f/PI - accel_roll_cal;
		if(sqrt(myAccelScaled.x*myAccelScaled.x) <= Total_acc)  
			pitchangle_acc = asin(myAccelScaled.x/Total_acc)*180.0f/PI - accel_pitch_cal;
		
		//Complimentary filter			
		roll  = 0.9996f * roll  + 0.0004f * rollangle_acc;
		pitch = 0.9996f * pitch + 0.0004f * pitchangle_acc;
				
		Mag_Pitch = roll * PI/180.0f;
		Mag_Roll  = pitch * PI/180.0f;
		
		Yh = myMagScaled.x*cos(Mag_Roll) + myMagScaled.y*sin(Mag_Roll)*sin(Mag_Pitch) - myMagScaled.z*cos(Mag_Pitch)*sin(Mag_Roll);
		Xh = myMagScaled.y*cos(Mag_Pitch) + myMagScaled.z*sin(Mag_Pitch);
		
		Mag_x_dampened = Xh;
		Mag_y_dampened = Yh;
	  
		Mag_x_dampened = 0.9f*Mag_x_dampened + 0.1f*Xh;
		Mag_y_dampened = 0.9f*Mag_y_dampened + 0.1f*Yh;
		
		if(Yh > 0) Heading = 90 - atan(Mag_x_dampened/Mag_y_dampened) * 180.0f/PI;	
		if(Yh < 0) Heading = 270 - atan(Mag_x_dampened/Mag_y_dampened) * 180.0f/PI;
		if(Yh == 0 && Xh < 0) Heading = 180.0f;
		if(Yh == 0 && Xh > 0) Heading = 0.0f;
		
		Heading += declinationAngle;
		Heading -= 90.0f;
		//Heading -= 16.0f;
		
		if (Heading > 360.0f) Heading -= 360.0f;
		if (Heading < 0.0f) Heading += 360.0f;
		
		yaw -= course_deviation(yaw, Heading) / 1200.0f;       			//Calculate the difference between the gyro and compass heading and make a small correction.
		if (yaw < 0.0f) yaw += 360.0f;                              //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
		else if (yaw >= 360.0f) yaw -= 360.0f;                      //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
 			
		pitch_level_adjust = pitch * 15.0f;                                           //Calculate the pitch angle correction.
    roll_level_adjust = roll * 15.0f;                                             //Calculate the roll angle correction.
		
		channel_1_base = channel_1;                                                   //Normally channel_1 is the pid_roll_setpoint input.
		channel_2_base = channel_2;                                                   //Normally channel_2 is the pid_pitch_setpoint input.                                             
		
		//When the heading_lock mode is activated the roll and pitch pid setpoints are heading dependent.
		//At startup the heading is registerd in the variable course_lock_heading.
		//First the course deviation is calculated between the current heading and the course_lock_heading.
		//Based on this deviation the pitch and roll controls are calculated so the responce is the same as on startup.
		if (heading_lock == 1) {
			heading_lock_course_deviation = course_deviation(yaw, course_lock_heading);
			channel_1_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * PI/180.0f)) + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * PI/180.0f));
			channel_2_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * PI/180.0f)) + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * PI/180.0f));
		}
			
		pid_roll_setpoint_base = channel_1_base;
		pid_pitch_setpoint_base = channel_2_base;

		//Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
		if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
		if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
		if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
		if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;
		
		//The PID set point in degrees per second is determined by the roll receiver input.
		//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_roll_setpoint = 0.0f;
		
		//We need a little dead band of 16us for better results.
		if (pid_roll_setpoint_base > 1508)	pid_roll_setpoint = pid_roll_setpoint_base - 1508;
		else if (pid_roll_setpoint_base < 1492)	pid_roll_setpoint = pid_roll_setpoint_base - 1492;

		pid_roll_setpoint -= roll_level_adjust;                                           //Subtract the angle correction from the standardized receiver roll input value.
		pid_roll_setpoint /= 3.0f;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the pitch receiver input.
		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_pitch_setpoint = 0.0f;
		
		//We need a little dead band of 16us for better results.
		if (pid_pitch_setpoint_base > 1508)	pid_pitch_setpoint = pid_pitch_setpoint_base - 1508;
		else if (pid_pitch_setpoint_base < 1492)	pid_pitch_setpoint = pid_pitch_setpoint_base - 1492;

		pid_pitch_setpoint -= pitch_level_adjust;                                         //Subtract the angle correction from the standardized receiver pitch input value.
		pid_pitch_setpoint /= 3.0f;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the yaw receiver input.
		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_yaw_setpoint = 0.0f;
		
		if (channel_3 > 1050) 
		{ 
			//Do not yaw when turning off the motors.
			if (channel_4 > 1508)	pid_yaw_setpoint = (channel_4 - 1508) / 3.0f;
			else if (channel_4 < 1492)	pid_yaw_setpoint = (channel_4 - 1492) / 3.0f;
		}
		
		calculate_pid();
		start_stop_takeoff();
	
//**************************************************************************//
		if(start == 2) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);                             //Set output PD14 LOW
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}
		if(start == 0) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);                               //Set output PD14 LOW
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		
		//The variable base_throttle is calculated in the following part. It forms the base throttle for every motor.
		if (takeoff_detected == 1 && start == 2) {                                       //If the quadcopter is started and flying.
			throttle = channel_3 + takeoff_throttle;                                       //The base throttle is the receiver throttle channel + the detected take-off throttle.
			if (flight_mode >= 2) {                                                          //If altitude mode is active.
				throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;    //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
			}
		}                                                  

		if (start == 2) 
		{                                                                								 	//The motors are started.  
			if (throttle > 1800) throttle = 1800;                                          	//Limit full throttle.
			
				esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
				esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
				esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
				esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).
							
				if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
				if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
				if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
				if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

				if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
				if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
				if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
				if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
		}

		else 
		{
			esc_1 = 1000;                                                                  	 //If start is not 2 keep a 1000us pulse for ess-1.
			esc_2 = 1000;                                                                  	 //If start is not 2 keep a 1000us pulse for ess-2.
			esc_3 = 1000;                                                                  	 //If start is not 2 keep a 1000us pulse for ess-3.
			esc_4 = 1000;                                                                  	 //If start is not 2 keep a 1000us pulse for ess-4.
		}

		TIM3->CCR1 = esc_1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
		TIM3->CCR2 = esc_2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
		TIM3->CCR3 = esc_3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
		TIM3->CCR4 = esc_4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.
		
		//send_telemetry_data 
		uint16_t size = 0;
		char tele_buf = 0;
				
		if(tele_time == 0){			
			sprintf(&tele_buf, "E""%d\n", check);
			size = strlen(&tele_buf);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);
		}
		if(tele_time == 10){
			sprintf(&tele_buf, "F""%d\n", flight_mode);
			size = strlen(&tele_buf);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);
		}
		if(tele_time == 20){
			sprintf(&tele_buf, "R""%.2f\n", roll);
			size = strlen(&tele_buf);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);		
		}
		if(tele_time == 30){			
			sprintf(&tele_buf, "P""%.2f\n", pitch);
			size = strlen(&tele_buf);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);		
		}
		if(tele_time == 40){
			sprintf(&tele_buf, "Y""%.2f\n", yaw);
			size = strlen(&tele_buf);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);
		}
		if(tele_time == 50){
			sprintf(&tele_buf, "S""%d\n", start);
			size = strlen(&tele_buf);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);
		}
		if(tele_time == 60){
			if(flight_mode >= 2){
				sprintf(&tele_buf, "G""%.2f\n", pid_altitude_setpoint);
				size = strlen(&tele_buf);
				HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);
			}
		}
		if(tele_time == 70){
			sprintf(&tele_buf, "H""%.2f\n", actual_pressure);
			size = strlen(&tele_buf);
			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);
		}
//		if(tele_time == 35){
//			sprintf(&tele_buf, "H""%.2f\n", actual_pressure);
//			size = strlen(&tele_buf);
//			HAL_UART_Transmit(&huart2, (uint8_t *)&tele_buf, size, 1);
//		}
		
		if(tele_time > 70) tele_time = 0;
		else tele_time++;
		
		MSCount++;
		if(MSCount > 3) MSCount = 0;
		
		loop = getCurrentMicros() - time;
		if(loop > 4050) {
			check = 1;
		}
		//else check = 0;
		while(loop < 4000) loop = getCurrentMicros() - time;

	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
