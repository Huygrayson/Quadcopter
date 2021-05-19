#include "PID.h"
#include "stm32f4xx_hal.h"
#include <math.h>

extern float pid_p_gain_roll;               //Gain setting for the pitch and roll P-controller (default = 0.6).
extern float pid_i_gain_roll;               //Gain setting for the pitch and roll I-controller (default = 0.000005).
extern float pid_d_gain_roll;               //Gain setting for the pitch and roll D-controller (default = 100.0).
extern int pid_max_roll;                    //Maximum output of the PID-controller (+/-).

extern float pid_p_gain_pitch;  						//Gain setting for the pitch P-controller. 0.6
extern float pid_i_gain_pitch;  			      //Gain setting for the pitch I-controller. 0.000005
extern float pid_d_gain_pitch ;  					  //Gain setting for the pitch D-controller. 100.0
extern int pid_max_pitch ;          				//Maximum output of the PID-controller (+/-).

extern float pid_p_gain_yaw;                //Gain setting for the pitch P-controller (default = 4.0) 
extern float pid_i_gain_yaw;                //Gain setting for the pitch I-controller (default = 0.0005)
extern float pid_d_gain_yaw;                //Gain setting for the pitch D-controller (default = 0.0)
extern int pid_max_yaw;                     //Maximum output of the PID-controller (+/-).

extern float pid_error_temp_roll, pid_error_temp_pitch, pid_error_temp_yaw;
extern float pid_i_mem_roll, pid_last_roll_d_error;
extern float pid_i_mem_pitch, pid_last_pitch_d_error;
extern float pid_i_mem_yaw, pid_last_yaw_d_error;

extern float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
extern float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
extern float pid_output_roll, pid_output_pitch, pid_output_yaw;

//Altitude PID variables
extern int32_t channel_3;
extern uint8_t flight_mode, takeoff_detected;
extern int16_t manual_throttle;
extern float pid_error_gain_altitude;

extern float pid_p_gain_altitude;           //Gain setting for the altitude P-controller (default = 1.4).
extern float pid_i_gain_altitude;           //Gain setting for the altitude I-controller (default = 0.2).
extern float pid_d_gain_altitude;           //Gain setting for the altitude D-controller (default = 0.75).
extern int pid_max_altitude;                //Maximum output of the PID-controller (+/-).
extern float pid_error_temp;

extern uint8_t parachute_rotating_mem_location;
extern int32_t parachute_buffer[35], parachute_throttle;
extern float pressure_parachute_previous;
extern float actual_pressure;
extern uint8_t manual_altitude_change;
extern float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude;
//*********************//

void calculate_pid(void){
  //Roll calculations
  pid_error_temp_roll = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp_roll;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp_roll + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp_roll - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp_roll;

  //Pitch calculations
  pid_error_temp_pitch = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp_pitch;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp_pitch + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp_pitch - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp_pitch;

  //Yaw calculations
  pid_error_temp_yaw = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp_yaw;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp_yaw + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp_yaw - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp_yaw;
}

void calculate_altitude_pid(void){
		if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.
    
		if (flight_mode >= 2 && takeoff_detected == 1) {                                                          //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      if (channel_3 > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }
      if (channel_3 < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (sqrt(pid_error_temp*pid_error_temp) - 10) / 20.0f;//The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0f) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
     
			//To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
		}

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }
	}
