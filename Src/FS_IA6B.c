#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "FS_IA6B.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern int32_t channel_1_start, channel_2_start, channel_3_start, channel_4_start, channel_5_start, channel_6_start;
extern int32_t channel_1_end, channel_2_end, channel_3_end, channel_4_end, channel_5_end, channel_6_end;
extern int32_t channel_1, channel_2, channel_3, channel_4, channel_5, channel_6;

extern uint8_t Is_First_Captured_1; 
extern uint8_t Is_First_Captured_2;
extern uint8_t Is_First_Captured_3;
extern uint8_t Is_First_Captured_4;
extern uint8_t Is_First_Captured_5;
extern uint8_t Is_First_Captured_6;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
	{		
		// if the interrupt source is channel_1
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  
		{
			if (Is_First_Captured_1==0) // if the first value is not captured
			{
				channel_1_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
				Is_First_Captured_1 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured_1==1)   // if the first is already captured
			{
				channel_1_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
				//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
				
				channel_1 = channel_1_end - channel_1_start;
				if (channel_1 < 0)channel_1 += 0xFFFF;               //If the timer has rolled over a correction is needed.
				
				Is_First_Captured_1 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			}
		}
		
		// if the interrupt source is channel_2
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (Is_First_Captured_2==0) // if the first value is not captured
			{
				channel_2_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
				Is_First_Captured_2 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured_2==1)   // if the first is already captured
			{
				channel_2_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
				//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
				
				channel_2 = channel_2_end - channel_2_start;
				if (channel_2 < 0)channel_2 += 0xFFFF;               //If the timer has rolled over a correction is needed.
				
				Is_First_Captured_2 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

			}
		}
		
		// if the interrupt source is channel_3
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if (Is_First_Captured_3==0) // if the first value is not captured
			{
				channel_3_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
				Is_First_Captured_3 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured_3==1)   // if the first is already captured
			{
				channel_3_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
				//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				channel_3 = channel_3_end - channel_3_start;
				if (channel_3 < 0)channel_3 += 0xFFFF;               //If the timer has rolled over a correction is needed.
				
				Is_First_Captured_3 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

			}
		}
		
		// if the interrupt source is channel_4
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if (Is_First_Captured_4==0) // if the first value is not captured
			{
				channel_4_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
				Is_First_Captured_4 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured_4==1)   // if the first is already captured
			{
				channel_4_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value
				//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				channel_4 = channel_4_end - channel_4_start;
				if (channel_4 < 0)channel_4 += 0xFFFF;               //If the timer has rolled over a correction is needed.
				
				Is_First_Captured_4 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);

			}
		}
	}
	
	if(htim->Instance == htim4.Instance)
	{		
		// if the interrupt source is channel_5
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  
		{
			if (Is_First_Captured_5==0) // if the first value is not captured
			{
				channel_5_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
				Is_First_Captured_5 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured_5==1)   // if the first is already captured
			{
				channel_5_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
				//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				channel_5 = channel_5_end - channel_5_start;
				if (channel_5 < 0)channel_5 += 0xFFFF;               //If the timer has rolled over a correction is needed.
				
				Is_First_Captured_5 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

			}
		}
		
		// if the interrupt source is channel_6
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if (Is_First_Captured_6==0) // if the first value is not captured
			{
				channel_6_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
				Is_First_Captured_6 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured_6==1)   // if the first is already captured
			{
				channel_6_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value
				//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				channel_6 = channel_6_end - channel_6_start;
				if (channel_6 < 0)channel_6 += 0xFFFF;               //If the timer has rolled over a correction is needed.
				
				Is_First_Captured_6 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);

			}
		}
	}
}
