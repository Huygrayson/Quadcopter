#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#ifndef FS_IA6B
#define FS_IA6B

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* FS_IA6B */
