/**
  ******************************************************************************
  * @file    ahrs.c
  * @author  Oleg Gordiushenkov
  * @date    2026-Aug-29
  * @brief   Description
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "ahrs.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void ahrs_estimate_attitude(const vector_3f_t* const acceleration,
		ahrs_attitude_t* const attitude)
{
	attitude->roll = atan2(acceleration->y, acceleration->z);
	attitude->pitch = atan2(-acceleration->x,
			sqrtf(acceleration->y * acceleration->y + acceleration->z * acceleration->z));
}

void ahrs_estimate_regularized_attitude(const vector_3f_t* const acceleration,
		ahrs_attitude_t* const attitude, const float mu)
{
	attitude->roll = atan2(acceleration->y,
			copysignf(1.0f, acceleration->z) *
			sqrtf(mu * acceleration->x * acceleration->x + acceleration->z * acceleration->z));
	attitude->pitch = atan2(-acceleration->x,
			sqrtf(acceleration->y * acceleration->y + acceleration->z * acceleration->z));
}
