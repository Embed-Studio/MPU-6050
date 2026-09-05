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
#include <string.h>
#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
// Provides estimation of the attitude from the accelerometer measurements
void ahrs_estimate_attitude(const vector_3f_t* const acceleration,
		ahrs_attitude_t* const attitude)
{
	attitude->roll = atan2(acceleration->y, acceleration->z);
	attitude->pitch = atan2(-acceleration->x,
			sqrtf(acceleration->y * acceleration->y + acceleration->z * acceleration->z));
}

// Provides the regularized estimation of the attitude from the accelerometer
// measurements. Higher mu values increase the roll stability but increase the error
void ahrs_estimate_regularized_attitude(const vector_3f_t* const acceleration,
		ahrs_attitude_t* const attitude, const float mu)
{
	attitude->roll = atan2(acceleration->y,
			copysignf(1.0f, acceleration->z) *
			sqrtf(mu * acceleration->x * acceleration->x + acceleration->z * acceleration->z));
	attitude->pitch = atan2(-acceleration->x,
			sqrtf(acceleration->y * acceleration->y + acceleration->z * acceleration->z));
}

// Resets the orientation angles in gyro_angles to the value provided by gyro_init
void ahrs_gyro_reset(vector_3f_t* const gyro_angles, const vector_3f_t* const gyro_init)
{
	memcpy(gyro_angles, gyro_init, sizeof(vector_3f_t));
}

// Updates the orientation angles by integrating the angular velocities for each axis
// This implementation is not correct and is build for demonstration of how the gyro works
void ahrs_gyro_loop(const vector_3f_t* const gyro, const float dt_s,
		vector_3f_t* const angles)
{
	for (uint8_t i = 0; i < VECTOR_3F_N_ELEMENTS; ++i) {
		angles->v[i] += gyro->v[i] * dt_s;
	}
}
