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
#include "es_math.h"

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

void ahrs_gyro_reset(vector_3f_t* const gyro_angles, const vector_3f_t* const gyro_init)
{
	memcpy(gyro_angles, gyro_init, sizeof(vector_3f_t));
}

void ahrs_gyro_loop(const vector_3f_t* const gyro, const float dt_s,
		vector_3f_t* const angles)
{
	for (uint8_t i = 0; i < VECTOR_3F_N_ELEMENTS; ++i) {
		angles->v[i] += gyro->v[i] * dt_s;
	}
}

void ahrs_complementary_filter(const ahrs_attitude_t* const accel_attitude,
		const vector_3f_t* const gyro, vector_3f_t* const attitude, const float alpha,
		const float dt_s)
{
	attitude->x = alpha * (attitude->x + gyro->x * dt_s)
			+ (1.0f - alpha) * accel_attitude->roll;
	attitude->y = alpha * (attitude->y + gyro->y * dt_s)
			+ (1.0f - alpha) * accel_attitude->pitch;
	attitude->z += gyro->z * dt_s;
}


static const vector_3f_t gravity_world =
{
    .x = 0.0f,
    .y = 0.0f,
    .z = 1.0f
};


void ahrs_init(ahrs_t* const ahrs, const vector_3f_t* const accel,
    const float kp)
{
    const vector_3f_t a = vector_3f_normalize(accel);
    const vector_3f_t euler = {
    	.x = atan2f(a.y, a.z),
		.y = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z)),
		.z = 0
    };
    ahrs->orientation = quaternion_from_euler(&euler);
    ahrs->kp = kp;
}


void ahrs_update(ahrs_t* const ahrs, const vector_3f_t* const gyro_radps,
    const vector_3f_t* const accel, const float dt_s)
{
    const vector_3f_t accel_normalized = vector_3f_normalize(accel);
    const quaternion_t q_conjugate = quaternion_conjugate(&ahrs->orientation);

    ahrs->gravity_estimated =
    		quaternion_rotate_vector(&q_conjugate, &gravity_world);

    ahrs->error =
    		vector_3f_cross(&accel_normalized, &ahrs->gravity_estimated);
    ahrs->gravity_error_deg = rad_to_deg(acosf(vector_3f_dot(&accel_normalized, &ahrs->gravity_estimated)));
    ahrs->error_norm = vector_3f_norm(&ahrs->error);
    const vector_3f_t correction = vector_3f_scale(&ahrs->error, ahrs->kp);
    const vector_3f_t gyro_corrected = vector_3f_add(gyro_radps, &correction);
    quaternion_integrate(&ahrs->orientation, &gyro_corrected, dt_s);
}
