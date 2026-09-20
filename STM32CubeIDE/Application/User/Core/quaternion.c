/**
  ******************************************************************************
  * @file    quaternion.c
  * @author  Oleg Gordiushenkov
  * @date    2026-Sep-19
  * @brief   Description
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "quaternion.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
quaternion_t quaternion_identity(void)
{
    return (quaternion_t) {
        .w = 1.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f
    };
}


float quaternion_norm(const quaternion_t *q)
{
    return sqrtf(
        q->w * q->w +
        q->x * q->x +
        q->y * q->y +
        q->z * q->z);
}


void quaternion_normalize(quaternion_t *q)
{
    const float norm = quaternion_norm(q);

    if (norm > 0.0f) {
        const float inv_norm = 1.0f / norm;

        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

quaternion_t quaternion_conjugate(const quaternion_t* const q)
{
	return (quaternion_t) {
		.w = q->w,
		.x = -q->x,
		.y = -q->y,
		.z = -q->z
	};
}

quaternion_t quaternion_multiply(const quaternion_t* const q1,
								 const quaternion_t* const q2)
{
	return (quaternion_t) {
		.w =  q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z,
		.x =  q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y,
		.y =  q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x,
		.z =  q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w
	};
}

vector_3f_t quaternion_rotate_vector(const quaternion_t* const q,
    const vector_3f_t* const v)
{
    const float tx = 2.0f * (q->y * v->z - q->z * v->y);
    const float ty = 2.0f * (q->z * v->x - q->x * v->z);
    const float tz = 2.0f * (q->x * v->y - q->y * v->x);

    return (vector_3f_t) {
		.x = v->x + q->w * tx + q->y * tz - q->z * ty,
		.y = v->y + q->w * ty + q->z * tx - q->x * tz,
		.z = v->z + q->w * tz + q->x * ty - q->y * tx
    };
}

void quaternion_integrate(quaternion_t* const q,
		const vector_3f_t* const angular_velocity, const float dt)
{
    const float half_dt = 0.5f * dt;

    quaternion_t dq;

    dq.w = -(
        q->x * angular_velocity->x +
        q->y * angular_velocity->y +
        q->z * angular_velocity->z) * half_dt;

    dq.x = (
        q->w * angular_velocity->x +
        q->y * angular_velocity->z -
        q->z * angular_velocity->y) * half_dt;

    dq.y = (
        q->w * angular_velocity->y +
        q->z * angular_velocity->x -
        q->x * angular_velocity->z) * half_dt;

    dq.z = (
        q->w * angular_velocity->z +
        q->x * angular_velocity->y -
        q->y * angular_velocity->x) * half_dt;

    q->w += dq.w;
    q->x += dq.x;
    q->y += dq.y;
    q->z += dq.z;

    quaternion_normalize(q);
}

vector_3f_t quaternion_to_euler(const quaternion_t* const q)
{
    float sin_pitch =
        2.0f * (q->w * q->y - q->z * q->x);

    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    return (vector_3f_t) {
		.x = atan2f(2.0f * (q->w * q->x + q->y * q->z),
					1.0f - 2.0f * (q->x * q->x + q->y * q->y)),
		.y = asinf(sin_pitch),
		.z = atan2f(2.0f * (q->w * q->z + q->x * q->y),
					1.0f - 2.0f * (q->y * q->y + q->z * q->z))
    };
}

quaternion_t quaternion_from_euler(const vector_3f_t* const euler_rad)
{
    const float half_roll = 0.5f * euler_rad->x;
    const float half_pitch = 0.5f * euler_rad->y;
    const float half_yaw = 0.5f * euler_rad->z;

    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);

    return (quaternion_t) {
        .w = cr * cp * cy + sr * sp * sy,
        .x = sr * cp * cy - cr * sp * sy,
        .y = cr * sp * cy + sr * cp * sy,
        .z = cr * cp * sy - sr * sp * cy
    };
}

