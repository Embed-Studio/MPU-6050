/**
  ******************************************************************************
  * @file    vector_3f.c
  * @author  Oleg Gordiushenkov
  * @date    2026-Sep-19
  * @brief   Description
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "vector_3f.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
vector_3f_t vector_3f_zero(void)
{
    return (vector_3f_t) {
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f
    };
}

float vector_3f_norm(const vector_3f_t* const v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

vector_3f_t vector_3f_normalize(const vector_3f_t* const v)
{
    const float norm = vector_3f_norm(v);

    if (norm > 0.0f) {
        const float inv_norm = 1.0f / norm;

        return (vector_3f_t) {
            .x = v->x * inv_norm,
            .y = v->y * inv_norm,
            .z = v->z * inv_norm
        };
    }

    return vector_3f_zero();
}

float vector_3f_dot(const vector_3f_t* const a, const vector_3f_t* const b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

vector_3f_t vector_3f_cross(const vector_3f_t* const a, const vector_3f_t* const b)
{
    return (vector_3f_t) {
        .x = a->y * b->z - a->z * b->y,
        .y = a->z * b->x - a->x * b->z,
        .z = a->x * b->y - a->y * b->x
    };
}

vector_3f_t vector_3f_add(const vector_3f_t* const a, const vector_3f_t* const b)
{
    return (vector_3f_t) {
        .x = a->x + b->x,
        .y = a->y + b->y,
        .z = a->z + b->z
    };
}

vector_3f_t vector_3f_sub(const vector_3f_t* const a, const vector_3f_t* const b)
{
    return (vector_3f_t) {
        .x = a->x - b->x,
        .y = a->y - b->y,
        .z = a->z - b->z
    };
}

vector_3f_t vector_3f_scale(const vector_3f_t* const v, float scalar)
{
    return (vector_3f_t) {
        .x = v->x * scalar,
        .y = v->y * scalar,
        .z = v->z * scalar
    };
}
