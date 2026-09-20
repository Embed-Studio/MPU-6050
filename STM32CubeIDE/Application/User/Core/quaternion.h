/**
  ******************************************************************************
  * @file    quaternion.h
  * @author  Oleg Gordiushenkov
  * @date    2026-Sep-19
  * @brief   Description
  ******************************************************************************
  */
#ifndef QUATERNION_H_
#define QUATERNION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported includes ---------------------------------------------------------*/
#include "vector_3f.h"

/* Exported types ------------------------------------------------------------*/
enum {
    QUATERNION_INDEX_W = 0,
    QUATERNION_INDEX_X = 1,
    QUATERNION_INDEX_Y = 2,
    QUATERNION_INDEX_Z = 3,
    QUATERNION_N_ELEMENTS = 4,
};

typedef union {
    struct {
        float w;
        float x;
        float y;
        float z;
    };
    float v[QUATERNION_N_ELEMENTS];
} quaternion_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief Returns the identity quaternion.
 */
quaternion_t quaternion_identity(void);

/**
 * @brief Calculates the norm of a quaternion.
 */
float quaternion_norm(const quaternion_t* const q);

/**
 * @brief Normalizes a quaternion in place.
 */
void quaternion_normalize(quaternion_t* const q);

/**
 * @brief Calculates the conjugate of a quaternion.
 */
quaternion_t quaternion_conjugate(const quaternion_t* const q);

/**
 * @brief Multiplies two quaternions.
 * @return Product q1 * q2.
 */
quaternion_t quaternion_multiply(const quaternion_t* const q1,
    const quaternion_t* const q2);

/**
 * @brief Rotates a vector using a quaternion.
 * The quaternion is assumed to represent a unit rotation.
 * @param q Rotation quaternion.
 * @param v Vector to rotate.
 * @return Rotated vector.
 */
vector_3f_t quaternion_rotate_vector(const quaternion_t* const q,
    const vector_3f_t* const v);

/**
 * @brief Integrates quaternion orientation from angular velocity.
 * The angular velocity is expressed in radians per second and the
 * integration interval in seconds.
 * @param q Quaternion to update.
 * @param angular_velocity_radps Angular velocity vector [rad/s].
 * @param dt_s Integration interval [s].
 */
void quaternion_integrate(quaternion_t* const q,
		const vector_3f_t* const angular_velocity_radps, const float dt_s);

/**
 * @brief Converts a quaternion orientation to Euler angles.
 * The returned vector contains:
 *   x - roll  angle around the X axis [rad]
 *   y - pitch angle around the Y axis [rad]
 *   z - yaw   angle around the Z axis [rad]
 * The pitch calculation uses asinf(). Its argument is theoretically
 * limited to [-1, 1], but floating-point rounding can produce a value
 * slightly outside this range. The value is therefore clamped before
 * calling asinf().
 * @param q Quaternion representing the orientation.
 * @return Euler angles in radians as a vector_3f_t.
 */
vector_3f_t quaternion_to_euler(const quaternion_t* const q);

/**
 * @brief Converts Euler angles to a quaternion.
 * The input vector contains roll, pitch and yaw angles around the
 * X, Y and Z axes respectively. The angles are expressed in radians.
 * Uses conventional ZYX yaw-pitch-roll composition
 * @param euler Euler angles [rad].
 * @return Quaternion representing the same orientation.
 */
quaternion_t quaternion_from_euler(const vector_3f_t* const euler_rad);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_H_ */
