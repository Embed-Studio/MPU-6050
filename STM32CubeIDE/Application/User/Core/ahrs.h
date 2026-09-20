/**
  ******************************************************************************
  * @file    ahrs.h
  * @author  Oleg Gordiushenkov
  * @date    2026-Aug-29
  * @brief   Description
  ******************************************************************************
  */
#ifndef AHRS_H_
#define AHRS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported includes ---------------------------------------------------------*/
#include "vector_3f.h"
#include "quaternion.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float roll;
    float pitch;
} ahrs_attitude_t;

typedef struct {
    quaternion_t orientation;
    float 		kp;
    vector_3f_t gravity_estimated;
    vector_3f_t error;
    float		error_norm;
    float 		gravity_error_deg;
} ahrs_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief Provides estimation of the attitude from the accelerometer measurements
 */
void ahrs_estimate_attitude(const vector_3f_t* const acceleration,
		ahrs_attitude_t* const attitude);

/**
 * @brief Provides the regularized estimation of the attitude from the accelerometer
 * measurements. Higher mu values increase the roll stability but increase the error
 */
void ahrs_estimate_regularized_attitude(const vector_3f_t* const acceleration,
		ahrs_attitude_t* const attitude, const float mu);

/**
 * @brief Resets the orientation angles in gyro_angles to the value provided by gyro_init
 */
void ahrs_gyro_reset(vector_3f_t* const gyro, const vector_3f_t* const gyro_init);

/**
 * @brief Updates the orientation angles by integrating the angular velocities for each axis
 * This implementation is not correct and is build for demonstration of how the gyro works
 */
void ahrs_gyro_loop(const vector_3f_t* const gyro, const float dt_s,
		vector_3f_t* const angles);

/**
 * @brief Update Euler-angle attitude using complementary filter.
 * attitude.x = roll
 * attitude.y = pitch
 * attitude.z = yaw
 * Note: accelerometer attitude and gyro must have consistent units (both hold
 * values in radians or degrees)
 */
void ahrs_complementary_filter(const ahrs_attitude_t* const accel_attitude,
		const vector_3f_t* const gyro, vector_3f_t* const attitude, const float alpha,
		const float dt_s);

/**
 * @brief Initializes the AHRS state.
 * The initial orientation is determined from the accelerometer,
 * assuming that the sensor is stationary.
 * @param ahrs AHRS state.
 * @param accel Accelerometer measurement.
 * @param kp Accelerometer influence
 */
void ahrs_init(ahrs_t* const ahrs, const vector_3f_t* const accel,
    const float kp);

/**
 * @brief Updates the AHRS orientation estimate.
 * The gyroscope provides the short-term orientation propagation,
 * while the accelerometer provides a gravity reference for correction.
 * @param ahrs AHRS state.
 * @param gyro Angular velocity [rad/s].
 * @param accel Accelerometer measurement.
 * @param dt Integration interval [s].
 */
void ahrs_update(ahrs_t* const ahrs, const vector_3f_t* const gyro_radps,
    const vector_3f_t* const accel, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H_ */
