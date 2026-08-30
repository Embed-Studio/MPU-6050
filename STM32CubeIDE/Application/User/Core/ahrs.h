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

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float roll;
    float pitch;
} ahrs_attitude_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void ahrs_estimate_attitude(const vector_3f_t* const acceleration,
		ahrs_attitude_t* const attitude);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H_ */
