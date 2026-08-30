/**
  ******************************************************************************
  * @file    es_math.h
  * @author  Oleg Gordiushenkov
  * @date    2026-Aug-29
  * @brief   Description
  ******************************************************************************
  */
#ifndef ES_MATH_H_
#define ES_MATH_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported includes ---------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PI_F      (3.14159265358979323846f)
#define DEG2RAD_F (PI_F / 180.0f)
#define RAD2DEG_F (180.0f / PI_F)

static inline float deg_to_rad(float deg)
{
    return deg * DEG2RAD_F;
}

static inline float rad_to_deg(float rad)
{
    return rad * RAD2DEG_F;
}

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* ES_MATH_H_ */
