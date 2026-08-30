/**
  ******************************************************************************
  * @file    vector_3f.h
  * @author  Oleg Gordiushenkov
  * @date    2026-Aug-29
  * @brief   Description
  ******************************************************************************
  */
#ifndef VECTOR_3F_H_
#define VECTOR_3F_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported includes ---------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
 enum {
 	VECTOR_INDEX_X = 0,
 	VECTOR_INDEX_Y = 1,
 	VECTOR_INDEX_Z = 2,
 	VECTOR_3F_N_ELEMENTS = 3,
 };

 typedef union {
 	struct {
 		float x;
 		float y;
 		float z;
 	};
 	float v[VECTOR_3F_N_ELEMENTS];
 } vector_3f_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* VECTOR_3F_H_ */
