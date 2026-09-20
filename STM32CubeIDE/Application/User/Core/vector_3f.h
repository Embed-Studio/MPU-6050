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
/**
 * @brief Returns the zero vector.
 */
vector_3f_t vector_3f_zero(void);

/**
 * @brief Calculates the Euclidean norm of a vector.
 */
float vector_3f_norm(const vector_3f_t* const v);

/**
 * @brief Returns a normalized copy of a vector.
 * If the vector has zero magnitude, the zero vector is returned.
 */
vector_3f_t vector_3f_normalize(const vector_3f_t* const v);

/**
 * @brief Calculates the dot product of two vectors.
 */
float vector_3f_dot(const vector_3f_t* const a, const vector_3f_t* const b);

/**
 * @brief Calculates the cross product of two vectors.
 */
vector_3f_t vector_3f_cross(const vector_3f_t* const a, const vector_3f_t* const b);

/**
 * @brief Adds two vectors.
 */
vector_3f_t vector_3f_add(const vector_3f_t* const a, const vector_3f_t* const b);

/**
 * @brief Subtracts one vector from another.
 */
vector_3f_t vector_3f_sub(const vector_3f_t* const a, const vector_3f_t* const b);

/**
 * @brief Multiplies a vector by a scalar.
 */
vector_3f_t vector_3f_scale(const vector_3f_t* const v, float scalar);

#ifdef __cplusplus
}
#endif

#endif /* VECTOR_3F_H_ */
