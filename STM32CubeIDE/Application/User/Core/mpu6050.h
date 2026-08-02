/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  oleg
  * @date    2026-Jul-20
  * @brief   Description
  ******************************************************************************
  */
#ifndef APPLICATION_USER_CORE_MPU6050_H_
#define APPLICATION_USER_CORE_MPU6050_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported includes ---------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* MPU-6050 Device Address */
#define MPU6050_I2C_ADDR         (0x68 << 1)

/* MPU-6050 Register Map */
#define MPU6050_REG_INT_STATUS   0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_ACCEL_CONFIG 0x1C

/* Expected Identity Value */
#define MPU6050_WHO_AM_I_VAL     0x68

/* PWR_MGMT_1 bit: triggers a full device reset, self-clearing */
#define MPU6050_DEVICE_RESET     0x80

/* Longest we wait for the sensor to answer after a cold power-up.
   The MPU-6050 needs time from VDD ramp before it accepts register access,
   and the STM32 boots far faster than that. */
#define MPU6050_STARTUP_TIMEOUT_MS 500

/* I2C bus pins, used only by MPU6050_Reset_I2C_Bus() to bit-bang the bus free.
   They must match the pins CubeMX assigned to the I2C peripheral — the defaults
   below are I2C1 on PB6/PB9.

   These are guarded, so they can be overridden without editing this driver: give
   the pins a user label in the .ioc (say MPU6050_SCL / MPU6050_SDA), regenerate,
   and CubeMX will emit MPU6050_SCL_Pin / MPU6050_SCL_GPIO_Port … into main.h.
   Then define them ahead of this header, or add -DMPU6050_SCL_Pin=… to the build.

   If SCL and SDA ever sit on different ports, enable both clocks in
   MPU6050_I2C_GPIO_CLK_ENABLE. */
#ifndef MPU6050_SCL_GPIO_Port
#define MPU6050_SCL_GPIO_Port    GPIOB
#endif
#ifndef MPU6050_SCL_Pin
#define MPU6050_SCL_Pin          GPIO_PIN_6
#endif
#ifndef MPU6050_SDA_GPIO_Port
#define MPU6050_SDA_GPIO_Port    GPIOB
#endif
#ifndef MPU6050_SDA_Pin
#define MPU6050_SDA_Pin          GPIO_PIN_9
#endif
#ifndef MPU6050_I2C_GPIO_CLK_ENABLE
#define MPU6050_I2C_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

/* ---------------------------------------------------------------------------
   Accelerometer calibration
   ---------------------------------------------------------------------------
   Model, one line per axis:

       acceleration[g] = (raw - bias) / scale

   Bias is in LSB, scale in LSB/g. Both come from the six-position method:
   point each axis up, then down, average each position, then

       bias  = (raw_up + raw_down) / 2
       scale = (raw_up - raw_down) / 2

   Both constants are compiled in below. Note what that costs on the bias: it
   was measured to about +/-3 LSB, but moved by up to 37 LSB (2.2 mg) across a
   single power cycle, so a stored offset describes the device as it was rather
   than as it is. The scale factors have no such problem — they reproduced to
   better than 0.03 %. If the offset error ever matters, the fix is to re-measure
   the bias at startup instead of trusting the value here.
   --------------------------------------------------------------------------- */

/* Datasheet sensitivity at AFS_SEL = 0 (+/-2 g). The measured scale factors are
   compared against this; at any other full-scale range it is a different number. */
#define MPU6050_ACCEL_NOMINAL_LSB_PER_G   16384.0f

/* Constants measured on ONE specific module, at a die temperature of
   27.8-29.8 degC. They are a property of that part, not of the MPU-6050:
   X and Y landed within 0.15 % of nominal while Z was out by 1.39 %.

   Measure your own. These defaults are guarded so the values can be supplied
   from the build (-DMPU6050_ACCEL_SCALE_X_DEFAULT=...) without editing the driver. */
#ifndef MPU6050_ACCEL_SCALE_X_DEFAULT
#define MPU6050_ACCEL_SCALE_X_DEFAULT     16407.75f
#endif
#ifndef MPU6050_ACCEL_SCALE_Y_DEFAULT
#define MPU6050_ACCEL_SCALE_Y_DEFAULT     16401.25f
#endif
#ifndef MPU6050_ACCEL_SCALE_Z_DEFAULT
#define MPU6050_ACCEL_SCALE_Z_DEFAULT     16611.20f
#endif

#ifndef MPU6050_ACCEL_BIAS_X_DEFAULT
#define MPU6050_ACCEL_BIAS_X_DEFAULT        197.35f
#endif
#ifndef MPU6050_ACCEL_BIAS_Y_DEFAULT
#define MPU6050_ACCEL_BIAS_Y_DEFAULT        -70.25f
#endif
#ifndef MPU6050_ACCEL_BIAS_Z_DEFAULT
#define MPU6050_ACCEL_BIAS_Z_DEFAULT        -37.50f
#endif

/* Data Structures */
typedef struct {
    int16_t 	accel_x;
    int16_t 	accel_y;
    int16_t 	accel_z;
    int16_t 	gyro_x;
    int16_t 	gyro_y;
    int16_t 	gyro_z;
    float 		temperature;
    uint16_t	sample_time_us;
} MPU6050_t;

/* Held in RAM rather than as file-scope constants so the values can be adjusted
   live over SWD — changing a constant stays an edit rather than a rebuild. */
typedef struct {
    float scale[3];   /* LSB/g, X Y Z */
    float bias[3];    /* LSB,   X Y Z */
} MPU6050_Calibration_t;

typedef struct {
    float x;          /* g */
    float y;          /* g */
    float z;          /* g */
} MPU6050_Accel_g_t;

/* Exported Functions */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data);
HAL_StatusTypeDef MPU6050_Clear_Interrupt(I2C_HandleTypeDef *hi2c);
void MPU6050_Reset_I2C_Bus(I2C_HandleTypeDef *hi2c);

/* Calibration */
void  MPU6050_Calibration_Init(MPU6050_Calibration_t *cal);
void  MPU6050_Apply_Calibration(const MPU6050_Calibration_t *cal,
                                const MPU6050_t *raw,
                                MPU6050_Accel_g_t *out);
float MPU6050_Gravity_Magnitude(const MPU6050_Accel_g_t *accel_g);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_CORE_MPU6050_H_ */
