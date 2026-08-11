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
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

/* Expected Identity Value */
#define MPU6050_WHO_AM_I_VAL     0x68

/* INT_STATUS bit: a new sample has been written to the data registers.
   Reading INT_STATUS clears every bit in it, which is the only thing that
   releases the INT pin while INT_RD_CLEAR (INT_PIN_CFG) stays at its reset 0.
   MPU6050_Read_All() therefore starts its burst here rather than at
   ACCEL_XOUT_H, and hands this byte back to the caller. */
#define MPU6050_INT_DATA_RDY     0x01

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

/* ---------------------------------------------------------------------------
   Gyroscope calibration
   ---------------------------------------------------------------------------
   rate[deg/s] = (raw - bias) / scale, one line per axis. Same shape as the
   accelerometer model above, but the two constants come from opposite places:

     - SCALE is compiled in. It is a property of the part.
     - BIAS is measured at every startup by MPU6050_Gyro_Zero_Update() and never
       stored. It is a property of the moment: on this part it moved 0.043 deg/s
       between power-ups, and 1 LSB of error costs 7.3 deg of heading after
       16 minutes.
   --------------------------------------------------------------------------- */

/* Datasheet sensitivity at FS_SEL = 0 (+/-250 deg/s), which MPU6050_Init()
   writes to GYRO_CONFIG. Every LSB-to-deg/s conversion downstream assumes it. */
#define MPU6050_GYRO_NOMINAL_LSB_PER_DPS  131.0f

/* Measured on ONE module at 27-29 degC. Only X deviates, by 0.9 %; Y and Z are
   within 0.1 % of nominal, and all three are inside the datasheet's +/-3 %.
   Which axis is off is not predictable, so measure your own. Guarded, so they
   can come from the build without editing the driver. */
#ifndef MPU6050_GYRO_SCALE_X_DEFAULT
#define MPU6050_GYRO_SCALE_X_DEFAULT      132.2f
#endif
#ifndef MPU6050_GYRO_SCALE_Y_DEFAULT
#define MPU6050_GYRO_SCALE_Y_DEFAULT      131.1f
#endif
#ifndef MPU6050_GYRO_SCALE_Z_DEFAULT
#define MPU6050_GYRO_SCALE_Z_DEFAULT      130.9f
#endif

/* Averaging window in samples: 2 s at 1 kHz. Shorter leaves too much noise on
   the constant (1 LSB is 7.3 deg after 16 min); past ~5 s nothing improves,
   because what is left is the bias moving rather than the estimate. */
#ifndef MPU6050_GYRO_ZERO_SAMPLES
#define MPU6050_GYRO_ZERO_SAMPLES         2000u
#endif

/* Stillness is re-checked every this many samples (0.2 s at 1 kHz) and the
   window banks a block only if that block passed, so the gate means
   "continuously still for 2 s". Judging a fixed window as a whole instead needs
   the board still for up to twice its length, by luck of alignment. */
#ifndef MPU6050_GYRO_ZERO_CHECK
#define MPU6050_GYRO_ZERO_CHECK           200u
#endif

/* Gate 1 — rate sd, vector norm, LSB. Catches rotation. A board on a table
   measures 8-9, one being held 40 and up. Without this test a startup average
   taken in the hand came out 0.79 deg/s wrong, 18x the drift being removed. */
#ifndef MPU6050_GYRO_ZERO_SD_MAX
#define MPU6050_GYRO_ZERO_SD_MAX          30.0f
#endif

/* Gate 2 — sd of |a|, g. Catches vibration, and the case the gyroscope is blind
   to: a board slid or carried at a constant attitude is moving without
   rotating. A resting board tops out at 0.0023 g. */
#ifndef MPU6050_GYRO_ZERO_ACCEL_SD_MAX
#define MPU6050_GYRO_ZERO_ACCEL_SD_MAX    0.005f
#endif

/* Gate 3 — |mean |a| - 1 g|. Catches sustained linear acceleration, which
   shifts a mean without adding variance to either test above. The tolerance
   must clear the accelerometer's own aging rather than just its noise: the
   compiled-in Z offset was measured moving 16 mg over four days, and that lands
   straight on |a|. */
#ifndef MPU6050_GYRO_ZERO_ACCEL_ERR_MAX
#define MPU6050_GYRO_ZERO_ACCEL_ERR_MAX   0.05f
#endif

/* Both window lengths are overridable and are counted in uint16_t fields, so
   state where an override stops working. Within these bounds every intermediate
   in the variance path stays in range. The banked count can overshoot by one
   block, since a block is folded in whole once it passes — hence the sum. */
_Static_assert(MPU6050_GYRO_ZERO_CHECK >= 2u,
               "gyro zero: a check block needs at least 2 samples for a variance");
_Static_assert(MPU6050_GYRO_ZERO_CHECK <= 65535u,
               "gyro zero: check block must fit MPU6050_GyroZero_t.blk_n (uint16_t)");
_Static_assert(MPU6050_GYRO_ZERO_SAMPLES + MPU6050_GYRO_ZERO_CHECK - 1u <= 65535u,
               "gyro zero: window + one check block must fit MPU6050_GyroZero_t.n (uint16_t)");

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
    /* Counts *sensor updates*: incremented only when the burst read came back
       with DATA_RDY set in INT_STATUS, so a host can tell a fresh sample from
       the same reading seen twice. It is incremented LAST, after every other
       field here has been written — so a host that sees this change is looking
       at a fully written sample.

       It travels inside this struct so it cannot drift away from the data it
       counts.

       16 bits, which at 1 kHz wraps every 65 s. That costs nothing for the job
       this does: the test is whether two adjacent samples differ, and a wrap
       (65535 -> 0) is a difference like any other, so it can never fake a
       duplicate. Only a cumulative count over a window longer than 65 s needs
       the wrap undone, which is a mod-65536 sum of the per-sample deltas
       offline. The narrower field also keeps this struct at 20 bytes and keeps
       two bytes per sample off the SWD link — worth having on a channel whose
       whole purpose is to observe how well acquisition keeps up. */
    uint16_t	sample_count;
} MPU6050_t;

/* Held in RAM rather than as file-scope constants so the values can be adjusted
   live over SWD — changing a constant stays an edit rather than a rebuild.

   The accel_ fields were called scale/bias before the gyroscope joined them; the
   prefix is what stops the two models being confused at a call site. */
typedef struct {
    float accel_scale[3];  /* LSB/g,       X Y Z — compiled in */
    float accel_bias[3];   /* LSB,         X Y Z — compiled in */
    float gyro_scale[3];   /* LSB/(deg/s), X Y Z — compiled in */
    float gyro_bias[3];    /* LSB,         X Y Z — MEASURED AT STARTUP, not
                              compiled in. Zero until the stillness gate has
                              accepted a window; see MPU6050_GyroZero_t. */
} MPU6050_Calibration_t;

typedef struct {
    float x;          /* g */
    float y;          /* g */
    float z;          /* g */
} MPU6050_Accel_g_t;

typedef struct {
    float x;          /* deg/s */
    float y;          /* deg/s */
    float z;          /* deg/s */
} MPU6050_Gyro_dps_t;

/* Startup-zeroing state machine. Every field after `ready` is there to be
   watched live over SWD — the point of a gate is that you can see it rejecting
   things, and `restarts` climbing while `sd_norm` sits at 200 is a much better
   diagnosis than a silently wrong constant. */
typedef struct {
    uint8_t  ready;       /* 0 while hunting, 1 once a zero has been accepted */
    uint16_t n;           /* samples of continuous stillness banked so far */
    uint16_t restarts;      /* partial windows thrown away */
    uint16_t blocks_failed; /* every failed block, including those failing before
                               there was any progress to lose. Both counters are
                               needed: restarts alone reads 0 when the board was
                               never still at all. Fits the padding hole the
                               struct already had, so it moved no other field. */

    int32_t  sum[3];        /* banked window; only the sum is needed, since
                               stillness is judged per check block */

    /* Current check block. Integer accumulators, so the variance comes out
       exactly from n*sum_sq - sum*sum; in float the sums of squares of raw
       counts would lose the digits the variance lives in. The magnitude
       accumulators track |a| - 1 g for the same reason. */
    uint16_t blk_n;
    int32_t  blk_sum[3];
    int64_t  blk_sum_sq[3];
    float    blk_acc_sum;
    float    blk_acc_sum_sq;

    /* The most recently completed check block — how still the board is right
       now. Updated every MPU6050_GYRO_ZERO_CHECK samples, pass or fail, and
       frozen once ready, because Update() then returns immediately. */
    float    sd[3];         /* per-axis rate sd, LSB */
    float    sd_norm;       /* vector norm of sd[], the gated quantity */
    float    acc_sd;        /* sd of |a|, g */
    float    acc_err;       /* mean |a| - 1, g */
} MPU6050_GyroZero_t;

/* Exported Functions */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data,
                                   uint8_t *int_status);
HAL_StatusTypeDef MPU6050_Clear_Interrupt(I2C_HandleTypeDef *hi2c);
void MPU6050_Reset_I2C_Bus(I2C_HandleTypeDef *hi2c);

/* Calibration */
void  MPU6050_Calibration_Init(MPU6050_Calibration_t *cal);
void  MPU6050_Apply_Calibration(const MPU6050_Calibration_t *cal,
                                const MPU6050_t *raw,
                                MPU6050_Accel_g_t *out);
float MPU6050_Gravity_Magnitude(const MPU6050_Accel_g_t *accel_g);

/* Gyroscope: startup zeroing behind a stillness gate, then the rate model */
void    MPU6050_Gyro_Zero_Init(MPU6050_GyroZero_t *z);
uint8_t MPU6050_Gyro_Zero_Update(MPU6050_GyroZero_t *z,
                                 const MPU6050_t *raw,
                                 float accel_magnitude_g,
                                 MPU6050_Calibration_t *cal);
void    MPU6050_Apply_Gyro_Calibration(const MPU6050_Calibration_t *cal,
                                       const MPU6050_t *raw,
                                       MPU6050_Gyro_dps_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_CORE_MPU6050_H_ */
