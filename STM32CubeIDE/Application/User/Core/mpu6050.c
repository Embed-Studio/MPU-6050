/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  oleg
  * @date    2026-Jul-20
  * @brief   Description
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "mpu6050.h"

#include <math.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Deliberately non-static: read live over SWD as a capture probe. */
uint8_t mpu6050_id = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initializes MPU-6050, wakes it up, and configures the Data Ready interrupt.
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;
    HAL_StatusTypeDef status;

    // 0. Free the bus. A debugger-forced reset (i.e. every flash) can stop the
    // MCU mid-transaction with the sensor still holding SDA low, which makes
    // every transaction below fail and traps main() in Error_Handler().
    MPU6050_Reset_I2C_Bus(hi2c);

    // 1. Wait for the device to answer, then verify Device ID.
    // On a cold power-up the STM32 is ready long before the MPU-6050 is, so the
    // first transaction would fail. An MCU reset alone does not power-cycle the
    // sensor, which is why this only ever bites on a fresh power-on.
    uint32_t start_tick = HAL_GetTick();
    while (HAL_I2C_IsDeviceReady(hi2c, MPU6050_I2C_ADDR, 1, 10) != HAL_OK) {
        if ((HAL_GetTick() - start_tick) > MPU6050_STARTUP_TIMEOUT_MS) {
            return HAL_TIMEOUT;
        }
    }

    status = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, 1, &mpu6050_id, 1, 100);
    if (status != HAL_OK || mpu6050_id != MPU6050_WHO_AM_I_VAL) {
        return HAL_ERROR;
    }

	// 2. Reset the device to a known state.
	// An MCU reset does not power-cycle the MPU-6050, so without this the sensor
	// keeps whatever configuration the previous run left behind.
	data = MPU6050_DEVICE_RESET;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 100);
	if (status != HAL_OK) return status;
	HAL_Delay(100);	// Reset bit self-clears; allow the device to settle before configuring

    // 3. Wake up sensor (Clear sleep bit in PWR_MGMT_1, clock source - gyro z for improved stability of sampling time)
	data = 0x03;
//	data = 0x00;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 100);
	if (status != HAL_OK) return status;

	// 4. Configure Frame Synchronization and Digital Low Pass Filter (DLPF)
	// CONFIG (0x1A) = 0x03 -> DLPF_CFG = 3. One register sets both filters, but
	// they are not the same filter: 44 Hz / 4.9 ms delay on the accelerometer,
	// 42 Hz / 4.8 ms on the gyroscope. Gyro output rate becomes 1 kHz, which is
	// what makes SMPLRT_DIV below mean 1 kHz.
	data = 0x03;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, 1, &data, 1, 100);
	if (status != HAL_OK) return status;

	// 5. Set accelerometer full-scale range
	// ACCEL_CONFIG (0x1C) = 0x00 -> AFS_SEL = 0 -> +/-2 g, 16384 LSB/g nominal.
	// This matches the power-on default, but is written explicitly: the calibration
	// constants are only valid for the range they were measured at.
	data = 0x00;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, 100);
	if (status != HAL_OK) return status;

	// 6. Set gyroscope full-scale range
	// GYRO_CONFIG (0x1B) = 0x00 -> FS_SEL = 0 -> +/-250 deg/s, 131 LSB/(deg/s)
	// nominal. This is the power-on default and the device reset in step 2 also
	// restores it, so the register held this value before — but every gyro number
	// derived from a capture is scaled by it, and a value that important should be
	// written rather than inherited. Same argument as ACCEL_CONFIG above.
	// The narrowest range gives the finest resolution, and nothing here rotates
	// anywhere near 250 deg/s.
	data = 0x00;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, 100);
	if (status != HAL_OK) return status;

	// 7. Set Sample Rate Divider
	// SMPLRT_DIV (0x19) = 0x00 -> Sample Rate = 1 kHz / (1 + 0) = 1 kHz.
	// The base rate is 1 kHz only because the DLPF is enabled above; with
	// DLPF_CFG = 0 it becomes 8 kHz and the same divider yields a different rate.
	data = 0x0;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, 1, &data, 1, 100);
	if (status != HAL_OK) return status;

	// 8. Configure and enable Data Ready interrupt (Register 0x38 = 0x01)
	data = 0x01;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, 0x38, 1, &data, 1, 100);

	return status;
}

/**
 * @brief  Reads INT_STATUS plus the raw accelerometer, gyroscope and temperature
 *         measurements in one burst, and releases the sensor's INT pin.
 *
 * @param  int_status  receives the INT_STATUS byte; test it against
 *                     MPU6050_INT_DATA_RDY to find out whether this burst
 *                     carries a sample the sensor had not delivered before.
 */
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data,
                                   uint8_t *int_status)
{
    uint8_t buffer[15];
    HAL_StatusTypeDef status;

    // Read 15 consecutive registers starting from INT_STATUS (0x3A), which sits
    // directly below the 14 data registers at ACCEL_XOUT_H (0x3B).
    //
    // Starting one register early is what clears DATA_RDY and releases the INT
    // pin: with INT_RD_CLEAR at its reset value, nothing else does. Folding it
    // into this burst costs one byte on the wire (~23 us in Fast mode) instead
    // of a second addressed transaction, and the byte is not waste — it says
    // whether the 14 that follow are new.
    //
    // It has to come FIRST, not last. If the sensor updates mid-burst, the INT
    // it raises then is for a sample this read has not returned: the data
    // registers are shadowed while the bus is busy, so the 14 bytes below are
    // the set that was current when the burst began. Clearing at the front
    // leaves that new INT standing and the next pass collects it; clearing at
    // the end would wipe it and drop the sample.
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_INT_STATUS, 1, buffer, 15, 100);
    if (status != HAL_OK) return status;

    *int_status = buffer[0];

    // Combine high and low bytes
    data->accel_x = (int16_t)((buffer[1]  << 8) | buffer[2]);
    data->accel_y = (int16_t)((buffer[3]  << 8) | buffer[4]);
    data->accel_z = (int16_t)((buffer[5]  << 8) | buffer[6]);

    int16_t raw_temp = (int16_t)((buffer[7] << 8) | buffer[8]);
    // Temperature formula from MPU-6050 datasheet
    data->temperature = ((float)raw_temp / 340.0f) + 36.53f;

    data->gyro_x  = (int16_t)((buffer[9]  << 8) | buffer[10]);
    data->gyro_y  = (int16_t)((buffer[11] << 8) | buffer[12]);
    data->gyro_z  = (int16_t)((buffer[13] << 8) | buffer[14]);

    return HAL_OK;
}

/**
 * @brief  Loads the compiled-in calibration constants.
 */
void MPU6050_Calibration_Init(MPU6050_Calibration_t *cal)
{
    cal->accel_scale[0] = MPU6050_ACCEL_SCALE_X_DEFAULT;
    cal->accel_scale[1] = MPU6050_ACCEL_SCALE_Y_DEFAULT;
    cal->accel_scale[2] = MPU6050_ACCEL_SCALE_Z_DEFAULT;

    cal->accel_bias[0] = MPU6050_ACCEL_BIAS_X_DEFAULT;
    cal->accel_bias[1] = MPU6050_ACCEL_BIAS_Y_DEFAULT;
    cal->accel_bias[2] = MPU6050_ACCEL_BIAS_Z_DEFAULT;

    cal->gyro_scale[0] = MPU6050_GYRO_SCALE_X_DEFAULT;
    cal->gyro_scale[1] = MPU6050_GYRO_SCALE_Y_DEFAULT;
    cal->gyro_scale[2] = MPU6050_GYRO_SCALE_Z_DEFAULT;

    /* Left at zero on purpose: the gyroscope bias is measured at every startup
       by MPU6050_Gyro_Zero_Update(), never stored. Until it lands, the rate
       output carries the raw offset rather than hiding it behind a stale one. */
    cal->gyro_bias[0] = 0.0f;
    cal->gyro_bias[1] = 0.0f;
    cal->gyro_bias[2] = 0.0f;
}

/**
 * @brief  Converts one raw sample to g: acceleration = (raw - bias) / scale.
 */
void MPU6050_Apply_Calibration(const MPU6050_Calibration_t *cal,
                               const MPU6050_t *raw,
                               MPU6050_Accel_g_t *out)
{
    out->x = ((float)raw->accel_x - cal->accel_bias[0]) / cal->accel_scale[0];
    out->y = ((float)raw->accel_y - cal->accel_bias[1]) / cal->accel_scale[1];
    out->z = ((float)raw->accel_z - cal->accel_bias[2]) / cal->accel_scale[2];
}

/**
 * @brief  Converts one raw sample to deg/s: rate = (raw - bias) / scale.
 *
 * The bias is zero until the stillness gate accepts a window, so early samples
 * come out with the raw offset still in them.
 */
void MPU6050_Apply_Gyro_Calibration(const MPU6050_Calibration_t *cal,
                                    const MPU6050_t *raw,
                                    MPU6050_Gyro_dps_t *out)
{
    out->x = ((float)raw->gyro_x - cal->gyro_bias[0]) / cal->gyro_scale[0];
    out->y = ((float)raw->gyro_y - cal->gyro_bias[1]) / cal->gyro_scale[1];
    out->z = ((float)raw->gyro_z - cal->gyro_bias[2]) / cal->gyro_scale[2];
}

/**
 * @brief  Resets the startup-zeroing state and begins a fresh window.
 */
void MPU6050_Gyro_Zero_Init(MPU6050_GyroZero_t *z)
{
    memset(z, 0, sizeof(*z));
}

/* Clears the current check block; the banked window is untouched. */
static void gyro_zero_block_reset(MPU6050_GyroZero_t *z)
{
    z->blk_n = 0;
    for (int i = 0; i < 3; ++i) {
        z->blk_sum[i] = 0;
        z->blk_sum_sq[i] = 0;
    }
    z->blk_acc_sum = 0.0f;
    z->blk_acc_sum_sq = 0.0f;
}

/**
 * @brief  Feeds one sample to the startup zeroing. Call it once per new sample.
 * @param  accel_magnitude_g  |a| from MPU6050_Gravity_Magnitude(), used by the
 *                            two secondary stillness tests.
 * @retval 1 once a zero has been accepted and written to cal->gyro_bias,
 *         0 while still hunting for a still enough window.
 *
 * Averages MPU6050_GYRO_ZERO_SAMPLES of raw rate, banking samples only while
 * the board is still: one failing check block throws the whole banked window
 * away, so the constant comes from a stretch that was still throughout.
 *
 * The refusal is the point. It does not make the constant more accurate — it
 * stops the firmware committing to one while the board moves, because a moving
 * board's rate average is the motion, not the offset. See the gate thresholds
 * in the header for what each of the three tests catches.
 *
 * Once accepted it does not re-zero: that needs a policy about when the board is
 * at rest again, which belongs to the application rather than this driver.
 */
uint8_t MPU6050_Gyro_Zero_Update(MPU6050_GyroZero_t *z,
                                 const MPU6050_t *raw,
                                 float accel_magnitude_g,
                                 MPU6050_Calibration_t *cal)
{
    if (z->ready) {
        return 1;
    }

    const int32_t g[3] = { raw->gyro_x, raw->gyro_y, raw->gyro_z };
    for (int i = 0; i < 3; ++i) {
        z->blk_sum[i]    += g[i];
        z->blk_sum_sq[i] += (int64_t)g[i] * g[i];
    }
    const float d = accel_magnitude_g - 1.0f;   /* see MPU6050_GyroZero_t */
    z->blk_acc_sum    += d;
    z->blk_acc_sum_sq += d * d;

    if (++z->blk_n < MPU6050_GYRO_ZERO_CHECK) {
        return 0;
    }

    /* Check block complete: measure how still the board was over it. */
    const uint32_t n = z->blk_n;
    float var_sum = 0.0f;
    for (int i = 0; i < 3; ++i) {
        /* Exact, rather than the naive float form which subtracts two nearly
           equal large numbers and loses the digits the variance lives in. The
           (int64_t) matters: blk_sum reaches 6.5e6, so squaring it in 32 bits
           would overflow. blk_sum_sq is already int64_t, so n is promoted. */
        const int64_t num = n * z->blk_sum_sq[i] - (int64_t)z->blk_sum[i] * z->blk_sum[i];
        const float var = (float)num / (float)(n * (n - 1u));
        z->sd[i] = sqrtf(var);
        var_sum += var;
    }
    z->sd_norm = sqrtf(var_sum);

    const float fn = (float)z->blk_n;
    const float acc_mean = z->blk_acc_sum / fn;
    const float acc_var = (z->blk_acc_sum_sq - z->blk_acc_sum * acc_mean) / (fn - 1.0f);
    z->acc_sd = sqrtf(acc_var > 0.0f ? acc_var : 0.0f);
    z->acc_err = acc_mean;   /* already relative to 1 g */

    const uint8_t still = (z->sd_norm <= MPU6050_GYRO_ZERO_SD_MAX) &&
                          (z->acc_sd  <= MPU6050_GYRO_ZERO_ACCEL_SD_MAX) &&
                          (fabsf(z->acc_err) <= MPU6050_GYRO_ZERO_ACCEL_ERR_MAX);

    if (!still) {
        /* The board moved. Everything banked is discarded: the window has to be
           continuously still, not still on average. */
        ++z->blocks_failed;
        if (z->n != 0) {
            ++z->restarts;
            z->n = 0;
            for (int i = 0; i < 3; ++i) {
                z->sum[i] = 0;
            }
        }
        gyro_zero_block_reset(z);
        return 0;
    }

    /* Still: bank the block. */
    for (int i = 0; i < 3; ++i) {
        z->sum[i] += z->blk_sum[i];
    }
    z->n = (uint16_t)(z->n + z->blk_n);
    gyro_zero_block_reset(z);

    if (z->n < MPU6050_GYRO_ZERO_SAMPLES) {
        return 0;
    }

    for (int i = 0; i < 3; ++i) {
        cal->gyro_bias[i] = (float)z->sum[i] / (float)z->n;
    }
    z->ready = 1;
    return 1;
}

/**
 * @brief  Vector magnitude of a calibrated sample, in g.
 *
 * For a stationary sensor this is 1 g whichever way the board is turned, which
 * is what makes it a validation criterion that needs no reference hardware:
 * the true orientation never has to be known. Only valid at rest — any real
 * acceleration adds to gravity.
 */
float MPU6050_Gravity_Magnitude(const MPU6050_Accel_g_t *accel_g)
{
    return sqrtf(accel_g->x * accel_g->x +
                 accel_g->y * accel_g->y +
                 accel_g->z * accel_g->z);
}

/**
 * @brief  Reads the INT_STATUS register to clear the physical INT pin state.
 *
 * The sampling path does not need this — MPU6050_Read_All() already begins its
 * burst at INT_STATUS. It stays for the paths that do not go through a data
 * read, such as discarding a stale interrupt after a bus recovery.
 */
HAL_StatusTypeDef MPU6050_Clear_Interrupt(I2C_HandleTypeDef *hi2c)
{
    uint8_t dummy;
    return HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_INT_STATUS, 1, &dummy, 1, 100);
}

/**
 * @brief  Manually clocks out the I2C bus to release an SDA line held low by a stuck slave.
 */
void MPU6050_Reset_I2C_Bus(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. De-initialize I2C peripheral
    HAL_I2C_DeInit(hi2c);

    // 2. Enable the GPIO clock for the bus pins (default: GPIOB, holding PB6/PB9)
    MPU6050_I2C_GPIO_CLK_ENABLE();

    // 3. Configure SCL as Output Open-Drain, Pull-up
    GPIO_InitStruct.Pin = MPU6050_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MPU6050_SCL_GPIO_Port, &GPIO_InitStruct);

    // 4. Configure SDA as Input, Pull-up
    GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MPU6050_SDA_GPIO_Port, &GPIO_InitStruct);

    // 5. Generate up to 9 clock pulses if SDA is held low by the slave
    // A standard I2C frame is 9 bits; toggling 9 times forces the slave to finish its byte.
    for (int i = 0; i < 9; i++)
    {
        // Check if slave released the line
        if (HAL_GPIO_ReadPin(MPU6050_SDA_GPIO_Port, MPU6050_SDA_Pin) == GPIO_PIN_SET)
        {
            break;
        }

        // Toggle SCL
        HAL_GPIO_WritePin(MPU6050_SCL_GPIO_Port, MPU6050_SCL_Pin, GPIO_PIN_RESET);
        HAL_Delay(1); // Small delay for bus timing
        HAL_GPIO_WritePin(MPU6050_SCL_GPIO_Port, MPU6050_SCL_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // 6. Force and release the peripheral reset to clear internal state flags.
    // Taken from the handle the caller passed, so this follows whichever I2C is
    // in use instead of always resetting I2C1.
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_FORCE_RESET();
        HAL_Delay(2);
        __HAL_RCC_I2C1_RELEASE_RESET();
    } else if (hi2c->Instance == I2C2) {
        __HAL_RCC_I2C2_FORCE_RESET();
        HAL_Delay(2);
        __HAL_RCC_I2C2_RELEASE_RESET();
    } else if (hi2c->Instance == I2C3) {
        __HAL_RCC_I2C3_FORCE_RESET();
        HAL_Delay(2);
        __HAL_RCC_I2C3_RELEASE_RESET();
    }
    HAL_Delay(2);

    // 7. Re-initialize the I2C hardware using CubeMX generated settings
    HAL_I2C_Init(hi2c);
}
