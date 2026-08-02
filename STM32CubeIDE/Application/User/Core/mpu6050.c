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
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 100);
	if (status != HAL_OK) return status;

	// 4. Configure Frame Synchronization and Digital Low Pass Filter (DLPF)
	// Setting CONFIG (0x1A) to 0x03 sets the DLPF to ~42Hz bandwidth (1kHz output rate)
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

	// 6. Set Sample Rate Divider
	// SMPLRT_DIV (0x19) = 0x00 -> Sample Rate = 1 kHz / (1 + 0) = 1 kHz.
	// The base rate is 1 kHz only because the DLPF is enabled above; with
	// DLPF_CFG = 0 it becomes 8 kHz and the same divider yields a different rate.
	data = 0x0;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, 1, &data, 1, 100);
	if (status != HAL_OK) return status;

	// 7. Configure and enable Data Ready interrupt (Register 0x38 = 0x01)
	data = 0x01;
	status = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, 0x38, 1, &data, 1, 100);

	return status;
}

/**
 * @brief  Reads raw accelerometer, gyroscope, and temperature measurements in one burst.
 */
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data)
{
    uint8_t buffer[14];
    HAL_StatusTypeDef status;

    // Read 14 consecutive registers starting from ACCEL_XOUT_H
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, buffer, 14, 100);
    if (status != HAL_OK) return status;

    // Combine high and low bytes
    data->accel_x = (int16_t)((buffer[0]  << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2]  << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4]  << 8) | buffer[5]);

    int16_t raw_temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    // Temperature formula from MPU-6050 datasheet
    data->temperature = ((float)raw_temp / 340.0f) + 36.53f;

    data->gyro_x  = (int16_t)((buffer[8]  << 8) | buffer[9]);
    data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return HAL_OK;
}

/**
 * @brief  Reads the INT_STATUS register to clear the physical INT pin state.
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
