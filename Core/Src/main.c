/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "timer.h"
#include "ahrs.h"
#include "es_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	timer_interval_t sample_timer;
	uint32_t 	sample_time_ticks;
	float		sample_time_fl_us;
	uint16_t 	sample_time_us;
	uint16_t 	data_exchange_time_us;
	uint16_t	signal_processing_us;
} time_profiling_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum {
	US_IN_S = 1000000,
	N_SAMPLES_BEFORE_GYRO_INIT = 10,
	N_FILTER_CONSTANTS = 6,
	LOOP_TIME_MS = 1,
	N_ACCEL_SAMPLE_FOR_INITIAL_ESTIMATION = 100,
	N_KP_CONSTANTS = 3,
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t mpu_data;
MPU6050_Calibration_t mpu_cal;
vector_3f_t accel_g;
float accel_magnitude_g = 0.0f;
vector_3f_t gyro_dps;
vector_3f_t gyro_radps;
MPU6050_GyroZero_t gyro_zero;
ahrs_attitude_t attitude;
ahrs_attitude_t attitude_deg;
ahrs_attitude_t accu_accel_attitude_deg = {0};
vector_3f_t gyro_angles_deg;
volatile uint8_t mpu_data_ready = 0;
volatile time_profiling_t timings = {0};
uint8_t cnt_main_cycles = 0;
uint8_t cnt_read_failure = 0;
uint8_t mpu_int_status = 0;
uint8_t cnt_stale_reads = 0;
uint8_t error_trap = 0;
uint8_t gyro_inited = 0;
uint8_t gyro_init_request = 0;
uint8_t reset_orientation = 0;
HAL_StatusTypeDef i2c_status = HAL_OK;
uint16_t cnt_accel_samples = 0;
// Time constants for complementary filters
float tau_ms [N_FILTER_CONSTANTS] = {
	10,
	50,
	100,
	500,
	1000,
	5000,
};
// Complementary filter constants
float alpha [N_FILTER_CONSTANTS] = {0};
vector_3f_t complementary_filer_orientation_deg[N_FILTER_CONSTANTS];

// Quaternion-based orientation
float kp[N_KP_CONSTANTS] = {
	0.1, 1, 10
};
ahrs_t ahrs[N_KP_CONSTANTS] = {0};
vector_3f_t quaternion_orientation_deg[N_KP_CONSTANTS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Timing_ConfigureInterruptPriorities(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void process_orientation_reset_request(void) {
	vector_3f_t reset_orientation = {.x = 0, .y = 1, .z = 0};
	for (uint8_t i = 0; i < N_FILTER_CONSTANTS; ++i) {
		ahrs_gyro_reset(&complementary_filer_orientation_deg[i], &reset_orientation);
	}
	for (uint8_t i = 0; i < N_KP_CONSTANTS; ++i) {
		ahrs_init(&ahrs[i], &reset_orientation, kp[i]);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM6_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	/* After the MX_*_Init() calls, which set the EXTI priority and grouping. */
	Timing_ConfigureInterruptPriorities();

	timer_init();
	timer_interval_init(&timings.sample_timer);
	MPU6050_Calibration_Init(&mpu_cal);
	MPU6050_Gyro_Zero_Init(&gyro_zero);
	i2c_status = MPU6050_Init(&hi2c1);
	if (i2c_status != HAL_OK) {
		// Toggle an onboard LED or enter error loop if sensor is missing
		 Error_Handler();
	}

	for (uint8_t i = 0; i < N_FILTER_CONSTANTS; ++i) {
		alpha[i] = tau_ms[i] / (tau_ms[i] + LOOP_TIME_MS);
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (reset_orientation) {
			reset_orientation = 0;
			process_orientation_reset_request();
		}

		// Handling missed interrupt from sensor
		if (HAL_GPIO_ReadPin(MPU6050_INT_GPIO_Port, MPU6050_INT_Pin)) {
			mpu_data_ready = 1;
		}

		if (mpu_data_ready) {
			mpu_data_ready = 0; // Clear software flag
			TIMER_START(i2c_exchange);
			i2c_status = MPU6050_Read_All(&hi2c1, &mpu_data, &mpu_int_status);
			timings.data_exchange_time_us = timer_us(TIMER_ELAPSED(i2c_exchange));
			if (i2c_status == HAL_OK) {
				if (mpu_int_status & MPU6050_INT_DATA_RDY) {
					TIMER_START(sig_processing);
					mpu_data.sample_time_us = timings.sample_time_fl_us;

					MPU6050_Apply_Calibration(&mpu_cal, &mpu_data, &accel_g);

					accel_magnitude_g = MPU6050_Gravity_Magnitude(&accel_g);

					MPU6050_Gyro_Zero_Update(&gyro_zero, &mpu_data,
					                         accel_magnitude_g, &mpu_cal);

					MPU6050_Apply_Gyro_Calibration(&mpu_cal, &mpu_data, &gyro_dps);

					ahrs_estimate_attitude(&accel_g, &attitude);

					attitude_deg.roll = rad_to_deg(attitude.roll);
					attitude_deg.pitch = rad_to_deg(attitude.pitch);

					if (((!gyro_inited && (mpu_data.sample_count > N_SAMPLES_BEFORE_GYRO_INIT))
					|| gyro_init_request) && gyro_zero.ready) {
						cnt_accel_samples += 1;
						accu_accel_attitude_deg.roll += attitude_deg.roll;
						accu_accel_attitude_deg.pitch += attitude_deg.pitch;

						if (cnt_accel_samples > N_ACCEL_SAMPLE_FOR_INITIAL_ESTIMATION) {
							gyro_inited = 1;
							gyro_init_request = 0;
							vector_3f_t init_angle = {
									.x = accu_accel_attitude_deg.roll / cnt_accel_samples,
									.y = accu_accel_attitude_deg.pitch / cnt_accel_samples,
									.z = 0,
							};
							cnt_accel_samples = 0;
							accu_accel_attitude_deg.roll = 0;
							accu_accel_attitude_deg.pitch = 0;
							ahrs_gyro_reset(&gyro_angles_deg, &init_angle);
							for (uint8_t i = 0; i < N_FILTER_CONSTANTS; ++i) {
								ahrs_gyro_reset(&complementary_filer_orientation_deg[i], &init_angle);
							}
							for (uint8_t i = 0; i < N_KP_CONSTANTS; ++i) {
								ahrs_init(&ahrs[i], &accel_g, kp[i]);
							}
						}
					}

					ahrs_gyro_loop(&gyro_dps, (mpu_data.sample_time_us) / US_IN_S, &gyro_angles_deg);

					for (uint8_t i = 0; i < N_FILTER_CONSTANTS; ++i) {
						ahrs_complementary_filter(&attitude_deg, &gyro_dps,
								&complementary_filer_orientation_deg[i], alpha[i],
								(mpu_data.sample_time_us) / US_IN_S);
					}

					gyro_radps = vector_3f_scale(&gyro_dps, DEG2RAD_F);
					for (uint8_t i = 0; i < N_KP_CONSTANTS; ++i) {
						ahrs_update(&ahrs[i], &gyro_radps, &accel_g, (mpu_data.sample_time_us) / US_IN_S);
						vector_3f_t quaternion_orientation_rad = quaternion_to_euler(&ahrs[i].orientation);
						quaternion_orientation_deg[i] = vector_3f_scale(&quaternion_orientation_rad, RAD2DEG_F);
					}

					++mpu_data.sample_count;
					timings.signal_processing_us = timer_us(TIMER_ELAPSED(sig_processing));

				} else {
					++cnt_stale_reads;
				}
			} else {
				++cnt_read_failure;
				// Bus is locked up! (HAL_BUSY or HAL_ERROR)
				// 1. Clear the stuck SDA line physically
				MPU6050_Reset_I2C_Bus(&hi2c1);

				// 2. Re-awaken the sensor registers (Wake up, Filter, Sample Rate, Interrupts)
				MPU6050_Init(&hi2c1);
			}
		}
		cnt_main_cycles += 1;
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Apply the interrupt priorities the timing experiment depends on.
  *
  * Grouping first: HAL_NVIC_SetPriority() encodes against whatever grouping is
  * current, and the generated HAL_MspInit() leaves it at NVIC_PRIORITYGROUP_0,
  * which has no pre-emption bits. Lower number wins. TIM6 is pinned weak in
  * both modes so it cannot become a second source of delay.
  */
static void Timing_ConfigureInterruptPriorities(void)
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

#if TIMING_PRIORITY_MODE == TIMING_SYSTICK_WINS
    /* A tick landing just before a DATA_RDY edge delays the timestamp, and the
       interval that follows is short by the same amount. */
    HAL_NVIC_SetPriority(SysTick_IRQn,  0U, 0U);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15U, 0U);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15U, 0U);
#else
    /* Shipped: nothing outranks the measurement. */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn,  0U, 0U);
    HAL_NVIC_SetPriority(SysTick_IRQn, 15U, 0U);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15U, 0U);
#endif

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INT_Pin) { // Ensure this matches your CubeMX user label for PB8
        mpu_data_ready = 1;
        timings.sample_time_ticks = timer_interval_update(&timings.sample_timer);
        timings.sample_time_us = timer_us(timings.sample_time_ticks);
        timings.sample_time_fl_us = timer_us_f(timings.sample_time_ticks);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  error_trap = 1;
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
