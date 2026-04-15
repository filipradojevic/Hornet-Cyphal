/**
 * @file    task_imu.c
 * @brief   Task IMU - process inertial measurement unit data
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_work.h"
#include "types.h"

/* Peripherals */

/* External hardware drivers */
#include "icm42688p.h"

/* Lib */
#include "FusionMath.h"

/* Middleware */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "mav.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern SemaphoreHandle_t mutex_spi;
extern SemaphoreHandle_t semphr_imu_drdy;
extern QueueHandle_t queue_mav_scaled_imu;
extern QueueHandle_t queue_mav_scaled_imu2;
extern QueueHandle_t queue_fusion_data;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static inline void swap_vals(float *x, float *y);

static void acc_calibrate(float *raw, const float *A, const float *b,
						  float *calib);

static void gyro_calibrate(float *raw, const float *b, float *calib);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_imu(void *arg)
{
	TickType_t time_sent = xTaskGetTickCount();

#if !MAVLINK_OR_CYPHAL

	messages_cyphal_uavcan_common_ScaledImu_1_0 imu = {0};

#else

	mavlink_scaled_imu_t imu = {0};

#endif /* MAVLINK_OR_CYPHAL */

	icm42688p_int_status_t int_status;
	icm42688p_t *instance;
	float temperature = 0;
	float acc_raw[3] = {0};
	float gyro_raw[3] = {0};
	float acc_calib[3] = {0};
	float gyro_calib[3] = {0};

	instance = (icm42688p_t *)arg;

	for (;;) {
		/* wait for data ready */
		xSemaphoreTake(semphr_imu_drdy, portMAX_DELAY);

		xSemaphoreTake(mutex_spi, portMAX_DELAY);
		icm42688p_int_status(instance, &int_status);

		icm42688p_temp_get(instance, &temperature);
		icm42688p_accel_get(instance, &acc_raw[0], &acc_raw[1], &acc_raw[2]);
		icm42688p_gyro_get(instance, &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
		xSemaphoreGive(mutex_spi);

		/* perform transform */
		swap_vals(&acc_raw[0], &acc_raw[1]);
		swap_vals(&gyro_raw[0], &gyro_raw[1]);

		acc_raw[2] *= (-1);
		gyro_raw[0] *= (-1);
		gyro_raw[1] *= (-1);
		gyro_raw[2] *= (-1);

		/* there is no calibration procedure for now */
		memcpy(gyro_calib, gyro_raw, sizeof(gyro_calib));
		memcpy(acc_calib, acc_raw, sizeof(acc_calib));

		fusion_data_t fusion_data = {.accel[0] = acc_calib[0],
									 .accel[1] = acc_calib[1],
									 .accel[2] = acc_calib[2],
									 .gyro[0] = -gyro_calib[0],
									 .gyro[1] = -gyro_calib[1],
									 .gyro[2] = gyro_calib[2],
									 .dt = 1.0f / 200.0f};

		xQueueSendToBack(queue_fusion_data, &fusion_data, 0);

		/* pack scaled imu */
		if (xTaskGetTickCount() - time_sent >= SCALED_IMU_PERIOD_MS) {
			time_sent = xTaskGetTickCount();

			imu.time_boot_ms = xTaskGetTickCount();
			imu.xacc = (int16_t)(acc_calib[0] * 1000);
			imu.yacc = (int16_t)(acc_calib[1] * 1000);
			imu.zacc = (int16_t)(acc_calib[2] * 1000);
			imu.xgyro = (int16_t)(FusionDegreesToRadians(gyro_calib[0]) * 1000);
			imu.ygyro = (int16_t)(FusionDegreesToRadians(gyro_calib[1]) * 1000);
			imu.zgyro = (int16_t)(FusionDegreesToRadians(gyro_calib[2]) * 1000);
			imu.temperature = (int16_t)(temperature * 100);

			xQueueSendToBack(queue_mav_scaled_imu, &imu, 0);
		}
	}
}

static inline void swap_vals(float *x, float *y)
{
	float tmp;

	tmp = *x;
	*x = *y;
	*y = tmp;
}

static void acc_calibrate(float *raw, const float *A, const float *b,
						  float *calib)
{
	calib[0] = (raw[0] - b[0]) * A[0];
	calib[1] = (raw[1] - b[1]) * A[1];
	calib[2] = (raw[2] - b[2]) * A[2];
}

static void gyro_calibrate(float *raw, const float *b, float *calib)
{
	calib[0] = raw[0] - b[0];
	calib[1] = raw[1] - b[1];
	calib[2] = raw[2] - b[2];
}
