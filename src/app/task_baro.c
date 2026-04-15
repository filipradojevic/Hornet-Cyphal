/**
 * @file    task_baro.c
 * @brief   Task baro - process barometer data
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_baro.h"
#include "types.h"

/* Peripherals */

/* External hardware drivers */
#include "ms5611.h"

/* Lib */

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
extern SemaphoreHandle_t semphr_mag_drdy;
extern QueueHandle_t queue_mav_scaled_pressure;
extern QueueHandle_t queue_mav_altitude;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static float altitude_calc(float pressure);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_baro(void *arg)
{
	TickType_t last_wake = xTaskGetTickCount();
	lStatus_t status = lStatus_Fail;
	ms5611_t *instance;
	float temperature;
	float pressure;
	float altitude_amsl;

#if !MAVLINK_OR_CYPHAL

	messages_cyphal_uavcan_common_ScaledPressure_1_0 baro = {0};
	messages_cyphal_uavcan_common_Altitude_1_0 altitude = {0};
#else

	mavlink_scaled_pressure_t baro = {0};
	mavlink_altitude_t altitude = {0};

#endif /* MAVLINK_OR_CYPHAL */

	instance = (ms5611_t *)arg;

	for (;;) {
		/* downrate measurements from 70 to 50 Hz */
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

		uint32_t meas_period = 0;

		/*--------------------------- Temperature ----------------------------*/
		xSemaphoreTake(mutex_spi, portMAX_DELAY);
		status = ms5611_measure(instance, MS5611_MEAS_TEMPERATURE,
								MS5611_OSR_2048, &meas_period);
		xSemaphoreGive(mutex_spi);

		if (status != lStatus_Success)
			continue; /* measurement request failed */

		vTaskDelay(pdMS_TO_TICKS((TickType_t)ceil(meas_period / 1000.0f)));

		xSemaphoreTake(mutex_spi, portMAX_DELAY);
		status =
			ms5611_collect(instance, MS5611_MEAS_TEMPERATURE, &temperature);
		xSemaphoreGive(mutex_spi);

		if (status != lStatus_Success)
			continue; /* collect failed */

		/*----------------------------- Pressure -----------------------------*/
		xSemaphoreTake(mutex_spi, portMAX_DELAY);
		status = ms5611_measure(instance, MS5611_MEAS_PRESSURE, MS5611_OSR_4096,
								&meas_period);
		xSemaphoreGive(mutex_spi);

		if (status != lStatus_Success)
			continue; /* measurement request failed */

		vTaskDelay(pdMS_TO_TICKS((TickType_t)ceil(meas_period / 1000.0f)));

		xSemaphoreTake(mutex_spi, portMAX_DELAY);
		status = ms5611_collect(instance, MS5611_MEAS_PRESSURE, &pressure);
		xSemaphoreGive(mutex_spi);

		if (status != lStatus_Success)
			continue; /* collect failed */

		/* pack scaled pressure */
		baro.time_boot_ms = xTaskGetTickCount();
		baro.press_abs = pressure * .01f;
		baro.temperature = temperature * 100;

		xQueueSendToBack(queue_mav_scaled_pressure, &baro, 0);

		/* calculate and pack altitude */
		altitude_amsl = altitude_calc(pressure);

		altitude.time_usec = xTaskGetTickCount() * 1000;
		altitude.altitude_amsl = altitude_amsl;

		xQueueSendToBack(queue_mav_altitude, &altitude, 0);
	}
}

static float altitude_calc(float pressure)
{
	/* equation from BMP180 datasheet (revision 2.5) page 17 */
	return (44330 * (1 - pow(pressure / 101325.0f, 1 / 5.255)));
}
