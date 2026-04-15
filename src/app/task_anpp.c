/**
 * @file    task_anpp.c
 * @brief   Task ANPP - process airspeed sensor data
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_anpp.h"
#include "types.h"

/* Peripherals */
#include "uart.h"

/* External hardware drivers */
#include "anpp.h"

/* Lib */

/* Middleware */
#include "FreeRTOS.h"
#include "queue.h"

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

extern QueueHandle_t queue_mav_lisum_airspeed_data;
extern QueueHandle_t queue_anpp;

static anpp_t anpp;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void uart_cb(uart_hal_instance_t instance, uart_hal_handle_t *handle,
					lStatus_t status, void *userData);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_anpp(void *arg)
{

#if !MAVLINK_OR_CYPHAL

	messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0 airspeed_data = {
		0};

#else

	mavlink_lisum_sensor_airspeed_data_t airspeed_data = {0};

#endif /* MAVLINK_OR_CYPHAL */

	uart_hal_handle_t uart_handle;
	uint8_t process_byte;
	uint8_t receive_byte;
	int32_t ret = 0;
	uint32_t recv_status = 0x00;

	/* create new transfer handle */
	HAL_UART_TransferCreateHandle(ANPP_UART_INSTANCE, &uart_handle, uart_cb,
								  &queue_anpp);

	/* receive data from UART via ISR */
	HAL_UART_ReceiveISR(ANPP_UART_INSTANCE, &uart_handle, &receive_byte, 1);

	/* initialize ANPP */
	anpp_init(&anpp);

	for (;;) {
		/* wait to receive data */
		xQueueReceive(queue_anpp, &process_byte, portMAX_DELAY);

		/* parse data */
		ret = anpp_parse(&anpp, process_byte);
		if (ret != 0)
			continue;

		if (anpp.id == ANPP_PACKET_ID_RAW_SENSORS) {
			/* process ANPP raw sensors packet */
			anpp_raw_sensor_t raw_sensor = {0};

			if (anpp_raw_sensors_decode(&anpp, &raw_sensor))
				continue;

			airspeed_data.raw_press = raw_sensor.diff_press * .01f;
			airspeed_data.temperature = (int16_t)(raw_sensor.temperature * 100);
			airspeed_data.id = 0x00;

			/* every sample starts with ANPP Raw Sensor Packet */
			recv_status = 0;
			recv_status |= (1 << 0);
		} else if (anpp.id == ANPP_PACKET_ID_AIR_DATA) {
			/* process ANPP air data packet */
			anpp_air_data_t air_data = {0};

			if (anpp_air_data_decode(&anpp, &air_data))
				continue;

			airspeed_data.airspeed = air_data.true_airspeed;

			recv_status |= (1 << 1);
		}

		if (recv_status == 0x03) {
			/* all expected data is received */
			recv_status = 0x00;

			xQueueSendToBack(queue_mav_lisum_airspeed_data, &airspeed_data, 0);
		}
	}
}

static void uart_cb(uart_hal_instance_t instance, uart_hal_handle_t *handle,
					lStatus_t status, void *userData)
{
	BaseType_t taskWoken = pdFALSE;
	QueueHandle_t *queue = (QueueHandle_t *)userData;

	if (handle->rxDataSize && status == lStatus_Success) {
		/* send received byte to the back of the queue */
		xQueueSendToBackFromISR(*queue, handle->rxData, &taskWoken);
		/* receive another byte via interrupt */
		HAL_UART_ReceiveISR(instance, handle, handle->rxData, 1);
	}

	portYIELD_FROM_ISR(taskWoken);
}
