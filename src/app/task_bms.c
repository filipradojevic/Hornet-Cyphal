/**
 * @file    task_bms.c
 * @brief   Task BMS - process BMS status
 * @version 1.0.0
 * @date    07.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_bms.h"
#include "types.h"

/* Peripherals */
#include "uart.h"

/* External hardware drivers */
#include "bms.h"

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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* uart callback */
static void uart_cb(uart_hal_instance_t instance, uart_hal_handle_t *handle,
					lStatus_t status, void *userData);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_bms(void *arg)
{

#if MAVLINK_OR_CYPHAL
	mavlink_battery_status_t data;

#else
	messages_cyphal_uavcan_common_BatteryStatus_1_0 data;

#endif /* MAVLINK_OR_CYPHAL */

	uart_hal_handle_t uart_handle;
	task_bms_arg_t *cfg;
	uint8_t data_byte;
	uint8_t rx_byte;
	bms_t bms;

	cfg = (task_bms_arg_t *)arg;

	/* initialize bms parser */
	bms_init(&bms);

	/* create transfer handle */
	HAL_UART_TransferCreateHandle(cfg->instance, &uart_handle, uart_cb,
								  cfg->rx_queue);

	/* receive byte via interrupt */
	HAL_UART_ReceiveISR(cfg->instance, &uart_handle, &rx_byte, 1);

	TickType_t last_wake = xTaskGetTickCount();

	// Početne vrednosti da ne kreće od nule
	// Razlikuj početne vrednosti na osnovu instance UART-a ili nekog ID-a u cfg
	if (cfg->instance == BMS1_UART_INSTANCE) {
		bms.status.soc = 900; // BMS1 kreće skoro pun
		data.id = 0;
	} else {
		bms.status.soc = 300; // BMS2 kreće skoro prazan
		data.id = 1;
	}
	bms.status.temp = 25.0f;
	for (int i = 0; i < 14; i++)
		bms.status.cell_volt[i] = 3600;

	int charging = 1;

	for (;;) {

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
		if (charging) {
			bms.status.soc += 10;	 // Brži skok SOC-a (+1% po sekundi)
			bms.status.curr = 12.5f; // Veća struja punjenja
			bms.status.temp += 0.1f; // Brži rast temperature

			for (int i = 0; i < 14; i++) {
				bms.status.cell_volt[i] += 15; // Skok od 15mV
				if (bms.status.cell_volt[i] > 4200)
					bms.status.cell_volt[i] = 4200;
			}
			if (bms.status.soc >= 1000)
				charging = 0;
		} else {
			bms.status.soc -= 15;	  // Još brže pražnjenje
			bms.status.curr = -18.0f; // Jaka struja pražnjenja
			bms.status.temp -= 0.08f;

			for (int i = 0; i < 14; i++) {
				bms.status.cell_volt[i] -= 20; // Pad od 20mV
				if (bms.status.cell_volt[i] < 3200)
					bms.status.cell_volt[i] = 3200;
			}
			if (bms.status.soc <= 200)
				charging = 1;
		}

		// Pakovanje (Mavlink/Cyphal format)
		data.temperature = (int16_t)(bms.status.temp * 100);
		for (int i = 0; i < 10; i++)
			data.voltages[i] = bms.status.cell_volt[i];
		for (int i = 0; i < 4; i++)
			data.voltages_ext[i] = bms.status.cell_volt[10 + i];

		data.current_battery = (int16_t)(bms.status.curr * 100);
		data.battery_remaining = (uint8_t)(bms.status.soc / 10);
		data.charge_state = (charging ? 5 : 0);

		// 		data.id = 0;
		// data.battery_function = MAV_BATTERY_FUNCTION_ALL;
		// data.type_ = MAV_BATTERY_TYPE_LION;
		// data.temperature = (int16_t)(bms.status.temp * 100);
		// data.voltages[0] = bms.status.cell_volt[0];
		// data.voltages[1] = bms.status.cell_volt[1];
		// data.voltages[2] = bms.status.cell_volt[2];
		// data.voltages[3] = bms.status.cell_volt[3];
		// data.voltages[4] = bms.status.cell_volt[4];
		// data.voltages[5] = bms.status.cell_volt[5];
		// data.voltages[6] = bms.status.cell_volt[6];
		// data.voltages[7] = bms.status.cell_volt[7];
		// data.voltages[8] = bms.status.cell_volt[8];
		// data.voltages[9] = bms.status.cell_volt[9];
		// data.current_battery = bms.status.curr * 10;
		// data.energy_consumed = -1;
		// data.battery_remaining = bms.status.soc / 10;
		// data.time_remaining = 0;
		// data.charge_state = MAV_BATTERY_CHARGE_STATE_OK;
		// data.voltages_ext[0] = bms.status.cell_volt[10];
		// data.voltages_ext[1] = bms.status.cell_volt[11];
		// data.voltages_ext[2] = bms.status.cell_volt[12];
		// data.voltages_ext[3] = bms.status.cell_volt[13];
		// data.mode = MAV_BATTERY_MODE_UNKNOWN;
		// data.fault_bitmask = 0;

		xQueueSendToBack(*cfg->tx_queue, &data, pdMS_TO_TICKS(0));
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
