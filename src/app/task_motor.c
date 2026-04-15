/**
 * @file    task_motor.c
 * @brief   Task motor - process motor status
 * @version 1.0.0
 * @date    07.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_motor.h"
#include "types.h"

/* Peripherals */
#include "uart.h"

/* External hardware drivers */
#include "motor.h"

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

extern QueueHandle_t queue_mav_motor_scaled;
extern QueueHandle_t queue_motor_rx;

static motor_t motor;

#if MAVLINK_OR_CYPHAL

mavlink_lisum_power_motor_scaled_data_t data = {0};

#else

messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0 data = {0};

#endif /* MAVLINK_OR_CYPHAL */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* uart callback */
static void uart_cb(uart_hal_instance_t instance, uart_hal_handle_t *handle,
					lStatus_t status, void *userData);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_motor(void *arg)
{

	uart_hal_handle_t uart_handle;
	uint8_t parse_byte;
	uint8_t receive_byte;

	/* initialize motor parser */
	motor_init(&motor);

	/* create transfer handle */
	HAL_UART_TransferCreateHandle(MOTOR_UART_INSTANCE, &uart_handle, uart_cb,
								  &queue_motor_rx);

	/* receive byte via interrupt */
	HAL_UART_ReceiveISR(MOTOR_UART_INSTANCE, &uart_handle, &receive_byte, 1);

	TickType_t last_wake = xTaskGetTickCount();

	for (;;) {

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
		/* wait on queue */
		// xQueueReceive(queue_motor_rx, &parse_byte, portMAX_DELAY);

		/* parse byte */
		// if (motor_parse(&motor, parse_byte) == 0) {
		/* pack motor status */

		data.time = motor.status.data.time;
		data.w_gg = motor.status.data.w_gg;
		data.w_gg_N = motor.status.data.w_gg_N;
		data.w_ft = motor.status.data.w_ft;
		data.w_ft_N = motor.status.data.w_ft_N;
		data.warning = motor.status.data.warning;
		data._error = motor.status.data.error;
		data.fuel_flow = motor.status.data.fuel_flow * 100;
		data.oil_flow_arm = motor.status.data.oil_flow_arm * 100;
		data.oil_flow_reducer = motor.status.data.oil_flow_reducer * 100;
		data.battery_volt = motor.status.data.battery_volt * 1000;
		data.temp_front_bearing = motor.status.data.temp_front_bearing * 10;
		data.temp_rear_bearing = motor.status.data.temp_rear_bearing * 10;
		data.temp_elastic_bearing = motor.status.data.temp_elastic_bearing * 10;
		data.temp_rigid_bearing = motor.status.data.temp_rigid_bearing * 10;
		data.temp_input_oil = motor.status.data.temp_input_oil * 10;
		data.temp_arm_oil = motor.status.data.temp_arm_oil * 10;
		data.temp_reducer_oil = motor.status.data.temp_reducer_oil * 10;
		data.temp_exhaust_fume = motor.status.data.temp_exhaust_fume * 10;
		data.press_arm_oil = motor.status.data.press_arm_oil * 1000;
		data.fuel_level = motor.status.data.fuel_level * 100;
		data.oil_level_bearing = motor.status.data.oil_level_bearing * 100;
		data.oil_level_reducer = motor.status.data.oil_level_reducer * 100;
		data.PWM_fuel_pump = motor.status.data.PWM_fuel_pump * 100;
		data.current_fuel_pump = motor.status.data.current_fuel_pump * 100;
		data.PWM_oil_pump = motor.status.data.PWM_oil_pump * 100;
		data.current_oil_pump = motor.status.data.current_oil_pump * 100;
		data.PWM_oil_suction_pump_reducer =
			motor.status.data.PWM_oil_suction_pump_reducer * 100;
		data.PWM_oil_suction_pump_arm =
			motor.status.data.PWM_oil_suction_pump_arm * 100;
		data.PWM_oil_pressure_pump =
			motor.status.data.PWM_oil_pressure_pump * 100;
		data.turn_on = motor.status.data.turn_on;
		data.activate = motor.status.data.activate;
		data.valve_state = motor.status.data.valve_state;
		data.pressure_state = motor.status.data.pressure_state;
		memcpy(data.EvnC, &motor.status.data.EvnC,
			   sizeof(motor.status.data.EvnC));

		xQueueSendToBack(queue_mav_motor_scaled, &data, pdMS_TO_TICKS(0));

		// }
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
