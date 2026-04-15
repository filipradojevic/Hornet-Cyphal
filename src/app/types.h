/**
 * @file    types.h
 * @brief   Project common defines and types.
 * @version 1.0.0
 * @date    07.04.2025
 * @author  LisumLab
 */

#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
/* Standard types */
#include <stdbool.h>
#include <stdint.h>

/* Middleware */
#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"
#include "task.h"

/* Cyphal register publishers */
#include "cyphal_reg_publishers.h"

/* Task BMS */
#include "mavlink/messages_cyphal_uavcan/common/BatteryStatus_1_0.h"

/* Task Motor */
#include "mavlink/messages_cyphal_uavcan/lisum/LisumPowerMotorScaledData_1_0.h"

/* Heartbeat */
#include "mavlink/messages_cyphal_uavcan/common/CommandLong_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/ComponentInformationBasic_1_0.h"
#include "uavcan/node/Mode_1_0.h"

#include "uavcan/primitive/array/Integer8_1_0.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/

#define MAVLINK_OR_CYPHAL 0 /* 1: MAVLink + Cyphal, 0: only MAVLink */

/* blinky period [ms] */
#define BLINKY_PERIOD_MS 500

/* Motor UART instance */
#define MOTOR_UART_INSTANCE UART_HAL_INSTANCE_1
/* Motor UART baud rate */
#define MOTOR_UART_BAUD_RATE 200000
/* Motor UART queue length in bytes */
#define MOTOR_UART_QUEUE_LEN 512

/* BMS1 UART instance */
#define BMS1_UART_INSTANCE UART_HAL_INSTANCE_2
/* BMS1 UART baud rate */
#define BMS1_UART_BAUD_RATE 9600
/* BMS1 UART queue length in bytes */
#define BMS1_UART_QUEUE_LEN 256

/* BMS2 UART instance */
#define BMS2_UART_INSTANCE UART_HAL_INSTANCE_3
/* BMS2 UART baud rate */
#define BMS2_UART_BAUD_RATE 9600
/* BMS2 UART queue length in bytes */
#define BMS2_UART_QUEUE_LEN 256

/* Cyphal timeouts [us] */
#define CYPHAL_CRITICAL_TIMEOUT 10000U
#define CYPHAL_MEDIUM_TIMEOUT 50000U
#define CYPHAL_LOW_TIMEOUT 1000000U

/* ------------------------- Ardupilot Versrion ----------------------------- */

/* DATA FOR ARDUPILOT VERSION MESSAGE */
#define PAYLOAD_LENGTH 8

/* Read from src/middleware/FreeRTOS/manifest.yml */
#define OS_CUSTOM_VERSION "v10.5.1"

/* BlackBox Board Version */
#define BOARD_TYPE 6	 // Read ethernet port marked by marker
#define BOARD_REVISION 2 // This is Hardware Version V0.2

/* Vendor and product IDs*/
#define VENDOR_ID 0x4254 // 'B' 'T' Inicialize of company
#define PRODUCT_ID 6	 // Number of model

#define VENDOR_NAME "BetaTehPro"
#define MODEL_NAME "HW_PWR_MAN"
#define SOFTWARE_VERSION FLIGHT_CUSTOM_VERSION
#define HARDWARE_VERSION "DEV-Board V0.2"
#define SERIAL_NUMBER "S_N: 016"

/* Mutex for Cyphal TX queuesss */
extern SemaphoreHandle_t tx_queue_mutex;

/* Mutex macros for Cyphal TX queue */
#define TX_QUEUE_MUTEX_TAKE                                                    \
	if (xSemaphoreTake(tx_queue_mutex, portMAX_DELAY) == pdTRUE)
#define TX_QUEUE_MUTEX_GIVE xSemaphoreGive(tx_queue_mutex)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef struct cyphal_subscription_messages_t {
	const enum CanardTransferKind kind;
	const CanardPortID port_id;
	const size_t serialization_buffer_size;
	struct CanardRxSubscription *subscription;
} cyphal_subscription_messages_t;

typedef enum {
	NODE_MODE_OPERATIONAL = uavcan_node_Mode_1_0_OPERATIONAL,
	NODE_MODE_INITIALIZATION = uavcan_node_Mode_1_0_INITIALIZATION,
	NODE_MODE_MAINTENANCE = uavcan_node_Mode_1_0_MAINTENANCE,
	NODE_MODE_SOFTWARE_UPDATE = uavcan_node_Mode_1_0_SOFTWARE_UPDATE,
	NODE_MODE_IDLE = 4,
	NODE_MODE_CRITICAL_FAILURE = 5
} node_mode_state_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TYPES_H */