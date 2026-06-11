/**
 * @file    types.h
 * @brief   Project common defines and types.
 * @version 1.0.0
 * @date    26.02.2025
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

/* Middleware */
#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"
#include "task.h"

/* External hardware drivers */
#include "cyphal_mavlink_publishers.h"
#include "cyphal_reg_publishers.h"
#include "cyphal_uavcan_publishers.h"

/* Lib */
#include "can.h"
#include "canard.h"
#include "cyphal_utility.h"

#include "mavlink/messages_cyphal_uavcan/lisum/LisumPowerHornetActData_1_0.h"

#include "mavlink/messages_cyphal_uavcan/lisum/LisumManualCtrlHornet_1_0.h"

#include "mavlink/messages_cyphal_uavcan/common/CommandAck_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/CommandLong_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/Statustext_1_0.h"

/* Heartbeat */
#include "mavlink/messages_cyphal_uavcan/common/ComponentInformationBasic_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/FileTransferProtocol_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/ServoOutputRaw_1_0.h"
#include "mavlink/messages_cyphal_uavcan/minimal/Heartbeat_1_0.h"
#include "uavcan/node/Mode_1_0.h"
#include "uavcan/primitive/array/Integer8_1_0.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define MAVLINK_OR_CYPHAL 0 /* 1: MAVLink + Cyphal, 0: only MAVLink */

#define ACTUATOR_NUMBER_TRACKING 4

/* Cyphal timeouts [us] */
#define CYPHAL_CRITICAL_TIMEOUT 10000U
#define CYPHAL_MEDIUM_TIMEOUT 50000U
#define CYPHAL_LOW_TIMEOUT 1000000U

/* blinky period [ms] */
#define BLINKY_PERIOD_MS 500

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
#define MODEL_NAME "HW_ACT_MASTER"
#define SOFTWARE_VERSION FLIGHT_CUSTOM_VERSION
#define HARDWARE_VERSION "DEV-Board V0.2"
#define SERIAL_NUMBER "S_N: 018"

/* Mutex for Cyphal TX queuesss */
extern SemaphoreHandle_t tx_queue_mutex;

/* Mutex macros for Cyphal TX queue */
#define TX_QUEUE_MUTEX_TAKE                                                    \
	if (xSemaphoreTake(tx_queue_mutex, portMAX_DELAY) == pdTRUE)
#define TX_QUEUE_MUTEX_GIVE xSemaphoreGive(tx_queue_mutex)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

#pragma pack(push, 1)

typedef struct time_measurement_t {
	uint32_t receive_can_packet_time_us;
	uint32_t processed_data_time_us;
	uint32_t synchro_time_us;
	uint32_t latency_time_per_packet_us;
	uint32_t latency_receive_message_time_per_packet_us;
	uint32_t average_receive_message_time_us;
	uint32_t average_time_us;
	uint32_t max_time_us;
	uint32_t min_time_us;
	uint16_t packet_count;
} time_measurement_t;

#pragma pack(pop)

typedef enum actuator_modes_e {
	ACT_MODE_CYPHAL = 0,
	ACT_MODE_RC = 1,
	ACT_MODE_ONESHOOT125 = 2,
	ACT_MODE_ONESHOOT42 = 3,
	ACT_MODE_DSHOOT = 4
} actuator_modes_e;

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

void gpio_sync_callback(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* TYPES_H */