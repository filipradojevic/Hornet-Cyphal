/**
 * @file    types.h
 * @brief   Project common defines and types.
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes: Cyphal/UAVCAN & MAVLink message types
 ******************************************************************************/

/* Middleware */
#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"
#include "task.h"

/* Cyphal register publishers */
#include "cyphal_reg_publishers.h"

/* Task ANPP*/
#include "mavlink/messages_cyphal_uavcan/lisum/LisumSensorAirspeedData_1_0.h"

/* Task Baro */
#include "mavlink/messages_cyphal_uavcan/common/Altitude_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/ScaledPressure_1_0.h"

/* Task Fusion */
#include "mavlink/messages_cyphal_uavcan/common/Attitude_1_0.h"

/* Task GNSS*/
#include "mavlink/messages_cyphal_uavcan/lisum/LisumGnssRecvData_1_0.h"

/* Task IMU */
#include "mavlink/messages_cyphal_uavcan/common/ScaledImu_1_0.h"

/* Task MAV */
#include "mavlink/messages_cyphal_uavcan/minimal/Heartbeat_1_0.h"

/* Heartbeat */
#include "mavlink/messages_cyphal_uavcan/common/CommandLong_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/ComponentInformationBasic_1_0.h"
#include "uavcan/node/Mode_1_0.h"
#include "uavcan/primitive/array/Integer8_1_0.h"

/* Standard types */
#include <stdint.h>

/*******************************************************************************
 * Defines: Periods, hardware, timeouts, mutex macros
 ******************************************************************************/

#define MAVLINK_OR_CYPHAL 0 /* 1: MAVLink + Cyphal, 0: only MAVLink */

/* Task periods [ms] */
#define BLINKY_PERIOD_MS 500
#define MAV_HB_PERIOD_MS 1000
#define ATTITUDE_PERIOD_MS 20
#define SCALED_IMU_PERIOD_MS 20
#define SCALED_IMU2_PERIOD_MS 20

/* MS5611 barometer hardware */
#define MS5611_CS_PORT GPIO_HAL_INSTANCE_0
#define MS5611_CS_PIN 11
#define MS5611_SPI_INSTANCE SPI_HAL_INSTANCE_0

/* ICM42688P IMU hardware */
#define ICM42688P_CS_PORT GPIO_HAL_INSTANCE_0
#define ICM42688P_CS_PIN 16
#define ICM42688P_INT_PORT GPIO_HAL_INSTANCE_0
#define ICM42688P_INT_PIN 4
#define ICM42688P_SPI_INSTANCE SPI_HAL_INSTANCE_0

/* GNSS UART */
#define GNSS_UART_INSTANCE UART_HAL_INSTANCE_2
#define GNSS_UART_BAUD_RATE 230400

/* ANPP UART */
#define ANPP_UART_INSTANCE UART_HAL_INSTANCE_1
#define ANPP_UART_BAUD_RATE 115200

/* IMU calibration */
#define IMU_DISABLE_CALIB 1

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
#define MODEL_NAME "HW_INS"
#define SOFTWARE_VERSION FLIGHT_CUSTOM_VERSION
#define HARDWARE_VERSION "DEV-Board V0.2"
#define SERIAL_NUMBER "S_N: 014"

/* Mutex for Cyphal TX queuesss */
extern SemaphoreHandle_t tx_queue_mutex;

/* Mutex macros for Cyphal TX queue */
#define TX_QUEUE_MUTEX_TAKE                                                    \
	if (xSemaphoreTake(tx_queue_mutex, portMAX_DELAY) == pdTRUE)
#define TX_QUEUE_MUTEX_GIVE xSemaphoreGive(tx_queue_mutex)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief fusion data */
typedef struct fusion_data_t {
	float gyro[3];	// Gyroscope data [deg/s]
	float accel[3]; // Accelerometer data [g]
	float mag[3];	// Magnetometer data - arbitrary units
	float dt;		// Time elapsed since last sample [s]
} fusion_data_t;

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
 * API
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TYPES_H */