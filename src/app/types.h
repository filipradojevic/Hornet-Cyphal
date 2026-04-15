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
 * Includes
 ******************************************************************************/
/* External hardware drivers */
#include "cyphal_mavlink_publishers.h"
#include "cyphal_reg_publishers.h"
#include "cyphal_uavcan_publishers.h"

/* Lib */
#include "can.h"
#include "canard.h"
#include "cyphal_utility.h"

/* MAVLink messages */
#include "mavlink/messages_cyphal_uavcan/common/AdsbVehicle_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/AisVessel_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/Attitude_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/AutopilotVersion_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/CommandLong_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/FileTransferProtocol_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/NamedValueFloat_1_0.h"
#include "mavlink/messages_cyphal_uavcan/common/ScaledImu_1_0.h"
#include "mavlink/messages_cyphal_uavcan/lisum/LisumGnssRecvData_1_0.h"

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

/* Task BMS */
#include "mavlink/messages_cyphal_uavcan/common/BatteryStatus_1_0.h"

/* Task Motor */
#include "mavlink/messages_cyphal_uavcan/lisum/LisumPowerMotorScaledData_1_0.h"

/* Task ACTUATOR */
#include "mavlink/messages_cyphal_uavcan/common/CommandAck_1_0.h"
#include "mavlink/messages_cyphal_uavcan/lisum/LisumManualCtrlHornet_1_0.h"
#include "mavlink/messages_cyphal_uavcan/lisum/LisumPowerHornetActData_1_0.h"

/* UAVCAN / Reg */
#include "reg/udral/physics/thermodynamics/PressureTempVarTs_0_1.h"

#include "uavcan/si/sample/angular_velocity/Vector3_1_0.h"
#include "uavcan/si/sample/length/Scalar_1_0.h"

/* Heartbeat */
#include "mavlink/messages_cyphal_uavcan/minimal/Heartbeat_1_0.h"

/* UAVCAN standard types */
#include "uavcan/uavcan/node/GetInfo_1_0.h"
#include "uavcan/uavcan/node/Heartbeat_1_0.h"
#include "uavcan/uavcan/node/Mode_1_0.h"

#include "uavcan/primitive/scalar/Integer16_1_0.h"
#include "uavcan/primitive/scalar/Integer64_1_0.h"
#include "uavcan/primitive/scalar/Integer8_1_0.h"
#include "uavcan/uavcan/primitive/array/Integer8_1_0.h"

#include "uavcan/si/unit/angle/Quaternion_1_0.h"
#include "uavcan/si/unit/angular_acceleration/Vector3_1_0.h"
#include "uavcan/si/unit/angular_velocity/Vector3_1_0.h"
#include "uavcan/si/unit/pressure/Scalar_1_0.h"
#include "uavcan/si/unit/temperature/Scalar_1_0.h"
#include "uavcan/si/unit/velocity/Scalar_1_0.h"

#include "uavcan/si/sample/angle/Scalar_1_0.h"
#include "uavcan/si/sample/length/WideVector3_1_0.h"
#include "uavcan/si/sample/magnetic_field_strength/Vector3_1_0.h"
#include "uavcan/si/sample/velocity/Vector3_1_0.h"

#include "mavlink/messages_cyphal_uavcan/common/ComponentInformationBasic_1_0.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* ------------------------------- Bootloader ------------------------------- */
#define BL_FTP_DEV_ID_POS 20

/* blinky period [ms] */
#define BLINKY_PERIOD_MS 500

/* Telemetry UART instance */
#define TELEMETRY_UART_INSTANCE UART_HAL_INSTANCE_3
/* Telemetry UART baud rate */
#define TELEMETRY_UART_BAUD_RATE 230400
/* Telemetry UART receive queue length in bytes */
#define TELEMETRY_RX_QUEUE_LEN 1024
/* Telemetry UART transmit queue length in bytes */
#define TELEMETRY_TX_QUEUE_LEN 1024

/* DATA FOR ARDUPILOT VERSION MESSAGE */
#define PAYLOAD_LENGTH 8

/* Read from src/middleware/FreeRTOS/manifest.yml */
#define OS_CUSTOM_VERSION OS_CUSTOM_HASH

/* BlackBox Board Version */
#define BOARD_TYPE 8	 // Read ethernet port marked by marker
#define BOARD_REVISION 2 // This is Hardware Version V0.2

/* Vendor and product IDs*/
#define VENDOR_ID 0x4254 // 'B' 'T' Inicialize of company
#define PRODUCT_ID 8	 // Number of model

#define VENDOR_NAME "BetaTehPro"
#define MODEL_NAME "HW_GW_SKY"
#define SOFTWARE_VERSION FLIGHT_CUSTOM_VERSION
#define HARDWARE_VERSION "DEV-Board V0.2"
#define SERIAL_NUMBER "S_N: 008"

#define MAVLINK_OR_CYPHAL 0 /* 1 is for MAVLink & UDP, 0 is for Cyphal CAN Bus */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef struct cyphal_subscription_messages_t {
	const enum CanardTransferKind kind;
	const CanardPortID port_id;
	const size_t serialization_buffer_size;
	struct CanardRxSubscription* subscription;
} cyphal_subscription_messages_t;

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