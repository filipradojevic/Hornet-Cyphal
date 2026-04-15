/**
 * @file    task_mav.c
 * @brief   Task MAV - handle MAVLink packets
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "ftp_common.h"
#include "main.h"
#include "task_mav.h"
#include "task_work.h"
#include "types.h"

/* Peripherals */
#include "bl.h"
#include "gpio.h"

/* Middleware */
#include "FreeRTOS.h"
#include "common.h"
#include "mav.h"
#include "queue.h"
#include "timers.h"

#include "udp_tl.h"

#include "uart_tl.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* MAVLink heartbeat period [ms] */
#define MAV_HB_PERIOD_MS 1000

/* MAVLink gateway ground track count */
#define MAV_GW_GND_TRACK_COUNT 7

/* MAVLink ground station helicopter application track count */
#define MAV_HELI_TRACK_COUNT 2

#if MAVLINK_OR_CYPHAL

/* MAVLink power management track count */
#define MAV_PWR_MAN_TRACK_COUNT 3

/* MAVLink ins track count */
#define MAV_INS_TRACK_COUNT 10

/* MAVLink actuator master track count */
#define MAV_ACT_MASTER_TRACK_COUNT 4

/* MAVLink ins cots track count */
#define MAV_INS_COTS_TRACK_COUNT 3

/* MAVLink black_box track count */
#define MAV_BLACK_BOX_TRACK_COUNT 3

#else

#define CYPHAL_SUBSCRIPTION_MESSAGES_COUNT_GW_SKY 14

#endif /* MAVLINK_OR_CYPHAL */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern SemaphoreHandle_t mutex_mav;
extern QueueSetHandle_t queueset_mav;
extern QueueHandle_t queue_mav_hb;

extern uart_tl_t mav_gw_gnd_uart_tl;
extern udp_tl_t mav_gw_gnd_udp_tl;

#if MAVLINK_OR_CYPHAL

extern udp_tl_t mav_pwr_man_tl;
extern udp_tl_t mav_black_box_tl;
extern udp_tl_t mav_ins_tl;
extern udp_tl_t mav_act_master_tl;
extern udp_tl_t mav_ins_cots_tl;

#endif /* MAVLINK_OR_CYPHAL */

/* Extern temporary variables */
extern uint64_t boot_time_ms;

// MAVLink information
static const uint8_t mav_key[32] = "Kolega zatvorite vrata";

const uint8_t dev_mav_sysid = 0;
const uint8_t dev_mav_compid = MAV_COMP_ID_USER1;

const uint8_t gw_gnd_mav_sysid = 1;
const uint8_t gw_gnd_mav_compid = MAV_COMP_ID_AUTOPILOT1;

const uint8_t heli_mav_sysid = 0;
const uint8_t heli_mav_compid = MAV_COMP_ID_USER52;

const uint8_t mp_mav_sysid = 255;
const uint8_t mp_mav_compid = MAV_COMP_ID_MISSIONPLANNER;

const uint8_t bb_mav_sysid = 0;
const uint8_t bb_mav_compid = MAV_COMP_ID_USER3;

const uint8_t ins_mav_sysid = 0;
const uint8_t ins_mav_compid = MAV_COMP_ID_USER4;

const uint8_t pwr_man_mav_sysid = 0;
const uint8_t pwr_man_mav_compid = MAV_COMP_ID_USER2;

const uint8_t act_master_mav_sysid = 0;
const uint8_t act_master_mav_compid = MAV_COMP_ID_USER5;

const uint8_t ins_cots_mav_sysid = 0;
const uint8_t ins_cots_mav_compid = MAV_COMP_ID_USER14;

const uint8_t batt1_mav_sysid = 0;
const uint8_t batt1_mav_compid = MAV_COMP_ID_BATTERY;

const uint8_t batt2_mav_sysid = 0;
const uint8_t batt2_mav_compid = MAV_COMP_ID_BATTERY2;

static const mav_track_t mav_gw_gnd_tracks[MAV_GW_GND_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_HEARTBEAT, .sysid = gw_gnd_mav_sysid, .compid = gw_gnd_mav_compid},
	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG, .sysid = heli_mav_sysid, .compid = heli_mav_compid},
	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG, .sysid = mp_mav_sysid, .compid = mp_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid},
	{.msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid},
	{.msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid}};

static const mav_track_t mav_heli_tracks[MAV_HELI_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG, .sysid = heli_mav_sysid, .compid = heli_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid}};

#if MAVLINK_OR_CYPHAL

static const mav_track_t mav_pwr_man_tracks[MAV_PWR_MAN_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS, .sysid = batt1_mav_sysid, .compid = batt1_mav_compid},
	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS, .sysid = batt2_mav_sysid, .compid = batt2_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA,
	 .sysid = pwr_man_mav_sysid,
	 .compid = pwr_man_mav_compid}};

static const mav_track_t mav_ins_tracks[MAV_INS_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_SCALED_IMU, .sysid = ins_mav_sysid, .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_SCALED_IMU2, .sysid = ins_mav_sysid, .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_SCALED_PRESSURE, .sysid = ins_mav_sysid, .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_ALTITUDE, .sysid = ins_mav_sysid, .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_ATTITUDE, .sysid = ins_mav_sysid, .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION, .sysid = ins_mav_sysid, .compid = ins_mav_compid},
	{.msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, .sysid = mp_mav_sysid, .compid = mp_mav_compid}};

static const mav_track_t mav_act_master_tracks[MAV_ACT_MASTER_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_COMMAND_ACK,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid},
	{.msgid = MAVLINK_MSG_ID_HEARTBEAT,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid}};

static const mav_track_t mav_ins_cots_tracks[MAV_INS_COTS_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid},
	{.msgid = MAVLINK_MSG_ID_ATTITUDE, .sysid = ins_cots_mav_sysid, .compid = ins_cots_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid}};

static const mav_track_t mav_black_box_tracks[MAV_BLACK_BOX_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC,
	 .sysid = bb_mav_sysid,
	 .compid = bb_mav_compid},
	{.msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
	 .sysid = bb_mav_sysid,
	 .compid = bb_mav_compid},
	{.msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION, .sysid = bb_mav_sysid, .compid = bb_mav_compid}};

#endif /* MAVLINK_OR_CYPHAL */

mav_t mav_gw_gnd_handle;

#if MAVLINK_OR_CYPHAL

static mav_t mav_pwr_man_handle;
static mav_t mav_black_box_handle;
static mav_t mav_ins_handle;
static mav_t mav_act_master_handle __attribute__((section("AHBSRAM0")));
static mav_t mav_ins_cots_handle __attribute__((section("AHBSRAM0")));

#endif /* MAVLINK_OR_CYPHAL */

static mav_link_t mav_link_gw_gnd_udp __attribute__((section("AHBSRAM0")));
// static mav_link_t mav_link_gw_gnd_uart __attribute__((section("AHBSRAM0")));

#if MAVLINK_OR_CYPHAL

static mav_link_t mav_link_pwr_man_udp __attribute__((section("AHBSRAM0")));
static mav_link_t mav_link_black_box_udp __attribute__((section("AHBSRAM0")));
static mav_link_t mav_link_ins_udp __attribute__((section("AHBSRAM0")));
static mav_link_t mav_link_act_master_udp __attribute__((section("AHBSRAM0")));
static mav_link_t mav_link_ins_cots_udp __attribute__((section("AHBSRAM0")));

#endif /* MAVLINK_OR_CYPHAL */

/* MAVLink Component Information Basic defines */
const char vendor_name[] = VENDOR_NAME;
const char model_name[] = MODEL_NAME;
const char software_version[] = SOFTWARE_VERSION;
const char hardware_version[] = HARDWARE_VERSION;
const char serial_number[] = SERIAL_NUMBER;

#if !MAVLINK_OR_CYPHAL
/* CANARD DATA STRUCTURES */
struct CanardInstance canard __attribute__((section("AHBSRAM0")));
struct CanardTxQueue tx_queue __attribute__((section("AHBSRAM0")));

/* Canard Transfer ID */
/* static CanardTransferID heartbeat_tid = 0; */ /* Unused */
static CanardTransferID tid_command_long = 0;
static CanardTransferID tid_ftp = 0;
static CanardTransferID tid_node_mode_1_0 = 0;
static CanardTransferID tid_common_command_long_1_0 = 0;
static CanardTransferID tid_lisum_manual_ctrl_hornet_1_0 = 0;

/* Canard Transfer ID */
// static struct CanardRxSubscription sub_heartbeat;
// static struct CanardRxSubscription sub_getinfo_response_bb;
// static struct CanardRxSubscription sub_getinfo_response_ins;

static struct CanardRxSubscription sub_integer8_array_1_0 __attribute__((section("AHBSRAM0")));
static struct CanardRxSubscription sub_node_mode_1_0 __attribute__((section("AHBSRAM0")));
static struct CanardRxSubscription sub_common_autopilot_version_1_0;

/* INS IMU */
static struct CanardRxSubscription sub_scaled_imu;

/* INS GNSS */
static struct CanardRxSubscription sub_lisum_gnss_recv_data;

/* INS Fusion */
static struct CanardRxSubscription sub_attitude;

/* INS Baro */
static struct CanardRxSubscription sub_scaled_pressure;
static struct CanardRxSubscription sub_altitude;

/* INS ANPP */
// static struct CanardRxSubscription sub_lisum_sensor_airspeed_data;

/* Task BMS */
static struct CanardRxSubscription sub_common_battery_status;

/* Task Motor */
static struct CanardRxSubscription sub_lisum_power_motor_scaled_data;

/* Task Actuator*/
// static struct CanardRxSubscription sub_lisum_manual_ctrl_hornet;
static struct CanardRxSubscription sub_lisum_power_hornet_act_data;
static struct CanardRxSubscription sub_common_command_ack;
static struct CanardRxSubscription sub_minimal_heartbeat;
static struct CanardRxSubscription sub_common_component_information_basic_1_0;

cyphal_subscription_messages_t cyphal_subscriptions[CYPHAL_SUBSCRIPTION_MESSAGES_COUNT_GW_SKY] = {

	/* Actuator */
	// {.kind = CanardTransferKindMessage,
	//  .port_id = messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_FIXED_PORT_ID_,
	//  .serialization_buffer_size =
	// 	 messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	//  .subscription = &sub_lisum_manual_ctrl_hornet},

	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_lisum_power_hornet_act_data},

	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_CommandAck_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_CommandAck_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_common_command_ack},

	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_minimal_Heartbeat_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_minimal_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_minimal_heartbeat},

	/* INS IMU */
	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_ScaledImu_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_ScaledImu_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_scaled_imu},

	/* INS GNSS */
	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_lisum_gnss_recv_data},

	/* INS Fusion */
	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_Attitude_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_Attitude_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_attitude},

	/* INS Baro */
	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_ScaledPressure_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_ScaledPressure_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_scaled_pressure},

	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_Altitude_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_Altitude_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_altitude},

	/* INS ANPP */
	// {.kind = CanardTransferKindMessage,
	//  .port_id = messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0_FIXED_PORT_ID_,
	//  .serialization_buffer_size =
	// 	 messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	//  .subscription = &sub_lisum_sensor_airspeed_data},

	/* INS BMS */
	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_BatteryStatus_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_BatteryStatus_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_common_battery_status},

	/* INS Motor */
	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_lisum_power_motor_scaled_data},

	/* GW SKY */
	{.kind = CanardTransferKindMessage,
	 .port_id = uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_integer8_array_1_0},

	// {.kind = CanardTransferKindMessage,
	//  .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
	//  .serialization_buffer_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	//  .subscription = &sub_heartbeat},

	{.kind = CanardTransferKindMessage,
	 .port_id = uavcan_node_Mode_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size = uavcan_node_Mode_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_node_mode_1_0},

	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_AutopilotVersion_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_common_autopilot_version_1_0},

	{.kind = CanardTransferKindMessage,
	 .port_id = messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_FIXED_PORT_ID_,
	 .serialization_buffer_size =
		 messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
	 .subscription = &sub_common_component_information_basic_1_0},

	// {.kind = CanardTransferKindResponse,
	//  .port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
	//  .serialization_buffer_size = 73,
	//  .subscription = &sub_getinfo_response_bb},

	// {.kind = CanardTransferKindResponse,
	//  .port_id = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
	//  .serialization_buffer_size = 96,
	//  .subscription = &sub_getinfo_response_ins}

};

extern const uint8_t CYPHAL_NODES_ID_ARRAY[CYPHAL_NODES_IDX_MAX];
cyphal_nodes_ids_t cyphal_nodes_ids;

#endif /* MAVLINK_OR_CYPHAL */

const uint32_t crc32_table[256] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
	0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
	0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
	0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
	0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
	0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
	0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
	0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
	0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
	0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
	0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
	0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
	0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
	0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
	0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
	0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
	0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
	0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
	0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
	0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

extern volatile UAVCAN_NODE_MODE current_node_mode;
static bl_t bl;
messages_cyphal_uavcan_common_CommandLong_1_0 cyphal_data;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* MAVLink gateway ground receive callback */
static void mav_gw_gnd_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

#if MAVLINK_OR_CYPHAL
/* MAVLink power management receive callback */
static void mav_pwr_man_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

/* MAVLink ins receive callback */
static void mav_ins_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

/* MAVLink actuator master receive callback */
static void mav_act_master_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

static void mav_ins_cots_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

/* MAVLink black_box receive callback */
static void mav_black_box_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg);

#endif /* MAVLINK_OR_CYPHAL */

// /* heartbeat timer callback */
static void timer_mav_hb_cb(TimerHandle_t xTimer);

#if !MAVLINK_OR_CYPHAL
static void send_ftp_cyphal(const mavlink_file_transfer_protocol_t* ftp, uint8_t* payload,
							size_t payload_size)
{
	uavcan_primitive_array_Integer8_1_0 ftp_array;
	memset(&ftp_array, 0, sizeof(ftp_array));

	ftp_array.value.elements[0] = ftp->target_network;
	ftp_array.value.elements[1] = ftp->target_system;
	ftp_array.value.elements[2] = ftp->target_component;

	memcpy(&ftp_array.value.elements[3], payload, payload_size);
	ftp_array.value.count = payload_size + 3;

	uint8_t buf[uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
	size_t buf_sz = sizeof(buf);

	if (uavcan_primitive_array_Integer8_1_0_serialize_(&ftp_array, buf, &buf_sz) >= 0) {
		(void)cyphal_publish_primitive_array_integer8_subject(
			&canard, &tx_queue, CanardPriorityExceptional,
			uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_, buf, buf_sz, &tid_ftp, 1000000U);
	}
	cyphal_drain_tx_queue(&canard, &tx_queue, 64);
}
#endif /* !MAVLINK_OR_CYPHAL */

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_mav(void* arg)
{
	QueueSetMemberHandle_t queue_member = NULL;
	mavlink_message_t msg;
	TimerHandle_t timer_mav_hb;
	mavlink_file_transfer_protocol_t ftp_data_test;

	/*--------------------------------- MAV ----------------------------------*/
	/* initialize MAVLink handle */
	mav_init(&mav_gw_gnd_handle, dev_mav_sysid, dev_mav_compid);

#if MAVLINK_OR_CYPHAL

	mav_init(&mav_pwr_man_handle, dev_mav_sysid, dev_mav_compid);

	mav_init(&mav_black_box_handle, dev_mav_sysid, dev_mav_compid);

	mav_init(&mav_ins_handle, dev_mav_sysid, dev_mav_compid);

	mav_init(&mav_act_master_handle, dev_mav_sysid, dev_mav_compid);

	mav_init(&mav_ins_cots_handle, dev_mav_sysid, dev_mav_compid);

#endif /* MAVLINK_OR_CYPHAL */

	/* initialize MAVLink UDP links */
	mav_link_init(&mav_link_gw_gnd_udp, MAVLINK_COMM_0, mav_gw_gnd_recv_cb, NULL,
				  (tl_t*)&mav_gw_gnd_udp_tl);

	// mav_link_init(&mav_link_gw_gnd_uart, MAVLINK_COMM_1, mav_gw_gnd_recv_cb, NULL,
	// 			  (tl_t*)&mav_gw_gnd_uart_tl);

#if MAVLINK_OR_CYPHAL

	mav_link_init(&mav_link_pwr_man_udp, MAVLINK_COMM_2, mav_pwr_man_recv_cb, NULL,
				  (tl_t*)&mav_pwr_man_tl);

	mav_link_init(&mav_link_black_box_udp, MAVLINK_COMM_2, mav_black_box_recv_cb, NULL,
				  (tl_t*)&mav_black_box_tl);

	mav_link_init(&mav_link_ins_udp, MAVLINK_COMM_2, mav_ins_recv_cb, NULL, (tl_t*)&mav_ins_tl);

	mav_link_init(&mav_link_act_master_udp, MAVLINK_COMM_2, mav_act_master_recv_cb, NULL,
				  (tl_t*)&mav_act_master_tl);

	mav_link_init(&mav_link_ins_cots_udp, MAVLINK_COMM_2, mav_ins_cots_recv_cb, NULL,
				  (tl_t*)&mav_ins_cots_tl);

#endif /* MAVLINK_OR_CYPHAL */

	/* connect MAVLink links to MAVLink handles */
	mav_link(&mav_gw_gnd_handle, &mav_link_gw_gnd_udp);

	// mav_link(&mav_gw_gnd_handle, &mav_link_gw_gnd_uart);

#if MAVLINK_OR_CYPHAL

	mav_link(&mav_pwr_man_handle, &mav_link_pwr_man_udp);

	mav_link(&mav_black_box_handle, &mav_link_black_box_udp);

	mav_link(&mav_ins_handle, &mav_link_ins_udp);

	mav_link(&mav_act_master_handle, &mav_link_act_master_udp);

	mav_link(&mav_ins_cots_handle, &mav_link_ins_cots_udp);

#endif /* MAVLINK_OR_CYPHAL */

	/* configure signing */
	mav_sign(&mav_gw_gnd_handle, mav_key, NULL);

	/* track gateway ground messages */
	for (uint8_t i = 0; i < MAV_GW_GND_TRACK_COUNT; i++) {
		mav_track_t* track;

		track = (mav_track_t*)&mav_gw_gnd_tracks[i];
		mav_track(&mav_gw_gnd_handle, track->msgid, track->sysid, track->compid);
	}

	/* track heli messages */
	for (uint8_t i = 0; i < MAV_HELI_TRACK_COUNT; i++) {
		mav_track_t* track;

		track = (mav_track_t*)&mav_heli_tracks[i];
		mav_track(&mav_gw_gnd_handle, track->msgid, track->sysid, track->compid);
	}

#if MAVLINK_OR_CYPHAL

	/* track ins messages */
	for (uint8_t i = 0; i < MAV_INS_TRACK_COUNT; i++) {
		mav_track_t* track;

		track = (mav_track_t*)&mav_ins_tracks[i];
		mav_track(&mav_ins_handle, track->msgid, track->sysid, track->compid);
	}

	/* track power management messages */
	for (uint8_t i = 0; i < MAV_PWR_MAN_TRACK_COUNT; i++) {
		mav_track_t* track;

		track = (mav_track_t*)&mav_pwr_man_tracks[i];
		mav_track(&mav_pwr_man_handle, track->msgid, track->sysid, track->compid);
	}

	/* track actuator master messages */
	for (uint8_t i = 0; i < MAV_ACT_MASTER_TRACK_COUNT; i++) {
		mav_track_t* track;

		track = (mav_track_t*)&mav_act_master_tracks[i];
		mav_track(&mav_act_master_handle, track->msgid, track->sysid, track->compid);
	}

	/* track ins cots messages */
	for (uint8_t i = 0; i < MAV_INS_COTS_TRACK_COUNT; i++) {
		mav_track_t* track;

		track = (mav_track_t*)&mav_ins_cots_tracks[i];
		mav_track(&mav_ins_cots_handle, track->msgid, track->sysid, track->compid);
	}

	/* track black box messages */
	for (uint8_t i = 0; i < MAV_BLACK_BOX_TRACK_COUNT; i++) {
		mav_track_t* track;

		track = (mav_track_t*)&mav_black_box_tracks[i];
		mav_track(&mav_black_box_handle, track->msgid, track->sysid, track->compid);
	}

#endif /* MAVLINK_OR_CYPHAL */

#if !MAVLINK_OR_CYPHAL

	/* Initialize deterministic memory pools and Cyphal */
	cyphal_pool_init();

	canard = canardInit(cyphal_instance_memory);
	canard.node_id = CYPHAL_GW_SKY_ID; /* Local node-ID (adjust as needed) */

	/* TX queue for one CAN interface (classic CAN MTU=8). Keep capacity moderate to fit RAM. */
	tx_queue = canardTxInit(512, CANARD_MTU_CAN_CLASSIC, cyphal_txq_memory);

	/* ------------ Subscribe to all messages that we will track ----------- */

	for (uint8_t i = 0; i < CYPHAL_SUBSCRIPTION_MESSAGES_COUNT_GW_SKY; i++) {
		(void)canardRxSubscribe(
			&canard, cyphal_subscriptions[i].kind, cyphal_subscriptions[i].port_id,
			cyphal_subscriptions[i].serialization_buffer_size,
			CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, cyphal_subscriptions[i].subscription);
	}

	/* Initialize Cyphal node ID whitelist */
	for (uint8_t i = 0; i < CYPHAL_NODES_IDX_MAX; i++) {
		cyphal_nodes_ids.uavcan_node_id[i] = CYPHAL_NODES_ID_ARRAY[i];
	}

#endif /* MAVLINK_OR_CYPHAL */

	bl_process_update(&bl, &ftp_data_test);
	/* create MAVLink heartbeat timer */
	timer_mav_hb =
		xTimerCreate("tim_mav_hb", pdMS_TO_TICKS(MAV_HB_PERIOD_MS), pdTRUE, NULL, timer_mav_hb_cb);

	/* start heartbeat timer */
	xTimerStart(timer_mav_hb, 0);

	current_node_mode = UAVCAN_NODE_MODE_OPERATIONAL;

	for (;;) {
		queue_member = xQueueSelectFromSet(queueset_mav, portMAX_DELAY);

		TickType_t now = xTaskGetTickCount();

		if (queue_member == queue_mav_hb) {

			mavlink_heartbeat_t data;

			xQueueReceive(queue_mav_hb, &data, portMAX_DELAY);

			xSemaphoreTake(mutex_mav, portMAX_DELAY);

			mavlink_msg_heartbeat_encode_chan(dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &msg,
											  &data);

			mav_send(&mav_gw_gnd_handle, &msg);

			xSemaphoreGive(mutex_mav);
		}
	}
}

static void mav_gw_gnd_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{
	mavlink_message_t tx_msg;

	xSemaphoreTake(mutex_mav, portMAX_DELAY);

	switch (current_node_mode) {

	/* ======================== OPERATIONAL =========================== */
	case UAVCAN_NODE_MODE_OPERATIONAL: {

		switch (msg->msgid) {
		// case MAVLINK_MSG_ID_HEARTBEAT: {
		// 	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
		// 						 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));

		// 	break;
		// }
		case MAVLINK_MSG_ID_COMMAND_LONG: {

			mavlink_command_long_t data;

			mavlink_msg_command_long_decode(msg, &data);

			if (data.command == MAV_CMD_REQUEST_MESSAGE) {

				if ((uint16_t)data.param1 == MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC) {

					mavlink_message_t reply_msg;

					mavlink_component_information_basic_t reply_info;

					reply_info = make_comp_info_basic();

					mavlink_msg_component_information_basic_encode_chan(
						dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &reply_msg, &reply_info);

					/* Respond, then forward to gnd answer */
					mav_send(&mav_gw_gnd_handle, &reply_msg);

#if MAVLINK_OR_CYPHAL

					/* Then broadcast message to all the same request */
					mavlink_msg_command_long_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_2,
														 &tx_msg, &data);

					mav_send(&mav_pwr_man_handle, &tx_msg);
					mav_send(&mav_black_box_handle, &tx_msg);
					mav_send(&mav_ins_handle, &tx_msg);
					mav_send(&mav_act_master_handle, &tx_msg);
					mav_send(&mav_ins_cots_handle, &tx_msg);

#else

					messages_cyphal_uavcan_common_CommandLong_1_0 command_long;
					memcpy(&command_long, &data, sizeof(command_long));

					(void)cyphal_publish_common_command_long(
						&canard, &tx_queue, CanardPriorityExceptional, &command_long,
						sizeof(command_long), &tid_command_long, 1000000U);

					/* Drain the first request now, schedule the second later to
					 * space transmissions */
					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

#endif /* MAVLINK_OR_CYPHAL */

					break;
				} else if ((uint16_t)data.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {

					mavlink_message_t reply_msg;

					mavlink_autopilot_version_t reply_info;

					reply_info = make_autopilot_version();

					mavlink_msg_autopilot_version_encode_chan(
						dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &reply_msg, &reply_info);

					mav_send(&mav_gw_gnd_handle, &reply_msg);

#if MAVLINK_OR_CYPHAL
					mavlink_msg_command_long_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_2,
														 &tx_msg, &data);

					mav_send(&mav_black_box_handle, &tx_msg);
					mav_send(&mav_ins_handle, &tx_msg);

					if (data.target_system == act_master_mav_sysid &&
						data.target_component == act_master_mav_compid)
						mav_send(&mav_act_master_handle, &tx_msg);

#else

					memset(&cyphal_data, 0, sizeof(cyphal_data));
					memcpy(&cyphal_data, &data, sizeof(cyphal_data));

					uint8_t buf
						[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

					size_t buf_size =
						messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
						&cyphal_data, buf, &buf_size);
					if (rc >= 0) {
						/* Use actual serialized size and original request metadata.
						 */
						cyphal_publish_common_command_long(&canard, &tx_queue,
														   CanardPriorityExceptional, buf, buf_size,
														   &tid_common_command_long_1_0, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

#endif /* MAVLINK_OR_CYPHAL */

					break;
				}
			} else if (data.command == MAV_CMD_DO_SET_MODE) {
				if (data.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {

					switch ((uint8_t)data.param2) {
					case 0:
						current_node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
						break;
					case 1:
						current_node_mode = UAVCAN_NODE_MODE_INITIALIZATION;
						break;
					case 2:
						current_node_mode = UAVCAN_NODE_MODE_MAINTENANCE;
						break;
					case 3:
						current_node_mode = UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
						break;
					default:
						break;
					}

					memcpy(&cyphal_data, &data, sizeof(cyphal_data));

					uint8_t buf
						[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
					size_t buf_size =
						messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
						&cyphal_data, buf, &buf_size);

					if (rc >= 0) {
						/* Use actual serialized size and original request metadata.
						 */
						cyphal_publish_common_command_long(&canard, &tx_queue,
														   CanardPriorityExceptional, buf, buf_size,
														   &tid_common_command_long_1_0, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

					break;
				}
			} else if (data.command == MAV_CMD_HORNET_INIT ||
					   data.command == MAV_CMD_HORNET_ARM_DISARM ||
					   data.command == MAV_CMD_HORNET_FAULT_RESET ||
					   data.command == MAV_CMD_HORNET_SWEEP ||
					   data.command == MAV_CMD_HORNET_HOMING) {

				memset(&cyphal_data, 0, sizeof(cyphal_data));
				memcpy(&cyphal_data, &data, sizeof(cyphal_data));

				uint8_t buf
					[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
				size_t buf_size =
					messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

				int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
					&cyphal_data, buf, &buf_size);

				if (rc >= 0) {
					/* Use actual serialized size and original request metadata.
					 */
					cyphal_publish_common_command_long(&canard, &tx_queue,
													   CanardPriorityExceptional, buf, buf_size,
													   &tid_common_command_long_1_0, 1000000U);
				}

				cyphal_drain_tx_queue(&canard, &tx_queue, 64);

				break;
			}

			break;
		}

		case MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET: {
			/* route lisum manual control hornet */
			mavlink_lisum_manual_ctrl_hornet_t data;

			mavlink_msg_lisum_manual_ctrl_hornet_decode(msg, &data);

#if MAVLINK_OR_CYPHAL

			mavlink_msg_lisum_manual_ctrl_hornet_encode_chan(msg->sysid, msg->compid,
															 MAVLINK_COMM_0, &tx_msg, &data);

			mav_send(&mav_act_master_handle, &tx_msg);

#else

			messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0 cyphal_data;
			memset(&cyphal_data, 0, sizeof(cyphal_data));

			cyphal_data.pos_sp_act1 = data.pos_sp_act1;
			cyphal_data.pos_sp_act2 = data.pos_sp_act2;
			cyphal_data.pos_sp_act3 = data.pos_sp_act3;
			cyphal_data.pos_sp_act4 = data.pos_sp_act4;
			cyphal_data.mode = data.mode;

			uint8_t buf
				[messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

			size_t buf_size =
				messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

			int8_t rc = messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_serialize_(
				&cyphal_data, buf, &buf_size);
			if (rc >= 0) {
				/* Use actual serialized size and original message metadata. */
				cyphal_publish_lisum_lisum_manual_ctrl_hornet_subject(
					&canard, &tx_queue, CanardPriorityImmediate,
					messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_FIXED_PORT_ID_, buf,
					buf_size, &tid_lisum_manual_ctrl_hornet_1_0, 1000000U);
			}

			cyphal_drain_tx_queue(&canard, &tx_queue, 64);

#endif /* MAVLINK_OR_CYPHAL */

			break;
		}

		case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {

			mavlink_file_transfer_protocol_t data;

			mavlink_msg_file_transfer_protocol_decode(msg, &data);

#if MAVLINK_OR_CYPHAL

			mavlink_msg_file_transfer_protocol_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
														   &tx_msg, &data);

			mav_send(&mav_black_box_handle, &tx_msg);

#else

			send_ftp_cyphal(&data, &data.payload, sizeof(data.payload));

#endif /* MAVLINK_OR_CYPHAL */

			break;
		}
		default:
			break;
		}

		break; // case OPERATIONAL
	} // UAVCAN_NODE_MODE_OPERATIONAL

	/* ======================== MAINTENANCE =========================== */
	case UAVCAN_NODE_MODE_MAINTENANCE: {

		switch (msg->msgid) {
			// case MAVLINK_MSG_ID_HEARTBEAT: {
			// 	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
			// 						 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));

			// 	break;
			// }

		case MAVLINK_MSG_ID_COMMAND_LONG: {

			mavlink_command_long_t data;

			mavlink_msg_command_long_decode(msg, &data);

			if (data.command == MAV_CMD_REQUEST_MESSAGE) {

				if ((uint16_t)data.param1 == MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC) {

					mavlink_message_t reply_msg;

					mavlink_component_information_basic_t reply_info;

					reply_info = make_comp_info_basic();

					mavlink_msg_component_information_basic_encode_chan(
						dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &reply_msg, &reply_info);

					/* Respond, then forward to gnd answer */
					mav_send(&mav_gw_gnd_handle, &reply_msg);

#if MAVLINK_OR_CYPHAL
					/* Then broadcast message to all the same request */
					mavlink_msg_command_long_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_2,
														 &tx_msg, &data);

					mav_send(&mav_pwr_man_handle, &tx_msg);
					mav_send(&mav_black_box_handle, &tx_msg);
					mav_send(&mav_ins_handle, &tx_msg);
					mav_send(&mav_act_master_handle, &tx_msg);
					mav_send(&mav_ins_cots_handle, &tx_msg);

#else

					messages_cyphal_uavcan_common_CommandLong_1_0 command_long;
					memcpy(&command_long, &data, sizeof(command_long));

					(void)cyphal_publish_common_command_long(
						&canard, &tx_queue, CanardPriorityNominal, &command_long,
						sizeof(command_long), &tid_command_long, 1000000U);

					/* Drain the first request now, schedule the second later to
					 * space transmissions */
					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

#endif /* MAVLINK_OR_CYPHAL */

					break;
				} else if ((uint16_t)data.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {

					mavlink_message_t reply_msg;

					mavlink_autopilot_version_t reply_info;

					reply_info = make_autopilot_version();

					mavlink_msg_autopilot_version_encode_chan(
						dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &reply_msg, &reply_info);

					mav_send(&mav_gw_gnd_handle, &reply_msg);

#if MAVLINK_OR_CYPHAL
					mavlink_msg_command_long_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_2,
														 &tx_msg, &data);

					mav_send(&mav_black_box_handle, &tx_msg);
					mav_send(&mav_ins_handle, &tx_msg);

					if (data.target_system == act_master_mav_sysid &&
						data.target_component == act_master_mav_compid)
						mav_send(&mav_act_master_handle, &tx_msg);

#else

					memset(&cyphal_data, 0, sizeof(cyphal_data));
					memcpy(&cyphal_data, &data, sizeof(cyphal_data));

					uint8_t buf
						[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

					size_t buf_size =
						messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
						&cyphal_data, buf, &buf_size);
					if (rc >= 0) {
						/* Use actual serialized size and original request metadata.
						 */
						cyphal_publish_common_command_long(&canard, &tx_queue,
														   CanardPriorityNominal, buf, buf_size,
														   &tid_common_command_long_1_0, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

#endif /* MAVLINK_OR_CYPHAL */

					break;
				}
			} else if (data.command == MAV_CMD_DO_SET_MODE) {
				if (data.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {

					uavcan_node_Mode_1_0 cyphal_data;
					memset(&cyphal_data, 0, sizeof(cyphal_data));

					switch ((uint8_t)data.param2) {
					case 0:
						current_node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
						break;
					case 1:
						current_node_mode = UAVCAN_NODE_MODE_INITIALIZATION;
						break;
					case 2:
						current_node_mode = UAVCAN_NODE_MODE_MAINTENANCE;
						break;
					case 3:
						current_node_mode = UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
						break;
					default:
						break;
					}

					memcpy(&cyphal_data, &data, sizeof(cyphal_data));

					uint8_t buf
						[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
					size_t buf_size =
						messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
						&cyphal_data, buf, &buf_size);

					if (rc >= 0) {
						/* Use actual serialized size and original request metadata.
						 */
						cyphal_publish_common_command_long(&canard, &tx_queue,
														   CanardPriorityExceptional, buf, buf_size,
														   &tid_common_command_long_1_0, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

					break;
				}
			} else if (data.command == MAV_CMD_HORNET_INIT ||
					   data.command == MAV_CMD_HORNET_ARM_DISARM ||
					   data.command == MAV_CMD_HORNET_FAULT_RESET ||
					   data.command == MAV_CMD_HORNET_SWEEP ||
					   data.command == MAV_CMD_HORNET_HOMING) {

				memset(&cyphal_data, 0, sizeof(cyphal_data));
				memcpy(&cyphal_data, &data, sizeof(cyphal_data));

				uint8_t buf
					[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
				size_t buf_size =
					messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

				int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
					&cyphal_data, buf, &buf_size);

				if (rc >= 0) {
					/* Use actual serialized size and original request metadata.
					 */
					cyphal_publish_common_command_long(&canard, &tx_queue,
													   CanardPriorityExceptional, buf, buf_size,
													   &tid_common_command_long_1_0, 1000000U);
				}

				cyphal_drain_tx_queue(&canard, &tx_queue, 64);

				break;
			}

			break;
		}

		case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {

			mavlink_file_transfer_protocol_t data;

			mavlink_msg_file_transfer_protocol_decode(msg, &data);

#if MAVLINK_OR_CYPHAL

			mavlink_msg_file_transfer_protocol_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
														   &tx_msg, &data);

			mav_send(&mav_black_box_handle, &tx_msg);

#else

			send_ftp_cyphal(&data, &data.payload, sizeof(data.payload));

#endif /* MAVLINK_OR_CYPHAL */

			break;
		}
		default:
			break;
		}

		break; // case MAINTENANCE
	} // UAVCAN_NODE_MODE_MAINTENANCE

	/* ======================== SOFTWARE UPDATE =========================== */
	case UAVCAN_NODE_MODE_SOFTWARE_UPDATE: {

		switch (msg->msgid) {
			// case MAVLINK_MSG_ID_HEARTBEAT: {
			// 	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
			// 						 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));

			// 	break;
			// }

		case MAVLINK_MSG_ID_COMMAND_LONG: {

			mavlink_command_long_t data;

			mavlink_msg_command_long_decode(msg, &data);

			if (data.command == MAV_CMD_REQUEST_MESSAGE) {

				if ((uint16_t)data.param1 == MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC) {

					mavlink_message_t reply_msg;

					mavlink_component_information_basic_t reply_info;

					reply_info = make_comp_info_basic();

					mavlink_msg_component_information_basic_encode_chan(
						dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &reply_msg, &reply_info);

					/* Respond, then forward to gnd answer */
					mav_send(&mav_gw_gnd_handle, &reply_msg);

#if MAVLINK_OR_CYPHAL
					/* Then broadcast message to all the same request */
					mavlink_msg_command_long_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_2,
														 &tx_msg, &data);

					mav_send(&mav_pwr_man_handle, &tx_msg);
					mav_send(&mav_black_box_handle, &tx_msg);
					mav_send(&mav_ins_handle, &tx_msg);
					mav_send(&mav_act_master_handle, &tx_msg);
					mav_send(&mav_ins_cots_handle, &tx_msg);

#else

					memcpy(&cyphal_data, &data, sizeof(cyphal_data));

					uint8_t buf
						[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
					size_t buf_size =
						messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
						&cyphal_data, buf, &buf_size);

					if (rc >= 0) {
						/* Use actual serialized size and original request metadata.
						 */
						cyphal_publish_common_command_long(&canard, &tx_queue,
														   CanardPriorityExceptional, buf, buf_size,
														   &tid_common_command_long_1_0, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

#endif /* MAVLINK_OR_CYPHAL */

					break;
				} else if ((uint16_t)data.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {

					mavlink_message_t reply_msg;

					mavlink_autopilot_version_t reply_info;

					reply_info = make_autopilot_version();

					mavlink_msg_autopilot_version_encode_chan(
						dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &reply_msg, &reply_info);

					mav_send(&mav_gw_gnd_handle, &reply_msg);

#if MAVLINK_OR_CYPHAL
					mavlink_msg_command_long_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_2,
														 &tx_msg, &data);

					mav_send(&mav_black_box_handle, &tx_msg);
					mav_send(&mav_ins_handle, &tx_msg);

					if (data.target_system == act_master_mav_sysid &&
						data.target_component == act_master_mav_compid)
						mav_send(&mav_act_master_handle, &tx_msg);

#else

					memset(&cyphal_data, 0, sizeof(cyphal_data));
					memcpy(&cyphal_data, &data, sizeof(cyphal_data));

					uint8_t buf
						[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

					size_t buf_size =
						messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
						&cyphal_data, buf, &buf_size);
					if (rc >= 0) {
						/* Use actual serialized size and original request metadata.
						 */
						cyphal_publish_common_command_long(&canard, &tx_queue,
														   CanardPriorityNominal, buf, buf_size,
														   &tid_common_command_long_1_0, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

#endif /* MAVLINK_OR_CYPHAL */

					break;
				}
			} else if (data.command == MAV_CMD_DO_SET_MODE) {
				if (data.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {

					switch ((uint8_t)data.param2) {
					case 0:
						current_node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
						break;
					case 1:
						current_node_mode = UAVCAN_NODE_MODE_INITIALIZATION;
						break;
					case 2:
						current_node_mode = UAVCAN_NODE_MODE_MAINTENANCE;
						break;
					case 3:
						current_node_mode = UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
						break;
					default:
						break;
					}

					memcpy(&cyphal_data, &data, sizeof(cyphal_data));

					uint8_t buf
						[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
					size_t buf_size =
						messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
						&cyphal_data, buf, &buf_size);

					if (rc >= 0) {
						/* Use actual serialized size and original request metadata.
						 */
						cyphal_publish_common_command_long(&canard, &tx_queue,
														   CanardPriorityExceptional, buf, buf_size,
														   &tid_common_command_long_1_0, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);

					break;
				}
			} else if (data.command == MAV_CMD_HORNET_INIT ||
					   data.command == MAV_CMD_HORNET_ARM_DISARM ||
					   data.command == MAV_CMD_HORNET_FAULT_RESET ||
					   data.command == MAV_CMD_HORNET_SWEEP ||
					   data.command == MAV_CMD_HORNET_HOMING) {

				memset(&cyphal_data, 0, sizeof(cyphal_data));
				memcpy(&cyphal_data, &data, sizeof(cyphal_data));

				uint8_t buf
					[messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
				size_t buf_size =
					messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

				int8_t rc = messages_cyphal_uavcan_common_CommandLong_1_0_serialize_(
					&cyphal_data, buf, &buf_size);

				if (rc >= 0) {
					/* Use actual serialized size and original request metadata.
					 */
					cyphal_publish_common_command_long(&canard, &tx_queue,
													   CanardPriorityExceptional, buf, buf_size,
													   &tid_common_command_long_1_0, 1000000U);
				}

				cyphal_drain_tx_queue(&canard, &tx_queue, 64);

				break;
			}

			break;
		}

		case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {

			mavlink_file_transfer_protocol_t data;

			mavlink_msg_file_transfer_protocol_decode(msg, &data);

			if (data.target_component == dev_mav_compid) {

				bl_process_mav_ftp(&bl, &data);

				mavlink_msg_file_transfer_protocol_encode_chan(dev_mav_sysid, dev_mav_compid,
															   MAVLINK_COMM_0, &tx_msg, &data);
				mav_send(&mav_gw_gnd_handle, &tx_msg);

				if (bl.flags & BL_FLAG_REBOOT) {
					for (volatile uint32_t i = 0; i < 1000000; i++)
						;
					HAL_NVIC_SystemReset();
				}

				break;
			}

			else {
#if MAVLINK_OR_CYPHAL

				mavlink_msg_file_transfer_protocol_encode_chan(msg->sysid, msg->compid,
															   MAVLINK_COMM_0, &tx_msg, &data);

				mav_send(&mav_black_box_handle, &tx_msg);

#else

				send_ftp_cyphal(&data, &data.payload, sizeof(data.payload));

#endif /* MAVLINK_OR_CYPHAL */

				break;
			}
		}

		default:
			break;
		}

		break; // case SOFTWARE_UPDATE
	} // UAVCAN_NODE_MODE_SOFTWARE_UPDATE

	} // switch (current_node_mode)
	xSemaphoreGive(mutex_mav);
}

#if MAVLINK_OR_CYPHAL

static void mav_pwr_man_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{
	mavlink_message_t tx_msg;

	xSemaphoreTake(mutex_mav, portMAX_DELAY);
	/* route message to black box */
	mav_send(&mav_black_box_handle, msg);

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_BATTERY_STATUS: {
		/* route battery status */
		mavlink_battery_status_t data;

		mavlink_msg_battery_status_decode(msg, &data);

		mavlink_msg_battery_status_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg,
											   &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA: {
		/* route lisum power motor scaled data */
		mavlink_lisum_power_motor_scaled_data_t data;

		mavlink_msg_lisum_power_motor_scaled_data_decode(msg, &data);

		mavlink_msg_lisum_power_motor_scaled_data_encode_chan(msg->sysid, msg->compid,
															  MAVLINK_COMM_0, &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC: {
		mavlink_component_information_basic_t data;

		mavlink_msg_component_information_basic_decode(msg, &data);

		mavlink_msg_component_information_basic_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
															&tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	default:
		break;
	}

	xSemaphoreGive(mutex_mav);
}

static void mav_ins_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{
	mavlink_message_t tx_msg;

	xSemaphoreTake(mutex_mav, portMAX_DELAY);
	/* route message to black box */
	mav_send(&mav_black_box_handle, msg);

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_SCALED_IMU: {
		/* route scaled imu */
		mavlink_scaled_imu_t data;

		mavlink_msg_scaled_imu_decode(msg, &data);

		mavlink_msg_scaled_imu_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_SCALED_IMU2: {
		/* route scaled imu2 */
		mavlink_scaled_imu2_t data;

		mavlink_msg_scaled_imu2_decode(msg, &data);

		mavlink_msg_scaled_imu2_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg,
											&data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_SCALED_PRESSURE: {
		/* route scaled pressure */
		mavlink_scaled_pressure_t data;

		mavlink_msg_scaled_pressure_decode(msg, &data);

		mavlink_msg_scaled_pressure_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg,
												&data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_ALTITUDE: {
		/* route altitude */
		mavlink_altitude_t data;

		mavlink_msg_altitude_decode(msg, &data);

		mavlink_msg_altitude_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_ATTITUDE: {
		/* route attitude */
		mavlink_attitude_t data;

		mavlink_msg_attitude_decode(msg, &data);

		mavlink_msg_attitude_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA: {
		/* route lisum gnss recv data */
		mavlink_lisum_gnss_recv_data_t data;

		mavlink_msg_lisum_gnss_recv_data_decode(msg, &data);

		mavlink_msg_lisum_gnss_recv_data_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
													 &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA: {
		/* route lisum sensor airspeed data */
		mavlink_lisum_sensor_airspeed_data_t data;

		mavlink_msg_lisum_sensor_airspeed_data_decode(msg, &data);

		mavlink_msg_lisum_sensor_airspeed_data_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
														   &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC: {
		mavlink_component_information_basic_t data;

		mavlink_msg_component_information_basic_decode(msg, &data);

		mavlink_msg_component_information_basic_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
															&tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
		mavlink_autopilot_version_t data;

		mavlink_msg_autopilot_version_decode(msg, &data);

		mavlink_msg_autopilot_version_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg,
												  &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
		mavlink_named_value_float_t data;

		mavlink_msg_named_value_float_decode(msg, &data);

		mavlink_msg_named_value_float_encode_chan(mp_mav_sysid, mp_mav_compid, MAVLINK_COMM_0,
												  &tx_msg, &data);

		mav_send(&mav_black_box_handle, &tx_msg);
		break;
	}
	default:
		break;
	}

	xSemaphoreGive(mutex_mav);
}

static void mav_act_master_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{
	mavlink_message_t tx_msg;

	xSemaphoreTake(mutex_mav, portMAX_DELAY);
	/* route message to black box */
	mav_send(&mav_black_box_handle, msg);

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA: {
		/* route lisum power hornet actuator data */
		mavlink_lisum_power_hornet_act_data_t data;

		mavlink_msg_lisum_power_hornet_act_data_decode(msg, &data);

		mavlink_msg_lisum_power_hornet_act_data_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
															&tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_COMMAND_ACK: {
		/* route command acknowledge */
		mavlink_command_ack_t data;

		mavlink_msg_command_ack_decode(msg, &data);

		mavlink_msg_command_ack_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg,
											&data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET: {
		/* route lisum manual control hornet */
		mavlink_lisum_manual_ctrl_hornet_t data;

		mavlink_msg_lisum_manual_ctrl_hornet_decode(msg, &data);

		mavlink_msg_lisum_manual_ctrl_hornet_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
														 &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_HEARTBEAT: {
		/* route heartbeat */
		mavlink_heartbeat_t data;

		mavlink_msg_heartbeat_decode(msg, &data);

		mavlink_msg_heartbeat_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC: {
		mavlink_component_information_basic_t data;

		mavlink_msg_component_information_basic_decode(msg, &data);

		mavlink_msg_component_information_basic_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
															&tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	default:
		break;
	}

	xSemaphoreGive(mutex_mav);
}

static void mav_ins_cots_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{
	mavlink_message_t tx_msg;

	xSemaphoreTake(mutex_mav, portMAX_DELAY);
	/* route message to black box */
	mav_send(&mav_black_box_handle, msg);

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_SCALED_IMU: {
		/* route scaled imu */
		mavlink_scaled_imu_t data;

		mavlink_msg_scaled_imu_decode(msg, &data);

		mavlink_msg_scaled_imu_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_ATTITUDE: {
		/* route attitude */
		mavlink_attitude_t data;

		mavlink_msg_attitude_decode(msg, &data);

		mavlink_msg_attitude_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA: {
		/* route lisum gnss recv data */
		mavlink_lisum_gnss_recv_data_t data;

		mavlink_msg_lisum_gnss_recv_data_decode(msg, &data);

		mavlink_msg_lisum_gnss_recv_data_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
													 &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC: {
		mavlink_component_information_basic_t data;

		mavlink_msg_component_information_basic_decode(msg, &data);

		mavlink_msg_component_information_basic_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
															&tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}

	default:
		break;
	}

	xSemaphoreGive(mutex_mav);
}

static void mav_black_box_recv_cb(mav_t* mav, mavlink_message_t* msg, void* arg)
{
	mavlink_message_t tx_msg;

	xSemaphoreTake(mutex_mav, portMAX_DELAY);

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC: {
		mavlink_component_information_basic_t data;

		mavlink_msg_component_information_basic_decode(msg, &data);

		mavlink_msg_component_information_basic_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
															&tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {

		mavlink_file_transfer_protocol_t data;

		mavlink_msg_file_transfer_protocol_decode(msg, &data);

		mavlink_msg_file_transfer_protocol_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0,
													   &tx_msg, &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {

		mavlink_autopilot_version_t data;

		mavlink_msg_autopilot_version_decode(msg, &data);

		mavlink_msg_autopilot_version_encode_chan(msg->sysid, msg->compid, MAVLINK_COMM_0, &tx_msg,
												  &data);

		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}
	default:
		break;
	}
	xSemaphoreGive(mutex_mav);
}

#endif /* !MAVLINK_OR_CYPHAL */

/* ---------------------- MAVLink callback functions ------------------------*/

static void timer_mav_hb_cb(TimerHandle_t xTimer)
{
	MAV_STATE current_node_mode_mav;

	switch (current_node_mode) {
	case UAVCAN_NODE_MODE_OPERATIONAL:
		current_node_mode_mav = MAV_STATE_ACTIVE;
		break;
	case UAVCAN_NODE_MODE_INITIALIZATION:
		current_node_mode_mav = MAV_STATE_BOOT;
		break;
	case UAVCAN_NODE_MODE_MAINTENANCE:
		current_node_mode_mav = MAV_STATE_STANDBY;
		break;
	case UAVCAN_NODE_MODE_SOFTWARE_UPDATE:
		current_node_mode_mav = MAV_STATE_CRITICAL;
		break;
	default:
		break;
	}

	mavlink_heartbeat_t mav_hb = {.type = MAV_TYPE_ONBOARD_CONTROLLER,
								  .autopilot = MAV_AUTOPILOT_GENERIC,
								  .base_mode = 0,
								  .custom_mode = 0,
								  .system_status = current_node_mode_mav};

	xQueueSendToBack(queue_mav_hb, &mav_hb, 0);
}