
/*******************************************************************************
 * Includes (ordered)
 ******************************************************************************/

// Standard C
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Project application
#include "bl.h"
#include "ftp_handler.h"
#include "ftp_helper.h"
#include "main.h"
#include "protocol_utility.h"
#include "task_log.h"
#include "task_work.h"
#include "types.h"

// ESP-IDF core
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_system.h"

// ESP-IDF drivers
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/spi_master.h"
#include "driver/twai.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// External hardware drivers
#include "flash.h"
#include "gd5f2gq5ue.h"
#include "psram_ring.h"

// Libraries
#include "canard.h"
#include "cyphal_mavlink_publishers.h"
#include "cyphal_uavcan_publishers.h"

// Middleware
#include "cyphal_utility.h"
#include "lfs.h"
#include "lfs_def.h"
#include "mav.h"
#include "udp.h"
#include "udp_tl.h"

// ULog formats
#include "ulog.h"
#include "ulog_adsb_vehicle.h"
#include "ulog_ais_vessel.h"
#include "ulog_altitude.h"
#include "ulog_attitude.h"
#include "ulog_battery_status.h"
#include "ulog_def.h"
#include "ulog_lisum_esc_status.h"
#include "ulog_lisum_gnss_recv_data.h"
#include "ulog_lisum_manual_ctrl_hornet.h"
#include "ulog_lisum_power_hornet_act_data.h"
#include "ulog_lisum_power_motor_scaled_data.h"
// #include "ulog_lisum_power_motor_vesc_data.h"
#include "ulog_lisum_propulsion_tb_status.h"
#include "ulog_lisum_sensor_airspeed_data.h"
#include "ulog_named_value_float.h"
#include "ulog_scaled_imu.h"
#include "ulog_scaled_pressure.h"

// Imu
#include "common/Altitude_1_0.h"
#include "common/Attitude_1_0.h"
#include "common/BatteryStatus_1_0.h"
#include "common/ScaledImu_1_0.h"
#include "common/ScaledPressure_1_0.h"
#include "lisum/LisumGnssRecvData_1_0.h"
#include "lisum/LisumPowerMotorScaledData_1_0.h"
#include "lisum/LisumSensorAirspeedData_1_0.h"

#include "common/AutopilotVersion_1_0.h"
#include "common/CommandLong_1_0.h"
#include "common/FileTransferProtocol_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/Mode_1_0.h"
#include "uavcan/primitive/array/Integer8_1_0.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define HEARTBEAT_PERIOD_MS 1000 /* gratuitous ARP period [ms] */

#if MAVLINK_OR_CYPHAL

#define MAV_GW_SKY_TRACK_COUNT 19 /* MAVLink gateway sky track count */
#define MAV_INS_TRACK_COUNT 9	  /* MAVLink Ins track count */
#define MAV_MOTOR_CONTROL_COUNT 5 /* MAVLink motor control track count */
#define MAV_VESC_CONTROL_COUNT 4  /* MAVLink VESC control track count */

#else

#define mutex_lock(mutex)                     \
	{                                         \
		xSemaphoreTake(mutex, portMAX_DELAY); \
	}
#define mutex_unlock(mutex)    \
	{                          \
		xSemaphoreGive(mutex); \
	}

#endif /* MAVLINK_OR_CYPHAL */

#define MAV_PROTOCOL_CAPABILITY_MAVLINK2 ((uint64_t)1 << 0)
#define MAV_PROTOCOL_CAPABILITY_FTP ((uint64_t)1 << 10)

#define CYPHAL_SUBSCRIPTION_MESSAGES_COUNT_BLACKBOX 11

/*******************************************************************************
 * Ulog ID Defines
 ******************************************************************************/

#define PAGE_SIZE_FLASH 2048U /* Flash page flash size in bytes */
#define PAGE_SIZE_SD 512U	  /* Flash page flash size in bytes */
#define LOOKAHEAD_SIZE 256U	  /* LittleFS lookahead buffer size in bytes */

#define TASK_PRIO_LOG 3 /* Task priority */

#define FILE_SYNC_THRESHOLD 100 * 2048 /* Threshold for file sync in bytes */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

// MAVLink information
const uint8_t dev_mav_sysid = 0;
const uint8_t dev_mav_compid = MAV_COMP_ID_USER3;

const uint8_t pwr_man_mav_sysid = 0;
const uint8_t pwr_man_mav_compid = MAV_COMP_ID_USER2;

const uint8_t ins_mav_sysid = 0;
const uint8_t ins_mav_compid = MAV_COMP_ID_USER4;

const uint8_t act_master_mav_sysid = 0;
const uint8_t act_master_mav_compid = MAV_COMP_ID_USER5;

const uint8_t ins_cots_mav_sysid = 0;
const uint8_t ins_cots_mav_compid = MAV_COMP_ID_USER14;

const uint8_t heli_mav_sysid = 0;
const uint8_t heli_mav_compid = MAV_COMP_ID_USER52;

const uint8_t batt1_mav_sysid = 0;
const uint8_t batt1_mav_compid = MAV_COMP_ID_BATTERY;

const uint8_t batt2_mav_sysid = 0;
const uint8_t batt2_mav_compid = MAV_COMP_ID_BATTERY2;

const uint8_t motor_control_mav_sysid = 0;
const uint8_t motor_control_mav_compid = MAV_COMP_ID_USER26;

const uint8_t mp_mav_sysid = 255;
const uint8_t mp_mav_compid = MAV_COMP_ID_MISSIONPLANNER;

const uint8_t vesc_control_mav_sysid = 1;
const uint8_t vesc_control_mav_compid = MAV_COMP_ID_AUTOPILOT1;

#if MAVLINK_OR_CYPHAL

static const mav_track_t mav_gw_sky_tracks[MAV_GW_SKY_TRACK_COUNT] = {

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = batt1_mav_sysid,
	 .compid = batt1_mav_compid},

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = batt2_mav_sysid,
	 .compid = batt2_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA,
	 .sysid = pwr_man_mav_sysid,
	 .compid = pwr_man_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_PRESSURE,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ALTITUDE, .sysid = ins_mav_sysid, .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ATTITUDE, .sysid = ins_mav_sysid, .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET,
	 .sysid = act_master_mav_sysid,
	 .compid = act_master_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ATTITUDE,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA,
	 .sysid = ins_cots_mav_sysid,
	 .compid = ins_cots_mav_compid},

	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid},

	{.msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid},

	{.msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid}};

static const mav_track_t mav_ins_tracks[MAV_INS_TRACK_COUNT] = {

	{.msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_PRESSURE,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ALTITUDE, .sysid = mp_mav_sysid, .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = mp_mav_sysid,
	 .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_BATTERY_STATUS,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU, .sysid = mp_mav_sysid, .compid = mp_mav_compid},

	{.msgid = MAVLINK_MSG_ID_SCALED_IMU,
	 .sysid = ins_mav_sysid,
	 .compid = ins_mav_compid}};

static const mav_track_t mav_motor_control_tracks[MAV_MOTOR_CONTROL_COUNT] = {

	{.msgid = MAVLINK_MSG_ID_LISUM_PROPULSION_TB_STATUS,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid},

	{.msgid = MAVLINK_MSG_ID_LISUM_ESC_STATUS,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid},

	{.msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid},

	{.msgid = MAVLINK_MSG_ID_AIS_VESSEL,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ADSB_VEHICLE,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid}};

static const mav_track_t mav_vesc_control_tracks[MAV_VESC_CONTROL_COUNT] = {

	{.msgid = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_VESC_DATA,
	 .sysid = vesc_control_mav_sysid,
	 .compid = vesc_control_mav_compid},

	{.msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid},

	{.msgid = MAVLINK_MSG_ID_AIS_VESSEL,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid},

	{.msgid = MAVLINK_MSG_ID_ADSB_VEHICLE,
	 .sysid = motor_control_mav_sysid,
	 .compid = motor_control_mav_compid}};

mav_t mav_gw_sky_handle;
mav_t mav_ins_handle;
mav_t mav_motor_control_handle;
mav_t mav_vesc_control_handle;

mav_link_t mav_link_ins_udp;
mav_link_t mav_link_gw_sky_udp;
mav_link_t mav_link_motor_control_udp;
mav_link_t mav_link_vesc_control_udp;

/* UDP Mavlink */
extern udp_t udp;

extern udp_tl_t mav_gw_sky_tl;
extern udp_tl_t mav_ins_tl;
extern udp_tl_t mav_motor_control_tl;
extern udp_tl_t mav_vesc_control_tl;

#else

/* System need this for File Transfer Protocol */
mav_t mav_gw_sky_handle;
mavlink_file_transfer_protocol_t ftp;

/* CANARD DATA STRUCTURES */
struct CanardInstance canard;
struct CanardTxQueue tx_queue;

/* Canard Transfer ID */
CanardTransferID tid_ftp = 0;
static CanardTransferID heartbeat_tid = 0;
static CanardTransferID tid_node_mode_1_0 = 0;
static CanardTransferID tid_autopilot = 0;
static CanardTransferID tid_command_long = 0;

/* Canard Rx Subscriptions */

// Imu
static struct CanardRxSubscription sub_common_scaled_imu_1_0;
static struct CanardRxSubscription sub_common_scaled_pressure_1_0;
static struct CanardRxSubscription sub_common_altitude_1_0;
static struct CanardRxSubscription sub_common_attitude_1_0;
static struct CanardRxSubscription sub_lisum_sensor_airspeed_data_1_0;
static struct CanardRxSubscription sub_lisum_gnss_recv_data_1_0;

static struct CanardRxSubscription sub_common_battery_status_1_0;
static struct CanardRxSubscription sub_lisum_power_motor_scaled_data_1_0;

// Sky
static struct CanardRxSubscription sub_integer8_array_1_0;
static struct CanardRxSubscription sub_node_mode_1_0;
static struct CanardRxSubscription sub_common_command_long_1_0;

/* Cyphal Node IDs */
extern const uint8_t CYPHAL_NODES_ID_ARRAY[CYPHAL_NODES_IDX_MAX];
static cyphal_nodes_ids_t cyphal_nodes_ids;

/* Cyphal Subscriptions Messages */
cyphal_subscription_messages_t
	cyphal_subscriptions[CYPHAL_SUBSCRIPTION_MESSAGES_COUNT_BLACKBOX] = {

		/* -------------------------------- INS -------------------------------*/

		{.kind = CanardTransferKindMessage,
		 .port_id = common_ScaledImu_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 common_ScaledImu_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_common_scaled_imu_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id = common_ScaledPressure_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 common_ScaledPressure_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_common_scaled_pressure_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id = common_Altitude_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 common_Altitude_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_common_altitude_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id = common_Attitude_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 common_Attitude_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_common_attitude_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id = lisum_LisumGnssRecvData_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 lisum_LisumGnssRecvData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_lisum_gnss_recv_data_1_0},

		// {.kind = CanardTransferKindMessage,
		//  .port_id = lisum_LisumGnssRecvData_1_0_FIXED_PORT_ID_,
		//  .serialization_buffer_size =
		// 	 lisum_LisumGnssRecvData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		//  .subscription = &sub_lisum_gnss_recv_data_1_0},

		/* ------------------------------- PWR MEN --------------------------- */
		{.kind = CanardTransferKindMessage,
		 .port_id = common_BatteryStatus_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 common_BatteryStatus_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_common_battery_status_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id = lisum_LisumPowerMotorScaledData_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 lisum_LisumPowerMotorScaledData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_lisum_power_motor_scaled_data_1_0},

		/* -------------------------------- SKY -------------------------------*/

		{.kind = CanardTransferKindMessage,
		 .port_id = common_CommandLong_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_common_command_long_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id = uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_integer8_array_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id = uavcan_node_Mode_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 uavcan_node_Mode_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_node_mode_1_0}};

static uavcan_node_Mode_1_0 node_mode_data;

/* Fallback unique ID if board-specific UID macro/array not provided. */
#ifndef CYPHAL_UID_ARRAY
const uint8_t CYPHAL_UID_ARRAY[16] = {CYPHAL_BLACK_BOX_ID};
#endif

#endif /* MAVLINK_OR_CYPHAL */

/* Time for Node mode */
static volatile TickType_t current_node_mode_time = 0;

/* FSM State of the Node device */
extern volatile node_mode_state_t current_node_mode;

/* Flags for system */
volatile bool mission_planner_connected = false;
volatile bool create_file = false;
uint8_t file_sync = 1;

/* MAVLink Component Information Basic defines */
const char vendor_name[] = VENDOR_NAME;
const char model_name[] = MODEL_NAME;
const char software_version[] = SOFTWARE_VERSION;
const char hardware_version[] = HARDWARE_VERSION;
const char serial_number[] = SERIAL_NUMBER;

static bl_t bl;

/*******************************************************************************
 * FreeRTOS Variables
 ******************************************************************************/

// Semaphores
extern SemaphoreHandle_t mutex_mav_ftp;

// Tasks Handles
extern TaskHandle_t task_work_handle;

/* FTP Sessions */
session_t sessions[FTP_MAX_SESSIONS];
int session_id;

/* Ring Buffer */
extern ring_t ring_buffer;

/* SD Card */
extern sd_t sd;

/* LittleFS File System */
extern lfs_t lfs;
extern lfs_file_t lfs_file;
extern struct lfs_file_config file_cfg;
extern struct lfs_config lfs_cfg;

// LittleFS buffers
extern uint8_t read_buffer;
extern uint8_t prog_buffer;
extern uint8_t lookahead_buffer;
extern uint8_t file_buffer;

extern rmt_item32_t led_data[24 * LED_NUM];

uint64_t imu_cnt = 0;
uint64_t imu_cnt2 = 0;
uint8_t imu_cnt3 = 0;
bool toggle = 0;

/*******************************************************************************
 * Ulog system variables
 ******************************************************************************/
extern ulog_t ulog;
extern uint32_t boot_count;
uint64_t boot_time_ms = 0;

/* Global Status Variables */
esp_err_t ret;
uint32_t err_cnt = 0x00;

volatile uint8_t flag_unsubscribed = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if MAVLINK_OR_CYPHAL

static void mav_gw_sky_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg);

static void mav_ins_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg);

static void mav_motor_control_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg);

static void mav_vesc_control_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg);

void mavlink_ulog_lfs_initialization(void);

#else

void cyphal_task_priority(const node_mode_state_t current_mode);

void cyphal_ulog_lfs_initialization(void);

void print_node_mode(uint8_t mode);

void build_getinfo_response(uavcan_node_GetInfo_Response_1_0 *resp);

#endif /* MAVLINK_OR_CYPHAL */

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_work(void *arg)
{

#if MAVLINK_OR_CYPHAL

	/* initialize MAVLink handle */
	mav_init(&mav_gw_sky_handle, dev_mav_sysid, dev_mav_compid);

	/* initialize MAVLink links */
	mav_link_init(&mav_link_gw_sky_udp, MAVLINK_COMM_2, mav_gw_sky_recv_cb, NULL,
				  (tl_t *)&mav_gw_sky_tl);

	mav_link_init(&mav_link_ins_udp, MAVLINK_COMM_0, mav_ins_recv_cb, NULL,
				  (tl_t *)&mav_ins_tl);

	mav_link_init(&mav_link_motor_control_udp, MAVLINK_COMM_2, mav_motor_control_recv_cb,
				  NULL, (tl_t *)&mav_motor_control_tl);

	mav_link_init(&mav_link_vesc_control_udp, MAVLINK_COMM_2, mav_vesc_control_recv_cb,
				  NULL, (tl_t *)&mav_vesc_control_tl);

	/* connect MAVLink links to MAVLink handles */
	mav_link(&mav_gw_sky_handle, &mav_link_gw_sky_udp);
	mav_link(&mav_ins_handle, &mav_link_ins_udp);
	mav_link(&mav_motor_control_handle, &mav_link_motor_control_udp);
	mav_link(&mav_vesc_control_handle, &mav_link_vesc_control_udp);

	/* track gateway sky messages */
	for (uint8_t i = 0; i < MAV_GW_SKY_TRACK_COUNT; i++)
	{
		mav_track_t *track;

		track = (mav_track_t *)&mav_gw_sky_tracks[i];

		mav_track(&mav_gw_sky_handle, track->msgid, track->sysid, track->compid);
	}

	for (uint8_t i = 0; i < MAV_INS_TRACK_COUNT; i++)
	{
		mav_track_t *track;

		track = (mav_track_t *)&mav_ins_tracks[i];

		mav_track(&mav_ins_handle, track->msgid, track->sysid, track->compid);
	}

	for (uint8_t i = 0; i < MAV_MOTOR_CONTROL_COUNT; i++)
	{
		mav_track_t *track;

		track = (mav_track_t *)&mav_motor_control_tracks[i];

		mav_track(&mav_motor_control_handle, track->msgid, track->sysid, track->compid);
	}

	for (uint8_t i = 0; i < MAV_VESC_CONTROL_COUNT; i++)
	{
		mav_track_t *track;

		track = (mav_track_t *)&mav_vesc_control_tracks[i];

		mav_track(&mav_vesc_control_handle, track->msgid, track->sysid, track->compid);
	}

#else

	/* ---------------------------- Cyphal Init --------------------------- */

	/* Initialize deterministic memory pools and Cyphal */
	cyphal_pool_init();

	/* Canard struct init */
	canard = canardInit(cyphal_instance_memory);

	/* Local node-ID (adjust as needed) */
	canard.node_id = CYPHAL_BLACK_BOX_ID;

	/* TX queue for one CAN interface (classic CAN MTU=8) */
	tx_queue = canardTxInit(512, CANARD_MTU_CAN_CLASSIC, cyphal_txq_memory);

	/* ------------ Subscribe to all messages that we will track ----------- */

	for (uint8_t i = 0; i < CYPHAL_SUBSCRIPTION_MESSAGES_COUNT_BLACKBOX; i++)
	{
		(void)canardRxSubscribe(&canard, cyphal_subscriptions[i].kind,
								cyphal_subscriptions[i].port_id,
								cyphal_subscriptions[i].serialization_buffer_size,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								cyphal_subscriptions[i].subscription);
	}

	/* --------------- Initialize Cyphal node ID whitelist  ---------------- */

	for (uint8_t i = 0; i < CYPHAL_NODES_IDX_MAX; i++)
	{
		cyphal_nodes_ids.uavcan_node_id[i] = CYPHAL_NODES_ID_ARRAY[i];
	}

	/* -------------------- Initialize node mode data  --------------------- */

	// Bootloader init
	bl_process_update(&bl, &ftp);

	current_node_mode_time = xTaskGetTickCount();

	current_node_mode = NODE_MODE_OPERATIONAL;

#endif /* MAVLINK_OR_CYPHAL */

	ESP_LOGI(TAG, "Initialized task_work and ready for operating!\n");

	vTaskDelay(pdMS_TO_TICKS(1000));

	TickType_t last_wake = xTaskGetTickCount();

	for (;;)
	{

		/* Sample current tick once per loop for both branches */
		TickType_t now = xTaskGetTickCount();

#if MAVLINK_OR_CYPHAL

		/* Initialize Mavlink ULog and LittleFS 1st time */
		if (!create_file)
		{
			create_file = true;
			mavlink_ulog_lfs_initialization();
		}

		/* Handle network processing for MAVLink build */
		udp_process(&udp);

		/* periodic ARP unchanged */
		if ((now - current_node_mode_time) >= pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS))
		{

			current_node_mode_time = now;

			udp_arp_grat(&udp);
		}

#else

		if ((current_node_mode == NODE_MODE_OPERATIONAL) && (!create_file))
		{
			create_file = true;
			cyphal_ulog_lfs_initialization();
			if (mission_planner_connected == true)
			{
				current_node_mode = NODE_MODE_MAINTENANCE;
			}
		}

		if (((current_node_mode == NODE_MODE_MAINTENANCE) ||
			 (current_node_mode == NODE_MODE_SOFTWARE_UPDATE)) &&
			(flag_unsubscribed == 0))
		{
			for (uint8_t i = 0; i < CYPHAL_SUBSCRIPTION_MESSAGES_COUNT_BLACKBOX; i++)
			{
				CanardPortID pid = cyphal_subscriptions[i].port_id;
				if (pid == uavcan_node_Mode_1_0_FIXED_PORT_ID_ ||
					pid == common_CommandLong_1_0_FIXED_PORT_ID_ ||
					pid == uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_)
				{

					continue;
				}

				int8_t res =
					canardRxUnsubscribe(&canard, cyphal_subscriptions[i].kind, pid);
				if (res < 0)
				{
					printf("Failed to unsubscribe port %u\n", pid);
				}
			}
			flag_unsubscribed = 1;
		}

		cyphal_process();

		if ((now - current_node_mode_time) >= pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS))
		{

			current_node_mode_time = now;
			node_mode_data.value = current_node_mode;

			uint8_t ptvt_buf[uavcan_node_Mode_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
			size_t pt_sz = sizeof(ptvt_buf);

			if (uavcan_node_Mode_1_0_serialize_(&node_mode_data, ptvt_buf, &pt_sz) >= 0)
			{

				(void)cyphal_publish_node_mode_subject(
					&canard, &tx_queue, CanardPriorityFast,
					uavcan_node_Mode_1_0_FIXED_PORT_ID_, ptvt_buf, pt_sz,
					&tid_node_mode_1_0, 1000000U);
			}

			cyphal_drain_tx_queue(&canard, &tx_queue, 64);
		}

#endif
	}
}

#if MAVLINK_OR_CYPHAL

static void mav_gw_sky_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg)
{

	switch (msg->msgid)
	{

	case MAVLINK_MSG_ID_HEARTBEAT:
	{

		gpio_set_level((gpio_num_t)26, !gpio_get_level((gpio_num_t)26));

		break;
	}
	case MAVLINK_MSG_ID_BATTERY_STATUS:
	{

		mavlink_battery_status_t data;

		mavlink_msg_battery_status_decode(msg, &data);

		ulog_battery_status_t log_data = {.timestamp = HAL_GetTimeUS(),
										  .id = data.id,
										  .battery_function = data.battery_function,
										  .type = data.type,
										  .temperature = data.temperature,
										  .voltages[0] = data.voltages[0],
										  .voltages[1] = data.voltages[1],
										  .voltages[2] = data.voltages[2],
										  .voltages[3] = data.voltages[3],
										  .voltages[4] = data.voltages[4],
										  .voltages[5] = data.voltages[5],
										  .voltages[6] = data.voltages[6],
										  .voltages[7] = data.voltages[7],
										  .voltages[8] = data.voltages[8],
										  .voltages[9] = data.voltages[9],
										  .current_battery = data.current_battery,
										  .current_consumed = data.current_consumed,
										  .energy_consumed = data.energy_consumed,
										  .battery_remaining = data.battery_remaining,
										  .time_remaining = data.time_remaining,
										  .charge_state = data.charge_state,
										  .voltages_ext[0] = data.voltages_ext[0],
										  .voltages_ext[1] = data.voltages_ext[1],
										  .voltages_ext[2] = data.voltages_ext[2],
										  .voltages_ext[3] = data.voltages_ext[3],
										  .mode = data.mode,
										  .fault_bitmask = data.fault_bitmask};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_battery_status_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_BATTERY_STATUS,
									.length = sizeof(ulog_battery_status_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_battery_status_t));

		break;
	}
	case MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA:
	{

		mavlink_lisum_power_motor_scaled_data_t data;

		mavlink_msg_lisum_power_motor_scaled_data_decode(msg, &data);

		ulog_lisum_power_motor_scaled_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.time = data.time,
			.w_gg = data.w_gg,
			.w_gg_N = data.w_gg_N,
			.w_ft = data.w_ft,
			.w_ft_N = data.w_ft_N,
			.fuel_flow = data.fuel_flow,
			.oil_flow_arm = data.oil_flow_arm,
			.oil_flow_reducer = data.oil_flow_reducer,
			.battery_volt = data.battery_volt,
			.temp_front_bearing = data.temp_front_bearing,
			.temp_rear_bearing = data.temp_rear_bearing,
			.temp_elastic_bearing = data.temp_elastic_bearing,
			.temp_rigid_bearing = data.temp_rigid_bearing,
			.temp_input_oil = data.temp_input_oil,
			.temp_arm_oil = data.temp_arm_oil,
			.temp_reducer_oil = data.temp_reducer_oil,
			.temp_exhaust_fume = data.temp_exhaust_fume,
			.press_arm_oil = data.press_arm_oil,
			.fuel_level = data.fuel_level,
			.oil_level_bearing = data.oil_level_bearing,
			.oil_level_reducer = data.oil_level_reducer,
			.PWM_fuel_pump = data.PWM_fuel_pump,
			.current_fuel_pump = data.current_fuel_pump,
			.PWM_oil_pump = data.PWM_oil_pump,
			.current_oil_pump = data.current_oil_pump,
			.PWM_oil_suction_pump_reducer = data.PWM_oil_suction_pump_reducer,
			.PWM_oil_suction_pump_arm = data.PWM_oil_suction_pump_arm,
			.PWM_oil_pressure_pump = data.PWM_oil_pressure_pump,
			.activate = data.activate,
			.turn_on = data.turn_on,
			.valve_state = data.valve_state,
			.pressure_state = data.pressure_state,
			.warning = data.warning,
			.error = data.error,
			.EvnC[0] = data.EvnC[0],
			.EvnC[1] = data.EvnC[1],
			.EvnC[2] = data.EvnC[2],
			.EvnC[3] = data.EvnC[3],
			.EvnC[4] = data.EvnC[4],
			.EvnC[5] = data.EvnC[5],
			.EvnC[6] = data.EvnC[6],
			.EvnC[7] = data.EvnC[7],
			.res[0] = data.res[0],
			.res[1] = data.res[1],
			.res[2] = data.res[2],
			.res[3] = data.res[3],
			.res[4] = data.res[4],
			.res[5] = data.res[5],
			.res[6] = data.res[6],
			.res[7] = data.res[7]};

		uint32_t payload_crc = calculate_crc32(
			(uint8_t *)&log_data, sizeof(ulog_lisum_power_motor_scaled_data_t));

		rb_entry_header_t header = {
			.message_id = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA,
			.length = sizeof(ulog_lisum_power_motor_scaled_data_t),
			.component_id = msg->compid,
			.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_power_motor_scaled_data_t));

		break;
	}
	case MAVLINK_MSG_ID_SCALED_IMU:
	{
		mavlink_scaled_imu_t data;

		mavlink_msg_scaled_imu_decode(msg, &data);

		ulog_scaled_imu_t log_data = {.timestamp = HAL_GetTimeUS(),
									  .time_boot_ms = data.time_boot_ms,
									  .xacc = data.xacc,
									  .yacc = data.yacc,
									  .zacc = data.zacc,
									  .xgyro = data.xgyro,
									  .ygyro = data.ygyro,
									  .zgyro = data.zgyro,
									  .xmag = data.xmag,
									  .ymag = data.ymag,
									  .zmag = data.zmag,
									  .temperature = data.temperature};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_scaled_imu_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_SCALED_IMU,
									.length = sizeof(ulog_scaled_imu_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_scaled_imu_t));

		break;
	}
	case MAVLINK_MSG_ID_SCALED_PRESSURE:
	{

		mavlink_scaled_pressure_t data;

		mavlink_msg_scaled_pressure_decode(msg, &data);

		ulog_scaled_pressure_t log_data = {.timestamp = HAL_GetTimeUS(),
										   .time_boot_ms = data.time_boot_ms,
										   .press_abs = data.press_abs,
										   .press_diff = data.press_diff,
										   .temperature = data.temperature,
										   .temperature_press_diff =
											   data.temperature_press_diff};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_scaled_pressure_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_SCALED_PRESSURE,
									.length = sizeof(ulog_scaled_pressure_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_scaled_pressure_t));

		break;
	}
	case MAVLINK_MSG_ID_ALTITUDE:
	{

		mavlink_altitude_t data;

		mavlink_msg_altitude_decode(msg, &data);

		ulog_altitude_t log_data = {.timestamp = HAL_GetTimeUS(),
									.time_usec = data.time_usec,
									.altitude_monotonic = data.altitude_monotonic,
									.altitude_amsl = data.altitude_amsl,
									.altitude_local = data.altitude_local,
									.altitude_relative = data.altitude_relative,
									.altitude_terrain = data.altitude_terrain,
									.bottom_clearance = data.bottom_clearance};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_altitude_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_ALTITUDE,
									.length = sizeof(ulog_altitude_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_altitude_t));

		break;
	}
	case MAVLINK_MSG_ID_ATTITUDE:
	{

		mavlink_attitude_t data;

		mavlink_msg_attitude_decode(msg, &data);

		ulog_attitude_t log_data = {.timestamp = HAL_GetTimeUS(),
									.time_boot_ms = data.time_boot_ms,
									.roll = data.roll,
									.pitch = data.pitch,
									.yaw = data.yaw,
									.rollspeed = data.rollspeed,
									.pitchspeed = data.pitchspeed,
									.yawspeed = data.yawspeed};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_attitude_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_ATTITUDE,
									.length = sizeof(ulog_attitude_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_attitude_t));

		break;
	}
	case MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA:
	{

		mavlink_lisum_gnss_recv_data_t data;

		mavlink_msg_lisum_gnss_recv_data_decode(msg, &data);

		ulog_lisum_gnss_recv_data_t log_data = {.timestamp = HAL_GetTimeUS(),
												.dev_id = data.dev_id,
												.lon = data.lon,
												.lat = data.lat,
												.alt = data.alt,
												.head = data.head,
												.head_rtk = data.head_rtk,
												.vel_ne = data.vel_ne,
												.vel_d = data.vel_d,
												.fix_type = data.fix_type,
												.carr_sol = data.carr_sol,
												.rtk_head_valid = data.rtk_head_valid,
												.sv_num = data.sv_num};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_lisum_gnss_recv_data_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_LISUM_GNSS_RECV_DATA,
									.length = sizeof(ulog_lisum_gnss_recv_data_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_lisum_gnss_recv_data_t));

		break;
	}
	case MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA:
	{

		mavlink_lisum_sensor_airspeed_data_t data;

		mavlink_msg_lisum_sensor_airspeed_data_decode(msg, &data);

		ulog_lisum_sensor_airspeed_data_t log_data = {.timestamp = HAL_GetTimeUS(),
													  .id = data.id,
													  .airspeed = data.airspeed,
													  .temperature = data.temperature,
													  .raw_press = data.raw_press,
													  .flags = data.flags};

		uint32_t payload_crc = calculate_crc32((uint8_t *)&log_data,
											   sizeof(ulog_lisum_sensor_airspeed_data_t));

		rb_entry_header_t header = {.message_id =
										MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA,
									.length = sizeof(ulog_lisum_sensor_airspeed_data_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_sensor_airspeed_data_t));

		break;
	}
	case MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA:
	{

		mavlink_lisum_power_hornet_act_data_t data;

		mavlink_msg_lisum_power_hornet_act_data_decode(msg, &data);

		ulog_lisum_power_hornet_act_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.pos_act1 = data.pos_act1,
			.abs_pos_act1 = data.abs_pos_act1,
			.vel_act1 = data.vel_act1,
			.curr_act1 = data.curr_act1,
			.sw_act1 = data.sw_act1,
			.abs_enc_sw_act1 = data.abs_enc_sw_act1,
			.pos_act2 = data.pos_act2,
			.abs_pos_act2 = data.abs_pos_act2,
			.vel_act2 = data.vel_act2,
			.curr_act2 = data.curr_act2,
			.sw_act2 = data.sw_act2,
			.abs_enc_sw_act2 = data.abs_enc_sw_act2,
			.pos_act3 = data.pos_act3,
			.abs_pos_act3 = data.abs_pos_act3,
			.vel_act3 = data.vel_act3,
			.curr_act3 = data.curr_act3,
			.sw_act3 = data.sw_act3,
			.abs_enc_sw_act3 = data.abs_enc_sw_act3,
			.pos_act4 = data.pos_act4,
			.abs_pos_act4 = data.abs_pos_act4,
			.vel_act4 = data.vel_act4,
			.curr_act4 = data.curr_act4,
			.sw_act4 = data.sw_act4,
			.abs_enc_sw_act4 = data.abs_enc_sw_act4};

		uint32_t payload_crc = calculate_crc32(
			(uint8_t *)&log_data, sizeof(ulog_lisum_power_hornet_act_data_t));

		rb_entry_header_t header = {.message_id =
										MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA,
									.length = sizeof(ulog_lisum_power_hornet_act_data_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_power_hornet_act_data_t));

		break;
	}
	case MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET:
	{

		mavlink_lisum_manual_ctrl_hornet_t data;

		mavlink_msg_lisum_manual_ctrl_hornet_decode(msg, &data);

		ulog_lisum_manual_ctrl_hornet_t log_data = {.timestamp = HAL_GetTimeUS(),
													.mode = data.mode,
													.pos_sp_act1 = data.pos_sp_act1,
													.pos_sp_act2 = data.pos_sp_act2,
													.pos_sp_act3 = data.pos_sp_act3,
													.pos_sp_act4 = data.pos_sp_act4};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_lisum_manual_ctrl_hornet_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET,
									.length = sizeof(ulog_lisum_manual_ctrl_hornet_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_manual_ctrl_hornet_t));

		break;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{

		mavlink_command_long_t data;

		mavlink_msg_command_long_decode(msg, &data);

		if (data.command == MAV_CMD_REQUEST_MESSAGE)
		{

			if (data.param1 == MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC)
			{

				mavlink_message_t tx_msg;

				mavlink_component_information_basic_t reply_info;

				reply_info = make_comp_info_basic();

				mavlink_msg_component_information_basic_encode_chan(
					dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_2, &tx_msg, &reply_info);

				mav_send(&mav_gw_sky_handle, &tx_msg);

				break;
			}

			else if (data.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION)
			{
				mavlink_message_t tx_msg;

				mavlink_autopilot_version_t reply_info;

				reply_info = make_autopilot_version();

				mavlink_msg_autopilot_version_encode_chan(
					dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_2, &tx_msg, &reply_info);

				mav_send(&mav_gw_sky_handle, &tx_msg);

				break;
			}
		}

		break;
	}
	case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
	{

		mavlink_file_transfer_protocol_t data;
		mavlink_message_t tx_msg;

		mavlink_msg_file_transfer_protocol_decode(msg, &data);

		if (file_sync != 0)
		{

			vTaskPrioritySet(task_work_handle, 2);

			file_sync = 0;
		}

		if (ftp_process_opcode(&data, &lfs, sessions, &file_cfg, &mav_gw_sky_handle) !=
			false)
		{

			mavlink_msg_file_transfer_protocol_encode_chan(
				dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_2, &tx_msg, &data);

			if (data.payload[BURST_COMPLETE] != 0U)
			{
				ftp_timeout(&mav_gw_sky_handle, &tx_msg, FTP_TRIES, FTP_DELAY);
			}
			else
			{
				mav_send(&mav_gw_sky_handle, &tx_msg);
			}
		}

		break;
	}

	default:
		break;
	}
}

static void mav_ins_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg)
{
	switch (msg->msgid)
	{

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
	{

		mavlink_named_value_float_t data;

		mavlink_msg_named_value_float_decode(msg, &data);

		ulog_named_value_float_t log_data = {.time_boot_ms = data.time_boot_ms,
											 .timestamp = HAL_GetTimeUS(),
											 .value = data.value};

		memcpy(log_data.name, data.name, sizeof(log_data.name));

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_named_value_float_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
									.length = sizeof(ulog_named_value_float_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_named_value_float_t));

		break;
	}
	case MAVLINK_MSG_ID_SCALED_PRESSURE:
	{

		mavlink_scaled_pressure_t data;

		mavlink_msg_scaled_pressure_decode(msg, &data);

		ulog_scaled_pressure_t log_data = {.time_boot_ms = data.time_boot_ms,
										   .timestamp = HAL_GetTimeUS(),
										   .press_abs = data.press_abs,
										   .press_diff = data.press_diff,
										   .temperature = data.temperature,
										   .temperature_press_diff =
											   data.temperature_press_diff};

		uint16_t message_id = MAVLINK_MSG_ID_SCALED_PRESSURE;
		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_scaled_pressure_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_SCALED_PRESSURE,
									.length = sizeof(ulog_scaled_pressure_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_scaled_pressure_t));

		break;
	}

	case MAVLINK_MSG_ID_ALTITUDE:
	{

		mavlink_altitude_t data;

		mavlink_msg_altitude_decode(msg, &data);

		ulog_altitude_t log_data = {.timestamp = HAL_GetTimeUS(),
									.time_usec = data.time_usec,
									.altitude_monotonic = data.altitude_monotonic,
									.altitude_amsl = data.altitude_amsl,
									.altitude_local = data.altitude_local,
									.altitude_relative = data.altitude_relative,
									.altitude_terrain = data.altitude_terrain,
									.bottom_clearance = data.bottom_clearance};

		uint16_t message_id = MAVLINK_MSG_ID_ALTITUDE;
		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_altitude_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_ALTITUDE,
									.length = sizeof(ulog_altitude_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_altitude_t));

		break;
	}

	case MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA:
	{

		mavlink_lisum_power_motor_scaled_data_t data;

		mavlink_msg_lisum_power_motor_scaled_data_decode(msg, &data);

		ulog_lisum_power_motor_scaled_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.time = data.time,
			.w_gg = data.w_gg,
			.w_gg_N = data.w_gg_N,
			.w_ft = data.w_ft,
			.w_ft_N = data.w_ft_N,
			.fuel_flow = data.fuel_flow,
			.oil_flow_arm = data.oil_flow_arm,
			.oil_flow_reducer = data.oil_flow_reducer,
			.battery_volt = data.battery_volt,
			.temp_front_bearing = data.temp_front_bearing,
			.temp_rear_bearing = data.temp_rear_bearing,
			.temp_elastic_bearing = data.temp_elastic_bearing,
			.temp_rigid_bearing = data.temp_rigid_bearing,
			.temp_input_oil = data.temp_input_oil,
			.temp_arm_oil = data.temp_arm_oil,
			.temp_reducer_oil = data.temp_reducer_oil,
			.temp_exhaust_fume = data.temp_exhaust_fume,
			.press_arm_oil = data.press_arm_oil,
			.fuel_level = data.fuel_level,
			.oil_level_bearing = data.oil_level_bearing,
			.oil_level_reducer = data.oil_level_reducer,
			.PWM_fuel_pump = data.PWM_fuel_pump,
			.current_fuel_pump = data.current_fuel_pump,
			.PWM_oil_pump = data.PWM_oil_pump,
			.current_oil_pump = data.current_oil_pump,
			.PWM_oil_suction_pump_reducer = data.PWM_oil_suction_pump_reducer,
			.PWM_oil_suction_pump_arm = data.PWM_oil_suction_pump_arm,
			.PWM_oil_pressure_pump = data.PWM_oil_pressure_pump,
			.activate = data.activate,
			.turn_on = data.turn_on,
			.valve_state = data.valve_state,
			.pressure_state = data.pressure_state,
			.warning = data.warning,
			.error = data.error,
			.EvnC[0] = data.EvnC[0],
			.EvnC[1] = data.EvnC[1],
			.EvnC[2] = data.EvnC[2],
			.EvnC[3] = data.EvnC[3],
			.EvnC[4] = data.EvnC[4],
			.EvnC[5] = data.EvnC[5],
			.EvnC[6] = data.EvnC[6],
			.EvnC[7] = data.EvnC[7],
			.res[0] = data.res[0],
			.res[1] = data.res[1],
			.res[2] = data.res[2],
			.res[3] = data.res[3],
			.res[4] = data.res[4],
			.res[5] = data.res[5],
			.res[6] = data.res[6],
			.res[7] = data.res[7]};

		uint16_t message_id = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA;
		uint32_t payload_crc = calculate_crc32(
			(uint8_t *)&log_data, sizeof(ulog_lisum_power_motor_scaled_data_t));

		rb_entry_header_t header = {
			.message_id = MAVLINK_MSG_ID_LISUM_POWER_MOTOR_SCALED_DATA,
			.length = sizeof(ulog_lisum_power_motor_scaled_data_t),
			.component_id = msg->compid,
			.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_power_motor_scaled_data_t));

		break;
	}

	case MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA:
	{

		mavlink_lisum_power_hornet_act_data_t data;

		mavlink_msg_lisum_power_hornet_act_data_decode(msg, &data);

		ulog_lisum_power_hornet_act_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.pos_act1 = data.pos_act1,
			.abs_pos_act1 = data.abs_pos_act1,
			.vel_act1 = data.vel_act1,
			.curr_act1 = data.curr_act1,
			.sw_act1 = data.sw_act1,
			.abs_enc_sw_act1 = data.abs_enc_sw_act1,
			.pos_act2 = data.pos_act2,
			.abs_pos_act2 = data.abs_pos_act2,
			.vel_act2 = data.vel_act2,
			.curr_act2 = data.curr_act2,
			.sw_act2 = data.sw_act2,
			.abs_enc_sw_act2 = data.abs_enc_sw_act2,
			.pos_act3 = data.pos_act3,
			.abs_pos_act3 = data.abs_pos_act3,
			.vel_act3 = data.vel_act3,
			.curr_act3 = data.curr_act3,
			.sw_act3 = data.sw_act3,
			.abs_enc_sw_act3 = data.abs_enc_sw_act3,
			.pos_act4 = data.pos_act4,
			.abs_pos_act4 = data.abs_pos_act4,
			.vel_act4 = data.vel_act4,
			.curr_act4 = data.curr_act4,
			.sw_act4 = data.sw_act4,
			.abs_enc_sw_act4 = data.abs_enc_sw_act4};

		uint32_t payload_crc = calculate_crc32(
			(uint8_t *)&log_data, sizeof(ulog_lisum_power_hornet_act_data_t));

		rb_entry_header_t header = {.message_id =
										MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA,
									.length = sizeof(ulog_lisum_power_hornet_act_data_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_power_hornet_act_data_t));

		break;
	}

	case MAVLINK_MSG_ID_BATTERY_STATUS:
	{

		mavlink_battery_status_t data;

		mavlink_msg_battery_status_decode(msg, &data);

		ulog_battery_status_t log_data = {.timestamp = HAL_GetTimeUS(),
										  .id = data.id,
										  .battery_function = data.battery_function,
										  .type = data.type,
										  .temperature = data.temperature,
										  .voltages[0] = data.voltages[0],
										  .voltages[1] = data.voltages[1],
										  .voltages[2] = data.voltages[2],
										  .voltages[3] = data.voltages[3],
										  .voltages[4] = data.voltages[4],
										  .voltages[5] = data.voltages[5],
										  .voltages[6] = data.voltages[6],
										  .voltages[7] = data.voltages[7],
										  .voltages[8] = data.voltages[8],
										  .voltages[9] = data.voltages[9],
										  .current_battery = data.current_battery,
										  .current_consumed = data.current_consumed,
										  .energy_consumed = data.energy_consumed,
										  .battery_remaining = data.battery_remaining,
										  .time_remaining = data.time_remaining,
										  .charge_state = data.charge_state,
										  .voltages_ext[0] = data.voltages_ext[0],
										  .voltages_ext[1] = data.voltages_ext[1],
										  .voltages_ext[2] = data.voltages_ext[2],
										  .voltages_ext[3] = data.voltages_ext[3],
										  .mode = data.mode,
										  .fault_bitmask = data.fault_bitmask};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_battery_status_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_BATTERY_STATUS,
									.length = sizeof(ulog_battery_status_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_battery_status_t));

		break;
	}
	case MAVLINK_MSG_ID_SCALED_IMU:
	{

		mavlink_scaled_imu_t data;

		mavlink_msg_scaled_imu_decode(msg, &data);

		ulog_scaled_imu_t log_data = {.timestamp = HAL_GetTimeUS(),
									  .time_boot_ms = data.time_boot_ms,
									  .xacc = data.xacc,
									  .yacc = data.yacc,
									  .zacc = data.zacc,
									  .xgyro = data.xgyro,
									  .ygyro = data.ygyro,
									  .zgyro = data.zgyro,
									  .xmag = data.xmag,
									  .ymag = data.ymag,
									  .zmag = data.zmag,
									  .temperature = data.temperature};

		uint32_t payload_crc =
			calculate_crc32((uint8_t *)&log_data, sizeof(ulog_scaled_imu_t));

		rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_SCALED_IMU,
									.length = sizeof(ulog_scaled_imu_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_scaled_imu_t));

		break;
	}

	default:
		break;
	}
}

static void mav_motor_control_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg)
{

	switch (msg->msgid)
	{

	case MAVLINK_MSG_ID_LISUM_PROPULSION_TB_STATUS:
	{

		mavlink_lisum_propulsion_tb_status_t data;

		mavlink_msg_lisum_propulsion_tb_status_decode(msg, &data);

		ulog_lisum_propulsion_tb_status_t log_data = {.timestamp = HAL_GetTimeUS(),
													  .current = data.current,
													  .rpm = data.rpm,
													  .temp_ambient = data.temp_ambient,
													  .temp_inverter = data.temp_inverter,
													  .temp_motor = data.temp_motor,
													  .throttle = data.throttle,
													  .thrust = data.thrust,
													  .torque = data.torque,
													  .voltage = data.voltage};

		uint32_t payload_crc = calculate_crc32((uint8_t *)&log_data,
											   sizeof(ulog_lisum_propulsion_tb_status_t));

		rb_entry_header_t header = {.message_id =
										MAVLINK_MSG_ID_LISUM_PROPULSION_TB_STATUS,
									.length = sizeof(ulog_lisum_propulsion_tb_status_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_propulsion_tb_status_t));

		break;
	}

	default:
		break;
	}
}

static void mav_vesc_control_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg)
{

	switch (msg->msgid)
	{

	case MAVLINK_MSG_ID_LISUM_POWER_MOTOR_VESC_DATA:
	{

		mavlink_lisum_power_motor_vesc_data_t data;

		mavlink_msg_lisum_power_motor_vesc_data_decode(msg, &data);

		ulog_lisum_power_motor_vesc_data_t log_data = {
			.timestamp = HAL_GetTimeUS(),
			.vesc_id = data.vesc_id,
			.mcu_id = data.mcu_id,
			.comm_name = data.comm_name,
			.current = data.current,
			.duty = data.duty,
			.rpm = data.rpm,
			.pos = data.pos,
			.current_rel = data.current_rel,
			.amp_hours_charged = data.amp_hours_charged,
			.amp_hours = data.amp_hours,
			.watt_hours_charged = data.watt_hours_charged,
			.watt_hours = data.watt_hours,
			.motor_temp = data.motor_temp,
			.fet_temp = data.fet_temp,
			.voltage = data.voltage,
			.tachometar = data.tachometar,
			.adc1 = data.adc1,
			.adc2 = data.adc2,
			.adc3 = data.adc3,
			.ppm = data.ppm,
			.max = data.max,
			.min = data.min,
			.foc_openloop_rpm = data.foc_openloop_rpm,
			.foc_sl_erpm = data.foc_sl_erpm,
			.max_power_loss = data.max_power_loss,
			.active_status_msgs = data.active_status_msgs,
			.len = data.len,
			.data[0] = data.data[0],
			.data[1] = data.data[1],
			.data[2] = data.data[2],
			.data[3] = data.data[3],
			.data[4] = data.data[4],
			.data[5] = data.data[5],
			.data[6] = data.data[6],
			.data[7] = data.data[7],
		};

		uint32_t payload_crc = calculate_crc32(
			(uint8_t *)&log_data, sizeof(ulog_lisum_power_motor_vesc_data_t));

		rb_entry_header_t header = {.message_id =
										MAVLINK_MSG_ID_LISUM_POWER_MOTOR_VESC_DATA,
									.length = sizeof(ulog_lisum_power_motor_vesc_data_t),
									.component_id = msg->compid,
									.crc32 = payload_crc};

		ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
		ring_push(&ring_buffer, (uint8_t *)&log_data,
				  sizeof(ulog_lisum_power_motor_vesc_data_t));

		break;
	}

	default:
		break;
	}
}

#else

void handle_cyphal_message(const struct CanardRxTransfer *tr)
{
	switch (current_node_mode)
	{

	case NODE_MODE_IDLE:

		switch (tr->metadata.port_id)
		{

		case common_CommandLong_1_0_FIXED_PORT_ID_:
		{

			printf("Received CommandLong message\n");

			common_CommandLong_1_0 arr;

			memset(&arr, 0, sizeof(arr));

			size_t in_size = tr->payload.size;

			int8_t rc = common_CommandLong_1_0_deserialize_(
				&arr, (const uint8_t *)tr->payload.data, &in_size);

			if (rc == 0)
			{

				if (arr.command == MAV_CMD_DO_SET_MODE)
				{
					if (arr.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
					{
						switch ((uint8_t)arr.param2)
						{
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
					}
				}
				else if (arr.command == MAV_CMD_REQUEST_MESSAGE)
				{
					if ((uint16_t)arr.param1 ==
						MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC)
					{

						common_ComponentInformationBasic_1_0 command_long;

						command_long = make_comp_info_basic();

						vTaskDelay(pdMS_TO_TICKS(2));

						uint8_t command_long_buf[common_ComponentInformationBasic_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
						size_t command_long_sz = sizeof(command_long_buf);

						if (common_ComponentInformationBasic_1_0_serialize_(
								&command_long, command_long_buf, &command_long_sz) >=
							0)
						{

							cyphal_publish_component_information_basic(
								&canard, &tx_queue, CanardPriorityExceptional,
								command_long_buf, command_long_sz,
								&tid_command_long, 1000000U);
						}
						cyphal_drain_tx_queue(&canard, &tx_queue, 64);
					}
				}

				if (arr.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION)
				{

					current_node_mode = NODE_MODE_MAINTENANCE;

					mission_planner_connected = true;

					common_AutopilotVersion_1_0 reply_info;

					reply_info = make_autopilot_version();

					uint8_t
						buf[common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

					size_t buf_size =
						common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = common_AutopilotVersion_1_0_serialize_(&reply_info, buf,
																	   &buf_size);

					if (rc >= 0)
					{

						cyphal_publish_autopilot_version(
							&canard, &tx_queue, CanardPriorityNominal, buf, buf_size,
							&tid_autopilot, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);
				}
			}
			break;
		}

		default:
		{
			break;
		}
		}

		break;

	case NODE_MODE_OPERATIONAL:

		Diode_Toggle(&imu_cnt2, 50, &toggle, DIODE_COLOR_RED);

		switch (tr->metadata.port_id)
		{

		case common_CommandLong_1_0_FIXED_PORT_ID_:
		{

			printf("Received CommandLong message\n");

			common_CommandLong_1_0 arr;

			memset(&arr, 0, sizeof(arr));

			size_t in_size = tr->payload.size;

			int8_t rc = common_CommandLong_1_0_deserialize_(
				&arr, (const uint8_t *)tr->payload.data, &in_size);

			if (rc == 0)
			{

				if (arr.command == MAV_CMD_DO_SET_MODE)
				{
					if (arr.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
					{
						switch ((uint8_t)arr.param2)
						{
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
					}
				}
				else if (arr.command == MAV_CMD_REQUEST_MESSAGE)
				{
					if ((uint16_t)arr.param1 ==
						MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC)
					{

						common_ComponentInformationBasic_1_0 command_long;

						command_long = make_comp_info_basic();

						vTaskDelay(pdMS_TO_TICKS(2));

						uint8_t command_long_buf[common_ComponentInformationBasic_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
						size_t command_long_sz = sizeof(command_long_buf);

						if (common_ComponentInformationBasic_1_0_serialize_(
								&command_long, command_long_buf, &command_long_sz) >=
							0)
						{

							cyphal_publish_component_information_basic(
								&canard, &tx_queue, CanardPriorityExceptional,
								command_long_buf, command_long_sz,
								&tid_command_long, 1000000U);
						}
						cyphal_drain_tx_queue(&canard, &tx_queue, 64);
					}
				}

				if (arr.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION)
				{

					common_AutopilotVersion_1_0 reply_info;

					reply_info = make_autopilot_version();

					uint8_t
						buf[common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

					size_t buf_size =
						common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = common_AutopilotVersion_1_0_serialize_(&reply_info, buf,
																	   &buf_size);

					if (rc >= 0)
					{

						cyphal_publish_autopilot_version(
							&canard, &tx_queue, CanardPriorityNominal, buf, buf_size,
							&tid_autopilot, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);
				}
			}
			break;
		}

			/* -------------------------------------------------------------------------------*/

		case uavcan_node_GetInfo_1_0_FIXED_PORT_ID_:
		{

			uavcan_node_GetInfo_Response_1_0 resp;

			build_getinfo_response(&resp);

			uint8_t
				buf[uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

			size_t buf_size =
				uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

			int8_t rc =
				uavcan_node_GetInfo_Response_1_0_serialize_(&resp, buf, &buf_size);

			if (rc >= 0)
			{

				cyphal_respond_cyphal_publish_node_get_info(
					&canard, &tx_queue, CanardPriorityNominal,
					tr->metadata.remote_node_id, buf, buf_size, tr->metadata.transfer_id,
					1000000U);
			}

			cyphal_drain_tx_queue(&canard, &tx_queue, 64);

			break;
		}

		case common_ScaledImu_1_0_FIXED_PORT_ID_:
		{

			common_ScaledImu_1_0 data;

			memset(&data, 0, sizeof(data));

			size_t sz = tr->payload.size;

			int8_t rc = common_ScaledImu_1_0_deserialize_(
				&data, (const uint8_t *)tr->payload.data, &sz);

			if (rc == 0)
			{

				ulog_scaled_imu_t log_data = {.timestamp = HAL_GetTimeUS(),
											  .time_boot_ms = data.time_boot_ms,
											  .xacc = data.xacc,
											  .yacc = data.yacc,
											  .zacc = data.zacc,
											  .xgyro = data.xgyro,
											  .ygyro = data.ygyro,
											  .zgyro = data.zgyro,
											  .xmag = data.xmag,
											  .ymag = data.ymag,
											  .zmag = data.zmag,
											  .temperature = data.temperature};

				uint32_t payload_crc =
					calculate_crc32((uint8_t *)&log_data, sizeof(ulog_scaled_imu_t));

				rb_entry_header_t header = {.message_id =
												common_ScaledImu_1_0_FIXED_PORT_ID_,
											.length = sizeof(ulog_scaled_imu_t),
											.component_id = ins_mav_compid,
											.crc32 = payload_crc};

				ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
				ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_scaled_imu_t));
			}

			break;
		}

		case common_ScaledPressure_1_0_FIXED_PORT_ID_:
		{

			common_ScaledPressure_1_0 data;

			memset(&data, 0, sizeof(data));

			size_t sz = tr->payload.size;

			int8_t rc = common_ScaledPressure_1_0_deserialize_(
				&data, (const uint8_t *)tr->payload.data, &sz);

			if (rc == 0)
			{

				ulog_scaled_pressure_t log_data = {.timestamp = HAL_GetTimeUS(),
												   .time_boot_ms = data.time_boot_ms,
												   .press_abs = data.press_abs,
												   .press_diff = data.press_diff,
												   .temperature = data.temperature,
												   .temperature_press_diff =
													   data.temperature_press_diff};

				uint32_t payload_crc =
					calculate_crc32((uint8_t *)&log_data, sizeof(ulog_scaled_pressure_t));

				rb_entry_header_t header = {.message_id =
												common_ScaledPressure_1_0_FIXED_PORT_ID_,
											.length = sizeof(ulog_scaled_pressure_t),
											.component_id = ins_mav_compid,
											.crc32 = payload_crc};

				ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
				ring_push(&ring_buffer, (uint8_t *)&log_data,
						  sizeof(ulog_scaled_pressure_t));
			}

			break;
		}

		case common_Attitude_1_0_FIXED_PORT_ID_:
		{

			common_Attitude_1_0 data;

			memset(&data, 0, sizeof(data));

			size_t sz = tr->payload.size;

			int8_t rc = common_Attitude_1_0_deserialize_(
				&data, (const uint8_t *)tr->payload.data, &sz);

			if (rc == 0)
			{

				ulog_attitude_t log_data = {.timestamp = HAL_GetTimeUS(),
											.time_boot_ms = data.time_boot_ms,
											.roll = data.roll,
											.pitch = data.pitch,
											.yaw = data.yaw,
											.rollspeed = data.rollspeed,
											.pitchspeed = data.pitchspeed,
											.yawspeed = data.yawspeed};

				uint32_t payload_crc =
					calculate_crc32((uint8_t *)&log_data, sizeof(ulog_attitude_t));

				rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_ATTITUDE,
											.length = sizeof(ulog_attitude_t),
											.component_id = ins_mav_compid,
											.crc32 = payload_crc};

				ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
				ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_attitude_t));

				break;
			}
		}

		case common_Altitude_1_0_FIXED_PORT_ID_:
		{

			common_Altitude_1_0 data;

			memset(&data, 0, sizeof(data));

			size_t sz = tr->payload.size;

			int8_t rc = common_Altitude_1_0_deserialize_(
				&data, (const uint8_t *)tr->payload.data, &sz);

			if (rc == 0)
			{

				ulog_altitude_t log_data = {.timestamp = HAL_GetTimeUS(),
											.time_usec = data.time_usec,
											.altitude_monotonic = data.altitude_monotonic,
											.altitude_amsl = data.altitude_amsl,
											.altitude_local = data.altitude_local,
											.altitude_relative = data.altitude_relative,
											.altitude_terrain = data.altitude_terrain,
											.bottom_clearance = data.bottom_clearance};

				uint32_t payload_crc =
					calculate_crc32((uint8_t *)&log_data, sizeof(ulog_altitude_t));

				rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_ALTITUDE,
											.length = sizeof(ulog_altitude_t),
											.component_id = ins_mav_compid,
											.crc32 = payload_crc};

				ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
				ring_push(&ring_buffer, (uint8_t *)&log_data, sizeof(ulog_altitude_t));

				break;
			}
		}

		case lisum_LisumGnssRecvData_1_0_FIXED_PORT_ID_:
		{

			lisum_LisumGnssRecvData_1_0 data;

			memset(&data, 0, sizeof(data));

			size_t sz = tr->payload.size;

			int8_t rc = lisum_LisumGnssRecvData_1_0_deserialize_(
				&data, (const uint8_t *)tr->payload.data, &sz);

			if (rc == 0)
			{

				ulog_lisum_gnss_recv_data_t log_data = {.timestamp = HAL_GetTimeUS(),
														.alt = data.alt,
														.carr_sol = data.carr_sol,
														.dev_id = data.dev_id,
														.fix_type = data.fix_type,
														.head = data.head,
														.head_rtk = data.head_rtk,
														.lat = data.lat,
														.lon = data.lon,
														.rtk_head_valid =
															data.rtk_head_valid,
														.sv_num = data.sv_num,
														.vel_d = data.vel_d,
														.vel_ne = data.vel_ne};

				uint32_t payload_crc = calculate_crc32(
					(uint8_t *)&log_data, sizeof(ulog_lisum_gnss_recv_data_t));

				rb_entry_header_t header = {.message_id = 600,
											.length = sizeof(ulog_lisum_gnss_recv_data_t),
											.component_id = ins_mav_compid,
											.crc32 = payload_crc};

				ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
				ring_push(&ring_buffer, (uint8_t *)&log_data,
						  sizeof(ulog_lisum_gnss_recv_data_t));

				break;
			}
		}

			// case lisum_LisumSensorAirspeedData_1_0_FIXED_PORT_ID_: {

			// 	lisum_LisumSensorAirspeedData_1_0 data;

			// 	memset(&data, 0, sizeof(data));

			// 	size_t sz = tr->payload.size;

			// 	int8_t rc = lisum_LisumSensorAirspeedData_1_0_deserialize_(
			// 		&data, (const uint8_t*)tr->payload.data, &sz);

			// 	if (rc == 0) {

			// 		ulog_lisum_sensor_airspeed_data_t log_data = {
			// 			.timestamp = HAL_GetTimeUS(),
			// 			.id = data.id,
			// 			.airspeed = data.airspeed,
			// 			.temperature = data.temperature,
			// 			.raw_press = data.raw_press,
			// 			.flags = data.flags};

			// 		uint32_t payload_crc = calculate_crc32(
			// 			(uint8_t*)&log_data,
			// sizeof(ulog_lisum_sensor_airspeed_data_t));

			// 		rb_entry_header_t header = {
			// 			.message_id =
			// lisum_LisumSensorAirspeedData_1_0_FIXED_PORT_ID_, 			.length =
			// sizeof(ulog_lisum_sensor_airspeed_data_t), 			.component_id =
			// ins_mav_compid, 			.crc32 = payload_crc};

			// 		ring_push(&ring_buffer, (uint8_t*)&header, sizeof(header));
			// 		ring_push(&ring_buffer, (uint8_t*)&log_data,
			// 				  sizeof(ulog_lisum_sensor_airspeed_data_t));

			// 		break;
			// 	}
			// }

		case common_BatteryStatus_1_0_FIXED_PORT_ID_:
		{

			common_BatteryStatus_1_0 data;

			memset(&data, 0, sizeof(data));

			size_t sz = tr->payload.size;

			int8_t rc = common_BatteryStatus_1_0_deserialize_(
				&data, (const uint8_t *)tr->payload.data, &sz);

			if (rc == 0)
			{
				ulog_battery_status_t log_data;

				if (tr->metadata.priority == CanardPriorityExceptional)
				{

					log_data = (ulog_battery_status_t){
						.timestamp = HAL_GetTimeUS(),
						.id = 1,
						.battery_function = data.battery_function,
						.type = data.type_,
						.temperature = data.temperature,
						.voltages[0] = data.voltages[0],
						.voltages[1] = data.voltages[1],
						.voltages[2] = data.voltages[2],
						.voltages[3] = data.voltages[3],
						.voltages[4] = data.voltages[4],
						.voltages[5] = data.voltages[5],
						.voltages[6] = data.voltages[6],
						.voltages[7] = data.voltages[7],
						.voltages[8] = data.voltages[8],
						.voltages[9] = data.voltages[9],
						.current_battery = data.current_battery,
						.current_consumed = data.current_consumed,
						.energy_consumed = data.energy_consumed,
						.battery_remaining = data.battery_remaining,
						.time_remaining = data.time_remaining,
						.charge_state = data.charge_state,
						.voltages_ext[0] = data.voltages_ext[0],
						.voltages_ext[1] = data.voltages_ext[1],
						.voltages_ext[2] = data.voltages_ext[2],
						.voltages_ext[3] = data.voltages_ext[3],
						.mode = data.mode,
						.fault_bitmask = data.fault_bitmask};
				}
				else
				{
					log_data = (ulog_battery_status_t){
						.timestamp = HAL_GetTimeUS(),
						.id = 2,
						.battery_function = data.battery_function,
						.type = data.type_,
						.temperature = data.temperature,
						.voltages[0] = data.voltages[0],
						.voltages[1] = data.voltages[1],
						.voltages[2] = data.voltages[2],
						.voltages[3] = data.voltages[3],
						.voltages[4] = data.voltages[4],
						.voltages[5] = data.voltages[5],
						.voltages[6] = data.voltages[6],
						.voltages[7] = data.voltages[7],
						.voltages[8] = data.voltages[8],
						.voltages[9] = data.voltages[9],
						.current_battery = data.current_battery,
						.current_consumed = data.current_consumed,
						.energy_consumed = data.energy_consumed,
						.battery_remaining = data.battery_remaining,
						.time_remaining = data.time_remaining,
						.charge_state = data.charge_state,
						.voltages_ext[0] = data.voltages_ext[0],
						.voltages_ext[1] = data.voltages_ext[1],
						.voltages_ext[2] = data.voltages_ext[2],
						.voltages_ext[3] = data.voltages_ext[3],
						.mode = data.mode,
						.fault_bitmask = data.fault_bitmask};
				}

				uint32_t payload_crc =
					calculate_crc32((uint8_t *)&log_data, sizeof(ulog_battery_status_t));

				rb_entry_header_t header = {.message_id = MAVLINK_MSG_ID_BATTERY_STATUS,
											.length = sizeof(ulog_battery_status_t),
											.component_id = pwr_man_mav_compid,
											.crc32 = payload_crc};

				ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
				ring_push(&ring_buffer, (uint8_t *)&log_data,
						  sizeof(ulog_battery_status_t));

				break;
			}
		}
		case lisum_LisumPowerMotorScaledData_1_0_FIXED_PORT_ID_:
		{

			lisum_LisumPowerMotorScaledData_1_0 data;

			memset(&data, 0, sizeof(data));

			size_t sz = tr->payload.size;

			int8_t rc = lisum_LisumPowerMotorScaledData_1_0_deserialize_(
				&data, (const uint8_t *)tr->payload.data, &sz);

			if (rc == 0)
			{
				ulog_lisum_power_motor_scaled_data_t log_data = {
					.timestamp = HAL_GetTimeUS(),
					.time = data.time,
					.w_gg = data.w_gg,
					.w_gg_N = data.w_gg_N,
					.w_ft = data.w_ft,
					.w_ft_N = data.w_ft_N,
					.fuel_flow = data.fuel_flow,
					.oil_flow_arm = data.oil_flow_arm,
					.oil_flow_reducer = data.oil_flow_reducer,
					.battery_volt = data.battery_volt,
					.temp_front_bearing = data.temp_front_bearing,
					.temp_rear_bearing = data.temp_rear_bearing,
					.temp_elastic_bearing = data.temp_elastic_bearing,
					.temp_rigid_bearing = data.temp_rigid_bearing,
					.temp_input_oil = data.temp_input_oil,
					.temp_arm_oil = data.temp_arm_oil,
					.temp_reducer_oil = data.temp_reducer_oil,
					.temp_exhaust_fume = data.temp_exhaust_fume,
					.press_arm_oil = data.press_arm_oil,
					.fuel_level = data.fuel_level,
					.oil_level_bearing = data.oil_level_bearing,
					.oil_level_reducer = data.oil_level_reducer,
					.PWM_fuel_pump = data.PWM_fuel_pump,
					.current_fuel_pump = data.current_fuel_pump,
					.PWM_oil_pump = data.PWM_oil_pump,
					.current_oil_pump = data.current_oil_pump,
					.PWM_oil_suction_pump_reducer = data.PWM_oil_suction_pump_reducer,
					.PWM_oil_suction_pump_arm = data.PWM_oil_suction_pump_arm,
					.PWM_oil_pressure_pump = data.PWM_oil_pressure_pump,
					.activate = data.activate,
					.turn_on = data.turn_on,
					.valve_state = data.valve_state,
					.pressure_state = data.pressure_state,
					.warning = data.warning,
					.error = data._error,
					.EvnC[0] = data.EvnC[0],
					.EvnC[1] = data.EvnC[1],
					.EvnC[2] = data.EvnC[2],
					.EvnC[3] = data.EvnC[3],
					.EvnC[4] = data.EvnC[4],
					.EvnC[5] = data.EvnC[5],
					.EvnC[6] = data.EvnC[6],
					.EvnC[7] = data.EvnC[7],
					.res[0] = data.res[0],
					.res[1] = data.res[1],
					.res[2] = data.res[2],
					.res[3] = data.res[3],
					.res[4] = data.res[4],
					.res[5] = data.res[5],
					.res[6] = data.res[6],
					.res[7] = data.res[7]};

				uint32_t payload_crc = calculate_crc32(
					(uint8_t *)&log_data, sizeof(ulog_lisum_power_motor_scaled_data_t));

				rb_entry_header_t header = {
					.message_id = 605,
					.length = sizeof(ulog_lisum_power_motor_scaled_data_t),
					.component_id = pwr_man_mav_compid,
					.crc32 = payload_crc};

				ring_push(&ring_buffer, (uint8_t *)&header, sizeof(header));
				ring_push(&ring_buffer, (uint8_t *)&log_data,
						  sizeof(ulog_lisum_power_motor_scaled_data_t));

				break;
			}
		}

		default:
			break;
		}

		break;

	case NODE_MODE_MAINTENANCE:

		Diode_Toggle(&imu_cnt, 1, &toggle, DIODE_COLOR_BLUE);

		switch (tr->metadata.port_id)
		{

		case common_CommandLong_1_0_FIXED_PORT_ID_:
		{

			printf("Received CommandLong message\n");

			// current_node_mode = NODE_MODE_INITIALIZATION;

			mission_planner_connected = true;

			common_CommandLong_1_0 arr;

			memset(&arr, 0, sizeof(arr));

			size_t in_size = tr->payload.size;

			int8_t rc = common_CommandLong_1_0_deserialize_(
				&arr, (const uint8_t *)tr->payload.data, &in_size);

			if (rc == 0)
			{

				if (arr.command == MAV_CMD_DO_SET_MODE)
				{
					if (arr.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
					{
						switch ((uint8_t)arr.param2)
						{
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
					}
				}
				else if (arr.command == MAV_CMD_REQUEST_MESSAGE)
				{
					if ((uint16_t)arr.param1 ==
						MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC)
					{

						common_ComponentInformationBasic_1_0 command_long;

						command_long = make_comp_info_basic();

						vTaskDelay(pdMS_TO_TICKS(2));

						uint8_t command_long_buf[common_ComponentInformationBasic_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
						size_t command_long_sz = sizeof(command_long_buf);

						if (common_ComponentInformationBasic_1_0_serialize_(
								&command_long, command_long_buf, &command_long_sz) >=
							0)
						{

							cyphal_publish_component_information_basic(
								&canard, &tx_queue, CanardPriorityExceptional,
								command_long_buf, command_long_sz,
								&tid_command_long, 1000000U);
						}
						cyphal_drain_tx_queue(&canard, &tx_queue, 64);
					}
				}

				if (arr.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION)
				{

					common_AutopilotVersion_1_0 reply_info;

					reply_info = make_autopilot_version();

					uint8_t
						buf[common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

					size_t buf_size =
						common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = common_AutopilotVersion_1_0_serialize_(&reply_info, buf,
																	   &buf_size);

					if (rc >= 0)
					{

						cyphal_publish_autopilot_version(
							&canard, &tx_queue, CanardPriorityNominal, buf, buf_size,
							&tid_autopilot, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);
				}
			}

			break;
		}

		case uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_:
		{

			mutex_lock(mutex_mav_ftp);

			uavcan_primitive_array_Integer8_1_0 arr;

			memset(&arr, 0, sizeof(arr));

			size_t in_size = tr->payload.size;

			int8_t rc = uavcan_primitive_array_Integer8_1_0_deserialize_(
				&arr, (const uint8_t *)tr->payload.data, &in_size);

			if (rc < 0)
			{
				mutex_unlock(mutex_mav_ftp);

				return; // Invalid message
			}

			if (arr.value.count == 0)
			{
				mutex_unlock(mutex_mav_ftp);

				return; // No data received
			}

			memset(&ftp, 0, sizeof(ftp));

			memcpy(ftp.payload, arr.value.elements, sizeof(ftp.payload));

			if (ftp_process_opcode(&ftp, &lfs, sessions, &file_cfg, &mav_gw_sky_handle))
			{

				uavcan_primitive_array_Integer8_1_0 ftp_array;

				memset(&ftp_array, 0, sizeof(ftp_array));

				memcpy(&ftp_array.value.elements[0], &ftp.payload[0],
					   sizeof(ftp.payload));

				ftp_array.value.count = (size_t)sizeof(ftp.payload);

				uint8_t ptvt_buf
					[uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

				size_t pt_sz = sizeof(ptvt_buf);

				if (uavcan_primitive_array_Integer8_1_0_serialize_(&ftp_array, ptvt_buf,
																   &pt_sz) >= 0)
				{

					(void)cyphal_publish_primitive_array_integer8_subject(
						&canard, &tx_queue, CanardPriorityExceptional,
						uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_, ptvt_buf,
						pt_sz, &tid_ftp, 1000000U);
				}
				cyphal_drain_tx_queue(&canard, &tx_queue, 64);
			}

			mutex_unlock(mutex_mav_ftp);

			break;
		}

		default:
			// Ignore other messages in this mode
			break;
		}

		break;

	case NODE_MODE_SOFTWARE_UPDATE:
		Diode_Toggle(&imu_cnt, 5, &toggle, DIODE_COLOR_ORANGE);

		switch (tr->metadata.port_id)
		{

		case common_CommandLong_1_0_FIXED_PORT_ID_:
		{

			printf("Received CommandLong message\n");

			// current_node_mode = NODE_MODE_INITIALIZATION;

			mission_planner_connected = true;

			common_CommandLong_1_0 arr;

			memset(&arr, 0, sizeof(arr));

			size_t in_size = tr->payload.size;

			int8_t rc = common_CommandLong_1_0_deserialize_(
				&arr, (const uint8_t *)tr->payload.data, &in_size);

			if (rc == 0)
			{

				if (arr.command == MAV_CMD_DO_SET_MODE)
				{
					if (arr.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
					{
						switch ((uint8_t)arr.param2)
						{
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
					}
				}
				else if (arr.command == MAV_CMD_REQUEST_MESSAGE)
				{
					if ((uint16_t)arr.param1 ==
						MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC)
					{

						common_ComponentInformationBasic_1_0 command_long;

						command_long = make_comp_info_basic();

						vTaskDelay(pdMS_TO_TICKS(2));

						uint8_t command_long_buf[common_ComponentInformationBasic_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
						size_t command_long_sz = sizeof(command_long_buf);

						if (common_ComponentInformationBasic_1_0_serialize_(
								&command_long, command_long_buf, &command_long_sz) >=
							0)
						{

							cyphal_publish_component_information_basic(
								&canard, &tx_queue, CanardPriorityExceptional,
								command_long_buf, command_long_sz,
								&tid_command_long, 1000000U);
						}
						cyphal_drain_tx_queue(&canard, &tx_queue, 64);
					}
				}

				if (arr.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION)
				{

					common_AutopilotVersion_1_0 reply_info;

					reply_info = make_autopilot_version();

					uint8_t
						buf[common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

					size_t buf_size =
						common_AutopilotVersion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

					int8_t rc = common_AutopilotVersion_1_0_serialize_(&reply_info, buf,
																	   &buf_size);

					if (rc >= 0)
					{

						cyphal_publish_autopilot_version(
							&canard, &tx_queue, CanardPriorityNominal, buf, buf_size,
							&tid_autopilot, 1000000U);
					}

					cyphal_drain_tx_queue(&canard, &tx_queue, 64);
				}
			}

			break;
		}

		case uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_:
		{

			mutex_lock(mutex_mav_ftp);

			uavcan_primitive_array_Integer8_1_0 arr;

			memset(&arr, 0, sizeof(arr));

			size_t in_size = tr->payload.size;

			int8_t rc = uavcan_primitive_array_Integer8_1_0_deserialize_(
				&arr, (const uint8_t *)tr->payload.data, &in_size);

			if (rc < 0)
			{
				mutex_unlock(mutex_mav_ftp);

				return; // Invalid message
			}

			if (arr.value.count == 0)
			{
				mutex_unlock(mutex_mav_ftp);

				return; // No data received
			}

			ftp.target_network = arr.value.elements[0];
			ftp.target_system = arr.value.elements[1];
			ftp.target_component = arr.value.elements[2];

			memcpy(ftp.payload, &arr.value.elements[3], sizeof(ftp.payload));

			// printf("payload_size=%zu\n", sizeof(ftp.payload));

			if (ftp.target_component != dev_mav_compid)
			{
				printf(
					"FTP message not intended for this component (target_component=%d, "
					"dev_mav_compid=%d), ignoring\n",
					ftp.target_component, dev_mav_compid);
				mutex_unlock(mutex_mav_ftp);
				break;
			}

			// Process bootloader message
			bl_process_mav_ftp(&bl, &ftp);

			uavcan_primitive_array_Integer8_1_0 ftp_array;
			memset(&ftp_array, 0, sizeof(ftp_array));

			ftp_array.value.elements[0] = ftp.target_network;
			ftp_array.value.elements[1] = ftp.target_system;
			ftp_array.value.elements[2] = dev_mav_compid;

			memcpy(&ftp_array.value.elements[3], ftp.payload, sizeof(ftp.payload));
			ftp_array.value.count = sizeof(ftp.payload) + 3;

			uint8_t
				buf[uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
			size_t buf_sz = sizeof(buf);

			if (uavcan_primitive_array_Integer8_1_0_serialize_(&ftp_array, buf,
															   &buf_sz) >= 0)
			{

				cyphal_publish_primitive_array_integer8_subject(
					&canard, &tx_queue, CanardPriorityExceptional,
					uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_, buf, buf_sz,
					&tid_ftp, 1000000U);
			}

			cyphal_drain_tx_queue(&canard, &tx_queue, 64);

			mutex_unlock(mutex_mav_ftp);

			if (bl.flags & BL_FLAG_REBOOT)
			{
				printf("Rebooting to bootloader...\n");
				for (volatile uint32_t i = 0; i < 1000000; i++)
					; // Short delay before rebooting
				esp_restart();
			}

			break;
		}

		default:
		{
			break;
		}

		break;
		}

	default:
	{
		break;
	}
	}
}

#endif /* MAVLINK_OR_CYPHAL */
