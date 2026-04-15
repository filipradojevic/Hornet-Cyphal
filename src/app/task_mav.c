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

/* Standard types */
#include <stdint.h>
#include <string.h>

/* Project headers */
#include "main.h"	  // Main project configuration
#include "task_mav.h" // Task and interface definitions for MAV
#include "types.h"	  // Project-wide type definitions

#include "common.h"

/* External hardware drivers */
#include "bl.h"
#include "ftp_common.h"
#include "mav.h"

/* Peripherals */
#include "gpio.h"

/* Cyphal/MAVLink publishers (application-level) */
#include "cyphal_mavlink_publishers.h"
#include "cyphal_uavcan_publishers.h"

/* CAN/Cyphal/Canard core libraries */
#include "can.h"			// CAN driver
#include "canard.h"			// Libcanard core
#include "cyphal_utility.h" // Cyphal utility functions

/* External hardware drivers */

/* Lib */

/* Middleware */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

#include "udp_tl.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* MAVLink client track count */
#define MAV_CLIENT_TRACK_COUNT 1

#define CYPHAL_SUBSCRIPTION_MESSAGES_COUNT 2

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* FreeRTOS objects */
extern QueueSetHandle_t queueset_mav;
extern QueueHandle_t queue_mav_hb;
extern QueueHandle_t queue_mav_scaled_imu;
extern QueueHandle_t queue_mav_scaled_pressure;
extern QueueHandle_t queue_mav_altitude;
extern QueueHandle_t queue_mav_attitude;
extern QueueHandle_t queue_mav_lisum_gnss_data;
extern QueueHandle_t queue_mav_lisum_airspeed_data;
extern SemaphoreHandle_t tx_queue_mutex;
extern QueueHandle_t queue_mav_ftp;
extern QueueHandle_t queue_command_long;

static bl_t bl;

#if !MAVLINK_OR_CYPHAL
static const uint8_t dev_mav_sysid = 0;
static const uint8_t dev_mav_compid = MAV_COMP_ID_USER4;
#endif /* MAVLINK_OR_CYPHAL */

#if MAVLINK_OR_CYPHAL

extern udp_tl_t mav_gw_sky_tl;

// MAVLink information
static const uint8_t dev_mav_sysid = 0;
static const uint8_t dev_mav_compid = MAV_COMP_ID_USER4;

static const uint8_t gw_sky_mav_sysid = 0;
static const uint8_t gw_sky_mav_compid = MAV_COMP_ID_USER1;

static const mav_track_t mav_gw_sky_tracks[MAV_CLIENT_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_HEARTBEAT,
	 .sysid = gw_sky_mav_sysid,
	 .compid = gw_sky_mav_compid}};

static mav_t mav_gw_sky_handle;
static mav_link_t mav_gw_sky_link_udp;

#else

/* Canard core instances */
struct CanardInstance canard;
struct CanardTxQueue tx_queue;

/* Cyphal node whitelist */
extern const uint8_t CYPHAL_NODES_ID_ARRAY[CYPHAL_NODES_IDX_MAX];
static cyphal_nodes_ids_t cyphal_nodes_ids;

static CanardTransferID tid_lisum_sensor_airspeed_data = 0; /* Task ANPP */
static CanardTransferID tid_common_scaled_pressure = 0;		/* Task Baro */
static CanardTransferID tid_common_altitude = 0;			/* Task Baro */
static CanardTransferID tid_common_attitude = 0;			/* Task Fusion */
static CanardTransferID tid_lisum_gnss_recv_data = 0;		/* Task GNSS */
static CanardTransferID tid_common_scaled_imu = 0;			/* Task Imu */
static CanardTransferID tid_node_mode = 0;					/* Task MAV */
static CanardTransferID tid_command_long = 0;				/* Task MAV */
static CanardTransferID tid_ftp = 0;
/* Serialization buffers and sizes */

/* Task ANPP */
static uint8_t lisum_airspeed_data_buf
	[messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t lisum_airspeed_data_sz = sizeof(lisum_airspeed_data_buf);

/* Task Baro */
static uint8_t altitude_buf
	[messages_cyphal_uavcan_common_Altitude_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t altitude_sz = sizeof(altitude_buf);

static uint8_t scaled_pressure_buf
	[messages_cyphal_uavcan_common_ScaledPressure_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t scaled_pressure_sz = sizeof(scaled_pressure_buf);

/* Task Fusion */
static uint8_t attitude_buf
	[messages_cyphal_uavcan_common_Attitude_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t attitude_sz = sizeof(attitude_buf);

/* Task GNSS */
static uint8_t gnss_data_buf
	[messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t gnss_data_sz = sizeof(gnss_data_buf);

/* Task Imu */
static uint8_t scaled_imu_buf
	[messages_cyphal_uavcan_common_ScaledImu_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t scaled_imu_sz = sizeof(scaled_imu_buf);

/* Task MAV */
static uint8_t
	node_mode_buf[uavcan_node_Mode_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t node_mode_sz = sizeof(node_mode_buf);

static uint8_t command_long_buf
	[messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t command_long_sz = sizeof(command_long_buf);

static struct CanardRxSubscription sub_integer8_array_1_0;
static struct CanardRxSubscription sub_command_long_1_0;

/* Cyphal Subscriptions Messages */
cyphal_subscription_messages_t
	cyphal_subscriptions[CYPHAL_SUBSCRIPTION_MESSAGES_COUNT] = {

		{.kind = CanardTransferKindMessage,
		 .port_id = uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_integer8_array_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id =
			 messages_cyphal_uavcan_common_CommandLong_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 messages_cyphal_uavcan_common_CommandLong_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_command_long_1_0}};

/* Global/volatile flags */
volatile int validate = 0;

#endif /* MAVLINK_OR_CYPHAL */

extern volatile node_mode_state_t current_node_mode;
static uint8_t
	buf[uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

/* MAVLink Component Information Basic defines */
const char vendor_name[] = VENDOR_NAME;
const char model_name[] = MODEL_NAME;
const char software_version[] = SOFTWARE_VERSION;
const char hardware_version[] = HARDWARE_VERSION;
const char serial_number[] = SERIAL_NUMBER;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* MAVLink receive callback */
static void mav_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg);

/* heartbeat timer callback */
static void timer_mav_hb_cb(TimerHandle_t xTimer);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_mav(void *arg)
{
	TimerHandle_t timer_mav_hb;

	TickType_t last_wake_time = xTaskGetTickCount();

#if MAVLINK_OR_CYPHAL

	mavlink_message_t msg;
	mavlink_heartbeat_t heartbeat;
	mavlink_scaled_imu_t scaled_imu;
	mavlink_scaled_pressure_t scaled_pressure;
	mavlink_altitude_t altitude;
	mavlink_attitude_t attitude;
	mavlink_lisum_gnss_recv_data_t lisum_gnss_recv_data;
	mavlink_lisum_sensor_airspeed_data_t lisum_airspeed_data;

	/*--------------------------------- MAV ----------------------------------*/
	/* initialize MAVLink handle */
	mav_init(&mav_gw_sky_handle, dev_mav_sysid, dev_mav_compid);

	/* initialize MAVLink UDP link */
	mav_link_init(&mav_gw_sky_link_udp, MAVLINK_COMM_0, mav_recv_cb, NULL,
				  (tl_t *)&mav_gw_sky_tl);

	/* connect MAVLink link to MAVLink handle */
	mav_link(&mav_gw_sky_handle, &mav_gw_sky_link_udp);

	/* track client messages */
	for (uint8_t i = 0; i < MAV_CLIENT_TRACK_COUNT; i++) {
		mav_track_t *track;

		track = (mav_track_t *)&mav_gw_sky_tracks[i];
		mav_track(&mav_gw_sky_handle, track->msgid, track->sysid,
				  track->compid);
	}

#else

	messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0
		lisum_airspeed_data;
	messages_cyphal_uavcan_common_ScaledPressure_1_0 scaled_pressure;
	messages_cyphal_uavcan_common_Altitude_1_0 altitude;
	messages_cyphal_uavcan_common_Attitude_1_0 attitude;
	messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0 lisum_gnss_recv_data;
	messages_cyphal_uavcan_common_ScaledImu_1_0 scaled_imu;
	uavcan_node_Mode_1_0 heartbeat;
	mavlink_file_transfer_protocol_t ftp;
	messages_cyphal_uavcan_common_ComponentInformationBasic_1_0 command_long;

	bl_process_update(&bl, &ftp);

	/* Initialize deterministic memory pools and Cyphal */
	cyphal_pool_init();

	/* Canard struct init */
	canard = canardInit(cyphal_instance_memory);

	/* Local node-ID (adjust as needed) */
	canard.node_id = CYPHAL_INS_ID;

	/* TX queue for one CAN interface (classic CAN MTU=8) */
	tx_queue = canardTxInit(512, CANARD_MTU_CAN_CLASSIC, cyphal_txq_memory);

	/* ------------ Subscribe to all messages that we will track ----------- */

	for (uint8_t i = 0; i < CYPHAL_SUBSCRIPTION_MESSAGES_COUNT; i++) {
		(void)canardRxSubscribe(
			&canard, cyphal_subscriptions[i].kind,
			cyphal_subscriptions[i].port_id,
			cyphal_subscriptions[i].serialization_buffer_size,
			CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
			cyphal_subscriptions[i].subscription);
	}

	/* --------------- Initialize Cyphal node ID whitelist  ---------------- */

	for (uint8_t i = 0; i < CYPHAL_NODES_IDX_MAX; i++) {
		cyphal_nodes_ids.uavcan_node_id[i] = CYPHAL_NODES_ID_ARRAY[i];
	}

	current_node_mode = NODE_MODE_OPERATIONAL;

#endif /* MAVLINK_OR_CYPHAL */

	/* create MAVLink heartbeat timer */
	timer_mav_hb = xTimerCreate("tim_mav_hb", pdMS_TO_TICKS(MAV_HB_PERIOD_MS),
								pdTRUE, NULL, timer_mav_hb_cb);

	/* start heartbeat timer */
	xTimerStart(timer_mav_hb, 0);

	for (;;) {

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));

		if (xQueueReceive(queue_mav_hb, &heartbeat, 0)) {

#if !MAVLINK_OR_CYPHAL

			heartbeat.value = current_node_mode;

			if (uavcan_node_Mode_1_0_serialize_(&heartbeat, node_mode_buf,
												&node_mode_sz) >= 0) {
				TX_QUEUE_MUTEX_TAKE
				{
					validate = cyphal_publish_node_mode_subject(
						&canard, &tx_queue, CanardPriorityHigh,
						uavcan_node_Mode_1_0_FIXED_PORT_ID_, node_mode_buf,
						node_mode_sz, &tid_node_mode, CYPHAL_MEDIUM_TIMEOUT);

					TX_QUEUE_MUTEX_GIVE;
				}
			}
#else

			mavlink_msg_heartbeat_encode(mav_gw_sky_handle.sysid,
										 mav_gw_sky_handle.compid, &msg,
										 &heartbeat);

			mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
		}

		if (current_node_mode == NODE_MODE_MAINTENANCE) {
			continue;
		} else if (current_node_mode == NODE_MODE_SOFTWARE_UPDATE) {

			if (xQueueReceive(queue_mav_ftp, &ftp, 0)) {

				if (ftp.target_component != dev_mav_compid) {
					continue;
				} else {
					bl_process_mav_ftp(&bl, &ftp);

					uavcan_primitive_array_Integer8_1_0 ftp_array;
					memset(&ftp_array, 0, sizeof(ftp_array));

					ftp_array.value.elements[0] = ftp.target_network;
					ftp_array.value.elements[1] = ftp.target_system;
					ftp_array.value.elements[2] = dev_mav_compid;

					memcpy(&ftp_array.value.elements[3], ftp.payload,
						   sizeof(ftp.payload));
					ftp_array.value.count = sizeof(ftp.payload) + 3;

					uint8_t buf
						[uavcan_primitive_array_Integer8_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
					size_t buf_sz = sizeof(buf);

					if (uavcan_primitive_array_Integer8_1_0_serialize_(
							&ftp_array, buf, &buf_sz) >= 0) {
						TX_QUEUE_MUTEX_TAKE
						{
							validate =
								cyphal_publish_primitive_array_integer8_subject(
									&canard, &tx_queue,
									CanardPriorityExceptional,
									uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_,
									buf, buf_sz, &tid_ftp, 1000000U);
							TX_QUEUE_MUTEX_GIVE;
						}
					}

					if (bl.flags & BL_FLAG_REBOOT) {
						for (volatile uint32_t i = 0; i < 1000000; i++)
							;
						HAL_NVIC_SystemReset();
					}
				}
			}
		}

		else {

			if (xQueueReceive(queue_command_long, &command_long, 0)) {
				if (messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_serialize_(
						&command_long, command_long_buf, &command_long_sz) >=
					0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate =
							cyphal_publish_common_component_information_basic(
								&canard, &tx_queue, CanardPriorityExceptional,
								command_long_buf, command_long_sz,
								&tid_command_long, CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
			}

			if (xQueueReceive(queue_mav_scaled_imu, &scaled_imu, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_common_ScaledImu_1_0_serialize_(
						&scaled_imu, scaled_imu_buf, &scaled_imu_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate = cyphal_publish_common_scaled_imu(
							&canard, &tx_queue, CanardPriorityExceptional,
							scaled_imu_buf, scaled_imu_sz,
							&tid_common_scaled_imu, CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}

#else

				mavlink_msg_scaled_imu_encode(mav_gw_sky_handle.sysid,
											  mav_gw_sky_handle.compid, &msg,
											  &scaled_imu);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}

			if (xQueueReceive(queue_mav_scaled_pressure, &scaled_pressure, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_common_ScaledPressure_1_0_serialize_(
						&scaled_pressure, scaled_pressure_buf,
						&scaled_pressure_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate = cyphal_publish_common_scaled_pressure(
							&canard, &tx_queue, CanardPriorityExceptional,
							scaled_pressure_buf, scaled_pressure_sz,
							&tid_common_scaled_pressure, CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else

				mavlink_msg_scaled_pressure_encode(mav_gw_sky_handle.sysid,
												   mav_gw_sky_handle.compid,
												   &msg, &scaled_pressure);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}

			if (xQueueReceive(queue_mav_altitude, &altitude, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_common_Altitude_1_0_serialize_(
						&altitude, altitude_buf, &altitude_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate = cyphal_publish_common_altitude(
							&canard, &tx_queue, CanardPriorityExceptional,
							altitude_buf, altitude_sz, &tid_common_altitude,
							CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else

				mavlink_msg_altitude_encode(mav_gw_sky_handle.sysid,
											mav_gw_sky_handle.compid, &msg,
											&altitude);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}

			if (xQueueReceive(queue_mav_attitude, &attitude, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_common_Attitude_1_0_serialize_(
						&attitude, attitude_buf, &attitude_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate = cyphal_publish_common_attitude(
							&canard, &tx_queue, CanardPriorityExceptional,
							attitude_buf, attitude_sz, &tid_common_attitude,
							CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else

				mavlink_msg_attitude_encode(mav_gw_sky_handle.sysid,
											mav_gw_sky_handle.compid, &msg,
											&attitude);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}

			if (xQueueReceive(queue_mav_lisum_gnss_data, &lisum_gnss_recv_data,
							  0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0_serialize_(
						&lisum_gnss_recv_data, gnss_data_buf, &gnss_data_sz) >=
					0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate =
							cyphal_publish_lisum_lisum_gnss_recv_data_subject(
								&canard, &tx_queue, CanardPriorityExceptional,
								messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0_FIXED_PORT_ID_,
								gnss_data_buf, gnss_data_sz,
								&tid_lisum_gnss_recv_data,
								CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}

#else
				mavlink_msg_lisum_gnss_recv_data_encode(
					mav_gw_sky_handle.sysid, mav_gw_sky_handle.compid, &msg,
					&lisum_gnss_recv_data);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}

			if (xQueueReceive(queue_mav_lisum_airspeed_data,
							  &lisum_airspeed_data, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0_serialize_(
						&lisum_airspeed_data, lisum_airspeed_data_buf,
						&lisum_airspeed_data_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate =
							cyphal_publish_lisum_lisum_sensor_airspeed_data(
								&canard, &tx_queue, CanardPriorityExceptional,
								lisum_airspeed_data_buf, lisum_airspeed_data_sz,
								&tid_lisum_sensor_airspeed_data,
								CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}

#else

				mavlink_msg_lisum_sensor_airspeed_data_encode(
					mav_gw_sky_handle.sysid, mav_gw_sky_handle.compid, &msg,
					&lisum_airspeed_data);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}
		}
	}
}

static void mav_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg)
{
	if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
		HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
							 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));
	}
}

static void timer_mav_hb_cb(TimerHandle_t xTimer)
{
	mavlink_heartbeat_t mav_hb = {.type = MAV_TYPE_GENERIC,
								  .autopilot = MAV_AUTOPILOT_GENERIC,
								  .base_mode = 0,
								  .custom_mode = 0,
								  .system_status = MAV_STATE_STANDBY};

	xQueueSendToBack(queue_mav_hb, &mav_hb, 0);
}
