/**
 * @file    task_mav.c
 * @brief   Task MAV - handle MAVLink packets
 * @version 1.0.0
 * @date    07.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "main.h"
#include "task_mav.h"
#include "types.h" // Project-wide type definitions

/* Peripherals */
#include "gpio.h"

/* External hardware drivers */

/* Lib */

/* Cyphal/MAVLink publishers (application-level) */
#include "cyphal_mavlink_publishers.h"
#include "cyphal_uavcan_publishers.h"

/* CAN/Cyphal/Canard core libraries */
#include "can.h"			// CAN driver
#include "canard.h"			// Libcanard core
#include "cyphal_utility.h" // Cyphal utility functions

/* Middleware */
#include "FreeRTOS.h"
#include "bl.h"

#include "queue.h"
#include "timers.h"
#include "udp_tl.h"

#include "mav.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* MAVLink heartbeat period [ms] */
#define MAV_HB_PERIOD_MS 1000

/* MAVLink gateway sky track count */
#define MAV_GW_SKY_TRACK_COUNT 2

#define CYPHAL_SUBSCRIPTION_MESSAGES_COUNT 2

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern QueueHandle_t queue_mav_hb;
extern QueueHandle_t queue_mav_motor_scaled;
extern QueueHandle_t queue_mav_batt_status1;
extern QueueHandle_t queue_mav_batt_status2;
extern QueueHandle_t queue_command_long;

/* Extern temporary variables */
extern uint64_t boot_time_ms;

#if !MAVLINK_OR_CYPHAL
static const uint8_t dev_mav_sysid = 0;
static const uint8_t dev_mav_compid = MAV_COMP_ID_USER2;
#endif /* MAVLINK_OR_CYPHAL */

#if MAVLINK_OR_CYPHAL

extern udp_tl_t mav_gw_sky_tl;

// MAVLink information
static const uint8_t dev_mav_sysid = 0;
static const uint8_t dev_mav_compid = MAV_COMP_ID_USER2;

static const uint8_t gw_sky_mav_sysid = 0;
static const uint8_t gw_sky_mav_compid = MAV_COMP_ID_USER1;

static const uint8_t heli_mav_sysid = 0;
static const uint8_t heli_mav_compid = MAV_COMP_ID_USER52;

static const uint8_t batt1_mav_sysid = 0;
static const uint8_t batt1_mav_compid = MAV_COMP_ID_BATTERY;

static const uint8_t batt2_mav_sysid = 0;
static const uint8_t batt2_mav_compid = MAV_COMP_ID_BATTERY2;

static const mav_track_t mav_gw_sky_tracks[MAV_GW_SKY_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_HEARTBEAT,
	 .sysid = gw_sky_mav_sysid,
	 .compid = gw_sky_mav_compid},
	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG,
	 .sysid = heli_mav_sysid,
	 .compid = heli_mav_compid}};

static mav_t mav_gw_sky_handle;
static mav_link_t mav_link_gw_sky_udp;

/* MAVLink Component Information Basic defines */
static const char vendor_name[] = "BetaTechPRO";
static const char model_name[] = "HW_PWR_MAN";
static const char software_version[] = "v1.0.0";
static const char hardware_version[] = "DEV-Board V0.2";
static const char serial_number[] = "S_N: XXX";
static const uint32_t time_manufacture_s = BUILD_UNIX_TIMESTAMP;

#else

/* Canard core instances */
struct CanardInstance canard;
struct CanardTxQueue tx_queue;

static bl_t bl;

extern QueueHandle_t queue_mav_ftp;

/* Cyphal node whitelist */
extern const uint8_t CYPHAL_NODES_ID_ARRAY[CYPHAL_NODES_IDX_MAX];
static cyphal_nodes_ids_t cyphal_nodes_ids;

static CanardTransferID tid_battery_status = 0;
static CanardTransferID tid_lisum_power_motor_scaled_data = 0;
static CanardTransferID tid_node_mode = 0;
static CanardTransferID tid_ftp = 0;
static CanardTransferID tid_command_long = 0; /* Task MAV */

/* Serialization buffers and sizes */

/* Task ANPP */
static uint8_t battery_status_buf
	[messages_cyphal_uavcan_common_BatteryStatus_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t battery_status_sz = sizeof(battery_status_buf);

/* Task Baro */
static uint8_t lisum_power_motor_scaled_data_buf
	[messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t lisum_power_motor_scaled_data_sz =
	sizeof(lisum_power_motor_scaled_data_buf);

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
mavlink_file_transfer_protocol_t ftp;

messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0 battery_status;
messages_cyphal_uavcan_common_BatteryStatus_1_0 lisum_power_motor_scaled_data;
messages_cyphal_uavcan_common_ComponentInformationBasic_1_0 command_long_data;
uavcan_node_Mode_1_0 heartbeat;

const char vendor_name[] = VENDOR_NAME;
const char model_name[] = MODEL_NAME;
const char software_version[] = SOFTWARE_VERSION;
const char hardware_version[] = HARDWARE_VERSION;
const char serial_number[] = SERIAL_NUMBER;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* MAVLink gateway sky receive callback */
static void mav_gw_sky_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg);

/* heartbeat timer callback */
static void timer_mav_hb_cb(TimerHandle_t xTimer);

// mavlink_component_information_basic_t make_comp_info_basic(void)
// {
// 	mavlink_component_information_basic_t data;
// 	/* Initialized structure */
// 	memset(&data, 0, sizeof(data));

// 	strncpy(data.vendor_name, vendor_name, sizeof(data.vendor_name));

// 	strncpy(data.model_name, model_name, sizeof(data.model_name));

// 	strncpy(data.software_version, software_version,
// 			sizeof(data.software_version));

// 	strncpy(data.hardware_version, hardware_version,
// 			sizeof(data.hardware_version));

// 	strncpy(data.serial_number, serial_number, sizeof(data.serial_number));

// 	data.capabilities = 0;

// 	data.time_manufacture_s = time_manufacture_s;

// 	/* Conversion to ms */
// 	data.time_boot_ms = boot_time_ms;

// 	return data;
// }

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
	mavlink_battery_status_t battery_status;
	mavlink_lisum_power_motor_scaled_data_t lisum_power_motor_scaled_data;

	/*--------------------------------- MAV ----------------------------------*/
	/* initialize MAVLink handle */
	mav_init(&mav_gw_sky_handle, dev_mav_sysid, dev_mav_compid);

	/* initialize MAVLink links */
	mav_link_init(&mav_link_gw_sky_udp, MAVLINK_COMM_0, mav_gw_sky_recv_cb,
				  NULL, (tl_t *)&mav_gw_sky_tl);

	/* connect MAVLink links to MAVLink handles */
	mav_link(&mav_gw_sky_handle, &mav_link_gw_sky_udp);

	/* track gateway sky messages */
	for (uint8_t i = 0; i < MAV_GW_SKY_TRACK_COUNT; i++) {
		mav_track_t *track;

		track = (mav_track_t *)&mav_gw_sky_tracks[i];
		mav_track(&mav_gw_sky_handle, track->msgid, track->sysid,
				  track->compid);
	}

#else

	mavlink_file_transfer_protocol_t ftp_data;

	bl_process_update(&bl, &ftp_data);

	/* Initialize deterministic memory pools and Cyphal */
	cyphal_pool_init();

	/* Canard struct init */
	canard = canardInit(cyphal_instance_memory);

	/* Local node-ID (adjust as needed) */
	canard.node_id = CYPHAL_POWER_MANAGEMENT_ID;

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

	vTaskDelay(pdMS_TO_TICKS(1000));

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
						&canard, &tx_queue, CanardPriorityExceptional,
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

			if (xQueueReceive(queue_command_long, &command_long_data, 0)) {
				if (messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_serialize_(
						&command_long_data, command_long_buf,
						&command_long_sz) >= 0) {
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

			if (xQueueReceive(queue_mav_motor_scaled,
							  &lisum_power_motor_scaled_data, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0_serialize_(
						&lisum_power_motor_scaled_data,
						lisum_power_motor_scaled_data_buf,
						&lisum_power_motor_scaled_data_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate =
							cyphal_publish_lisum_lisum_power_motor_scaled_data_subject(
								&canard, &tx_queue, CanardPriorityExceptional,
								messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0_FIXED_PORT_ID_,
								lisum_power_motor_scaled_data_buf,
								lisum_power_motor_scaled_data_sz,
								&tid_lisum_power_motor_scaled_data,
								CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else

				mavlink_msg_lisum_power_motor_scaled_data_encode(
					dev_mav_sysid, dev_mav_compid, &msg,
					&lisum_power_motor_scaled_data);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}

			if (xQueueReceive(queue_mav_batt_status1, &battery_status, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_common_BatteryStatus_1_0_serialize_(
						&battery_status, battery_status_buf,
						&battery_status_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate = cyphal_publish_common_battery_status(
							&canard, &tx_queue, CanardPriorityExceptional,
							battery_status_buf, battery_status_sz,
							&tid_battery_status, CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else

				mavlink_msg_battery_status_encode(
					batt1_mav_sysid, batt1_mav_compid, &msg, &battery_status);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}

			if (xQueueReceive(queue_mav_batt_status2, &battery_status, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_common_BatteryStatus_1_0_serialize_(
						&battery_status, battery_status_buf,
						&battery_status_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate = cyphal_publish_common_battery_status(
							&canard, &tx_queue, CanardPriorityImmediate,
							battery_status_buf, battery_status_sz,
							&tid_battery_status, CYPHAL_MEDIUM_TIMEOUT);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else

				mavlink_msg_battery_status_encode(
					batt2_mav_sysid, batt2_mav_compid, &msg, &battery_status);

				mav_send(&mav_gw_sky_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}
		}
	}
}

static void mav_gw_sky_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg)
{
	mavlink_message_t tx_msg;

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: {
		HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
							 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));
		break;
	}
		// case MAVLINK_MSG_ID_COMMAND_LONG: {
		// 	/* route command long message */
		// 	mavlink_command_long_t data;

		// 	mavlink_msg_command_long_decode(msg, &data);

		// 	if (data.command == MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC) {
		// 		/* It's a request for software/hardware version */
		// 		mavlink_message_t reply_msg;

		// 		mavlink_component_information_basic_t reply_info;

		// 		reply_info = make_comp_info_basic();

		// 		mavlink_msg_component_information_basic_encode(
		// 			msg->sysid, msg->compid, &reply_msg, &reply_info);

		// 		mav_send(&mav_gw_sky_handle, &reply_msg);

		// 		break;
		// 	}
		// 	break;
		// }

	default:
		break;
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
