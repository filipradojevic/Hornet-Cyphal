/**
 * @file    task_mav.c
 * @brief   Task MAV - handle MAVLink packets
 * @version 1.0.0
 * @date    26.02.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "bl.h"
#include "common.h"
#include "main.h"
#include "task_epos.h"
#include "task_mav.h"
#include "types.h"

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
#include "mav.h"
#include "queue.h"
#include "timers.h"
#include "udp_tl.h"
#include <mavlink_types.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* MAVLink heartbeat period [ms] */
#define MAV_HB_PERIOD_MS 1000

/* MAVLink client track count (ground station helicopter application) */
#define MAV_CLIENT_TRACK_COUNT 4

#define CYPHAL_SUBSCRIPTION_MESSAGES_COUNT 5

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern QueueHandle_t queue_mav_hb;
extern QueueHandle_t queue_mav_ack;
extern QueueHandle_t queue_mav_act_data;
extern QueueHandle_t queue_mav_manual_ctrl;
extern SemaphoreHandle_t tx_queue_mutex;
extern QueueHandle_t queue_mav_ftp;
extern QueueHandle_t queue_command_long;
extern QueueHandle_t queue_status_text_report;

extern TaskHandle_t task_tx_can_handle;

extern task_epos_state_e task_epos_state;

/* Extern temporary variables */
extern uint64_t boot_time_ms;

extern task_epos_state_e task_epos_state;

#if !MAVLINK_OR_CYPHAL
static const uint8_t dev_mav_sysid = 0;
static const uint8_t dev_mav_compid = MAV_COMP_ID_USER5;
#endif /* MAVLINK_OR_CYPHAL */

#if MAVLINK_OR_CYPHAL

extern udp_tl_t mav_udp_tl;

// MAVLink information
static const uint8_t dev_mav_sysid = 0;
static const uint8_t dev_mav_compid = MAV_COMP_ID_USER5;

static const uint8_t client_mav_sysid = 0;
static const uint8_t client_mav_compid = MAV_COMP_ID_USER52;

static const mav_track_t mav_client_tracks[MAV_CLIENT_TRACK_COUNT] = {
	{.msgid = MAVLINK_MSG_ID_HEARTBEAT,
	 .sysid = client_mav_sysid,
	 .compid = client_mav_compid},
	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG,
	 .sysid = client_mav_sysid,
	 .compid = client_mav_compid},
	{.msgid = MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET,
	 .sysid = client_mav_sysid,
	 .compid = client_mav_compid},
	{.msgid = MAVLINK_MSG_ID_COMMAND_LONG,
	 .sysid = client_mav_sysid,
	 .compid = client_mav_compid}};

static mav_t mav_handle;
static mav_link_t mav_link_udp;

#else

/* Canard core instances */
struct CanardInstance canard;
struct CanardTxQueue tx_queue;

/* Cyphal node whitelist */
extern const uint8_t CYPHAL_NODES_ID_ARRAY[CYPHAL_NODES_IDX_MAX];
static cyphal_nodes_ids_t cyphal_nodes_ids;

static CanardTransferID tid_lisum_manual_ctrl_hornet = 0;
static CanardTransferID tid_lisum_power_hornet_ack = 0;
static CanardTransferID tid_common_command_ack = 0;
static CanardTransferID tid_node_mode = 0;
static CanardTransferID tid_command_long = 0;		/* Task MAV */
static CanardTransferID tid_common_status_text = 0; /* Task MAV */
static CanardTransferID tid_ftp = 0;

static struct CanardRxSubscription sub_command_long_1_0;
static struct CanardRxSubscription sub_lisum_manual_ctrl_hornet_1_0;
static struct CanardRxSubscription sub_servo_output_raw_1_0;

/* Serialization buffers and sizes */
static uint8_t common_status_text
	[messages_cyphal_uavcan_common_Statustext_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t common_status_text_sz = sizeof(common_status_text);

static uint8_t lisum_manual_ctrl_hornet_buf
	[messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t lisum_manual_ctrl_hornet_sz =
	sizeof(lisum_manual_ctrl_hornet_buf);

static uint8_t lisum_power_hornet_ack_buf
	[messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t lisum_power_hornet_ack_sz = sizeof(lisum_power_hornet_ack_buf);
static uint8_t command_ack_buf
	[messages_cyphal_uavcan_common_CommandAck_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t command_ack_sz = sizeof(command_ack_buf);
static uint8_t node_mode_buf
	[messages_cyphal_uavcan_minimal_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t node_mode_sz = sizeof(node_mode_buf);

static uint8_t command_long_buf
	[messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t command_long_sz = sizeof(command_long_buf);

static uint8_t component_info_buf
	[messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t component_info_sz = sizeof(component_info_buf);

static struct CanardRxSubscription sub_integer8_array_1_0;

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
		 .subscription = &sub_command_long_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id =
			 messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_lisum_manual_ctrl_hornet_1_0},

		{.kind = CanardTransferKindMessage,
		 .port_id =
			 messages_cyphal_uavcan_common_ServoOutputRaw_1_0_FIXED_PORT_ID_,
		 .serialization_buffer_size =
			 messages_cyphal_uavcan_common_ServoOutputRaw_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
		 .subscription = &sub_servo_output_raw_1_0},
};

/* Global/volatile flags */
volatile int validate = 0;

extern volatile node_mode_state_t current_node_mode;

#endif /* MAVLINK_OR_CYPHAL */

/* MAVLink Component Information Basic defines */
const char vendor_name[] = VENDOR_NAME;
const char model_name[] = MODEL_NAME;
const char software_version[] = SOFTWARE_VERSION;
const char hardware_version[] = HARDWARE_VERSION;
const char serial_number[] = SERIAL_NUMBER;
const uint32_t time_manufacture_s = BUILD_UNIX_TIMESTAMP;

static bl_t bl;

static const enum CanardPriority act_priorities[] = {
	CanardPriorityExceptional, CanardPriorityImmediate, CanardPriorityFast,
	CanardPriorityHigh,		   CanardPriorityNominal,	CanardPriorityLow,
	CanardPrioritySlow,		   CanardPriorityOptional,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* MAVLink receive callback */
static void mav_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg);

/* heartbeat timer callback */
static void timer_mav_hb_cb(TimerHandle_t xTimer);

/* process received MAVLink command */
static void mav_handle_cmd_long(mavlink_message_t *msg,
								mavlink_command_long_t *cmd_long);

mavlink_component_information_basic_t make_comp_info_basic(void)
{
	mavlink_component_information_basic_t data;
	/* Initialized structure */
	memset(&data, 0, sizeof(data));

	strncpy(data.vendor_name, vendor_name, sizeof(data.vendor_name));

	strncpy(data.model_name, model_name, sizeof(data.model_name));

	strncpy(data.software_version, software_version,
			sizeof(data.software_version));

	strncpy(data.hardware_version, hardware_version,
			sizeof(data.hardware_version));

	strncpy(data.serial_number, serial_number, sizeof(data.serial_number));

	data.capabilities = 0;

	data.time_manufacture_s = time_manufacture_s;

	/* Conversion to ms */
	data.time_boot_ms = boot_time_ms;

	return data;
}

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_mav(void *arg)
{

#if MAVLINK_OR_CYPHAL

	mavlink_message_t msg;
	mavlink_heartbeat_t heartbeat;
	mavlink_command_ack_t command_ack;
	mavlink_lisum_power_hornet_act_data_t lisum_power_hornet_ack;
	mavlink_lisum_manual_ctrl_hornet_t lisum_manual_ctrl_hornet;

	/*--------------------------------- MAV ----------------------------------*/
	/* initialize MAVLink handle */
	mav_init(&mav_handle, dev_mav_sysid, dev_mav_compid);

	/* initialize MAVLink UDP link */
	mav_link_init(&mav_link_udp, MAVLINK_COMM_0, mav_recv_cb, NULL,
				  (tl_t *)&mav_udp_tl);

	/* connect MAVLink link to MAVLink handle */
	mav_link(&mav_handle, &mav_link_udp);

	/* track MAVLink client messages */
	for (uint8_t i = 0; i < MAV_CLIENT_TRACK_COUNT; i++) {
		mav_track_t *track;

		track = (mav_track_t *)&mav_client_tracks[i];
		mav_track(&mav_handle, track->msgid, track->sysid, track->compid);
	}

#else

	messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0
		lisum_manual_ctrl_hornet;
	messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0
		lisum_power_hornet_ack;
	messages_cyphal_uavcan_common_CommandAck_1_0 command_ack;
	uavcan_node_Mode_1_0 heartbeat;
	mavlink_file_transfer_protocol_t ftp;
	messages_cyphal_uavcan_common_ComponentInformationBasic_1_0 command_long;
	messages_cyphal_uavcan_common_Statustext_1_0 status_text_report;

	bl_process_update(&bl, &ftp);

	/* Initialize deterministic memory pools and Cyphal */
	cyphal_pool_init();

	/* Canard struct init */
	canard = canardInit(cyphal_instance_memory);

	/* Local node-ID (adjust as needed) */
	canard.node_id = CYPHAL_ACTUATOR_MASTER_ID;

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

	TimerHandle_t timer_mav_hb;

	TickType_t last_wake_time = xTaskGetTickCount();
	TickType_t compontent_info_time = 0;

	/* create MAVLink heartbeat timer */
	timer_mav_hb = xTimerCreate("tim_mav_hb", pdMS_TO_TICKS(MAV_HB_PERIOD_MS),
								pdTRUE, NULL, timer_mav_hb_cb);

	/* start heartbeat timer */
	xTimerStart(timer_mav_hb, 0);

	for (;;) {

		// vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));

		if (compontent_info_time != 0 &&
			xTaskGetTickCount() - compontent_info_time > pdMS_TO_TICKS(10)) {
			TX_QUEUE_MUTEX_TAKE
			{
				compontent_info_time = 0;
				validate = cyphal_publish_common_component_information_basic(
					&canard, &tx_queue, CanardPriorityExceptional,
					component_info_buf, component_info_sz, &tid_command_long,
					CYPHAL_MEDIUM_TIMEOUT);

				xTaskNotifyGive(task_tx_can_handle);

				TX_QUEUE_MUTEX_GIVE;
			}
		}

		if (xQueueReceive(queue_status_text_report, &status_text_report, 0)) {

#if !MAVLINK_OR_CYPHAL

			if (messages_cyphal_uavcan_common_Statustext_1_0_serialize_(
					&status_text_report, common_status_text,
					&common_status_text_sz) >= 0) {
				TX_QUEUE_MUTEX_TAKE
				{
					validate = cyphal_publish_common_statustext(
						&canard, &tx_queue, CanardPriorityExceptional,
						common_status_text, common_status_text_sz,
						&tid_common_status_text, CYPHAL_MEDIUM_TIMEOUT);

					xTaskNotifyGive(task_tx_can_handle);

					TX_QUEUE_MUTEX_GIVE;
				}
			}
#else

			mavlink_msg_command_ack_encode(mav_handle.sysid, mav_handle.compid,
										   &msg, &command_ack);

			mav_send(&mav_handle, &msg);
#endif /* MAVLINK_OR_CYPHAL */
		}

		if (xQueueReceive(queue_mav_hb, &heartbeat, 0)) {

#if !MAVLINK_OR_CYPHAL

			heartbeat.value = current_node_mode;

			if (uavcan_node_Mode_1_0_serialize_(&heartbeat, node_mode_buf,
												&node_mode_sz) >= 0) {
				TX_QUEUE_MUTEX_TAKE
				{
					validate = cyphal_publish_node_mode_subject(
						&canard, &tx_queue, CanardPriorityImmediate,
						uavcan_node_Mode_1_0_FIXED_PORT_ID_, node_mode_buf,
						node_mode_sz, &tid_node_mode, CYPHAL_MEDIUM_TIMEOUT);

					xTaskNotifyGive(task_tx_can_handle);

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

							xTaskNotifyGive(task_tx_can_handle);
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
						&command_long, component_info_buf,
						&component_info_sz) >= 0) {

					compontent_info_time = xTaskGetTickCount();
				}
			}

			if (xQueueReceive(queue_mav_ack, &command_ack, 0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_common_CommandAck_1_0_serialize_(
						&command_ack, command_ack_buf, &command_ack_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate = cyphal_publish_common_command_ack(
							&canard, &tx_queue, CanardPriorityExceptional,
							command_ack_buf, command_ack_sz,
							&tid_common_command_ack, CYPHAL_MEDIUM_TIMEOUT);

						xTaskNotifyGive(task_tx_can_handle);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else

				mavlink_msg_command_ack_encode(
					mav_handle.sysid, mav_handle.compid, &msg, &command_ack);

				mav_send(&mav_handle, &msg);
#endif /* MAVLINK_OR_CYPHAL */
			}

			while (
				xQueueReceive(queue_mav_act_data, &lisum_power_hornet_ack, 0)) {

#if !MAVLINK_OR_CYPHAL
				uint8_t act_id = lisum_power_hornet_ack.act_id;

				if (messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0_serialize_(
						&lisum_power_hornet_ack, lisum_power_hornet_ack_buf,
						&lisum_power_hornet_ack_sz) >= 0) {
					TX_QUEUE_MUTEX_TAKE
					{
						validate =
							cyphal_publish_lisum_lisum_power_hornet_act_data_subject(
								&canard, &tx_queue, act_priorities[act_id],
								messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0_FIXED_PORT_ID_,
								lisum_power_hornet_ack_buf,
								lisum_power_hornet_ack_sz,
								&tid_lisum_power_hornet_ack,
								CYPHAL_LOW_TIMEOUT);

						xTaskNotifyGive(task_tx_can_handle);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else
				mavlink_msg_lisum_power_hornet_act_data_encode(
					mav_handle.sysid, mav_handle.compid, &msg,
					&lisum_power_hornet_ack);

				mav_send(&mav_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}
			if (xQueueReceive(queue_mav_manual_ctrl, &lisum_manual_ctrl_hornet,
							  0)) {

#if !MAVLINK_OR_CYPHAL

				if (messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_serialize_(
						&lisum_manual_ctrl_hornet, lisum_manual_ctrl_hornet_buf,
						&lisum_manual_ctrl_hornet_sz) >= 0) {

					TX_QUEUE_MUTEX_TAKE
					{
						validate =
							cyphal_publish_lisum_lisum_manual_ctrl_hornet_subject(
								&canard, &tx_queue, CanardPriorityImmediate,
								messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_FIXED_PORT_ID_,
								lisum_manual_ctrl_hornet_buf,
								lisum_manual_ctrl_hornet_sz,
								&tid_lisum_manual_ctrl_hornet,
								CYPHAL_MEDIUM_TIMEOUT);

						xTaskNotifyGive(task_tx_can_handle);

						TX_QUEUE_MUTEX_GIVE;
					}
				}
#else
				mavlink_msg_lisum_manual_ctrl_hornet_encode(
					mav_handle.sysid, mav_handle.compid, &msg,
					&lisum_manual_ctrl_hornet);

				mav_send(&mav_handle, &msg);

#endif /* MAVLINK_OR_CYPHAL */
			}
		}
	}
}

#if MAVLINK_OR_CYPHAL

static void mav_recv_cb(mav_t *mav, mavlink_message_t *msg, void *arg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: {
		HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
							 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));

		break;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG: {
		mavlink_command_long_t cmd_long = {0};

		mavlink_msg_command_long_decode(msg, &cmd_long);

		if (cmd_long.command == MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC) {
			/* It's a request for software/hardware version */
			mavlink_message_t reply_msg;

			mavlink_component_information_basic_t reply_info;

			reply_info = make_comp_info_basic();

			mavlink_msg_component_information_basic_encode(
				msg->sysid, msg->compid, &reply_msg, &reply_info);

			mav_send(&mav_handle, &reply_msg); /* Send data to gw_sky */

			break; /* We know what was the long command, end of case */
		}

		/* check target */
		if ((cmd_long.target_system != mav->sysid ||
			 cmd_long.target_component != mav->compid) &&
			(cmd_long.target_system != 0 && cmd_long.target_component != 0))
			break;

		mav_handle_cmd_long(msg, &cmd_long);

		break;
	}
	case MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET: {
		mavlink_lisum_manual_ctrl_hornet_t ctrl = {0};

		mavlink_msg_lisum_manual_ctrl_hornet_decode(msg, &ctrl);

		task_epos_ctrl(&ctrl);

		break;
	}
	default:
		break;
	}
}

#endif /* MAVLINK_OR_CYPHAL */

static void timer_mav_hb_cb(TimerHandle_t xTimer)
{
	mavlink_heartbeat_t mav_hb = {.type = MAV_TYPE_GENERIC,
								  .autopilot = MAV_AUTOPILOT_GENERIC,
								  .base_mode = 0,
								  .custom_mode = 0,
								  .system_status = MAV_STATE_STANDBY};

	mav_hb.custom_mode = task_epos_state;

	xQueueSendToBack(queue_mav_hb, &mav_hb, 0);
}

static void mav_handle_cmd_long(mavlink_message_t *msg,
								mavlink_command_long_t *cmd_long)
{
	switch (cmd_long->command) {
	case MAV_CMD_HORNET_HOMING:
	case MAV_CMD_HORNET_ARM_DISARM:
	case MAV_CMD_HORNET_INIT:
	case MAV_CMD_HORNET_FAULT_RESET:
	case MAV_CMD_HORNET_SWEEP:
		task_epos_cmd(cmd_long, msg->sysid, msg->compid);

		break;
	default: {
		mavlink_command_ack_t ack = {.command = cmd_long->command,
									 .result = MAV_RESULT_UNSUPPORTED,
									 .progress = 0xFF,
									 .result_param2 = 0,
									 .target_system = msg->sysid,
									 .target_component = msg->compid};

		xQueueSendToBack(queue_mav_ack, &ack, 0);

		break;
	}
	}
}
