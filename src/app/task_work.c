/*******************************************************************************
 * @file    task_work.c
 * @brief   Background task processing for the system
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 *
 * This file contains the main background task loop and Cyphal message handler.
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "task_work.h"
#include "types.h"
#include <stdint.h>
#include <string.h>

// Middleware and drivers
#include "FreeRTOS.h"
#include "gpio.h"
#include "mav.h"
#include "queue.h"
#include "timers.h"
#include "uart_tl.h"
#include "udp.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define ARP_GRAT_PERIOD_MS 10000

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
extern struct CanardInstance canard;
extern struct CanardTxQueue tx_queue;

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern udp_t udp;

#if !MAVLINK_OR_CYPHAL

extern SemaphoreHandle_t mutex_mav;
extern mav_t mav_gw_gnd_handle;
extern QueueHandle_t queue_cyphal_rx;

extern const uint8_t dev_mav_sysid;
extern const uint8_t dev_mav_compid;

extern const uint8_t gw_gnd_mav_sysid;
extern const uint8_t gw_gnd_mav_compid;

extern const uint8_t heli_mav_sysid;
extern const uint8_t heli_mav_compid;

extern const uint8_t mp_mav_sysid;
extern const uint8_t mp_mav_compid;

extern const uint8_t bb_mav_sysid;
extern const uint8_t bb_mav_compid;

extern const uint8_t ins_mav_sysid;
extern const uint8_t ins_mav_compid;

extern const uint8_t pwr_man_mav_sysid;
extern const uint8_t pwr_man_mav_compid;

extern const uint8_t act_master_mav_sysid;
extern const uint8_t act_master_mav_compid;

extern const uint8_t ins_cots_mav_sysid;
extern const uint8_t ins_cots_mav_compid;

extern const uint8_t batt1_mav_sysid;
extern const uint8_t batt1_mav_compid;

extern const uint8_t batt2_mav_sysid;
extern const uint8_t batt2_mav_compid;

#endif /* MAVLINK_OR_CYPHAL */

uint16_t cnt = 0;
uint16_t cnt2 = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_work(void* arg)
{
	TickType_t arp_grat_time = xTaskGetTickCount();
	can_hal_msg_t rx;

	for (;;) {
		// Process UDP data
		udp_process(&udp);

		// Periodically send gratuitous ARP
		if (xTaskGetTickCount() - arp_grat_time >= ARP_GRAT_PERIOD_MS) {
			arp_grat_time = xTaskGetTickCount();
			udp_arp_grat(&udp);
		}

#if !MAVLINK_OR_CYPHAL

		while (xQueueReceive(queue_cyphal_rx, &rx, 0) == pdPASS) {
			cyphal_process(&canard, rx);
		}

#endif
	}
}

void handle_cyphal_transfer(const struct CanardRxTransfer* tr)
{
	xSemaphoreTake(mutex_mav, portMAX_DELAY);

	switch (tr->metadata.port_id) {

	/* Actuator */
	case messages_cyphal_uavcan_minimal_Heartbeat_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_minimal_Heartbeat_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_minimal_Heartbeat_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			messages_cyphal_uavcan_minimal_Heartbeat_1_0 minimal_heartbeat;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&minimal_heartbeat, 0, sizeof(minimal_heartbeat));

			// Map Cyphal Manual Control fields to MAVLink fields
			minimal_heartbeat.autopilot = arr.autopilot;
			minimal_heartbeat.base_mode = arr.base_mode;
			minimal_heartbeat.custom_mode = arr.custom_mode;
			minimal_heartbeat.mavlink_version = arr.mavlink_version;
			minimal_heartbeat.system_status = arr.system_status;
			minimal_heartbeat.type_ = arr.type_;

			mavlink_msg_heartbeat_encode_chan(act_master_mav_sysid, act_master_mav_compid,
											  MAVLINK_COMM_0, &tx_msg, &minimal_heartbeat);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	case messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_ComponentInformationBasic_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_common_ComponentInformationBasic_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;

			mavlink_component_information_basic_t reply_info;
			memcpy(&reply_info, &arr, sizeof(reply_info));

			int val = (reply_info.serial_number[5] - '0') * 100 +
					  (reply_info.serial_number[6] - '0') * 10 +
					  (reply_info.serial_number[7] - '0');

			switch (val) {
			case 20: {
				mavlink_msg_component_information_basic_encode_chan(
					bb_mav_sysid, bb_mav_compid, MAVLINK_COMM_0, &tx_msg, &reply_info);
				break;
			}
			case 14: {
				mavlink_msg_component_information_basic_encode_chan(
					ins_mav_sysid, ins_mav_compid, MAVLINK_COMM_0, &tx_msg, &reply_info);
				break;
			}
			case 16: {
				mavlink_msg_component_information_basic_encode_chan(
					pwr_man_mav_sysid, pwr_man_mav_compid, MAVLINK_COMM_0, &tx_msg, &reply_info);
				break;
			}
			case 18: {
				mavlink_msg_component_information_basic_encode_chan(
					act_master_mav_sysid, act_master_mav_compid, MAVLINK_COMM_0, &tx_msg,
					&reply_info);
				break;
			}
			default: {
				mavlink_msg_component_information_basic_encode_chan(
					dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0, &tx_msg, &reply_info);
				break;
			}
			}

			/* Respond, then forward to gnd answer */
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	case messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_lisum_power_hornet_act_data_t power_hornet_act_data;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&power_hornet_act_data, 0, sizeof(power_hornet_act_data));

			// Map Cyphal Manual Control fields to MAVLink fields
			power_hornet_act_data.abs_enc_sw_act1 = arr.abs_enc_sw_act1;
			power_hornet_act_data.abs_enc_sw_act2 = arr.abs_enc_sw_act2;
			power_hornet_act_data.abs_enc_sw_act3 = arr.abs_enc_sw_act3;
			power_hornet_act_data.abs_enc_sw_act4 = arr.abs_enc_sw_act4;
			power_hornet_act_data.sw_act1 = arr.sw_act1;
			power_hornet_act_data.sw_act2 = arr.sw_act2;
			power_hornet_act_data.sw_act3 = arr.sw_act3;
			power_hornet_act_data.sw_act4 = arr.sw_act4;
			power_hornet_act_data.pos_act1 = arr.pos_act1;
			power_hornet_act_data.pos_act2 = arr.pos_act2;
			power_hornet_act_data.pos_act3 = arr.pos_act3;
			power_hornet_act_data.pos_act4 = arr.pos_act4;
			power_hornet_act_data.curr_act1 = arr.curr_act1;
			power_hornet_act_data.curr_act2 = arr.curr_act2;
			power_hornet_act_data.curr_act3 = arr.curr_act3;
			power_hornet_act_data.curr_act4 = arr.curr_act4;
			power_hornet_act_data.vel_act1 = arr.vel_act1;
			power_hornet_act_data.vel_act2 = arr.vel_act2;
			power_hornet_act_data.vel_act3 = arr.vel_act3;
			power_hornet_act_data.vel_act4 = arr.vel_act4;
			power_hornet_act_data.abs_pos_act1 = arr.abs_pos_act1;
			power_hornet_act_data.abs_pos_act2 = arr.abs_pos_act2;
			power_hornet_act_data.abs_pos_act3 = arr.abs_pos_act3;
			power_hornet_act_data.abs_pos_act4 = arr.abs_pos_act4;

			mavlink_msg_lisum_power_hornet_act_data_encode_chan(
				act_master_mav_sysid, act_master_mav_compid, MAVLINK_COMM_0, &tx_msg,
				&power_hornet_act_data);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	case messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_lisum_manual_ctrl_hornet_t manual_ctrl_hornet;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&manual_ctrl_hornet, 0, sizeof(manual_ctrl_hornet));

			// Map Cyphal Manual Control fields to MAVLink fields
			manual_ctrl_hornet.pos_sp_act1 = arr.pos_sp_act1;
			manual_ctrl_hornet.pos_sp_act2 = arr.pos_sp_act2;
			manual_ctrl_hornet.pos_sp_act3 = arr.pos_sp_act3;
			manual_ctrl_hornet.pos_sp_act4 = arr.pos_sp_act4;
			manual_ctrl_hornet.mode = arr.mode;

			mavlink_msg_lisum_manual_ctrl_hornet_encode_chan(act_master_mav_sysid,
															 act_master_mav_compid, MAVLINK_COMM_0,
															 &tx_msg, &manual_ctrl_hornet);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	case messages_cyphal_uavcan_common_CommandAck_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_CommandAck_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_common_CommandAck_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_command_ack_t command_ack;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&command_ack, 0, sizeof(command_ack));

			// Map Cyphal CommandAck fields to MAVLink fields
			command_ack.command = arr.command;
			command_ack.result = arr.result;
			command_ack.progress = arr.progress;
			command_ack.result_param2 = arr.result_param2;
			command_ack.target_system = arr.target_system;
			command_ack.target_component = arr.target_component;

			mavlink_msg_command_ack_encode_chan(act_master_mav_sysid, act_master_mav_compid,
												MAVLINK_COMM_0, &tx_msg, &command_ack);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	/* INS IMU */
	case messages_cyphal_uavcan_common_ScaledImu_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_ScaledImu_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_common_ScaledImu_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_scaled_imu_t scaled_imu;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&scaled_imu, 0, sizeof(scaled_imu));

			// Map Cyphal ScaledImu fields to MAVLink scaled_imu fields
			scaled_imu.xacc = arr.xacc;
			scaled_imu.yacc = arr.yacc;
			scaled_imu.zacc = arr.zacc;
			scaled_imu.xgyro = arr.xgyro;
			scaled_imu.ygyro = arr.ygyro;
			scaled_imu.zgyro = arr.zgyro;
			scaled_imu.xmag = arr.xmag;
			scaled_imu.ymag = arr.ymag;
			scaled_imu.zmag = arr.zmag;

			mavlink_msg_scaled_imu_encode_chan(ins_mav_sysid, ins_mav_compid, MAVLINK_COMM_0,
											   &tx_msg, &scaled_imu);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	/* INS GNSS */
	case messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_lisum_gnss_recv_data_t lisum_gnss_recv;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&lisum_gnss_recv, 0, sizeof(lisum_gnss_recv));

			// Map Cyphal GNSS fields to MAVLink fields (example, adjust as needed)
			lisum_gnss_recv.lat = arr.lat;
			lisum_gnss_recv.lon = arr.lon;
			lisum_gnss_recv.alt = arr.alt;
			lisum_gnss_recv.head = arr.head;
			lisum_gnss_recv.head_rtk = arr.head_rtk;
			lisum_gnss_recv.vel_ne = arr.vel_ne;
			lisum_gnss_recv.vel_d = arr.vel_d;
			lisum_gnss_recv.dev_id = arr.dev_id;
			lisum_gnss_recv.fix_type = arr.fix_type;
			lisum_gnss_recv.carr_sol = arr.carr_sol;
			lisum_gnss_recv.rtk_head_valid = arr.rtk_head_valid;
			lisum_gnss_recv.sv_num = arr.sv_num;

			// Add more field mappings as needed
			mavlink_msg_lisum_gnss_recv_data_encode_chan(ins_mav_sysid, ins_mav_compid,
														 MAVLINK_COMM_0, &tx_msg, &lisum_gnss_recv);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	/* INS Fusion */
	case messages_cyphal_uavcan_common_Attitude_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_Attitude_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_common_Attitude_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_attitude_t attitude;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&attitude, 0, sizeof(attitude));

			attitude.roll = arr.roll;
			attitude.pitch = arr.pitch;
			attitude.yaw = arr.yaw;
			attitude.rollspeed = arr.rollspeed;
			attitude.pitchspeed = arr.pitchspeed;
			attitude.yawspeed = arr.yawspeed;

			mavlink_msg_attitude_encode_chan(ins_mav_sysid, ins_mav_compid, MAVLINK_COMM_0, &tx_msg,
											 &attitude);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	/* INS Baro */
	case messages_cyphal_uavcan_common_ScaledPressure_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_ScaledPressure_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_common_ScaledPressure_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_scaled_pressure_t scaled_pressure;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&scaled_pressure, 0, sizeof(scaled_pressure));

			// Map Cyphal ScaledPressure fields to MAVLink fields
			scaled_pressure.time_boot_ms = arr.time_boot_ms;
			scaled_pressure.press_abs = arr.press_abs;
			scaled_pressure.press_diff = arr.press_diff;
			scaled_pressure.temperature = arr.temperature;
			scaled_pressure.temperature_press_diff = arr.temperature_press_diff;

			mavlink_msg_scaled_pressure_encode_chan(ins_mav_sysid, ins_mav_compid, MAVLINK_COMM_0,
													&tx_msg, &scaled_pressure);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	case messages_cyphal_uavcan_common_Altitude_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_Altitude_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;

		int8_t rc = messages_cyphal_uavcan_common_Altitude_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_altitude_t altitude;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&altitude, 0, sizeof(altitude));

			// Map Cyphal Altitude fields to MAVLink fields
			altitude.time_usec = arr.time_usec;
			altitude.altitude_monotonic = arr.altitude_monotonic;
			altitude.altitude_amsl = arr.altitude_amsl;
			altitude.altitude_local = arr.altitude_local;
			altitude.altitude_relative = arr.altitude_relative;
			altitude.altitude_terrain = arr.altitude_terrain;
			altitude.bottom_clearance = arr.bottom_clearance;

			mavlink_msg_altitude_encode_chan(ins_mav_sysid, ins_mav_compid, MAVLINK_COMM_0, &tx_msg,
											 &altitude);
			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	/* IMU ANPP*/
	// case messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0_FIXED_PORT_ID_: {

	// 	messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0 arr;

	// 	memset(&arr, 0, sizeof(arr));

	// 	memset(&tx_msg, 0, sizeof(tx_msg));

	// 	size_t in_size = tr->payload.size;
	// 	int8_t rc =
	// messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0_deserialize_( 		&arr,
	// (const uint8_t*)tr->payload.data, &in_size);

	// 	if (rc < 0) {
	// 		goto out; // Invalid message
	// 	}

	// 	if (rc == 0) {
	// 		mavlink_message_t tx_msg;
	// 		mavlink_lisum_sensor_airspeed_data_t lisum_sensor_airspeed;

	// 		memset(&lisum_sensor_airspeed, 0, sizeof(lisum_sensor_airspeed));

	// 		// Map Cyphal Airspeed fields to MAVLink fields (example, adjust as needed)
	// 		lisum_sensor_airspeed.airspeed = arr.airspeed;
	// 		lisum_sensor_airspeed.raw_press = arr.raw_press;
	// 		lisum_sensor_airspeed.temperature = arr.temperature;
	// 		lisum_sensor_airspeed.id = arr.id;
	// 		lisum_sensor_airspeed.flags = arr.flags;

	// 		// Add more field mappings as needed
	// 		mavlink_msg_lisum_sensor_airspeed_data_encode_chan(
	// 			bb_mav_sysid, bb_mav_compid, MAVLINK_COMM_0, &tx_msg,
	// &lisum_sensor_airspeed);

	// 		mav_send(&mav_gw_gnd_handle, &tx_msg);
	// 	}
	// 	break;
	// }

	/* INS BMS */
	case messages_cyphal_uavcan_common_BatteryStatus_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_BatteryStatus_1_0 arr;

		cnt2++;
		if (cnt2 >= 100) {
			cnt2 = 0;
			HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 26,
								 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 26));
		}

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;
		int8_t rc = messages_cyphal_uavcan_common_BatteryStatus_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_battery_status_t battery_status;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&battery_status, 0, sizeof(battery_status));

			// Map Cyphal Airspeed fields to MAVLink fields (example, adjust as needed)
			battery_status.current_consumed = arr.current_consumed;
			battery_status.energy_consumed = arr.energy_consumed;
			battery_status.temperature = arr.temperature;
			memcpy(battery_status.voltages, arr.voltages, sizeof(battery_status.voltages));
			battery_status.current_battery = arr.current_battery;
			battery_status.id = arr.id;
			battery_status.battery_function = arr.battery_function;
			battery_status.type = arr.type_;
			battery_status.battery_remaining = arr.battery_remaining;
			battery_status.time_remaining = arr.time_remaining;
			battery_status.charge_state = arr.charge_state;
			memcpy(battery_status.voltages_ext, arr.voltages_ext,
				   sizeof(battery_status.voltages_ext));
			battery_status.mode = arr.mode;
			battery_status.fault_bitmask = arr.fault_bitmask;

			if (tr->metadata.priority == CanardPriorityExceptional) {
				mavlink_msg_battery_status_encode_chan(batt1_mav_sysid, batt1_mav_compid,
													   MAVLINK_COMM_0, &tx_msg, &battery_status);
			} else {
				mavlink_msg_battery_status_encode_chan(batt2_mav_sysid, batt2_mav_compid,
													   MAVLINK_COMM_0, &tx_msg, &battery_status);
			}

			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	// /* INS Motor */
	case messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0 arr;

		cnt++;
		if (cnt >= 100) {
			cnt = 0;
			HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
								 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));
		}

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;
		int8_t rc = messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (rc == 0) {
			mavlink_message_t tx_msg;
			mavlink_lisum_power_motor_scaled_data_t motor_scaled_data;

			memset(&tx_msg, 0, sizeof(tx_msg));
			memset(&motor_scaled_data, 0, sizeof(motor_scaled_data));

			// Map Cyphal Airspeed fields to MAVLink fields (example, adjust as needed)
			// motor_scaled_data.airspeed = arr.airspeed;
			// motor_scaled_data.raw_press = arr.raw_press;
			// motor_scaled_data.temperature = arr.temperature;
			// motor_scaled_data.id = arr.id;
			// motor_scaled_data.flags = arr.flags;
			// Add more field mappings as needed
			mavlink_msg_lisum_power_motor_scaled_data_encode_chan(
				pwr_man_mav_sysid, pwr_man_mav_compid, MAVLINK_COMM_0, &tx_msg, &motor_scaled_data);

			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

		/* ----------------------- File Transfer Protocol --------------------------- */

	case uavcan_primitive_array_Integer8_1_0_FIXED_PORT_ID_: {

		uavcan_primitive_array_Integer8_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;
		int8_t rc = uavcan_primitive_array_Integer8_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {
			goto out; // Invalid message
		}

		if (arr.value.count == 0) {
			goto out; // No data received
		}

		if (rc == 0) {
			mavlink_file_transfer_protocol_t ftp;
			memset(&ftp, 0, sizeof(ftp));

			ftp.target_network = arr.value.elements[0];
			ftp.target_system = arr.value.elements[1];
			ftp.target_component = arr.value.elements[2];

			memcpy(&ftp.payload, &arr.value.elements[3], sizeof(ftp.payload));

			mavlink_message_t tx_msg;
			memset(&tx_msg, 0, sizeof(tx_msg));

			if (ftp.target_component == dev_mav_compid) {
				mavlink_msg_file_transfer_protocol_encode_chan(dev_mav_sysid, dev_mav_compid,
															   MAVLINK_COMM_0, &tx_msg, &ftp);
			} else if (ftp.target_component == act_master_mav_compid) {
				mavlink_msg_file_transfer_protocol_encode_chan(
					act_master_mav_sysid, act_master_mav_compid, MAVLINK_COMM_0, &tx_msg, &ftp);
			} else if (ftp.target_component == ins_cots_mav_compid) {
				mavlink_msg_file_transfer_protocol_encode_chan(
					ins_cots_mav_sysid, ins_cots_mav_compid, MAVLINK_COMM_0, &tx_msg, &ftp);
			} else if (ftp.target_component == pwr_man_mav_compid) {
				mavlink_msg_file_transfer_protocol_encode_chan(
					pwr_man_mav_sysid, pwr_man_mav_compid, MAVLINK_COMM_0, &tx_msg, &ftp);
			} else if (ftp.target_component == ins_mav_compid) {
				mavlink_msg_file_transfer_protocol_encode_chan(ins_mav_sysid, ins_mav_compid,
															   MAVLINK_COMM_0, &tx_msg, &ftp);
			} else if (ftp.target_component == bb_mav_compid) {
				mavlink_msg_file_transfer_protocol_encode_chan(bb_mav_sysid, bb_mav_compid,
															   MAVLINK_COMM_0, &tx_msg, &ftp);
			}

			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

		/* -------------------------- Node Mode ----------------------------- */

	case uavcan_node_Mode_1_0_FIXED_PORT_ID_: {
		mavlink_message_t tx_msg;
		mavlink_heartbeat_t mavlink_heartbeat_data;

		uavcan_node_Mode_1_0 data;

		memset(&tx_msg, 0, sizeof(tx_msg));
		memset(&data, 0, sizeof(data));

		size_t in_size = tr->payload.size;
		int8_t rc =
			uavcan_node_Mode_1_0_deserialize_(&data, (const uint8_t*)tr->payload.data, &in_size);

		if (rc < 0) {

			return; // Invalid message
		}

		mavlink_heartbeat_data = (mavlink_heartbeat_t){.type = MAV_TYPE_LOG,
													   .autopilot = MAV_AUTOPILOT_GENERIC,
													   .base_mode = 0,
													   .custom_mode = 0,
													   .system_status = MAV_STATE_STANDBY};

		switch (data.value) {
		case uavcan_node_Mode_1_0_OPERATIONAL:
			mavlink_heartbeat_data.system_status = MAV_STATE_ACTIVE;
			break;
		case uavcan_node_Mode_1_0_INITIALIZATION:
			mavlink_heartbeat_data.system_status = MAV_STATE_BOOT;
			break;
		case uavcan_node_Mode_1_0_MAINTENANCE:
			mavlink_heartbeat_data.system_status = MAV_STATE_STANDBY;
			break;
		case uavcan_node_Mode_1_0_SOFTWARE_UPDATE:
			mavlink_heartbeat_data.system_status = MAV_STATE_CRITICAL;
			break;
		case 4:
			mavlink_heartbeat_data.system_status = MAV_STATE_UNINIT;
			break;
		default:
			break;
		}

		if (tr->metadata.priority == CanardPriorityExceptional) {
			mavlink_msg_heartbeat_encode_chan(pwr_man_mav_sysid, pwr_man_mav_compid, MAVLINK_COMM_0,
											  &tx_msg, &mavlink_heartbeat_data);
		} else if (tr->metadata.priority == CanardPriorityImmediate) {
			mavlink_msg_heartbeat_encode_chan(act_master_mav_sysid, act_master_mav_compid,
											  MAVLINK_COMM_0, &tx_msg, &mavlink_heartbeat_data);
		} else if (tr->metadata.priority == CanardPriorityFast) {
			mavlink_msg_heartbeat_encode_chan(bb_mav_sysid, bb_mav_compid, MAVLINK_COMM_0, &tx_msg,
											  &mavlink_heartbeat_data);
		} else {
			mavlink_msg_heartbeat_encode_chan(ins_mav_sysid, ins_mav_compid, MAVLINK_COMM_0,
											  &tx_msg, &mavlink_heartbeat_data);
		}
		mav_send(&mav_gw_gnd_handle, &tx_msg);

		break;
	}

		/* -------------------------- Autopilot Version ----------------------------- */

	case messages_cyphal_uavcan_common_AutopilotVersion_1_0_FIXED_PORT_ID_: {

		messages_cyphal_uavcan_common_AutopilotVersion_1_0 arr;

		memset(&arr, 0, sizeof(arr));

		size_t in_size = tr->payload.size;
		int8_t rc = messages_cyphal_uavcan_common_AutopilotVersion_1_0_deserialize_(
			&arr, (const uint8_t*)tr->payload.data, &in_size);

		if (rc == 0) {

			mavlink_autopilot_version_t autopilot_version;

			memset(&autopilot_version, 0, sizeof(autopilot_version));

			autopilot_version.capabilities = arr.capabilities;
			autopilot_version.uid = arr.uid;
			autopilot_version.flight_sw_version = arr.flight_sw_version;
			autopilot_version.middleware_sw_version = arr.middleware_sw_version;
			autopilot_version.os_sw_version = arr.os_sw_version;
			autopilot_version.board_version = arr.board_version;
			autopilot_version.vendor_id = arr.vendor_id;
			autopilot_version.product_id = arr.product_id;
			memcpy(autopilot_version.flight_custom_version, arr.flight_custom_version, 8);
			memcpy(autopilot_version.middleware_custom_version, arr.middleware_custom_version, 8);
			memcpy(autopilot_version.os_custom_version, arr.os_custom_version, 8);
			memcpy(autopilot_version.uid2, arr.uid2, 18);

			mavlink_message_t tx_msg;
			memset(&tx_msg, 0, sizeof(tx_msg));

			mavlink_msg_autopilot_version_encode_chan(dev_mav_sysid, dev_mav_compid, MAVLINK_COMM_0,
													  &tx_msg, &autopilot_version);

			mav_send(&mav_gw_gnd_handle, &tx_msg);
		}
		break;
	}

	default:
		break;
	}

out:
	xSemaphoreGive(mutex_mav);
}

/*******************************************************************************
 * CAN interrupt callback
 ******************************************************************************/

void cyphal_cb(void* usr_arg, uint32_t int_status)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (int_status & CAN_HAL_INT_MASK_RI) {
		can_hal_msg_t rx;
		while (HAL_CAN_ReceiveMessage(CAN_HAL_INSTANCE_0, &rx, 0) == lStatus_Success) {
			if (xQueueSendFromISR(queue_cyphal_rx, &rx, &xHigherPriorityTaskWoken) != pdPASS) {
			}
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
