/**
 * @file    task_lob.c
 * @brief   Task log - log data
 * @version 1.0.0
 * @date    10.04.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * System Includes
 ******************************************************************************/

#include <inttypes.h>
#include <stdint.h>

/*******************************************************************************
 * Application Includes
 ******************************************************************************/

#include "ftp_handler.h"
#include "ftp_helper.h"
#include "main.h"
#include "protocol_utility.h"
#include "task_log.h"
#include "task_work.h"
#include "types.h"

/*******************************************************************************
 * ESP-IDF Core Includes
 ******************************************************************************/
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

/*******************************************************************************
 * ESP-IDF Driver Includes
 ******************************************************************************/
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/spi_master.h"

/*******************************************************************************
 * External Hardware Driver Includes
 ******************************************************************************/

#include "lfs.h"
#include "lfs_def.h"
#include "mav.h"
#include "psram_ring.h"

/*******************************************************************************
 * FreeRTOS Includes
 ******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*******************************************************************************
 * ULog Format Includes
 ******************************************************************************/

#include "cyphal/reg/udral/physics/thermodynamics/ulog_reg_udral_physics_thermodynamics_201_pressuretempvarts_0_1.h"
#include "cyphal/uavcan/primitive/scalar/ulog_uavcan_primitive_scalar_503_integer16_1_0.h"
#include "cyphal/uavcan/primitive/scalar/ulog_uavcan_primitive_scalar_510_integer64_1_0.h"
#include "cyphal/uavcan/si/sample/angle/ulog_uavcan_si_sample_angle_509_scalar_1_0.h"
#include "cyphal/uavcan/si/sample/length/ulog_uavcan_si_sample_length_202_scalar_1_0.h"
#include "cyphal/uavcan/si/sample/length/ulog_uavcan_si_sample_length_511_widevector3_1_0.h"
#include "cyphal/uavcan/si/sample/velocity/ulog_uavcan_si_sample_velocity_507_vector3_1_0.h"
#include "cyphal\uavcan\primitive\scalar\ulog_uavcan_primitive_scalar_510_integer64_1_0.h"
#include "cyphal\uavcan\si\sample\angular_velocity\ulog_uavcan_si_sample_angular_velocity_512_vector3_1_0.h"
#include "cyphal\uavcan\si\sample\magnetic_field_strength\ulog_uavcan_si_sample_magnetic_field_strength_517_vector3_1_0.h"
#include "cyphal\uavcan\si\unit\angle\ulog_uavcan_si_unit_angle_513_quaternion_1_0.h"
#include "cyphal\uavcan\si\unit\angular_acceleration\ulog_uavcan_si_unit_angular_acceleration_515_vector3_1_0.h"
#include "cyphal\uavcan\si\unit\angular_velocity\ulog_uavcan_si_unit_angular_velocity_516_vector3_1_0.h"
#include "cyphal\uavcan\si\unit\pressure\ulog_uavcan_si_unit_pressure_506_scalar_1_0.h"
#include "cyphal\uavcan\si\unit\temperature\ulog_uavcan_si_unit_temperature_505_scalar_1_0.h"
#include "cyphal\uavcan\si\unit\velocity\ulog_uavcan_si_unit_velocity_504_scalar_1_0.h"

#include "ulog.h"
#include "ulog_adsb_vehicle.h"
#include "ulog_ais_vessel.h"
#include "ulog_altitude.h"
#include "ulog_attitude.h"
#include "ulog_battery_status.h"
#include "ulog_def.h"
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

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define MAX_ULOG_DATA_SIZE                                                               \
	sizeof(lisum_LisumPowerMotorScaledData_1_0) + sizeof(rb_entry_header_t) +            \
		sizeof(uint32_t)		  // Maximum ULog data size TO LOG FILE !!!
#define LFS_SYNC_PERIOD_US 400000 // 400ms

#define mutex_lock(mutex)                                                                \
	{                                                                                    \
		xSemaphoreTake(mutex, portMAX_DELAY);                                            \
	}
#define mutex_unlock(mutex)                                                              \
	{                                                                                    \
		xSemaphoreGive(mutex);                                                           \
	}

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern volatile node_mode_state_t current_node_mode;

#if MAVLINK_OR_CYPHAL

/* UDP Mavlink */
extern mav_t mav_gw_sky_handle;
extern mav_link_t mav_link_gw_sky_udp;
extern mav_link_t mav_link_motor_control_udp;
extern mav_link_t mav_link_vesc_control_udp;

#endif /* MAVLINK_OR_CYPHAL */

/* FreeRTOS */

// Semaphores
extern SemaphoreHandle_t mutex_mav_ftp;
extern SemaphoreHandle_t semaphore_logging_ready;

// Tasks Handles
extern TaskHandle_t task_work_handle;
extern TaskHandle_t task_log_global;

extern ulog_t ulog;
extern uint32_t boot_count;
extern uint16_t batt_status1_id;
extern uint16_t batt_status2_id;
extern uint16_t motor_scaled_id;
extern uint16_t scaled_imu1_id;
extern uint16_t scaled_imu2_id;
extern uint16_t scaled_pressure_id;
extern uint16_t altitude_id;
extern uint16_t attitude1_id;
extern uint16_t attitude2_id;
extern uint16_t lisum_gnss_data1_id;
extern uint16_t lisum_gnss_data2_id;
extern uint16_t lisum_gnss_data3_id;
extern uint16_t lisum_airspeed_data_id;
extern uint16_t lisum_act_data_id;
extern uint16_t lisum_manual_ctrl_id;
extern uint16_t lisum_propulsion_tb_status_id;
extern uint16_t lisum_esc_status_id;
extern uint16_t named_value_float_id;
extern uint16_t lisum_power_motor_vesc_id;
extern uint16_t ais_vessel_id;
extern uint16_t adsb_vehicle_id;
extern uint16_t uavcan_lisum_lisum_gnss_recv_data_1_0_id;

extern uint8_t dev_mav_sysid;
extern uint8_t dev_mav_compid;

extern uint8_t pwr_man_mav_sysid;
extern uint8_t pwr_man_mav_compid;

extern uint8_t ins_mav_sysid;
extern uint8_t ins_mav_compid;

extern uint8_t act_master_mav_sysid;
extern uint8_t act_master_mav_compid;

extern uint8_t ins_cots_mav_sysid;
extern uint8_t ins_cots_mav_compid;

extern uint8_t heli_mav_sysid;
extern uint8_t heli_mav_compid;

extern uint8_t batt1_mav_sysid;
extern uint8_t batt1_mav_compid;

extern uint8_t batt2_mav_sysid;
extern uint8_t batt2_mav_compid;

extern uint8_t motor_control_mav_sysid;
extern uint8_t motor_control_mav_compid;

extern uint8_t mp_mav_sysid;
extern uint8_t mp_mav_compid;

extern uint8_t vesc_control_mav_sysid;
extern uint8_t vesc_control_mav_compid;

extern ring_t ring_buffer;

extern lfs_t lfs;
extern lfs_file_t lfs_file;

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_log(void* arg)
{

	TickType_t sync_time = xTaskGetTickCount();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	TickType_t log_t0;
#pragma GCC diagnostic pop
	int err = 0;

	uint8_t read_buffer[MAX_ULOG_DATA_SIZE];
	uint16_t msg_id;
	size_t data_len;

	const TickType_t xPeriod = pdMS_TO_TICKS(1);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	int64_t last_sync_time_us = esp_timer_get_time();
	int64_t now_us = esp_timer_get_time();

	bool data_was_read = false;

	for (;;) {

		vTaskDelayUntil(&xLastWakeTime, xPeriod);

		data_was_read = false;

		/* Non blocking loop for parsing data from ring buffer */
		while (ring_used(&ring_buffer) >= sizeof(rb_entry_header_t)) {

			if (xTaskGetTickCount() - sync_time >= LOG_SYNC_PERIOD_MS) {

				mutex_lock(mutex_mav_ftp);

				sync_time = xTaskGetTickCount();

				ulog_sync(&ulog);

				mutex_unlock(mutex_mav_ftp);
			}

			if (current_node_mode == NODE_MODE_OPERATIONAL) {
				now_us = esp_timer_get_time();
			}

			if (((now_us - last_sync_time_us) >= LFS_SYNC_PERIOD_US) &&
				(current_node_mode == NODE_MODE_OPERATIONAL)) {

				mutex_lock(mutex_mav_ftp);

				last_sync_time_us = now_us;

				int err = lfs_file_sync(&lfs, &lfs_file);

				mutex_unlock(mutex_mav_ftp);
			}

			/* Read header from Ring Buffer */
			rb_entry_header_t header;
			uint32_t header_read =
				ring_pop(&ring_buffer, (uint8_t*)&header, sizeof(rb_entry_header_t));

			if (header_read != sizeof(rb_entry_header_t)) {
				// printf("LOG TASK: Failed to read complete header\n");
				break;
			}

			/* Check lenght of payload */
			if (header.length > MAX_ULOG_DATA_SIZE || header.length == 0) {
				continue;
			}

			/* Check if it's enough data for valid payload */
			if (ring_used(&ring_buffer) < header.length) {
				// ESP_LOGE(TAG,
				// 		 "Not enough data for payload. Need: %u, Available: %u",
				// 		 header.length, ring_used(&ring_buffer));
				break;
			}

			/* Read payload from Ring Buffer */
			uint32_t payload_read = ring_pop(&ring_buffer, read_buffer, header.length);

			if (payload_read != header.length) {
				// printf("LOG TASK: Failed to read complete payload. Expected:
				// "
				// 	   "%" PRIu16 ", Got: %" PRIu32 "\n",
				// 	   header.length, payload_read);
				continue;
			}

			/* CRC Check */
			uint32_t calculated_crc = calculate_crc32(read_buffer, header.length);

			if (calculated_crc != header.crc32) {
				// printf("LOG TASK: CRC mismatch! Expected: 0x%08" PRIX32
				// 	   ", Got: 0x%08" PRIX32 ", "
				// 	   "Msg ID: %" PRIu16 "\n",
				// 	   header.crc32, calculated_crc, header.message_id);
				continue;
			}

			data_was_read = true;

			mutex_lock(mutex_mav_ftp);

			/* Parse the message and write data in ulog file */
			switch (header.message_id) {

			case common_BatteryStatus_1_0_FIXED_PORT_ID_: {

				ulog_battery_status_t* log_data = (ulog_battery_status_t*)read_buffer;

				if (log_data->id == 1) {
					ulog_write_battery_status(&ulog, batt_status1_id, log_data);
				} else {
					ulog_write_battery_status(&ulog, batt_status2_id, log_data);
				}
				break;
			}

				// case 605: {
				// 	ulog_lisum_power_motor_scaled_data_t* log_data =
				// 		(ulog_lisum_power_motor_scaled_data_t*)read_buffer;

				// 	ulog_write_lisum_power_motor_scaled_data(&ulog, motor_scaled_id,
				// 											 log_data);
				// 	break;
				// }

			case lisum_LisumPowerMotorScaledData_1_0_FIXED_PORT_ID_: {
				ulog_lisum_power_motor_scaled_data_t* log_data =
					(ulog_lisum_power_motor_scaled_data_t*)read_buffer;

				ulog_write_lisum_power_motor_scaled_data(&ulog, motor_scaled_id,
														 log_data);
				break;
			}

			case MAVLINK_MSG_ID_SCALED_IMU: {
				ulog_scaled_imu_t* log_data = (ulog_scaled_imu_t*)read_buffer;

				if (header.component_id == ins_mav_compid) {
					ulog_write_scaled_imu(&ulog, scaled_imu1_id, log_data);
				} else {
					ulog_write_scaled_imu(&ulog, scaled_imu2_id, log_data);
				}
				break;
			}

			case MAVLINK_MSG_ID_SCALED_PRESSURE: {
				ulog_scaled_pressure_t* log_data = (ulog_scaled_pressure_t*)read_buffer;

				ulog_write_scaled_pressure(&ulog, scaled_pressure_id, log_data);
				break;
			}

			case MAVLINK_MSG_ID_ATTITUDE: {
				ulog_attitude_t* log_data = (ulog_attitude_t*)read_buffer;

				if (header.component_id == ins_mav_compid) {
					ulog_write_attitude(&ulog, attitude1_id, log_data);
				} else {
					ulog_write_attitude(&ulog, attitude2_id, log_data);
				}
				break;
			}

			case common_Altitude_1_0_FIXED_PORT_ID_: {
				ulog_altitude_t* log_data = (ulog_altitude_t*)read_buffer;

				ulog_write_altitude(&ulog, altitude_id, log_data);
				break;
			}

			case lisum_LisumGnssRecvData_1_0_FIXED_PORT_ID_: {
				ulog_lisum_gnss_recv_data_t* log_data =
					(ulog_lisum_gnss_recv_data_t*)read_buffer;

				if (header.component_id == ins_mav_compid) {
					ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data1_id, log_data);
				} else if (header.component_id == ins_cots_mav_compid) {
					ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data2_id, log_data);
				} else {
					ulog_write_lisum_gnss_recv_data(&ulog, lisum_gnss_data3_id, log_data);
				}
				break;
			}

			case MAVLINK_MSG_ID_LISUM_SENSOR_AIRSPEED_DATA: {
				ulog_lisum_sensor_airspeed_data_t* log_data =
					(ulog_lisum_sensor_airspeed_data_t*)read_buffer;

				ulog_write_lisum_sensor_airspeed_data(&ulog, lisum_airspeed_data_id,
													  log_data);
				break;
			}

			case MAVLINK_MSG_ID_LISUM_POWER_HORNET_ACT_DATA: {
				ulog_lisum_power_hornet_act_data_t* log_data =
					(ulog_lisum_power_hornet_act_data_t*)read_buffer;

				ulog_write_lisum_power_hornet_act_data(&ulog, lisum_act_data_id,
													   log_data);
				break;
			}

			case MAVLINK_MSG_ID_LISUM_MANUAL_CTRL_HORNET: {
				ulog_lisum_manual_ctrl_hornet_t* log_data =
					(ulog_lisum_manual_ctrl_hornet_t*)read_buffer;

				ulog_write_lisum_manual_ctrl_hornet(&ulog, lisum_manual_ctrl_id,
													log_data);
				break;
			}

			case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
				ulog_named_value_float_t* log_data =
					(ulog_named_value_float_t*)read_buffer;

				ulog_write_named_value_float(&ulog, named_value_float_id, log_data);
				break;
			}

			case MAVLINK_MSG_ID_LISUM_PROPULSION_TB_STATUS: {
				ulog_lisum_propulsion_tb_status_t* log_data =
					(ulog_lisum_propulsion_tb_status_t*)read_buffer;

				ulog_write_lisum_propulsion_tb_status(
					&ulog, lisum_propulsion_tb_status_id, log_data);

				break;
			}

				// case MAVLINK_MSG_ID_LISUM_POWER_MOTOR_VESC_DATA: {

				// 	ulog_lisum_power_motor_vesc_data_t* log_data =
				// 		(ulog_lisum_power_motor_vesc_data_t*)read_buffer;

				// 	ulog_write_lisum_power_motor_vesc_data(&ulog,
				// lisum_power_motor_vesc_id, log_data); 	break;
				// }

			case MAVLINK_MSG_ID_ADSB_VEHICLE: {
				ulog_adsb_vehicle_t* log_data = (ulog_adsb_vehicle_t*)read_buffer;

				ulog_write_adsb_vehicle(&ulog, adsb_vehicle_id, log_data);
				break;
			}

			case MAVLINK_MSG_ID_AIS_VESSEL: {
				ulog_ais_vessel_t* log_data = (ulog_ais_vessel_t*)read_buffer;

				ulog_write_ais_vessel(&ulog, ais_vessel_id, log_data);
				break;
			}

				// case 600: {
				// 	ulog_lisum_gnss_recv_data_t* log_data =
				// 		(ulog_lisum_gnss_recv_data_t*)read_buffer;

				// 	ulog_write_lisum_gnss_recv_data(
				// 		&ulog, uavcan_lisum_lisum_gnss_recv_data_1_0_id, log_data);
				// 	break;
				// }

				// case MAVLINK_MSG_ID_LISUM_ESC_STATUS: {
				// 	ulog_lisum_esc_status_t* log_data =
				// 		(ulog_lisum_esc_status_t*)read_buffer;

				// 	ulog_write_lisum_esc_status(&ulog, lisum_esc_status_id,
				// &ulog, 								lisum_esc_status_id,
				// log_data); 	break;
				// }

			default:
				ESP_LOGW(TAG, "Unknown message ID (%u) with valid CRC",
						 header.message_id);
				break;
			}

			mutex_unlock(mutex_mav_ftp);
		}

		if (!data_was_read) {
			uint32_t used = ring_used(&ring_buffer);
			if (used > 0) {
				// ESP_LOGD(TAG,
				// 		 "Ring buffer has %u bytes, but not enough for "
				// 		 "complete message",
				// 		 used);
			}
		}
	}
}
