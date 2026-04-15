/**
 * @file    task_work.h
 * @brief   Task work - process background tasks
 * @version 1.0.0
 * @date    24.04.2025
 * @author  BetaTehPro
 */

#ifndef TASK_WORK_H
#define TASK_WORK_H

#ifdef __cplusplus
extern "C"
{
#endif

	/*******************************************************************************
	 * Includes
	 ******************************************************************************/

#include "main.h"
#include "types.h"
#include <stdint.h>

#include "lfs.h"
#include "mav.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

	/*******************************************************************************
	 * Defines
	 ******************************************************************************/

	/*******************************************************************************
	 * Typedefs
	 ******************************************************************************/

	/*******************************************************************************
	 * Variables
	 ******************************************************************************/
	extern const char vendor_name[];
	extern const char model_name[];
	extern const char software_version[];
	extern const char hardware_version[];
	extern const char serial_number[];

	/* Extern temporary variables */
	extern uint64_t boot_time_ms;

	extern const uint32_t crc32_table[256];

	/*******************************************************************************
	 * API
	 ******************************************************************************/

	/**
	 * @brief Parses a version string in the format "vX.Y.Z" to a 32-bit integer.
	 *
	 * @param version_str Pointer to the version string.
	 * @return Encoded version as 0x76MMmmpp ('v', major, minor, patch).
	 */
	static inline uint32_t parse_version_string(const char *version_str)
	{
		int major = 0, minor = 0, patch = 0;

		if (version_str[0] == 'v' || version_str[0] == 'V')
		{
			version_str++; // skip 'v'
		}

		sscanf(version_str, "%d.%d.%d", &major, &minor, &patch);

		return (((uint32_t)'v' & 0xFF) << 24) | ((major & 0xFF) << 16) |
			   ((minor & 0xFF) << 8) | ((patch & 0xFF) << 0);
	}

	/**
	 * @brief Creates and fills a mavlink_component_information_basic_t struct.
	 *
	 * @return Filled mavlink_component_information_basic_t structure.
	 */

#if !MAVLINK_OR_CYPHAL
	static inline common_ComponentInformationBasic_1_0 make_comp_info_basic(void)
	{
		common_ComponentInformationBasic_1_0 data;
		memset(&data, 0, sizeof(data));

		strncpy((char *)data.vendor_name, vendor_name, sizeof(data.vendor_name));
		strncpy((char *)data.model_name, model_name, sizeof(data.model_name));
		strncpy((char *)data.software_version, software_version, sizeof(data.software_version));
		strncpy((char *)data.hardware_version, hardware_version, sizeof(data.hardware_version));
		strncpy((char *)data.serial_number, serial_number, sizeof(data.serial_number));

		data.capabilities =
			MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_FTP;

		data.time_manufacture_s = BUILD_UNIX_TIMESTAMP;

		data.time_boot_ms = boot_time_ms;
		return data;
	}
#else

static inline mavlink_component_information_basic_t make_comp_info_basic(void)
{
	mavlink_component_information_basic_t data;
	memset(&data, 0, sizeof(data));
	strncpy(data.vendor_name, vendor_name, sizeof(data.vendor_name));
	strncpy(data.model_name, model_name, sizeof(data.model_name));
	strncpy(data.software_version, software_version, sizeof(data.software_version));
	strncpy(data.hardware_version, hardware_version, sizeof(data.hardware_version));
	strncpy(data.serial_number, serial_number, sizeof(data.serial_number));
	data.capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_FTP;
	data.time_manufacture_s = BUILD_UNIX_TIMESTAMP;
	data.time_boot_ms = boot_time_ms;
	return data;
}

#endif /* MAVLINK_OR_CYPHAL */

	/**
	 * @brief Creates and fills a mavlink_autopilot_version_t struct.
	 *
	 * @return Filled mavlink_autopilot_version_t structure.
	 */

#if MAVLINK_OR_CYPHAL

	static inline mavlink_autopilot_version_t make_autopilot_version(void)
	{

		mavlink_autopilot_version_t data;

#else

static inline common_AutopilotVersion_1_0 make_autopilot_version(void)
{
	common_AutopilotVersion_1_0 data;

#endif
		memset(&data, 0, sizeof(data));

		data.capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_FTP;
		data.uid = 0x0102030405060708ULL;
		data.board_version = (BOARD_TYPE << 16) | BOARD_REVISION;
		data.vendor_id = VENDOR_ID;
		data.product_id = PRODUCT_ID;
		data.flight_sw_version = parse_version_string(FLIGHT_CUSTOM_VERSION);
		data.os_sw_version = parse_version_string(OS_CUSTOM_VERSION);
		data.middleware_sw_version = (((uint32_t)'v' & 0xFF) << 24) |
									 ((LFS_VERSION_MAJOR & 0xFF) << 16) |
									 ((LFS_VERSION_MINOR & 0xFF) << 8) | (0 & 0xFF);

		for (int i = 0; i < PAYLOAD_LENGTH; i++)
		{
			data.flight_custom_version[i] = FLIGHT_CUSTOM_HASH[i];
			data.middleware_custom_version[i] = MIDDLEWARE_CUSTOM_HASH[i];
			data.os_custom_version[i] = OS_CUSTOM_HASH[i];
		}

		memset(data.uid2, 0, sizeof(data.uid2));
		strncpy((char *)data.uid2, serial_number, sizeof(data.uid2));

		return data;
	}

	static inline uint32_t calculate_crc32(const uint8_t *data, size_t length)
	{
		uint32_t crc = 0xFFFFFFFF;

		for (size_t i = 0; i < length; i++)
		{
			uint8_t byte = data[i];
			crc = crc32_table[(crc ^ byte) & 0xFF] ^ (crc >> 8);
		}

		return crc ^ 0xFFFFFFFF;
	}

	/*******************************************************************************
	 * Prototypes
	 ******************************************************************************/

	void task_work(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* TASK_WORK_H */