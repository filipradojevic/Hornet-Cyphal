/**
 * @file    bl.h
 * @brief   Bootloader driver.
 * @version	1.0.0
 * @date    24.03.2026
 * @author  Filip Radojevic
 */

#ifndef BL_H
#define BL_H

#ifdef __cplusplus
extern "C"
{
#endif

	/*******************************************************************************
	 * Includes
	 ******************************************************************************/

#include "bl_def.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include <stdint.h>
#include <string.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mav.h"
#pragma GCC diagnostic pop

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define BL_CRC_TABLE_SIZE 256
#define BL_CRC_POSITION_IN_PACKET 140
#define BL_MAGIC 0x505442 /* 'B' 'T' 'P' */
#define BL_FW_HEADER_SIZE 64

	/*******************************************************************************
	 * Typedefs
	 ******************************************************************************/

	/**
	 * @struct fw_header_t
	 * @brief  Firmware image header prepended to the .bin file at build time.
	 *
	 * The bootloader reads this header from the beginning of the update region
	 * to validate the image before copying the firmware into the program region.
	 * The actual firmware (vector table + code) starts at offset @ref
	 * BL_FW_HEADER_SIZE.
	 */
	typedef struct __attribute__((packed))
	{
		char project_name[16];	/**< Project name, e.g. "Bootloader-main". */
		uint32_t magic;			/**< Magic value (@ref BL_MAGIC). */
		uint16_t device_id;		/**< Target device identifier. */
		uint8_t version_major;	/**< Firmware major version. */
		uint8_t version_minor;	/**< Firmware minor version. */
		uint8_t version_patch;	/**< Firmware patch version. */
		uint8_t reserved[7];	/**< Reserved for future use. */
		uint32_t firmware_size; /**< Size of the firmware payload (without header). */
		uint32_t firmware_crc;	/**< CRC-32 of the firmware payload only. */
		char git_hash[8];		/**< Short git commit hash (8 chars). */
		char git_tag[16];		/**< Git tag string, e.g. "v1.2.3". */
	} fw_header_t;

	/**
	 * @enum bl_status_t
	 * @brief Return codes for bootloader operations.
	 */
	typedef enum bl_status_t
	{
		BL_STATUS_NONE = 0,		  /**< No status / not yet set. */
		BL_STATUS_SUCCESS = 1,	  /**< Operation completed successfully. */
		BL_STATUS_ERROR = 2,	  /**< Generic error. */
		BL_STATUS_TIMEOUT = 3,	  /**< Operation timed out. */
		BL_STATUS_INVALID_CRC = 4 /**< CRC mismatch detected. */
	} bl_status_t;

	/**
	 * @enum bl_flags_t
	 * @brief Bit-field flags representing bootloader state and results.
	 */
	typedef enum bl_flags_t
	{
		BL_FLAGS_NONE = 0,
		BL_FLAG_SUCCESS = 1 << 0,					  /**< Last operation succeeded. */
		BL_FLAG_VALID_FILE_IN_UPDATE_REGION = 1 << 1, /**< Valid firmware in update region. */
		BL_FLAG_INVALID_FILE_IN_UPDATE_REGION =
			1 << 2, /**< Invalid firmware in update region. */
		BL_FLAG_VALID_FILE_IN_APPLICATION_REGION =
			1 << 3, /**< Valid firmware in application region. */
		BL_FLAG_INVALID_FILE_IN_APPLICATION_REGION =
			1 << 4,							/**< Invalid firmware in application region. */
		BL_FLAG_REBOOT = 1 << 5,			/**< Reboot requested. */
		BL_FLAG_BOOTLOADER_FAILED = 1 << 6, /**< Bootloader encountered a fatal error. */
	} bl_flags_t;

	/**
	 * @struct bl_boot_info_t
	 * @brief  Persistent boot information stored in the settings flash region.
	 */
	typedef struct __attribute__((packed))
	{
		uint32_t bl_magic_flag;			  /**< Magic value (@ref BL_MAGIC) to validate the record. */
		uint32_t bl_size_of_new_firmware; /**< Size of the new firmware image in bytes. */
		uint32_t bl_boot_flags;			  /**< Combination of @ref bl_flags_t values. */
		uint8_t bl_crc[4];				  /**< CRC-32 of the firmware image (big-endian bytes). */
	} bl_boot_info_t;

	/**
	 * @enum bl_parser_state_e
	 * @brief States of the firmware-update state machine.
	 */
	typedef enum bl_parser_state_e
	{
		BL_PARSER_STATE_INIT = 0,						 /**< Initialization. */
		BL_PARSER_STATE_DATA = 1,						 /**< Receiving payload data. */
		BL_PARSER_STATE_FLASH_WRITE = 2,				 /**< Writing buffered data to flash. */
		BL_PARSER_STATE_CHECK_CRC_OF_FILE = 3,			 /**< Verifying full-image CRC. */
		BL_PARSER_STATE_COPY_INTO_APPLICATION_REGION = 4 /**< Committing OTA partition. */
	} bl_parser_state_e;

	/**
	 * @struct bl_t
	 * @brief  Runtime context for the bootloader update process.
	 */
	typedef struct bl_t
	{
		uint8_t arr[256];
		uint32_t offset;
		uint8_t payload_side;
		uint8_t len;
		uint16_t total_buf_len;

		uint32_t size_of_new_firmware;
		uint32_t ota_bytes_written;

		uint32_t crc_file;
		uint8_t crc_message[4];

		esp_ota_handle_t ota_handle;
		const esp_partition_t *partition;

		uint8_t state;
		uint8_t flags;
	} bl_t;

	/*******************************************************************************
	 * API
	 ******************************************************************************/

	/**
	 * @brief  Initialize the bootloader context and begin an OTA session.
	 *
	 * Zeroes the bl_t structure, selects the next OTA partition, and calls
	 * esp_ota_begin(). Sets BL_FLAG_BOOTLOADER_FAILED if esp_ota_begin() fails.
	 *
	 * @param[out] bl  Pointer to the bootloader context structure.
	 */
	void bl_init(bl_t *bl);

	/**
	 * @brief  Verify the CRC-32 of the current payload chunk.
	 *
	 * Computes a CRC-32 over the active 128-byte payload half and compares
	 * it against the received CRC value.
	 *
	 * @param[in] bl            Pointer to the bootloader context structure.
	 * @param[in] received_crc  Expected CRC-32 value.
	 * @return BL_STATUS_SUCCESS if the CRC matches, BL_STATUS_INVALID_CRC otherwise.
	 */
	bl_status_t bl_verify_crc(bl_t *bl, uint32_t received_crc);

	/**
	 * @brief  Main bootloader state-machine — process one FTP data packet.
	 *
	 * Drives the update process through the following states:
	 * - @c INIT        — reset context and start OTA session.
	 * - @c DATA        — buffer incoming payload, verify per-packet CRC.
	 * - @c FLASH_WRITE — write the 256-byte buffer via esp_ota_write().
	 * - @c CHECK_CRC   — validate the running CRC-32 against the received value.
	 * - @c COPY        — call esp_ota_end(), set boot partition, restart.
	 *
	 * @param[in,out] bl        Pointer to the bootloader context structure.
	 * @param[in]     ftp_data  Pointer to the received MAVLink FTP payload.
	 * @return BL_STATUS_SUCCESS on success, BL_STATUS_ERROR or
	 *         BL_STATUS_INVALID_CRC on failure.
	 */
	bl_status_t bl_process_update(bl_t *bl, mavlink_file_transfer_protocol_t *ftp_data);

	/**
	 * @brief  Process an FTP packet and prepare ACK/NAK response in-place.
	 *
	 * Calls @ref bl_process_update, then writes ACK or NAK fields into
	 * @p ftp->payload so the caller can send the response.
	 *
	 * @param[in,out] bl   Pointer to the bootloader context structure.
	 * @param[in,out] ftp  Pointer to the FTP message; payload is modified with the response.
	 * @return BL_STATUS_SUCCESS on success, error code otherwise.
	 */
	bl_status_t bl_process_mav_ftp(bl_t *bl, mavlink_file_transfer_protocol_t *ftp);

	/**
	 * @brief  Extract the 32-bit offset from an FTP payload.
	 *
	 * @param[in] ftp  Pointer to the received FTP message.
	 * @return Little-endian offset value from the payload.
	 */
	uint32_t bl_get_offset(const mavlink_file_transfer_protocol_t *ftp);

#ifdef __cplusplus
}
#endif

#endif /* BL_H */