/**
 * @file    bl.c
 * @brief   Bootloader driver.
 * @version	1.0.0
 * @date    24.03.2026
 * @author  Filip Radojevic
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bl.h"
#include "LPC17xx.h"
#include "bl_def.h"
#include "core_cm3.h"
#include "core_cmFunc.h"
#include "core_cmInstr.h"
#include <string.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern const uint32_t crc32_table[BL_CRC_TABLE_SIZE];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void prepare_ack_write(mavlink_file_transfer_protocol_t *ftp);
static void prepare_nak_simple(mavlink_file_transfer_protocol_t *ftp,
							   uint8_t error);

/*******************************************************************************
 * Code
 ******************************************************************************/

void bl_init(bl_t *bl)
{
	bl->status = BL_STATUS_NONE;
	bl->flags = BL_FLAGS_NONE;
	bl->state = BL_PARSER_STATE_INIT;
	bl->status_sector = IAP_HAL_CMD_SUCCESS;

	/* Point to the beginning of the update flash region */
	bl->current_write_addr = BL_START_ADDR_OF_UPDATE_REGION;
	bl->sector_num = HAL_IAP_GetSectorNumber(bl->current_write_addr);
	bl->sector_addr = HAL_IAP_GetSectorAddress(bl->sector_num);

	bl->offset = 0;
	bl->idx = 0;
	bl->len = 0;
	bl->payload_side = 0;
	memset(bl->arr, 0, sizeof(bl->arr));
	memset(bl->crc, 0, sizeof(bl->crc));
}

bl_status_t bl_process_update(bl_t *bl,
							  mavlink_file_transfer_protocol_t *ftp_data)
{
	switch (bl->state) {

	case BL_PARSER_STATE_INIT:

		bl_init(bl);

		/* Erase the first sector so it's ready for incoming data */
		if (bl->offset == 0) {
			bl_sector_update(bl);
		}

		bl->payload_side = 0;
		bl->state = BL_PARSER_STATE_DATA;

		return BL_STATUS_SUCCESS;

	case BL_PARSER_STATE_DATA: {

		uint8_t len = ftp_data->payload[4];
		uint16_t buffer_offset = (bl->payload_side == 0) ? 0 : 128;

		/* Extract the total firmware size from the FTP payload (little-endian)
		 */
		bl->size_of_new_firmware =
			ftp_data->payload[11] << 24 | (ftp_data->payload[10] << 16) |
			(ftp_data->payload[9] << 8) | ftp_data->payload[8];

		/* Guard against oversized chunks corrupting the buffer */
		if (bl->len > 128) {
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		/* Copy payload into the correct half of the 256-byte buffer */
		memcpy(&bl->arr[buffer_offset], &ftp_data->payload[12], len);
		bl->len = len;

		uint32_t received_crc;
		memcpy(&received_crc, &ftp_data->payload[140], 4);

		/* Empty payload means transfer is done, move to file CRC check */
		if (len == 0) {
			bl->state = BL_PARSER_STATE_CHECK_CRC_OF_FILE;
			return bl_process_update(bl, ftp_data);
		}

		if (bl_verify_crc(bl, received_crc) != BL_STATUS_SUCCESS) {
			return BL_STATUS_INVALID_CRC;
		}

		/* Short chunk — last piece before a flash write */
		if (len > 0 && len < 128) {
			bl->payload_side = 1;
			bl->state = BL_PARSER_STATE_FLASH_WRITE;
			return bl_process_update(bl, ftp_data);
		}

		/* First half received — wait for the second half */
		if (bl->payload_side == 0) {
			bl->payload_side = 1;
			return BL_STATUS_SUCCESS;
		} else {
			/* Both halves ready — flush to flash */
			bl->state = BL_PARSER_STATE_FLASH_WRITE;
			return bl_process_update(bl, ftp_data);
		}
	}

	case BL_PARSER_STATE_FLASH_WRITE: {
		iap_hal_status_code_t status;

		bl->sector_num = HAL_IAP_GetSectorNumber(bl->current_write_addr);
		bl->sector_addr = HAL_IAP_GetSectorAddress(bl->sector_num);

		/* Crossed into a new sector — erase it first */
		if (bl->sector_addr != bl->last_erased_sector_addr) {
			if (bl_sector_update(bl) != BL_STATUS_SUCCESS) {
				return BL_STATUS_ERROR;
			}
		}

		/* IAP requires prepare right before every write, otherwise Hard Fault
		 */
		__disable_irq();
		HAL_IAP_PrepareSector(bl->sector_num, bl->sector_num);

		/* Write the 256-byte buffer into flash */
		status = HAL_IAP_CopyRAM2Flash((uint8_t *)bl->current_write_addr,
									   bl->arr, 256);
		__enable_irq();

		if (status != IAP_HAL_CMD_SUCCESS) {
			bl->status = BL_STATUS_ERROR;
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		/* Read-back verify — make sure flash matches RAM */
		if (memcmp(bl->arr, (void *)bl->current_write_addr, 256) != 0) {
			bl->status = BL_STATUS_ERROR;
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		/* Clear the buffer and advance to the next 256-byte block */
		memset(bl->arr, 0, sizeof(bl->arr));
		bl->current_write_addr += 256;
		bl->offset += 256;
		bl->payload_side = 0;
		bl->state = BL_PARSER_STATE_DATA;

		return BL_STATUS_SUCCESS;
	}

	case BL_PARSER_STATE_CHECK_CRC_OF_FILE: {
		uint32_t received_crc;
		memcpy(&received_crc, &ftp_data->payload[BL_CRC_POSITION_IN_PACKET], 4);

		/* Validate CRC of the entire firmware image written to flash */
		if (bl_verify_file_crc(bl, received_crc) != BL_STATUS_SUCCESS) {
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_INVALID_CRC;
		}

		/* Mark firmware as valid and persist before rebooting */
		bl->flags |= BL_FLAG_VALID_FILE_IN_UPDATE_REGION | BL_FLAG_REBOOT;
		bl->state = BL_PARSER_STATE_COPY_INTO_APPLICATION_REGION;
		bl_save_boot_info(bl);

		return BL_STATUS_SUCCESS;
	}

	case BL_PARSER_STATE_COPY_INTO_APPLICATION_REGION: {
		if (bl_save_boot_info(bl) != BL_STATUS_SUCCESS) {
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		bl->flags |= BL_FLAG_REBOOT;
		return BL_STATUS_SUCCESS;
	}

	default:
		bl->state = BL_PARSER_STATE_INIT;
		break;
	}

	return BL_STATUS_SUCCESS;
}

bl_status_t bl_sector_update(bl_t *bl)
{
	bl->sector_num = HAL_IAP_GetSectorNumber(bl->current_write_addr);
	bl->sector_addr = HAL_IAP_GetSectorAddress(bl->sector_num);

	/* Remember which sector was erased to avoid redundant erases */
	bl->last_erased_sector_addr = bl->sector_addr;

	/* Prepare + erase must be atomic (no interrupts allowed during IAP) */
	__disable_irq();

	if (HAL_IAP_PrepareSector(bl->sector_num, bl->sector_num) !=
		IAP_HAL_CMD_SUCCESS) {
		bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
		return BL_STATUS_ERROR;
	}

	if (HAL_IAP_EraseSector(bl->sector_num, bl->sector_num) !=
		IAP_HAL_CMD_SUCCESS) {
		bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
		return BL_STATUS_ERROR;
	}

	__enable_irq();

	return BL_STATUS_SUCCESS;
}

bl_status_t bl_save_boot_info(bl_t *bl)
{
	bl_boot_info_t boot_info;
	uint8_t temp_buf[256] __attribute__((aligned(4)));

	/* Pack the current state into a boot info record */
	boot_info.bl_magic_flag = BL_MAGIC;
	boot_info.bl_size_of_new_firmware = bl->size_of_new_firmware;
	boot_info.bl_boot_flags = bl->flags;
	boot_info.bl_crc[0] = bl->crc[0];
	boot_info.bl_crc[1] = bl->crc[1];

	iap_hal_sector_num_t settings_sector =
		HAL_IAP_GetSectorNumber(BL_START_ADDR_OF_SETTINGS_REGION);

	/* Erase the settings sector before writing new data */
	__disable_irq();
	if (HAL_IAP_PrepareSector(settings_sector, settings_sector) !=
		IAP_HAL_CMD_SUCCESS) {
		__enable_irq();
		return BL_STATUS_ERROR;
	}
	if (HAL_IAP_EraseSector(settings_sector, settings_sector) !=
		IAP_HAL_CMD_SUCCESS) {
		__enable_irq();
		return BL_STATUS_ERROR;
	}

	/* Pad to 256 bytes (0xFF = erased flash) and write */
	memset(temp_buf, 0xFF, 256);
	memcpy(temp_buf, &boot_info, sizeof(bl_boot_info_t));

	HAL_IAP_PrepareSector(settings_sector, settings_sector);
	iap_hal_status_code_t status = HAL_IAP_CopyRAM2Flash(
		(uint8_t *)BL_START_ADDR_OF_SETTINGS_REGION, temp_buf, 256);
	__enable_irq();

	if (status != IAP_HAL_CMD_SUCCESS)
		return BL_STATUS_ERROR;

	return BL_STATUS_SUCCESS;
}

bl_status_t bl_verify_crc(bl_t *bl, uint32_t received_crc)
{
	uint32_t crc = 0x00000000;

	/* Pick the active 128-byte half of the buffer */
	uint8_t *data_ptr = (bl->payload_side == 0) ? &bl->arr[0] : &bl->arr[128];

	/* CRC-32 lookup-table computation over the chunk */
	for (int i = 0; i < bl->len; i++) {
		crc = (crc >> 8) ^ crc32_table[(crc ^ data_ptr[i]) & 0xFF];
	}

	return (crc == received_crc) ? BL_STATUS_SUCCESS : BL_STATUS_INVALID_CRC;
}

bl_status_t bl_verify_file_crc(bl_t *bl, uint32_t received_crc)
{
	uint32_t crc = 0x00000000;
	uint8_t *data_ptr = (uint8_t *)BL_START_ADDR_OF_UPDATE_REGION;

	/* CRC-32 over the entire firmware image in flash */
	for (uint32_t i = 0; i < bl->size_of_new_firmware; i++) {
		crc = (crc >> 8) ^ crc32_table[(crc ^ data_ptr[i]) & 0xFF];
	}

	return (crc == received_crc) ? BL_STATUS_SUCCESS : BL_STATUS_INVALID_CRC;
}

/* --------------------- Bootloader helper functions -----------------------*/

bl_status_t bl_process_mav_ftp(bl_t *bl, mavlink_file_transfer_protocol_t *ftp)
{
	bl_status_t status = bl_process_update(bl, ftp);

	if (status != BL_STATUS_SUCCESS) {
		prepare_nak_simple(ftp, status);
	} else {
		prepare_ack_write(ftp);
	}

	return status;
}

static void prepare_ack_write(mavlink_file_transfer_protocol_t *ftp)
{
	uint16_t seq = (uint16_t)ftp->payload[0] | ((uint16_t)ftp->payload[1] << 8);

	seq++;

	ftp->payload[0] = (uint8_t)(seq & 0xFF);
	ftp->payload[1] = (uint8_t)((seq >> 8) & 0xFF);

	ftp->payload[3] = 128;
	ftp->payload[5] = 7;
}

static void prepare_nak_simple(mavlink_file_transfer_protocol_t *ftp,
							   uint8_t error)
{
	memset(ftp->payload, 0, sizeof(ftp->payload));

	ftp->payload[3] = 129;
	ftp->payload[5] = error;
}

uint32_t bl_get_offset(const mavlink_file_transfer_protocol_t *ftp)
{
	return (uint32_t)ftp->payload[8] | ((uint32_t)ftp->payload[9] << 8) |
		   ((uint32_t)ftp->payload[10] << 16) |
		   ((uint32_t)ftp->payload[11] << 24);
}

/********************************* End Of File ********************************/