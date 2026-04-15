#include "bl.h"
#include "bl_def.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "main.h"

#include <string.h>

#define TAG "BL"

extern const uint32_t crc32_table[BL_CRC_TABLE_SIZE];

/******************************************************************************/
static void prepare_ack_write(mavlink_file_transfer_protocol_t *ftp);
static void prepare_nak_simple(mavlink_file_transfer_protocol_t *ftp, uint8_t error);
/******************************************************************************/

void bl_init(bl_t *bl)
{
	printf("[BL] bl_init: initializing bootloader context\n");

	memset(bl, 0, sizeof(bl_t));

	bl->state = BL_PARSER_STATE_INIT;

	bl->partition = esp_ota_get_next_update_partition(NULL);

	if (bl->partition == NULL)
	{
		printf("[BL] bl_init: ERROR - no OTA partition found\n");
		bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
		return;
	}

	printf("[BL] bl_init: OTA partition found: %s (offset: 0x%08lx, size: 0x%08lx)\n",
		   bl->partition->label, bl->partition->address, bl->partition->size);

	if (esp_ota_begin(bl->partition, OTA_SIZE_UNKNOWN, &bl->ota_handle) != ESP_OK)
	{
		printf("[BL] bl_init: ERROR - esp_ota_begin() failed\n");
		bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
		return;
	}

	bl->crc_file = 0x00000000;

	printf("[BL] bl_init: OTA session started successfully\n");
}

/******************************************************************************/

bl_status_t bl_process_update(bl_t *bl, mavlink_file_transfer_protocol_t *ftp_data)
{
	switch (bl->state)
	{

	case BL_PARSER_STATE_INIT:

		printf("[BL] STATE: INIT\n");

		bl_init(bl);

		bl->payload_side = 0;
		bl->state = BL_PARSER_STATE_DATA;

		return BL_STATUS_SUCCESS;

	case BL_PARSER_STATE_DATA:
	{
		uint8_t len = ftp_data->payload[4];
		uint16_t buffer_offset = (bl->payload_side == 0) ? 0 : 128;

		printf("[BL] STATE: DATA | offset: %lu | len: %u | side: %u\n",
			   bl->offset, len, bl->payload_side);

		/* Guard against oversized chunks corrupting the buffer */
		if (len > 128)
		{
			printf("[BL] DATA: ERROR - chunk too large (%u > 128)\n", len);
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		memcpy(&bl->arr[buffer_offset], &ftp_data->payload[12], len);
		bl->len = len;

		/* Track how many bytes are actually in the buffer */
		if (bl->payload_side == 0)
		{
			bl->total_buf_len = len;
		}
		else
		{
			bl->total_buf_len += len;
		}

		uint32_t received_crc;
		memcpy(&received_crc, &ftp_data->payload[140], 4);

		/* Empty payload means transfer is done, move to file CRC check */
		if (len == 0)
		{
			printf("[BL] DATA: len == 0, transfer complete, moving to CRC check\n");
			bl->state = BL_PARSER_STATE_CHECK_CRC_OF_FILE;
			return bl_process_update(bl, ftp_data);
		}

		if (bl_verify_crc(bl, received_crc) != BL_STATUS_SUCCESS)
		{
			printf("[BL] DATA: ERROR - packet CRC mismatch (received: 0x%08lX)\n",
				   received_crc);
			return BL_STATUS_INVALID_CRC;
		}

		printf("[BL] DATA: packet CRC OK (0x%08lX)\n", received_crc);

		/* Short chunk — last piece before a flash write */
		if (len > 0 && len < 128)
		{
			printf("[BL] DATA: short chunk (%u bytes), triggering flash write\n", len);
			bl->payload_side = 1;
			bl->state = BL_PARSER_STATE_FLASH_WRITE;
			return bl_process_update(bl, ftp_data);
		}

		/* First half received — wait for the second half */
		if (bl->payload_side == 0)
		{
			printf("[BL] DATA: first half buffered, waiting for second half\n");
			bl->payload_side = 1;
			return BL_STATUS_SUCCESS;
		}
		else
		{
			printf("[BL] DATA: both halves ready, triggering flash write\n");
			bl->state = BL_PARSER_STATE_FLASH_WRITE;
			return bl_process_update(bl, ftp_data);
		}
	}

	case BL_PARSER_STATE_FLASH_WRITE:
	{
		printf("[BL] STATE: FLASH_WRITE | offset: %lu\n", bl->offset);

		esp_err_t err;
		uint16_t valid_len = bl->total_buf_len;

		printf("[BL] FLASH_WRITE: offset=%lu, valid_len=%u, total_after=%lu\n",
			   bl->offset, valid_len, bl->offset + valid_len);

		if (bl->offset == 0)
		{
			uint16_t write_len = (valid_len > 64) ? valid_len - 64 : 0;

			printf("[BL] FLASH_WRITE: first block, skipping 64-byte header, writing %u bytes\n",
				   write_len);
			printf("[BL] FLASH_WRITE: arr[64]=0x%02X arr[65]=0x%02X arr[66]=0x%02X arr[67]=0x%02X\n",
				   bl->arr[64], bl->arr[65], bl->arr[66], bl->arr[67]);

			if (write_len > 0)
			{
				err = esp_ota_write(bl->ota_handle, &bl->arr[64], write_len);
				if (err == ESP_OK)
					bl->ota_bytes_written += write_len;
			}
			else
			{
				err = ESP_OK;
			}
		}
		else
		{
			err = esp_ota_write(bl->ota_handle, bl->arr, valid_len);
			if (err == ESP_OK)
				bl->ota_bytes_written += valid_len;
		}

		if (err != ESP_OK)
		{
			printf("[BL] FLASH_WRITE: ERROR - esp_ota_write() failed (err: 0x%x)\n", err);
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		/* Running CRC over full buffer including header */
		for (uint16_t i = 0; i < valid_len; i++)
		{
			bl->crc_file =
				(bl->crc_file >> 8) ^ crc32_table[(bl->crc_file ^ bl->arr[i]) & 0xFF];
		}

		printf("[BL] FLASH_WRITE: valid_len=%u, ota_written=%lu, running CRC=0x%08lX\n",
			   valid_len, bl->ota_bytes_written, bl->crc_file);

		memset(bl->arr, 0, sizeof(bl->arr));
		bl->offset += valid_len;
		bl->payload_side = 0;
		bl->total_buf_len = 0;
		bl->state = BL_PARSER_STATE_DATA;

		return BL_STATUS_SUCCESS;
	}

	case BL_PARSER_STATE_CHECK_CRC_OF_FILE:
	{
		printf("[BL] STATE: CHECK_CRC_OF_FILE\n");

		uint32_t received_crc;
		memcpy(&received_crc, &ftp_data->payload[BL_CRC_POSITION_IN_PACKET], 4);

		printf("[BL] CHECK_CRC: received:         0x%08lX\n", received_crc);
		printf("[BL] CHECK_CRC: calculated:        0x%08lX\n", bl->crc_file);
		printf("[BL] CHECK_CRC: total offset:      %lu\n", bl->offset);
		printf("[BL] CHECK_CRC: ota_bytes_written: %lu\n", bl->ota_bytes_written);
		printf("[BL] CHECK_CRC: expected esp size: %lu\n", bl->offset > 64 ? bl->offset - 64 : 0);
		printf("[BL] CHECK_CRC: diff (exp-written):%ld\n",
			   (int32_t)(bl->offset > 64 ? bl->offset - 64 : 0) - (int32_t)bl->ota_bytes_written);

		if (bl->crc_file != received_crc)
		{
			printf("[BL] CHECK_CRC: ERROR - file CRC mismatch\n");
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_INVALID_CRC;
		}

		printf("[BL] CHECK_CRC: CRC OK, marking firmware valid\n");

		bl->flags |= BL_FLAG_VALID_FILE_IN_UPDATE_REGION | BL_FLAG_REBOOT;
		bl->state = BL_PARSER_STATE_COPY_INTO_APPLICATION_REGION;

		return bl_process_update(bl, ftp_data);
	}

	case BL_PARSER_STATE_COPY_INTO_APPLICATION_REGION:
	{
		printf("[BL] STATE: COPY_INTO_APPLICATION_REGION\n");

		if (esp_ota_end(bl->ota_handle) != ESP_OK)
		{
			printf("[BL] COPY: ERROR - esp_ota_end() failed\n");
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		printf("[BL] COPY: OTA write finalized\n");

		if (esp_ota_set_boot_partition(bl->partition) != ESP_OK)
		{
			printf("[BL] COPY: ERROR - esp_ota_set_boot_partition() failed\n");
			bl->flags |= BL_FLAG_BOOTLOADER_FAILED;
			return BL_STATUS_ERROR;
		}

		printf("[BL] COPY: boot partition set, flags: 0x%02X, restarting...\n", bl->flags);

		bl->flags |= BL_FLAG_REBOOT;

		return BL_STATUS_SUCCESS;
	}

	default:
		printf("[BL] ERROR: unknown state %u, resetting to INIT\n", bl->state);
		bl->state = BL_PARSER_STATE_INIT;
		break;
	}

	return BL_STATUS_SUCCESS;
}

/******************************************************************************/

bl_status_t bl_verify_crc(bl_t *bl, uint32_t received_crc)
{
	uint32_t crc = 0x00000000;

	/* Pick the active 128-byte half of the buffer */
	uint8_t *data_ptr = (bl->payload_side == 0) ? &bl->arr[0] : &bl->arr[128];

	/* CRC-32 lookup-table computation over the chunk */
	for (int i = 0; i < bl->len; i++)
	{
		crc = (crc >> 8) ^ crc32_table[(crc ^ data_ptr[i]) & 0xFF];
	}

	return (crc == received_crc) ? BL_STATUS_SUCCESS : BL_STATUS_INVALID_CRC;
}

/******************************************************************************/
/**************** MAVLINK HELPERS ****************/
/******************************************************************************/

bl_status_t bl_process_mav_ftp(bl_t *bl, mavlink_file_transfer_protocol_t *ftp)
{
	bl_status_t status = bl_process_update(bl, ftp);

	if (status != BL_STATUS_SUCCESS)
	{
		printf("[BL] bl_process_mav_ftp: NAK, status: %d\n", status);
		prepare_nak_simple(ftp, status);
	}
	else
	{
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

static void prepare_nak_simple(mavlink_file_transfer_protocol_t *ftp, uint8_t error)
{
	memset(ftp->payload, 0, sizeof(ftp->payload));

	ftp->payload[3] = 129;
	ftp->payload[5] = error;
}

/******************************************************************************/

uint32_t bl_get_offset(const mavlink_file_transfer_protocol_t *ftp)
{
	return (uint32_t)ftp->payload[8] | ((uint32_t)ftp->payload[9] << 8) |
		   ((uint32_t)ftp->payload[10] << 16) | ((uint32_t)ftp->payload[11] << 24);
}