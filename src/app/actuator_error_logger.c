/**
 * @file    actuator_error_logger.c
 * @brief   Actuator flash error logger - implementation.
 * @details Implements logging of EPOS4 actuator errors into internal uC flash
 *          memory. Errors are written sequentially across 7 dedicated sectors
 *          (sectors 8-14) as fixed-size 8-byte slots. A separate header sector
 *          (sector 15) tracks whether logged errors have been reported to the
 *          host after each boot.
 *
 *          Storage model:
 *            - Each sector holds 512 slots of 8 bytes = 4096 bytes.
 *            - Slots are written in order; an erased slot reads as 0xFFFFFFFF.
 *            - When a sector is full the next sector is pre-erased before use.
 *            - When all 7 sectors are full, sector 0 is erased and reused
 *              (ring buffer, oldest data lost).
 *
 *          Header model:
 *            - Sector 15 contains up to 512 header slots of 8 bytes each.
 *            - Each slot has a write flag and a reported flag (both uint32_t).
 *            - One bit is cleared in the write flag the first time an error
 *              is logged per boot. The reported flag is updated to match once
 *              the host acknowledges the errors.
 *            - When all 32 bits of a slot are consumed, the next slot is used.
 *
 * @version 1.0.0
 * @date    18.05.2026
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "actuator_error_logger.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/** Size of a flash write page in bytes (minimum writable unit via IAP). */
#define ACTUATOR_ERROR_PAGE_SIZE 256U

/*******************************************************************************
 * Variables
 ******************************************************************************/

/** Base addresses of the 7 error log sectors, indexed 0-6. */
static const uint32_t s_error_sectors[ACTUATOR_ERROR_SECTOR_COUNT] = {
	ACTUATOR_ERROR_SECTOR_0, ACTUATOR_ERROR_SECTOR_1, ACTUATOR_ERROR_SECTOR_2,
	ACTUATOR_ERROR_SECTOR_3, ACTUATOR_ERROR_SECTOR_4, ACTUATOR_ERROR_SECTOR_5,
	ACTUATOR_ERROR_SECTOR_6,
};

/** Index into s_error_sectors[] pointing to the sector currently being written.
 */
static uint8_t s_current_sector;

/** Slot index within the current sector pointing to the next free slot. */
static uint16_t s_current_slot;

/** Boot counter restored from flash on init and incremented each boot. */
static uint8_t s_boot_cnt;

/** RAM copy of the 256-byte flash page currently being built for error storage.
 */
static uint8_t s_page_buffer[ACTUATOR_ERROR_PAGE_SIZE];

/** RAM copy of a 256-byte flash page used when updating the header sector. */
static uint8_t s_header_page[ACTUATOR_ERROR_PAGE_SIZE];

/** Set to 1 after the first error is flagged in the header this boot. */
static uint8_t s_boot_error_flagged;

/** Counts how many errors have been logged since the last boot. */
static uint32_t s_error_count;

/** Set to 1 at startup if the previous boot had unreported errors. */
uint8_t unreported_errors_at_boot;

/*******************************************************************************
 * Prototypes (private)
 ******************************************************************************/

static void actuator_error_store_fill_from_epos(actuator_error_t *err,
												epos_track_t *track,
												uint8_t boot_cnt);
static void actuator_error_store_load_page(void);
static void actuator_error_store_erase_sector(uint8_t sector_idx);

static void actuator_error_header_find(uint32_t *out_write,
									   uint32_t *out_reported,
									   uint32_t *out_idx);
static void actuator_error_header_erase(void);
static uint32_t actuator_error_header_next_flag(uint32_t current);

/*******************************************************************************
 * Private helpers - address calculation
 ******************************************************************************/

/**
 * @brief  Compute the flash address of a given slot.
 * @param  sector  Sector index (0 .. ACTUATOR_ERROR_SECTOR_COUNT-1).
 * @param  slot    Slot index within the sector.
 * @return Absolute flash address of the slot.
 */
static inline uint32_t actuator_error_slot_addr(uint8_t sector, uint16_t slot)
{
	return s_error_sectors[sector] + (uint32_t)slot * ACTUATOR_ERROR_SLOT_SIZE;
}

/**
 * @brief  Compute the flash address of the 256-byte page that contains a slot.
 * @param  sector  Sector index.
 * @param  slot    Slot index within the sector.
 * @return Absolute flash address of the containing page (256-byte aligned).
 */
static inline uint32_t actuator_error_page_addr(uint8_t sector, uint16_t slot)
{
	uint8_t page_idx = slot / ACTUATOR_ERROR_SLOTS_PER_PAGE;
	return s_error_sectors[sector] +
		   (uint32_t)page_idx * ACTUATOR_ERROR_PAGE_SIZE;
}

/*******************************************************************************
 * Code - Error Store (public)
 ******************************************************************************/

/**
 * @brief   Initialize the actuator error logger.
 * @details Scans all sectors slot by slot looking for the first erased entry
 *          (first 4 bytes == 0xFFFFFFFF). When found, the boot counter is
 *          read from the last written slot and incremented. If all sectors are
 *          full, sector 0 is erased and the logger restarts from slot 0.
 */
void actuator_error_store_init(void)
{
	/* Scan every sector and slot to locate the first free (erased) slot. */
	for (uint8_t s = 0U; s < ACTUATOR_ERROR_SECTOR_COUNT; s++) {
		for (uint16_t slot = 0U; slot < ACTUATOR_ERROR_SLOTS_PER_SECTOR;
			 slot++) {
			uint32_t addr = actuator_error_slot_addr(s, slot);
			uint32_t stored_num = *(volatile uint32_t *)(addr);

			/* An erased slot reads as all 0xFF - this is our write position. */
			if (stored_num != 0xFFFFFFFFU)
				continue;

			/* Restore boot counter from the last written slot. */
			if (slot > 0U) {
				/* Previous slot is in the same sector. */
				actuator_error_t *prev =
					(actuator_error_t *)actuator_error_slot_addr(s, slot - 1U);
				s_boot_cnt = prev->uc_boot_cnt;
			} else if (s > 0U) {
				/* First slot of this sector - read from last slot of previous
				 * sector. */
				actuator_error_t *prev =
					(actuator_error_t *)actuator_error_slot_addr(
						s - 1U, ACTUATOR_ERROR_SLOTS_PER_SECTOR - 1U);
				s_boot_cnt = prev->uc_boot_cnt;
			} else {
				/* Very first slot of all sectors - no previous data, start at
				 * 0. */
				s_boot_cnt = 0U;
			}

			/* Increment boot counter for the current boot session. */
			s_boot_cnt++;

			s_current_sector = s;
			s_current_slot = slot;
			actuator_error_store_load_page();
			return;
		}
	}

	/*
	 * All sectors are completely full.
	 * Read boot counter from the last slot of the last sector,
	 * then erase sector 0 and start over (ring buffer wrap-around).
	 */
	actuator_error_t *last = (actuator_error_t *)actuator_error_slot_addr(
		ACTUATOR_ERROR_SECTOR_COUNT - 1U, ACTUATOR_ERROR_SLOTS_PER_SECTOR - 1U);

	s_boot_cnt = last->uc_boot_cnt + 1U;
	s_current_sector = 0U;
	s_current_slot = 0U;

	/* Erase sector 0 to make room for new entries. */
	actuator_error_store_erase_sector(0U);

	actuator_error_store_load_page();
}

/**
 * @brief   Log an EPOS4 actuator error to flash.
 * @details Fills the error structure, updates the in-RAM page buffer, writes
 *          the page to flash, and advances the current slot pointer. If the
 *          sector boundary is crossed, the next sector is erased before use.
 *          On the first error of this boot the header sector is also updated.
 * @param   err       Destination error structure (filled by this function).
 * @param   track     EPOS track that generated the fault.
 * @param   boot_cnt  Boot counter to embed in the log entry.
 */
void actuator_error_store_write(actuator_error_t *err, epos_track_t *track,
								uint8_t boot_cnt)
{
	/* Step 1: Fill the error structure from the EPOS track data. */
	actuator_error_store_fill_from_epos(err, track, boot_cnt);

	/* Step 2: Determine where in the page buffer this slot belongs. */
	uint8_t page_slot = s_current_slot % ACTUATOR_ERROR_SLOTS_PER_PAGE;
	uint32_t page_addr =
		actuator_error_page_addr(s_current_sector, s_current_slot);

	/* Step 3: Copy the filled error structure into the correct position in the
	 * page buffer. */
	memcpy(s_page_buffer + page_slot * sizeof(actuator_error_t), err,
		   sizeof(actuator_error_t));

	/* Step 4: Write the updated 256-byte page back to flash. */
	__disable_irq();
	iap_hal_sector_num_t sec =
		HAL_IAP_GetSectorNumber(s_error_sectors[s_current_sector]);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_CopyRAM2Flash((uint8_t *)page_addr, s_page_buffer,
						  IAP_HAL_WRITE_256);
	__enable_irq();

	/* Step 5: Advance the slot pointer. */
	s_current_slot++;

	/* Step 6: If the current sector is now full, move to the next one. */
	if (s_current_slot >= ACTUATOR_ERROR_SLOTS_PER_SECTOR) {
		s_current_slot = 0U;
		s_current_sector =
			(s_current_sector + 1U) % ACTUATOR_ERROR_SECTOR_COUNT;

		/* Pre-erase the next sector so it is ready for writing. */
		actuator_error_store_erase_sector(s_current_sector);
	}

	/* Step 7: Load the page that corresponds to the new current slot position.
	 */
	actuator_error_store_load_page();

	/* Step 8: On the first error of this boot, mark the header sector as
	 * written. */
	if (s_error_count == 0U)
		actuator_error_header_mark_written();

	s_error_count++;
}

/*******************************************************************************
 * Code - Error Store (private)
 ******************************************************************************/

/**
 * @brief   Fill an error structure from the given EPOS track and boot counter.
 * @details Reads the EPOS error history object (0x1003) for the specific
 *          drive index. If the read fails the error code is set to 0xFFFF.
 * @param   err       Destination structure to fill.
 * @param   track     EPOS track that generated the fault.
 * @param   boot_cnt  Boot counter value to embed.
 */
static void actuator_error_store_fill_from_epos(actuator_error_t *err,
												epos_track_t *track,
												uint8_t boot_cnt)
{
	uint8_t epos_idx = (uint8_t)(track - epos.track);

	/* Encode the absolute slot number as the error identifier. */
	err->num_error =
		(uint16_t)(s_current_sector * ACTUATOR_ERROR_SLOTS_PER_SECTOR +
				   s_current_slot);
	err->statusword = track->status;
	err->task_state = (uint8_t)task_epos_state;
	err->uc_boot_cnt = boot_cnt;

	/* Read the 32-bit error code from the EPOS object dictionary and truncate
	 * to 16 bits. */
	if (epos_idx < epos.track_count) {
		uint32_t full_error_code = 0U;
		if (epos_obj_read(&epos, epos_idx, &ObjErrorHistory1,
						  &full_error_code) != 0)
			err->error_code = 0xFFFFU; /* Read failed - store sentinel value. */
		else
			err->error_code = (uint16_t)(full_error_code & 0xFFFFU);
	}
}

/**
 * @brief   Load the flash page for the current slot into the RAM page buffer.
 * @details The page buffer always mirrors the 256-byte flash page that
 *          contains s_current_slot. This must be called after any change
 *          to s_current_sector or s_current_slot.
 */
static void actuator_error_store_load_page(void)
{
	uint32_t page_addr =
		actuator_error_page_addr(s_current_sector, s_current_slot);
	memcpy(s_page_buffer, (const uint8_t *)page_addr, ACTUATOR_ERROR_PAGE_SIZE);
}

/**
 * @brief   Erase a single error log sector.
 * @param   sector_idx  Index into s_error_sectors[] of the sector to erase.
 */
static void actuator_error_store_erase_sector(uint8_t sector_idx)
{
	__disable_irq();
	iap_hal_sector_num_t sec =
		HAL_IAP_GetSectorNumber(s_error_sectors[sector_idx]);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_EraseSector(sec, sec);
	__enable_irq();
}

/*******************************************************************************
 * Code - Error Header (public)
 ******************************************************************************/

/**
 * @brief   Check whether the previous boot left unreported errors.
 * @details If the write flag in the active header slot has been decremented
 *          but the reported flag has not caught up, errors were logged last
 *          boot but never acknowledged by the host.
 */
void actuator_error_header_check_unreported(void)
{
	uint32_t w, r, idx;

	/* Locate the active header slot. */
	actuator_error_header_find(&w, &r, &idx);

	/* write has been decremented but reported has not been updated ->
	 * unreported errors. */
	if (w != ACTUATOR_ERROR_HEADER_FLAG_FRESH && w != r)
		unreported_errors_at_boot = 1U;
}

/**
 * @brief   Mark that at least one error was written during this boot.
 * @details Clears one bit in the write flag of the active header slot.
 *          Runs at most once per boot (guarded by s_boot_error_flagged).
 *          If the header sector is full it is erased before writing.
 */
void actuator_error_header_mark_written(void)
{
	/* Guard: execute at most once per boot. */
	if (s_boot_error_flagged)
		return;

	uint32_t w, r, idx;

	/* Step 1: Find the active header slot. */
	actuator_error_header_find(&w, &r, &idx);

	/* Step 2: If the sector is completely full, erase it and restart from slot
	 * 0. */
	if (idx >= ACTUATOR_ERROR_HEADER_SLOTS_MAX) {
		actuator_error_header_erase();
		idx = 0U;
		w = ACTUATOR_ERROR_HEADER_FLAG_FRESH;
	}

	uint32_t write_addr =
		ACTUATOR_ERROR_HEADER_ADDR + idx * ACTUATOR_ERROR_HEADER_SLOT_SIZE;

	/* Step 3: If the current slot's write flag is fully consumed, advance to
	 * the next slot. */
	if (w == 0x00000000U) {
		idx++;

		if (idx >= ACTUATOR_ERROR_HEADER_SLOTS_MAX) {
			/* Wrapped around - erase and start from the beginning. */
			actuator_error_header_erase();
			idx = 0U;
		}

		write_addr =
			ACTUATOR_ERROR_HEADER_ADDR + idx * ACTUATOR_ERROR_HEADER_SLOT_SIZE;
		w = ACTUATOR_ERROR_HEADER_FLAG_FRESH;
	}

	/* Step 4: Compute the next flag value (clear one bit to record this boot's
	 * errors). */
	uint32_t new_w = actuator_error_header_next_flag(w);

	/* Step 5: Read the containing 256-byte page into RAM (flash can only be
	 * written in pages). */
	uint32_t page_addr = write_addr & ~(ACTUATOR_ERROR_PAGE_SIZE - 1U);
	uint32_t offset_in_page = write_addr - page_addr;
	memcpy(s_header_page, (const uint8_t *)page_addr, ACTUATOR_ERROR_PAGE_SIZE);

	/* Step 6: Patch the write flag in the RAM copy of the page. */
	memcpy(s_header_page + offset_in_page, &new_w, sizeof(uint32_t));

	/* Step 7: Write the updated page back to flash. */
	__disable_irq();
	iap_hal_sector_num_t sec =
		HAL_IAP_GetSectorNumber(ACTUATOR_ERROR_HEADER_ADDR);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_CopyRAM2Flash((uint8_t *)page_addr, s_header_page,
						  IAP_HAL_WRITE_256);
	__enable_irq();

	/* Mark so subsequent errors this boot do not re-enter this function. */
	s_boot_error_flagged = 1U;
}

/**
 * @brief   Mark all errors from the current boot as reported to the host.
 * @details Copies the write flag value into the reported flag of the active
 *          header slot. After this call, write == reported, meaning the host
 *          has been informed of all errors logged this boot.
 */
void actuator_error_header_mark_reported(void)
{
	uint32_t w, r, idx;

	/* Step 1: Find the active header slot. */
	actuator_error_header_find(&w, &r, &idx);

	/* If no errors have been written, or everything is already reported,
	 * nothing to do. */
	if (w == ACTUATOR_ERROR_HEADER_FLAG_FRESH || w == r)
		return;

	/* Step 2: Compute address of the reported flag (second uint32_t in the
	 * slot). */
	uint32_t reported_addr = ACTUATOR_ERROR_HEADER_ADDR +
							 idx * ACTUATOR_ERROR_HEADER_SLOT_SIZE +
							 sizeof(uint32_t);

	/* Step 3: Read the containing page into RAM. */
	uint32_t page_addr = reported_addr & ~(ACTUATOR_ERROR_PAGE_SIZE - 1U);
	uint32_t offset_in_page = reported_addr - page_addr;
	memcpy(s_header_page, (const uint8_t *)page_addr, ACTUATOR_ERROR_PAGE_SIZE);

	/* Step 4: Patch the reported flag to match the write flag. */
	memcpy(s_header_page + offset_in_page, &w, sizeof(uint32_t));

	/* Step 5: Write the updated page back to flash. */
	__disable_irq();
	iap_hal_sector_num_t sec =
		HAL_IAP_GetSectorNumber(ACTUATOR_ERROR_HEADER_ADDR);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_CopyRAM2Flash((uint8_t *)page_addr, s_header_page,
						  IAP_HAL_WRITE_256);
	__enable_irq();
}

/*******************************************************************************
 * Code - Error Header (private)
 ******************************************************************************/

/**
 * @brief   Locate the currently active header slot.
 * @details Iterates header slots looking for:
 *            1. First fully fresh slot (0xFFFF/0xFFFF) -> previous slot is
 * active.
 *            2. First active slot (write != 0 or reported != 0, but not both
 * 0).
 *            3. If no active slot found, reports idx =
 * ACTUATOR_ERROR_HEADER_SLOTS_MAX to signal that the sector is full.
 * @param   out_write     Write flag value of the active slot.
 * @param   out_reported  Reported flag value of the active slot.
 * @param   out_idx       Index of the active slot within the header sector.
 */
static void actuator_error_header_find(uint32_t *out_write,
									   uint32_t *out_reported,
									   uint32_t *out_idx)
{
	for (uint32_t i = 0U; i < ACTUATOR_ERROR_HEADER_SLOTS_MAX; i++) {
		uint32_t addr =
			ACTUATOR_ERROR_HEADER_ADDR + i * ACTUATOR_ERROR_HEADER_SLOT_SIZE;
		uint32_t w = *(volatile uint32_t *)(addr);
		uint32_t r = *(volatile uint32_t *)(addr + sizeof(uint32_t));

		/* Both fields are erased - this slot has never been used. */
		if (w == ACTUATOR_ERROR_HEADER_FLAG_FRESH &&
			r == ACTUATOR_ERROR_HEADER_FLAG_FRESH) {
			if (i == 0U) {
				/* Entire sector is fresh - no errors have ever been logged. */
				*out_write = ACTUATOR_ERROR_HEADER_FLAG_FRESH;
				*out_reported = ACTUATOR_ERROR_HEADER_FLAG_FRESH;
				*out_idx = 0U;
			} else {
				/* Return the previous slot, which was the last one written. */
				uint32_t prev_addr = ACTUATOR_ERROR_HEADER_ADDR +
									 (i - 1U) * ACTUATOR_ERROR_HEADER_SLOT_SIZE;
				*out_write = *(volatile uint32_t *)(prev_addr);
				*out_reported =
					*(volatile uint32_t *)(prev_addr + sizeof(uint32_t));
				*out_idx = i - 1U;
			}
			return;
		}

		/* Both fields are zero - slot is fully consumed, move to the next one.
		 */
		if (w == 0x00000000U && r == 0x00000000U)
			continue;

		/* Partially written slot - this is the active one. */
		*out_write = w;
		*out_reported = r;
		*out_idx = i;
		return;
	}

	/* Reached the end without finding a free or active slot - sector is full.
	 */
	*out_write = 0x00000000U;
	*out_reported = 0x00000000U;
	*out_idx = ACTUATOR_ERROR_HEADER_SLOTS_MAX;
}

/**
 * @brief   Erase the entire header sector.
 * @details Called when the header sector is full (all slots consumed).
 *          After erasing, the logger starts from slot 0 with a fresh flag.
 */
static void actuator_error_header_erase(void)
{
	__disable_irq();
	iap_hal_sector_num_t sec =
		HAL_IAP_GetSectorNumber(ACTUATOR_ERROR_HEADER_ADDR);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_EraseSector(sec, sec);
	__enable_irq();
}

/**
 * @brief   Clear the lowest set bit in a header flag value.
 * @details Each boot that logs errors consumes one bit. When all 32 bits
 *          are cleared (returns 0x00000000) the slot is fully consumed and
 *          the logger moves to the next header slot.
 * @param   current  Current flag value.
 * @return  Flag value with one additional bit cleared, or 0x00000000 if
 *          all bits were already cleared.
 */
static uint32_t actuator_error_header_next_flag(uint32_t current)
{
	for (int i = 0; i < 32; i++) {
		if (current & (1U << i))
			return current & ~(1U << i);
	}
	return 0x00000000U; /* All bits are already cleared. */
}