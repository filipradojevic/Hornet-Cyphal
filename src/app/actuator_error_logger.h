/**
 * @file    actuator_error_logger.h
 * @brief   Actuator flash error logger - public interface.
 * @details Provides types, defines, and function declarations for logging
 *          EPOS4 actuator errors into internal uC flash memory. Errors are
 *          stored across 7 dedicated flash sectors and can be retrieved after
 *          reboot for diagnostics and fault analysis.
 *
 *          Flash layout:
 *            Sectors 8-14  : Error log storage (7 x 512 slots x 8 bytes)
 *            Sector 15     : Error header (write/reported flags per boot)
 *
 * @version 1.0.0
 * @date    18.05.2026
 * @author  LisumLab
 */

#ifndef ACTUATOR_ERROR_LOGGER_H
#define ACTUATOR_ERROR_LOGGER_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/** @defgroup actuator_error_store_cfg Error Store - Flash Geometry
 *  @brief Defines describing the physical layout of error log storage.
 *  @{
 */

/** Number of flash sectors dedicated to error log storage (sectors 8-14). */
#define ACTUATOR_ERROR_SECTOR_COUNT 7U

/** Size of one error slot in bytes. Must match sizeof(actuator_error_t). */
#define ACTUATOR_ERROR_SLOT_SIZE 8U

/** Number of error slots that fit in one 256-byte flash write page. */
#define ACTUATOR_ERROR_SLOTS_PER_PAGE 32U

/** Total number of error slots available per sector. */
#define ACTUATOR_ERROR_SLOTS_PER_SECTOR 512U

/** @} */

/** @defgroup actuator_error_header_cfg Error Header - Flash Geometry
 *  @brief Defines describing the physical layout of the error header sector.
 *         The header tracks whether errors from a given boot have been
 *         reported to the host. One slot is consumed per boot cycle.
 *  @{
 */

/** Base address of the error header sector (sector 15, reserved). */
#define ACTUATOR_ERROR_HEADER_ADDR IAP_HAL_SECTOR_15_ADDR

/** Size of one header slot in bytes: 4 bytes write flag + 4 bytes reported
 * flag. */
#define ACTUATOR_ERROR_HEADER_SLOT_SIZE 8U

/** Maximum number of header slots that fit in the 4096-byte header sector. */
#define ACTUATOR_ERROR_HEADER_SLOTS_MAX                                        \
	(4096U / ACTUATOR_ERROR_HEADER_SLOT_SIZE)

/** Header slot flag value indicating the slot has never been written (erased
 * state). */
#define ACTUATOR_ERROR_HEADER_FLAG_FRESH 0xFFFFFFFFU

/** Header slot flag value indicating the slot has been written at least once.
 */
#define ACTUATOR_ERROR_HEADER_FLAG_SET 0xFEFFFFFFU

/** @} */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/**
 * @brief   Flash sector base addresses for error log storage.
 * @details Each enumerator maps to the start address of a dedicated flash
 *          sector. Sectors are used sequentially; when all are full, sector 0
 *          is erased and reused (ring buffer behavior).
 */
typedef enum actuator_error_sector_e {
	ACTUATOR_ERROR_SECTOR_0 = IAP_HAL_SECTOR_8_ADDR,
	ACTUATOR_ERROR_SECTOR_1 = IAP_HAL_SECTOR_9_ADDR,
	ACTUATOR_ERROR_SECTOR_2 = IAP_HAL_SECTOR_10_ADDR,
	ACTUATOR_ERROR_SECTOR_3 = IAP_HAL_SECTOR_11_ADDR,
	ACTUATOR_ERROR_SECTOR_4 = IAP_HAL_SECTOR_12_ADDR,
	ACTUATOR_ERROR_SECTOR_5 = IAP_HAL_SECTOR_13_ADDR,
	ACTUATOR_ERROR_SECTOR_6 = IAP_HAL_SECTOR_14_ADDR,
} actuator_error_sector_e;

/**
 * @brief   Persistent header slot stored in sector 15.
 * @details Tracks whether errors logged during a boot cycle have been
 *          reported to the host. The write flag is decremented (one bit
 *          cleared) each boot an error occurs. The reported flag is updated
 *          to match the write flag once the host acknowledges the errors.
 *
 *          Slot states:
 *            write == 0xFFFFFFFF && reported == 0xFFFFFFFF  ->  slot is fresh
 * (unused) write != reported                               ->  unreported
 * errors exist write == reported                               ->  all errors
 * reported write == 0x00000000 && reported == 0x00000000  ->  slot fully
 * consumed
 */
#pragma pack(push, 1)
typedef struct {
	uint32_t
		last_error_flag; /**< Bit-field decremented each boot with errors. */
	uint32_t last_reported_flag; /**< Mirrors write flag after host
									acknowledgment. */
} actuator_last_error_t;
#pragma pack(pop)

/**
 * @brief   Single error log entry stored in flash.
 * @details Occupies exactly ACTUATOR_ERROR_SLOT_SIZE (8) bytes. One entry is
 *          written per detected EPOS4 fault. The num_error field encodes the
 *          absolute slot index across all sectors, allowing the host to
 *          determine the order and location of each error in flash.
 *
 *          Memory layout (packed, no padding):
 *            Offset 0  : num_error   (2 bytes)
 *            Offset 2  : error_code  (2 bytes)
 *            Offset 4  : statusword  (2 bytes)
 *            Offset 6  : task_state  (1 byte)
 *            Offset 7  : uc_boot_cnt (1 byte)
 */
#pragma pack(push, 1)
typedef struct {
	uint16_t num_error;	 /**< Absolute slot index: sector * SLOTS_PER_SECTOR +
							slot. */
	uint16_t error_code; /**< EPOS4 error code read from object 0x1003 (Error
							History). */
	uint16_t statusword; /**< EPOS4 statusword at the moment the fault was
							detected. */
	uint8_t task_state;	 /**< State of task_epos (task_epos_state_e) at fault
							time. */
	uint8_t uc_boot_cnt; /**< Boot counter value at fault time, for cross-boot
							correlation. */
} actuator_error_t;
#pragma pack(pop)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief   Initialize the actuator error logger.
 * @details Scans all error log sectors to locate the first free slot
 *          (0xFFFFFFFF marker). Restores the boot counter from the last
 *          written entry and loads the corresponding flash page into the
 *          RAM page buffer. Must be called once at startup before any
 *          call to actuator_error_store_write().
 */
void actuator_error_store_init(void);

/**
 * @brief   Log an EPOS4 actuator error to flash.
 * @details Fills the error structure from the EPOS track and boot counter,
 *          copies it into the RAM page buffer, and writes the updated 256-byte
 *          page back to flash. Advances the current slot; if the sector is
 *          full, erases the next sector and moves to it. On the first error
 *          of each boot, also marks the header sector as written.
 * @param   err       Pointer to an error structure to be filled and stored.
 * @param   track     Pointer to the EPOS track that generated the fault.
 * @param   boot_cnt  Current boot counter value to embed in the log entry.
 */
void actuator_error_store_write(actuator_error_t *err, epos_track_t *track,
								uint8_t boot_cnt);

/**
 * @brief   Check whether unreported errors from a previous boot exist.
 * @details Reads the active header slot and sets the unreported_errors_at_boot
 *          flag if the write flag has been decremented but the reported flag
 *          has not yet been updated to match. Call once at startup after
 *          actuator_error_store_init().
 */
void actuator_error_header_check_unreported(void);

/**
 * @brief   Mark that at least one error was written during this boot.
 * @details Clears one bit in the write flag of the active header slot to
 *          signal that new errors exist. Guarded by boot_error_flagged so
 *          it executes at most once per boot, regardless of how many errors
 *          are logged. Called internally by actuator_error_store_write().
 */
void actuator_error_header_mark_written(void);

/**
 * @brief   Mark all errors from the current boot as reported to the host.
 * @details Updates the reported flag in the active header slot to match the
 *          write flag. Call this after successfully transmitting all pending
 *          errors to the host application.
 */
void actuator_error_header_mark_reported(void);

#endif /* ACTUATOR_ERROR_LOGGER_H */