/**
 * @file    protocol_utility.h
 * @brief   Project common defines and types.
 * @version 1.0.0
 * @date    16.12.2025
 * @author  BetaTehPro
 */

#ifndef PROTOCOL_UTILITY_H
#define PROTOCOL_UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "types.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

typedef enum {
	NODE_MODE_OPERATIONAL = uavcan_node_Mode_1_0_OPERATIONAL,
	NODE_MODE_INITIALIZATION = uavcan_node_Mode_1_0_INITIALIZATION,
	NODE_MODE_MAINTENANCE = uavcan_node_Mode_1_0_MAINTENANCE,
	NODE_MODE_SOFTWARE_UPDATE = uavcan_node_Mode_1_0_SOFTWARE_UPDATE,
	NODE_MODE_IDLE = 4,
	NODE_MODE_CRITICAL_FAILURE = 5
} node_mode_state_t;

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API Prototypes
 ******************************************************************************/

/**
 * @brief Adjust task priority based on the current Cyphal node mode.
 *
 * This helper updates the priority of the main work task (`task_work_handle`) to
 * favour file/SD/LittleFS activity when the node is in maintenance mode and to
 * lower it when returning to normal operation. It uses the global `file_sync`
 * flag to perform the change only once per transition.
 *
 * @param current_mode Current node mode as `uavcan_node_Mode_1_0`.
 *
 * @note This function may call FreeRTOS API `vTaskPrioritySet()` and therefore
 *       must be called from a context that is allowed to change task
 *       priorities (typically a task context). It does not block.
 *
 * @since 1.0.0
 */
void cyphal_task_priority(const node_mode_state_t current_mode);

/**
 * @brief One-time initialization of LittleFS + ULog when entering INITIALIZATION
 *        mode for Cyphal builds.
 *
 * When called repeatedly, the function will perform real initialization only
 * once (guarded by the `create_file` flag). The implementation mounts the
 * filesystem and starts the ULog subsystem when `current_node_mode` indicates
 * initialization.
 *
 * @param current_mode Current node mode as `uavcan_node_Mode_1_0`.
 *
 * @note This function may call blocking filesystem and init APIs; call it from
 *       a task context. Side effects: sets `create_file` to true when the
 *       initialization succeeds.
 *
 * @since 1.0.0
 */

void cyphal_ulog_lfs_initialization(void);

/**
 * @brief One-time initialization of LittleFS + ULog for MAVLink builds.
 *
 * Ensures the log filesystem and ULog backend are initialized exactly once.
 * This variant is used when the project is compiled in MAVLink mode (that is,
 * `MAVLINK_OR_CYPHAL` is true) where no Cyphal-mode handshake is required.
 *
 * @note This function may call blocking filesystem and init APIs; call it from
 *       a task context. Side effects: sets `create_file` to true when the
 *       initialization succeeds.
 *
 * @since 1.0.0
 */
void mavlink_ulog_lfs_initialization(void);

/**
 * @brief Print a human-readable description of `uavcan_node_Mode_1_0`.
 *
 * Utility used for debugging and logging. The function prints a short
 * descriptive string for the provided `mode` value to standard output.
 *
 * @param mode The mode value (one of the `uavcan_node_Mode_1_0` enum values).
 *
 * @note This helper is intended for debug/console output and does not return
 *       any status.
 *
 * @since 1.0.0
 */
void print_node_mode(uint8_t mode);

/**
 * @brief Populate a `uavcan_node_GetInfo_Response_1_0` structure with build
 *        and runtime metadata.
 *
 * The function fills the provided `resp` structure with protocol version,
 * hardware/software version, VCS revision, unique ID, node name and other
 * informational fields. It does not perform serialization or transmit the
 * response; callers should serialize `resp` with the generated nunavut
 * serializer and publish it using the Cyphal transport layer.
 *
 * @param[out] resp Pointer to an allocated `uavcan_node_GetInfo_Response_1_0`
 *                  structure that will be populated by this function.
 *
 * @pre `resp` must point to valid writable memory large enough to hold the
 *      response object. The function does not allocate dynamic memory.
 *
 * @since 1.0.0
 */
void build_getinfo_response(uavcan_node_GetInfo_Response_1_0* resp);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_UTILITY_H */