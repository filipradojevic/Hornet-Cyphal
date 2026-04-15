/**
 * @file    task_epos.h
 * @brief   Task EPOS - handle EPOS4 actuators
 * @version 1.0.0
 * @date    26.02.2025
 * @author  LisumLab
 */

#ifndef TASK_EPOS_H
#define TASK_EPOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "types.h"
#include <stdint.h>

#include "mav.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* maximum number of retries during boot */
#define TASK_EPOS_BOOT_MAX_RETRY 5
/* maximum number of homing retries (homing command) */
#define TASK_EPOS_HOMING_MAX_RETRY 5
/* maximum number of initialization retries (init command) */
#define TASK_EPOS_INIT_MAX_RETRY 5
/* maximum number of fault reset retries (fault reset command) */
#define TASK_EPOS_FAULT_RESET_MAX_RETRY 100
/* maximum number of sweep retries (sweep command) */
#define TASK_EPOS_SWEEP_MAX_RETRY 5
/* homing timeout [ms] */
#define TASK_EPOS_HOMING_TIMEOUT_MS 20000

/* actuator master node id */
#define ACT_MASTER_NODE_ID 0
/* first actuator canopen node id */
#define ACT1_NODE_ID 1
/* second actuator canopen node id */
#define ACT2_NODE_ID 2
/* third actuator canopen node id */
#define ACT3_NODE_ID 3
/* fourth actuator canopen node id */
#define ACT4_NODE_ID 4

/* actuator sensor1 type */
#define ACT_SENSOR1_TYPE EPOS_SENSOR1_TYPE_NONE
/* actuator sensor2 type */
#define ACT_SENSOR2_TYPE EPOS_SENSOR2_TYPE_SSI_ABS_ENC
/* actuator sensor3 type */
#define ACT_SENSOR3_TYPE EPOS_SENSOR3_TYPE_DIGITAL_HALL_SENSOR
/* actuator main sensor */
#define ACT_MAIN_SENSOR EPOS_SENSOR2
/* actuator auxiliary sensor */
#define ACT_AUX_SENSOR EPOS_SENSOR_NONE

/* ssi absolute encoder data rate */
#define SSI_ABS_ENC_DATA_RATE 400
/* ssi absolute encoder special bits leading */
#define SSI_ABS_ENC_SPECIAL_BITS_LEADING 8
/* ssi absolute encoder multi turn bits */
#define SSI_ABS_ENC_MULTI_TURN_BITS 16
/* ssi absolute encoder single turn bits */
#define SSI_ABS_ENC_SINGLE_TURN_BITS 17
/* ssi absolute encoder special bits trailing */
#define SSI_ABS_ENC_SPECIAL_BITS_TRAILING 0
/* ssi absolute encoder direction */
#define SSI_ABS_ENC_DIRECTION EPOS_SENSOR_DIRECTION_MAXON
/* ssi absolute encoder ecoding type */
#define SSI_ABS_ENC_ENCODING_TYPE EPOS_SSI_ENCODING_TYPE_BINARY
/* ssi absolute encoder timeout */
#define SSI_ABS_ENC_TIMEOUT 7
/* ssi absolute encoder power up time */
#define SSI_ABS_ENC_POWER_UP_TIME 0
/* ssi absolute encoder multi turn bits used */
#define SSI_ABS_ENC_MULTI_TURN_BITS_USED 15
/* ssi absolute encoder single turn bits used */
#define SSI_ABS_ENC_SINGLE_TURN_BITS_USED 17

/* hall sensor method */
#define HALL_METHOD EPOS_SENSOR_METHOD1
/* hall sensor polarity */
#define HALL_POLARITY EPOS_SENSOR_DIRECTION_MAXON

/* motor nominal current */
#define MOTOR_NOM_CURR 6550
/* motor current limit */
#define MOTOR_CURR_LIMIT 2 * MOTOR_NOM_CURR
/* motor pole pair number */
#define MOTOR_PP_NUM 8
/* motor thermal time constant */
#define MOTOR_TT_CONSTANT 177
/* motor torque constant */
#define MOTOR_TORQUE_CONSTANT 89200
/* motor type */
#define MOTOR_TYPE EPOS_MOTOR_TYPE_SIN_PM_BL
/* motor maximum speed */
#define MOTOR_MAX_SPEED 6000

/* commutation sensor absolute */
#define COMMUTATION_SENSOR_ABS EPOS_COMMUTATION_SENSOR_ABS_SENSOR3
/* commutation sensor relative */
#define COMMUTATION_SENSOR_REL EPOS_COMMUTATION_SENSOR_REL_SENSOR2

/* homing speed for switch search */
#define HOMING_HSPEED_SW_SEARCH 100
/* homing speed zero search */
#define HOMING_HSPEED_ZERO_SEARCH 10
/* homing acceleration */
#define HOMING_HACCEL 1000
/* homing offset move distance */
#define HOMING_HOFF_MOVE_DIST 0
/* homing current threshold */
#define HOMING_HCURR_THRESHOLD 2000

/* csp configuration */
/* cyclic synchronous position mode follow error window */
#define CSP_FOLLOW_ERROR_WINDOW 0xFFFFFFFF
/* cyclic synchronous position mode maximum position */
#define CSP_MAX_POS 0
/* cyclic synchronous position mode minimum position */
#define CSP_MIN_POS 0
/* cyclic synchronous position mode deceleration */
#define CSP_PDECEL 10000
/* cyclic synchronous position mode quick stop deceleration */
#define CSP_QSDECEL 10000
/* cyclic synchronous position mode interpolation time period */
#define CSP_INTERPOLATION_TIME_PERIOD 5

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef enum task_epos_state_e {
	TASK_EPOS_STATE_INIT_FAILED = -1,
	TASK_EPOS_STATE_INIT = 0,
	TASK_EPOS_STATE_READY_TO_ARM = 1,
	TASK_EPOS_STATE_ARMED = 2
} task_epos_state_e;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

void task_epos(void *arg);

#if MAVLINK_OR_CYPHAL

void task_epos_cmd(mavlink_command_long_t *cmd, uint8_t sysid, uint8_t compid);

void task_epos_ctrl(mavlink_lisum_manual_ctrl_hornet_t *cmd);

#else

void task_epos_cmd(messages_cyphal_uavcan_common_CommandLong_1_0 *cmd,
				   uint8_t sysid, uint8_t compid);

void task_epos_ctrl(
	messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0 *cmd);

#endif

#ifdef __cplusplus
}
#endif

#endif /* TASK_EPOS_H */