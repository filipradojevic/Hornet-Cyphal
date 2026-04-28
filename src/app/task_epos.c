/**
 * @file    task_epos.c
 * @brief   Task EPOS - handle EPOS4 actuators
 * @version 1.0.0
 * @date    26.02.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <string.h>

#include "task_epos.h"
#include "types.h"

/* Peripherals */
#include "can.h"

/* External hardware drivers */
#include "epos.h"
#include "epos_csp.h"
#include "epos_eeprom.h"
#include "epos_homing.h"

/* Lib */
#include "canopen.h"

/* Middleware */
#include "FreeRTOS.h"
#include "mav.h"
#include "semphr.h"
#include "timers.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* mm to epos increments multiplier */
#define TASK_EPOS_MM_TO_INC (-131072.0f / 5.0f)
/* epos increments to mm multiplier */
#define TASK_EPOS_INC_TO_MM 1.0f / TASK_EPOS_MM_TO_INC
/* rpm to mm/s multiplier */
#define TASK_EPOS_RPM_TO_MMS (-131072.0f / 5.0f)
/* mm/s to rpm multiplier */
#define TASK_EPOS_MMS_TO_RPM 1.0f / TASK_EPOS_RPM_TO_MMS
/* A to mA multiplier */
#define TASK_EPOS_A_TO_MA 1000.0f
/* mA to A multiplier */
#define TASK_EPOS_MA_TO_A 1.0f / TASK_EPOS_A_TO_MA

/* setpoint conversion (from mm to inc*(-1)) */
#define TASK_EPOS_SP_CONV(x) (x * TASK_EPOS_MM_TO_INC)

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern QueueHandle_t mailbox_epos_ctrl;
extern QueueHandle_t queue_mav_ack;
extern QueueHandle_t queue_mav_act_data;
extern QueueHandle_t queue_mav_manual_ctrl;
extern QueueHandle_t queue_epos_cmd;

static const epos_cfg_t epos_cfg_data = {
	.unit_cfg.pos_prefix = EPOS_UNIT_PREFIX_NONE,
	.unit_cfg.vel_prefix = EPOS_UNIT_PREFIX_NONE,
	.unit_cfg.accel_prefix = EPOS_UNIT_PREFIX_NONE,

	.sensor_cfg.sensor1_type = ACT_SENSOR1_TYPE,
	.sensor_cfg.sensor2_type = ACT_SENSOR2_TYPE,
	.sensor_cfg.sensor3_type = ACT_SENSOR3_TYPE,
	.sensor_cfg.main_sensor = ACT_MAIN_SENSOR,
	.sensor_cfg.aux_sensor = ACT_AUX_SENSOR,

	.ssi_abs_enc_cfg.data_rate = SSI_ABS_ENC_DATA_RATE,
	.ssi_abs_enc_cfg.special_bits_leading = SSI_ABS_ENC_SPECIAL_BITS_LEADING,
	.ssi_abs_enc_cfg.multi_turn_bits = SSI_ABS_ENC_MULTI_TURN_BITS,
	.ssi_abs_enc_cfg.single_turn_bits = SSI_ABS_ENC_SINGLE_TURN_BITS,
	.ssi_abs_enc_cfg.special_bits_trailing = SSI_ABS_ENC_SPECIAL_BITS_TRAILING,
	.ssi_abs_enc_cfg.direction = SSI_ABS_ENC_DIRECTION,
	.ssi_abs_enc_cfg.encoding_type = SSI_ABS_ENC_ENCODING_TYPE,
	.ssi_abs_enc_cfg.timeout = SSI_ABS_ENC_TIMEOUT,
	.ssi_abs_enc_cfg.power_up_time = SSI_ABS_ENC_POWER_UP_TIME,
	.ssi_abs_enc_cfg.multi_turn_bits_used = SSI_ABS_ENC_MULTI_TURN_BITS_USED,
	.ssi_abs_enc_cfg.single_turn_bits_used = SSI_ABS_ENC_SINGLE_TURN_BITS_USED,

	.hall_cfg.method = HALL_METHOD,
	.hall_cfg.polarity = HALL_POLARITY,

	.motor_cfg.nom_curr = MOTOR_NOM_CURR,
	.motor_cfg.curr_limit = MOTOR_CURR_LIMIT,
	.motor_cfg.pp_num = MOTOR_PP_NUM,
	.motor_cfg.tt_constant = MOTOR_TT_CONSTANT,
	.motor_cfg.torque_constant = MOTOR_TORQUE_CONSTANT,
	.motor_cfg.type = MOTOR_TYPE,
	.motor_cfg.max_speed = MOTOR_MAX_SPEED,

	.commutation_sensor_abs = COMMUTATION_SENSOR_ABS,
	.commutation_sensor_rel = COMMUTATION_SENSOR_REL};

static const epos_homing_cfg_t homing_cfg = {
	.hspeed_sw_search = HOMING_HSPEED_SW_SEARCH,
	.hspeed_zero_search = HOMING_HSPEED_ZERO_SEARCH,
	.haccel = HOMING_HACCEL,
	.hoff_move_dist = HOMING_HOFF_MOVE_DIST,
	.hcurr_threshold = HOMING_HCURR_THRESHOLD};

static const epos_csp_cfg_t csp_cfg = {
	.follow_error_window = CSP_FOLLOW_ERROR_WINDOW,
	.max_pos = CSP_MAX_POS,
	.min_pos = CSP_MIN_POS,
	.pdecel = CSP_PDECEL,
	.qsdecel = CSP_QSDECEL,
	.interpolation_time_period = CSP_INTERPOLATION_TIME_PERIOD};

task_epos_state_e task_epos_state = TASK_EPOS_STATE_INIT;

static canopen_handle_t canopen;
static epos_t epos;

static epos_custom_eeprom_t eeprom[4];

/** @note to be removed */
static int32_t sp[4] = {0};
static uint8_t act_id = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* check all tracked EPOS devices */
static uint32_t _epos_check(void);

/* initialize tracked EPOS devices (actuator, homing and csp config)*/
static uint32_t _epos_init(void);

/* execute homing of tracked EPOS devices */
static uint32_t _epos_homing(LISUM_HORNET_HOMING_METHOD method);

/* reset all tracked EPOS devices faults */
static uint32_t _epos_fault_reset(void);

/* perform full sweep on all tracked EPOS devices */
static uint32_t _epos_sweep(void);

/* send setpoints to tracked EPOS devices */
static void _epos_ctrl(uint8_t act_id, int32_t *setpoints, uint16_t cw);

/* get custom eeprom of all tracked EPOS devices */
static uint32_t _epos_custom_eeprom_get(void);

/* execute init command */
static uint32_t _work_init(void);

/* execute homing command */
static uint32_t _work_homing(LISUM_HORNET_HOMING_METHOD method);

/* execute fault reset command */
static uint32_t _work_fault_reset(void);

/* execute arm/disarm command */
static uint32_t _work_arm_disarm(uint8_t arm);

/* process request */
static void _process_request(mavlink_command_long_t *cmd);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_epos(void *arg)
{

#if MAVLINK_OR_CYPHAL

	mavlink_lisum_manual_ctrl_hornet_t ctrl = {0};
	mavlink_command_long_t cmd = {0};
	mavlink_lisum_power_hornet_act_data_t act_data = {0};

#else

	messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0 ctrl = {0};
	messages_cyphal_uavcan_common_CommandLong_1_0 cmd = {0};
	messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0 act_data = {0};

#endif /* MAVLINK_OR_CYPHAL */

	TickType_t status_poll_time = xTaskGetTickCount();
	TickType_t last_wake = xTaskGetTickCount();
	uint8_t epos_id1;

	/*------------------------------- CANOpen --------------------------------*/
	canopen_init(&canopen, CAN_HAL_INSTANCE_0, ACT_MASTER_NODE_ID);

	/*-------------------------------- EPOS ----------------------------------*/
	epos_init(&epos, &canopen);
	epos_id1 = epos_track(&epos, ACT1_NODE_ID);

	if (_epos_check() || _epos_custom_eeprom_get())
		task_epos_state = TASK_EPOS_STATE_INIT_FAILED;
	else
		task_epos_state = TASK_EPOS_STATE_READY_TO_ARM;

	for (;;) {

		vTaskDelayUntil(&last_wake,
						pdMS_TO_TICKS(CSP_INTERPOLATION_TIME_PERIOD));

		/* process commands */
		if (pdPASS == xQueueReceive(queue_epos_cmd, &cmd, 0)) {
			_process_request(&cmd);
		}

		/* process setpoints */
		if (pdPASS == xQueueReceive(mailbox_epos_ctrl, &ctrl, 0)) {
			int32_t tmp_sp = 0;
			int32_t limit = 0;

			// Take tmp set point
			tmp_sp = (int32_t)TASK_EPOS_SP_CONV(ctrl.pos_sp_act);

			// Take actuator id
			act_id = ctrl.act_id;

			// Calculate real set point and check limits based on actuator id
			switch (ctrl.act_id) {
			case ACTUATOR_ID_0: {
				limit = (int32_t)(eeprom[ACTUATOR_ID_0].param1 * -1);
				if (tmp_sp <= 0 && tmp_sp >= limit) {
					sp[ACTUATOR_ID_0] = tmp_sp;
				}
				break;
			}
			case ACTUATOR_ID_1: {
				limit = (int32_t)(eeprom[ACTUATOR_ID_1].param1 * -1);
				if (tmp_sp <= 0 && tmp_sp >= limit) {
					sp[ACTUATOR_ID_1] = tmp_sp;
				}
				break;
			}
			case ACTUATOR_ID_2: {
				limit = (int32_t)(eeprom[ACTUATOR_ID_2].param1 * -1);
				if (tmp_sp <= 0 && tmp_sp >= limit) {
					sp[ACTUATOR_ID_2] = tmp_sp;
				}
				break;
			}
			case ACTUATOR_ID_3: {
				limit = (int32_t)(eeprom[ACTUATOR_ID_3].param1 * -1);
				if (tmp_sp <= 0 && tmp_sp >= limit) {
					sp[ACTUATOR_ID_3] = tmp_sp;
				}
				break;
			}
			default: {
				break;
			}
			}
		}

		switch (task_epos_state) {
		case TASK_EPOS_STATE_INIT_FAILED:
			/* do nothing, wait for init command */
			epos_sync(&epos);

			break;
		case TASK_EPOS_STATE_READY_TO_ARM:
			/* wait for arm */
			_epos_ctrl(act_id, sp, 0x000E);

			break;
		case TASK_EPOS_STATE_ARMED:
			_epos_ctrl(act_id, sp, 0x000F);

			break;
		default:
			break;
		}

		/* periodically poll status and check for errors */
		if (xTaskGetTickCount() - status_poll_time >= pdMS_TO_TICKS(100)) {
			status_poll_time = xTaskGetTickCount();

			for (uint8_t i = 0; i < epos.track_count; i++) {
				epos_track_t *track = NULL;

				track = &epos.track[i];

				/* get status */
				if (epos_status_get(&epos, i, &track->status))
					continue;

				track->state = track->status & EPOS_DEF_STATE_MASK;

				/* check remote bit in statusword */
				if (!(track->status & (1 << 9))) {
					epos_mode(&epos, i, EPOS_OP_MODE_CSP);
					continue;
				}

				/* check tracked EPOS device state */
				switch (track->state) {
				case EPOS_STATE_SWITCH_ON_DISABLED:
					epos_enter_ready_to_switch_on(&epos, i);

					break;
				case EPOS_STATE_FAULT:
					/* automatically reset fault in case of armed state */
					if (task_epos_state == TASK_EPOS_STATE_ARMED)
						epos_fault_reset(&epos, i);

					break;
				default:
					break;
				}
			}
		}

		/* send actuator data */
		act_data.pos_act1 = epos.track[epos_id1].pos * TASK_EPOS_INC_TO_MM;
		act_data.abs_pos_act1 = 0;
		act_data.vel_act1 = epos.track[epos_id1].vel * TASK_EPOS_MMS_TO_RPM;
		act_data.curr_act1 = epos.track[epos_id1].curr * TASK_EPOS_MA_TO_A;
		act_data.sw_act1 = epos.track[epos_id1].status;
		act_data.abs_enc_sw_act1 = 0;

		xQueueSendToBack(queue_mav_act_data, &act_data, 0);

		/* echo actuator control back */
		xQueueSendToBack(queue_mav_manual_ctrl, &ctrl, 0);
	}
}

#if MAVLINK_OR_CYPHAL

void task_epos_cmd(mavlink_command_long_t *cmd, uint8_t sysid, uint8_t compid)
{
	/* save information of target system id and component id */
	uint8_t old_sysid = cmd->target_system;
	uint8_t old_compid = cmd->target_component;

	cmd->target_system = sysid;
	cmd->target_component = compid;

	/* send command to the back of the queue */
	xQueueSendToBack(queue_epos_cmd, cmd, 0);

	/* revert to old system id and component id */
	cmd->target_system = old_sysid;
	cmd->target_component = old_compid;
}

void task_epos_ctrl(mavlink_lisum_manual_ctrl_hornet_t *ctrl)
{
	/* send control to back of the queue */
	xQueueOverwrite(mailbox_epos_ctrl, ctrl);
}

#else

void task_epos_cmd(messages_cyphal_uavcan_common_CommandLong_1_0 *cmd,
				   uint8_t sysid, uint8_t compid)
{
	/* save information of target system id and component id */
	uint8_t old_sysid = cmd->target_system;
	uint8_t old_compid = cmd->target_component;

	cmd->target_system = sysid;
	cmd->target_component = compid;

	/* send command to the back of the queue */
	xQueueSendToBack(queue_epos_cmd, cmd, 0);

	/* revert to old system id and component id */
	cmd->target_system = old_sysid;
	cmd->target_component = old_compid;
}

void task_epos_ctrl(
	messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0 *ctrl)
{
	/* send control to back of the queue */
	xQueueOverwrite(mailbox_epos_ctrl, ctrl);
}

#endif /* MAVLINK_OR_CYPHAL */

static uint32_t _epos_check(void)
{
	/* iterate over all tracked EPOS devices */
	for (uint8_t i = 0; i < epos.track_count; i++) {
		epos_cfg_t epos_cfg_rx = {0};

		/* check if EPOS configuration is valid */
		if (epos_cfg_get(&epos, i, &epos_cfg_rx))
			return 1;

		/* compare configurations */
		if (memcmp(&epos_cfg_rx, &epos_cfg_data, sizeof(epos_cfg_rx)) != 0)
			return 1;
	}

	return 0;
}

static uint32_t _epos_init(void)
{
	/* iterate over all tracked EPOS devices */
	for (uint8_t i = 0; i < epos.track_count; i++) {
		/* configure EPOS device */
		if (epos_cfg(&epos, i, (epos_cfg_t *)&epos_cfg_data))
			return 1;

		/* configure EPOS device homing parameters */
		if (epos_homing_cfg(&epos, i, (epos_homing_cfg_t *)&homing_cfg))
			return 1;

		/* configure EPOS device CSP parameters */
		if (epos_csp_cfg(&epos, i, (epos_csp_cfg_t *)&csp_cfg))
			return 1;
	}

	return 0;
}

static uint32_t _epos_homing(LISUM_HORNET_HOMING_METHOD method)
{
	switch (method) {
	case LISUM_HORNET_HOMING_METHOD_CURR_THR:
		/* iterate over all tracked EPOS devices */
		for (uint8_t i = 0; i < epos.track_count; i++) {
			/* perform current homing */
			if (epos_homing(&epos, i, EPOS_HOMING_METHOD_CURR_THR_POS_SPEED, 0,
							TASK_EPOS_HOMING_TIMEOUT_MS))
				return 1;

			/* enter CSP mode */
			if (epos_mode(&epos, i, EPOS_OP_MODE_CSP))
				return 1;

			/* save data to EEPROM */
			if (epos_eeprom_save(&epos, i))
				return 1;
		}

		break;
	case LISUM_HORNET_HOMING_METHOD_ACT_POS:
		/* actual position homing is not supported */
		return 1;
		break;
	default:
		return 1;
		break;
	}

	return 0;
}

static uint32_t _epos_fault_reset(void)
{
	/* iterate over all tracked EPOS devices */
	for (uint8_t i = 0; i < epos.track_count; i++) {
		epos_track_t *track;

		/* check if EPOS device is in fault state */
		track = (epos_track_t *)&epos.track[i];
		if (track->state != EPOS_STATE_FAULT)
			continue;

		/* reset faults */
		if (epos_fault_reset(&epos, i))
			return 1;
	}

	return 0;
}

static uint32_t _epos_sweep(void)
{
	/* iterate over all tracked EPOS devices */
	for (uint8_t i = 0; i < epos.track_count; i++) {
		int32_t pos0;
		int32_t pos1;
		uint32_t travel;

		/* go to minimum position */
		if (epos_homing(&epos, i, EPOS_HOMING_METHOD_CURR_THR_POS_SPEED, 0,
						TASK_EPOS_HOMING_TIMEOUT_MS))
			return 1;

		/* wait for 10ms to give time for sensors to acquire current position */
		vTaskDelay(pdMS_TO_TICKS(10));

		/* get minimum absolute position */
		if (epos_sensor_position_get(&epos, i, 2, &pos0))
			return 1;

		/* go to maximum position */
		if (epos_homing(&epos, i, EPOS_HOMING_METHOD_CURR_THR_NEG_SPEED, 0,
						TASK_EPOS_HOMING_TIMEOUT_MS))
			return 1;

		/* wait for 10ms to give time for sensors to acquire current position */
		vTaskDelay(pdMS_TO_TICKS(10));

		/* get maximum absolute position */
		if (epos_sensor_position_get(&epos, i, 2, &pos1))
			return 1;

		/* return to minimum position */
		if (epos_homing(&epos, i, EPOS_HOMING_METHOD_CURR_THR_POS_SPEED, 0,
						TASK_EPOS_HOMING_TIMEOUT_MS))
			return 1;

		/* calculate maximum travel */
		if ((pos1 - pos0) > 0)
			travel = pos1 - pos0;
		else
			travel = pos0 - pos1;

		memset(&eeprom[i], 0x00, sizeof(eeprom[i]));
		eeprom[i].param1 = travel;

		if (epos_custom_eeprom_write(&epos, i, &eeprom[i]))
			return 1;

		/* enter CSP mode */
		if (epos_mode(&epos, i, EPOS_OP_MODE_CSP))
			return 1;

		/* save EPOS data to EEPROM */
		if (epos_eeprom_save(&epos, i))
			return 1;
	}

	return 0;
}

static uint32_t _epos_custom_eeprom_get(void)
{
	/* iterate over all tracked EPOS devices */
	for (uint8_t i = 0; i < epos.track_count; i++) {
		/* read custom EEPROM of EPOS device */
		if (epos_custom_eeprom_read(&epos, i, eeprom + i))
			return 1;
	}

	return 0;
}

static void _epos_ctrl(uint8_t act_id, int32_t *setpoints, uint16_t cw)
{
	/* send CSP control */
	epos_csp_control(&epos, act_id, cw, *(setpoints + act_id));

	epos_sync(&epos);
}

static uint32_t _work_init(void)
{
	uint32_t ret = 0;
	uint32_t retry = 0;

	/* try to initialize tracked EPOS devices */
	retry = 0;
	do {
		ret = _epos_init();
	} while (ret && retry++ < TASK_EPOS_INIT_MAX_RETRY);

	if (retry >= TASK_EPOS_INIT_MAX_RETRY)
		return 1;

	/* try reading custom EEPROM of all tracked EPOS devices */
	retry = 0;
	do {
		ret = _epos_custom_eeprom_get();
	} while (ret && retry++ < TASK_EPOS_BOOT_MAX_RETRY);

	if (retry >= TASK_EPOS_BOOT_MAX_RETRY)
		return 1;

	return 0;
}

static uint32_t _work_homing(LISUM_HORNET_HOMING_METHOD method)
{
	uint32_t ret = 0;
	uint32_t retry = 0;

	/* try checking all tracked EPOS devices */
	retry = 0;
	do {
		ret = _epos_check();
	} while (ret && retry++ < TASK_EPOS_BOOT_MAX_RETRY);

	if (retry >= TASK_EPOS_BOOT_MAX_RETRY)
		return 1;

	/* try reading custom EEPROM of all tracked EPOS devices */
	retry = 0;
	do {
		ret = _epos_custom_eeprom_get();
	} while (ret && retry++ < TASK_EPOS_BOOT_MAX_RETRY);

	if (retry >= TASK_EPOS_BOOT_MAX_RETRY)
		return 1;

	/* try homing of all tracked EPOS devices */
	retry = 0;
	do {
		ret = _epos_homing(method);
	} while (ret && retry++ < TASK_EPOS_HOMING_MAX_RETRY);

	if (retry >= TASK_EPOS_HOMING_MAX_RETRY)
		return 1;

	return 0;
}

static uint32_t _work_fault_reset(void)
{
	uint32_t ret = 0;
	uint32_t retry = 0;

	/* try to reset faults of all tracked EPOS devices */
	retry = 0;
	do {
		ret = _epos_fault_reset();
	} while (ret && retry++ < TASK_EPOS_FAULT_RESET_MAX_RETRY);

	if (retry >= TASK_EPOS_FAULT_RESET_MAX_RETRY)
		return 1;

	return 0;
}

static uint32_t _work_sweep(void)
{
	uint32_t ret = 0;
	uint32_t retry = 0;

	/* try to reset faults of all tracked EPOS devices */
	retry = 0;
	do {
		ret = _epos_sweep();
	} while (ret && retry++ < TASK_EPOS_SWEEP_MAX_RETRY);

	if (retry >= TASK_EPOS_SWEEP_MAX_RETRY)
		return 1;

	return 0;
}

static uint32_t _work_arm_disarm(uint8_t arm)
{
	if (arm) {
		/* check if task is in valid state */
		if (task_epos_state != TASK_EPOS_STATE_READY_TO_ARM)
			return 1;

		task_epos_state = TASK_EPOS_STATE_ARMED;
	} else {
		/* check if task is in valid state */
		if (task_epos_state != TASK_EPOS_STATE_ARMED)
			return 1;

		task_epos_state = TASK_EPOS_STATE_READY_TO_ARM;
	}

	return 0;
}

static void _process_request(mavlink_command_long_t *cmd)
{
	uint32_t ret = 0;

	switch (cmd->command) {
	case MAV_CMD_HORNET_HOMING:
		ret = _work_homing(cmd->param1);
		if (ret) // go to init failed state
			task_epos_state = TASK_EPOS_STATE_INIT_FAILED;
		else // go to ready to arm state
			task_epos_state = TASK_EPOS_STATE_READY_TO_ARM;

		break;
	case MAV_CMD_HORNET_ARM_DISARM:
		ret = _work_arm_disarm(cmd->param1);

		break;
	case MAV_CMD_HORNET_INIT:
		ret = _work_init();
		if (ret) // go to init failed state
			task_epos_state = TASK_EPOS_STATE_INIT_FAILED;
		else // go to ready to arm state
			task_epos_state = TASK_EPOS_STATE_READY_TO_ARM;

		break;
	case MAV_CMD_HORNET_FAULT_RESET:
		ret = _work_fault_reset();

		break;
	case MAV_CMD_HORNET_SWEEP:
		ret = _work_sweep();
		if (ret) // go to init failed state
			task_epos_state = TASK_EPOS_STATE_INIT_FAILED;
		else // go to ready to arm state
			task_epos_state = TASK_EPOS_STATE_READY_TO_ARM;

		break;
	default:
		ret = -1; // unsupported command
		break;
	}

#if MAVLINK_OR_CYPHAL

	mavlink_command_ack_t ack = {0};

#else
	messages_cyphal_uavcan_common_CommandAck_1_0 ack = {0};

#endif /* MAVLINK_OR_CYPHAL */

	switch (ret) {
	case -1: // unsupported command
		ack.command = cmd->command;
		ack.result = MAV_RESULT_UNSUPPORTED;
		ack.progress = (uint8_t)-1;
		ack.result_param2 = 0;
		ack.target_system = cmd->target_system;
		ack.target_component = cmd->target_component;

		break;
	case 0: // command success
		ack.command = cmd->command;
		ack.result = MAV_RESULT_ACCEPTED;
		ack.progress = (uint8_t)-1;
		ack.result_param2 = 0;
		ack.target_system = cmd->target_system;
		ack.target_component = cmd->target_component;

		break;
	default: // command failed
		ack.command = cmd->command;
		ack.result = MAV_RESULT_FAILED;
		ack.progress = (uint8_t)-1;
		ack.result_param2 = 0;
		ack.target_system = cmd->target_system;
		ack.target_component = cmd->target_component;

		break;
	}

	xQueueSendToBack(queue_mav_ack, &ack, 0);
}
