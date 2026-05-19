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
#include "gpio.h"

/* External hardware drivers */
#include "epos.h"
#include "epos_csp.h"
#include "epos_eeprom.h"
#include "epos_homing.h"
#include "epos_od.h"
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
extern QueueHandle_t queue_status_text_report;

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
task_epos_state_e prev_task_epos_state = TASK_EPOS_STATE_INIT;

static canopen_handle_t canopen;
static epos_t epos;

static epos_custom_eeprom_t eeprom[4];

/** @note to be removed */
static int32_t sp[4] = {0};

static const uint32_t error_sectors[] = {
	ACTUATOR_ERROR_SECTOR_0, ACTUATOR_ERROR_SECTOR_1, ACTUATOR_ERROR_SECTOR_2,
	ACTUATOR_ERROR_SECTOR_3, ACTUATOR_ERROR_SECTOR_4, ACTUATOR_ERROR_SECTOR_5,
	ACTUATOR_ERROR_SECTOR_6,
};

static uint8_t current_sector = 0;
static uint8_t current_slot = 0;

static actuator_error_t current_error = {0};

static uint8_t current_page_buffer[256] __attribute__((aligned(4)));
static uint8_t error_header_page[256] __attribute__((aligned(4)));

static uint8_t boot_error_flagged = 0;
static uint8_t unreported_errors_at_boot = 0;

static uint32_t new_error_occures = 0;

static uint8_t lisum_manual_ctrl_hornet_buf
	[messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
static size_t lisum_manual_ctrl_hornet_sz =
	sizeof(lisum_manual_ctrl_hornet_buf);

static uint8_t global_boot_cnt = 0;

static uint8_t status_send_cnt = 0;

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

static void _epos_ctrl2(uint8_t act_id, int32_t *setpoints, uint16_t cw);

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

/* initialize actuator error logging */
static void actuator_error_init(void);

/* log actuator error to flash memory */
static void actuator_error_log(actuator_error_t *err, epos_track_t *track,
							   uint8_t boot_cnt);

/* log actuator error from EPOS to flash memory */
static void actuator_error_log_from_epos(actuator_error_t *err,
										 epos_track_t *track, uint8_t boot_cnt);

/* convert EPOS error code to string */
static const char *_epos_error_to_str(uint32_t error_code);

static const char *_epos_task_state_to_str(uint8_t task_epos_state);

static void _epos_statusword_to_str(uint16_t sw, char *buf, size_t buf_size);

static void load_current_page(void);

static void error_header_find(uint32_t *out_write, uint32_t *out_reported,
							  uint32_t *out_idx);

static void error_header_mark_written(void);

static void error_header_mark_reported(void);

static void check_if_error_occurred_in_previous_boot(void);

static uint32_t error_header_next_flag(uint32_t current);

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
	uint8_t epos_id1;
	uint8_t epos_id2 = 1;
	uint8_t epos_id3 = 2;
	uint8_t epos_id4 = 3;

	static uint32_t last_remote_log_time = 0;

	/*------------------------------- CANOpen --------------------------------*/
	canopen_init(&canopen, CAN_HAL_INSTANCE_0, ACT_MASTER_NODE_0_ID);

	/*-------------------------------- EPOS ----------------------------------*/
	epos_init(&epos, &canopen);
	epos_id1 = epos_track(&epos, ACT2_NODE_ID);

	// Initialization of actuator error logging in flash memory
	actuator_error_init();

	check_if_error_occurred_in_previous_boot();

	// Delay to ensure gnd and sky controllers are up and running before sending
	// status text report
	vTaskDelay(pdMS_TO_TICKS(5000));

	if (unreported_errors_at_boot) {
		messages_cyphal_uavcan_common_Statustext_1_0 statustext;
		statustext.severity = MAV_SEVERITY_ERROR;
		statustext.id = 0;
		statustext.chunk_seq = 0;
		snprintf((char *)statustext.text, sizeof(statustext.text),
				 "[EPOS ERROR] Error occures in last boot");

		xQueueSendToBack(queue_status_text_report, &statustext, 0);

		error_header_mark_reported();
	}

	if (_epos_check() || _epos_custom_eeprom_get()) {
		actuator_error_log(&current_error, &epos.track[0], global_boot_cnt);
		task_epos_state = TASK_EPOS_STATE_INIT_FAILED;
	} else {
		task_epos_state = TASK_EPOS_STATE_READY_TO_ARM;
	}

	for (;;) {
		/* process commands */
		if (pdPASS == xQueueReceive(queue_epos_cmd, &cmd, 0)) {
			_process_request(&cmd);
		}

		/* process setpoints */
		while (xQueueReceive(mailbox_epos_ctrl, &ctrl, 0) == pdPASS) {
			int32_t new_sp = (int32_t)TASK_EPOS_SP_CONV(ctrl.pos_sp_act);

			switch (ctrl.act_id) {
			case 0:
				sp[0] = new_sp;
				break;
			case 1:
				sp[1] = new_sp;
				break;
			case 2:
				sp[2] = new_sp;
				break;
			case 3:
				sp[3] = new_sp;
				break;
			default:
				break;
			}
		}

		switch (task_epos_state) {
		case TASK_EPOS_STATE_INIT_FAILED:

			if (prev_task_epos_state != TASK_EPOS_STATE_INIT_FAILED) {
				actuator_error_log(&current_error, &epos.track[0],
								   global_boot_cnt);
			}
			prev_task_epos_state = task_epos_state;

			/* do nothing, wait for init command */
			epos_sync(&epos);

			break;
		case TASK_EPOS_STATE_READY_TO_ARM:
			/* wait for arm */
			_epos_ctrl2(ctrl.act_id, sp, 0x000E);

			break;
		case TASK_EPOS_STATE_ARMED:
			_epos_ctrl(ctrl.act_id, sp, 0x000F);

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
					// Log just once in a second
					if (xTaskGetTickCount() - last_remote_log_time >
						pdMS_TO_TICKS(1000)) {
						last_remote_log_time = xTaskGetTickCount();
						actuator_error_log(&current_error, track,
										   global_boot_cnt);
					}
					epos_mode(&epos, i, EPOS_OP_MODE_CSP);
					continue;
				}

				/* check tracked EPOS device state */
				switch (track->state) {
				case EPOS_STATE_SWITCH_ON_DISABLED:
					if (prev_task_epos_state != EPOS_STATE_SWITCH_ON_DISABLED) {
						actuator_error_log(&current_error, track,
										   global_boot_cnt);
						prev_task_epos_state = EPOS_STATE_SWITCH_ON_DISABLED;
					}

					epos_enter_ready_to_switch_on(&epos, i);

					break;
				case EPOS_STATE_FAULT:
					if (prev_task_epos_state != EPOS_STATE_FAULT) {
						prev_task_epos_state = EPOS_STATE_FAULT;
						actuator_error_log(&current_error, track,
										   global_boot_cnt);
					}
					/* automatically reset fault in case of armed state */
					if (task_epos_state == TASK_EPOS_STATE_ARMED)
						epos_fault_reset(&epos, i);

					break;
				default:
					break;
				}
			}
		}

		if (status_send_cnt) {
			status_send_cnt = 0;

			/* send actuator data */
			for (uint8_t i = 0; i < 4; i++) { // hardcoded for 4 actuators
				act_data.act_id = i;
				act_data.pos_act = epos.track[i].pos * TASK_EPOS_INC_TO_MM;
				act_data.abs_pos_act = 0;
				act_data.vel_act = epos.track[i].vel * TASK_EPOS_MMS_TO_RPM;
				act_data.curr_act = epos.track[i].curr * TASK_EPOS_MA_TO_A;
				act_data.sw_act = epos.track[i].status;
				act_data.abs_enc_sw_act = 0;
				xQueueSendToBack(queue_mav_act_data, &act_data, 0);
			}
		} else {
			status_send_cnt++;
		}

		/* echo actuator control back */
		xQueueSendToBack(queue_mav_manual_ctrl, &ctrl, 0);

		vTaskDelay(pdMS_TO_TICKS(CSP_INTERPOLATION_TIME_PERIOD));
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
	xQueueSend(mailbox_epos_ctrl, ctrl, 0);
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

static void _epos_ctrl2(uint8_t act_id, int32_t *setpoints, uint16_t cw)
{
	/* iterate over all tracked EPOS devices */
	// for (uint8_t i = 0; i < epos.track_count; i++) {
	/* send CSP control */
	// epos_csp_control(&epos, act_id, cw, *(setpoints + act_id));
	// TODO set for other epos devices!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	epos_csp_control(&epos, 0, cw, *(setpoints + act_id));
	// }

	epos_sync(&epos);
}

static void _epos_ctrl(uint8_t act_id, int32_t *setpoints, uint16_t cw)
{
	/* iterate over all tracked EPOS devices */
	// for (uint8_t i = 0; i < epos.track_count; i++) {
	/* send CSP control */
	// epos_csp_control(&epos, act_id, cw, *(setpoints + act_id));
	// TODO set for other epos devices!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// if (act_id == 1) {
	epos_csp_control(&epos, 0, cw, *(setpoints + 0));
	// }

	epos_sync(&epos);
	// }

	for (uint8_t i = 0; i < ACTUATOR_NUMBER_TRACKING; i++) {
		HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_0, i + 4, 0);
	}
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
		prev_task_epos_state = task_epos_state;
	} else {
		/* check if task is in valid state */
		if (task_epos_state != TASK_EPOS_STATE_ARMED)
			return 1;

		task_epos_state = TASK_EPOS_STATE_READY_TO_ARM;
		prev_task_epos_state = task_epos_state;
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

		prev_task_epos_state = task_epos_state;

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

		prev_task_epos_state = task_epos_state;

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

		prev_task_epos_state = task_epos_state;

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
		actuator_error_log(&current_error, &epos.track[0], global_boot_cnt);
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

		actuator_error_log(&current_error, &epos.track[0], global_boot_cnt);
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

static void actuator_error_init(void)
{
	for (uint8_t s = 0; s < ERROR_SECTOR_COUNT; s++) {
		for (uint16_t slot = 0; slot < ERROR_SLOTS_PER_SECTOR; slot++) {

			uint32_t addr = error_sectors[s] + slot * ERROR_SLOT_SIZE;
			uint32_t stored_num = *(volatile uint32_t *)(addr);

			if (stored_num == 0xFFFFFFFF) {
				global_boot_cnt =
					*(volatile uint16_t *)(addr - 1); // get boot count

				global_boot_cnt++; // increment boot count for the new error log
				current_sector = s;
				current_slot = slot;
				load_current_page();
				return;
			}
		}
	}

	// Sve popunjeno - briši sektor 0, kreni iznova
	current_sector = 0;
	current_slot = 0;

	global_boot_cnt =
		*(volatile uint16_t *)(IAP_HAL_SECTOR_16_ADDR - 1); // get boot count

	global_boot_cnt++;

	__disable_irq();
	iap_hal_sector_num_t sec = HAL_IAP_GetSectorNumber(error_sectors[0]);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_EraseSector(sec, sec);
	__enable_irq();

	load_current_page();
}

static void actuator_error_log(actuator_error_t *err, epos_track_t *track,
							   uint8_t boot_cnt)
{
	actuator_error_log_from_epos(err, track, boot_cnt);

	uint8_t page_slot = current_slot % ERROR_SLOTS_PER_PAGE;
	uint8_t page_idx = current_slot / ERROR_SLOTS_PER_PAGE;
	uint32_t page_addr = error_sectors[current_sector] + page_idx * 256;

	memcpy(current_page_buffer + page_slot * sizeof(actuator_error_t), err,
		   sizeof(actuator_error_t));

	__disable_irq();
	iap_hal_sector_num_t sec =
		HAL_IAP_GetSectorNumber(error_sectors[current_sector]);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_CopyRAM2Flash((uint8_t *)page_addr, current_page_buffer,
						  IAP_HAL_WRITE_256);
	__enable_irq();

	current_slot++;

	if (current_slot >= ERROR_SLOTS_PER_SECTOR) {
		current_slot = 0;
		current_sector = (current_sector + 1) % ERROR_SECTOR_COUNT;

		__disable_irq();
		sec = HAL_IAP_GetSectorNumber(error_sectors[current_sector]);
		HAL_IAP_PrepareSector(sec, sec);
		HAL_IAP_EraseSector(sec, sec);
		__enable_irq();
	}

	load_current_page();

	if (new_error_occures == 0) {
		error_header_mark_written();
		new_error_occures++;
	} else {
		new_error_occures++;
	}
}

static void actuator_error_log_from_epos(actuator_error_t *err,
										 epos_track_t *track, uint8_t boot_cnt)
{
	uint8_t epos_idx = (uint8_t)(track - epos.track);

	// num_error = fiksna pozicija u flash-u, ne brojač
	err->num_error =
		(uint16_t)(current_sector * ERROR_SLOTS_PER_SECTOR + current_slot);
	err->statusword = track->status;
	err->task_state = (uint8_t)task_epos_state;
	err->uc_boot_cnt = boot_cnt;

	if (epos_idx < epos.track_count) {
		uint32_t full_error_code = 0;
		if (epos_obj_read(&epos, epos_idx, &ObjErrorHistory1,
						  &full_error_code) != 0) {
			err->error_code = 0xFFFF;
		} else {
			err->error_code = (uint16_t)(full_error_code & 0xFFFF);
		}
	}
}

static void load_current_page(void)
{
	uint8_t page_idx = current_slot / ERROR_SLOTS_PER_PAGE;
	uint32_t page_addr = error_sectors[current_sector] + page_idx * 256;
	memcpy(current_page_buffer, (const uint8_t *)page_addr, 256);
}

/* find current active header slot */
static void error_header_find(uint32_t *out_write, uint32_t *out_reported,
							  uint32_t *out_idx)
{
	for (uint32_t i = 0; i < ERROR_HEADER_SLOTS_MAX; i++) {
		uint32_t addr = ERROR_HEADER_ADDR + i * ERROR_HEADER_SLOT_SIZE;
		uint32_t w = *(volatile uint32_t *)(addr);
		uint32_t r = *(volatile uint32_t *)(addr + 4);

		if (w == ERROR_HEADER_FLAG_FRESH && r == ERROR_HEADER_FLAG_FRESH) {
			/* prvi slobodan slot - prethodni je aktivan */
			if (i == 0) {
				/* sektor potpuno prazan, nikad nije bilo errora */
				*out_write = ERROR_HEADER_FLAG_FRESH;
				*out_reported = ERROR_HEADER_FLAG_FRESH;
				*out_idx = 0;
			} else {
				uint32_t prev =
					ERROR_HEADER_ADDR + (i - 1) * ERROR_HEADER_SLOT_SIZE;
				*out_write = *(volatile uint32_t *)(prev);
				*out_reported = *(volatile uint32_t *)(prev + 4);
				*out_idx = i - 1;
			}
			return;
		}

		/* oba 0x00 - slot iskorišćen do kraja, nastavi */
		if (w == 0x00000000 && r == 0x00000000)
			continue;

		/* aktivan slot */
		*out_write = w;
		*out_reported = r;
		*out_idx = i;
		return;
	}

	/* sektor pun */
	*out_write = 0x00000000;
	*out_reported = 0x00000000;
	*out_idx = ERROR_HEADER_SLOTS_MAX;
}

/* spusti jedan bit u write flag - samo jednom po bootu */
static void error_header_mark_written(void)
{
	if (boot_error_flagged)
		return;

	uint32_t w, r, idx;
	error_header_find(&w, &r, &idx);

	uint32_t write_addr;

	if (idx >= ERROR_HEADER_SLOTS_MAX) {
		/* sektor pun -> erase */
		__disable_irq();
		iap_hal_sector_num_t sec = HAL_IAP_GetSectorNumber(ERROR_HEADER_ADDR);

		HAL_IAP_PrepareSector(sec, sec);
		HAL_IAP_EraseSector(sec, sec);
		__enable_irq();

		idx = 0;
		w = ERROR_HEADER_FLAG_FRESH; /* 0xFFFFFFFF */
	}

	/* trenutni slot */
	write_addr = ERROR_HEADER_ADDR + (idx * ERROR_HEADER_SLOT_SIZE);

	/* ako je stigao do 0 -> idi na sledeci slot */
	if (w == 0x00000000) {

		idx++;

		if (idx >= ERROR_HEADER_SLOTS_MAX) {

			/* erase i kreni ispocetka */
			__disable_irq();
			iap_hal_sector_num_t sec =
				HAL_IAP_GetSectorNumber(ERROR_HEADER_ADDR);

			HAL_IAP_PrepareSector(sec, sec);
			HAL_IAP_EraseSector(sec, sec);
			__enable_irq();

			idx = 0;
		}

		write_addr = ERROR_HEADER_ADDR + (idx * ERROR_HEADER_SLOT_SIZE);

		w = ERROR_HEADER_FLAG_FRESH;
	}

	/* spusti jedan bit */
	uint32_t new_w = error_header_next_flag(w);

	uint32_t page_addr = write_addr & ~(0xFF);
	memcpy(error_header_page, (const uint8_t *)page_addr, 256);

	uint32_t offset_in_page = write_addr - page_addr;

	memcpy(error_header_page + offset_in_page, &new_w, sizeof(uint32_t));

	__disable_irq();

	iap_hal_sector_num_t sec = HAL_IAP_GetSectorNumber(ERROR_HEADER_ADDR);

	HAL_IAP_PrepareSector(sec, sec);

	HAL_IAP_CopyRAM2Flash((uint8_t *)page_addr, error_header_page,
						  IAP_HAL_WRITE_256);

	__enable_irq();

	boot_error_flagged = 1;
}

static void error_header_mark_reported(void)
{
	uint32_t w, r, idx;
	error_header_find(&w, &r, &idx);

	if (w == ERROR_HEADER_FLAG_FRESH || w == r)
		return;

	/* reported = w, spuštamo iste bite kao w, bez erase! */
	uint32_t reported_addr =
		ERROR_HEADER_ADDR + idx * ERROR_HEADER_SLOT_SIZE + 4;

	uint32_t page_addr = reported_addr & ~(0xFF);
	memcpy(error_header_page, (const uint8_t *)page_addr, 256);

	uint32_t offset_in_page = reported_addr - page_addr;
	memcpy(error_header_page + offset_in_page, &w, sizeof(uint32_t));

	__disable_irq();
	iap_hal_sector_num_t sec = HAL_IAP_GetSectorNumber(ERROR_HEADER_ADDR);
	HAL_IAP_PrepareSector(sec, sec);
	HAL_IAP_CopyRAM2Flash((uint8_t *)page_addr, error_header_page,
						  IAP_HAL_WRITE_256);
	__enable_irq();
}

static void check_if_error_occurred_in_previous_boot(void)
{
	uint32_t w, r, idx;
	error_header_find(&w, &r, &idx);

	/* write spušten a reported nije → prošli boot imao errora */
	if (w != ERROR_HEADER_FLAG_FRESH && w != r)
		unreported_errors_at_boot = 1;
}

static uint32_t error_header_next_flag(uint32_t current)
{
	/* traži prvi bit koji je još 1, spusti ga */
	for (int i = 0; i < 32; i++) {
		if (current & (1u << i)) {
			return current & ~(1u << i);
		}
	}
	return 0x00000000; /* svi biti spušteni */
}