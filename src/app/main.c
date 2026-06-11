/**
 * @file    main.c
 * @brief   Actuator master project.
 * @details This project manages EPOS4 actuator drivers. Actuator configuration
 *          and data collection is done via CAN using CANopen. Acquired data is
 *          packed into MAVLink messages and transmitted via UDP to GW_SKY. This
 *          project also offers actuator initialization, homing, sweep,
 *          arming/disarming and fault reset via MAVLink commands.
 * @version 1.0.0
 * @date    26.02.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "act_config.h"
#include "servo.h"
#include "task_epos.h"
#include "task_mav.h"
#include "task_tx_can.h"
#include "task_work.h"
#include "types.h"

/* Peripherals */
#include "can.h"
#include "common.h"
#include "eth.h"
#include "gpio.h"
#include "lpc17xx_clkpwr.h"
#include "util.h"

/* External hardware drivers */

/* Lib */

/* Middleware */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "lpc17xx_uart.h"
#include "uart.h"
#include "udp.h"
#include "udp_tl.h"

#include "mav.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define ACTUATOR_CONTROL                                                       \
	1 // 0 - Control via lisumManualCtrlHornet, 1 - via PWM capture
/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef struct {
	uint32_t t_rise;
	uint32_t t_period_rise;
	uint32_t pulse_us;
	uint32_t period_us;
	uint8_t percent;
	bool valid;
	bool rise_valid;
} capture_ch_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Temporary variables */
uint64_t boot_time_ms = 0;

#if MAVLINK_OR_CYPHAL

/* Device information */
// UDP
const uint8_t dev_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x04};
const uint8_t dev_ip[4] = {192, 168, 1, 104};

// UDP Client information
const uint8_t client_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x00};
const uint8_t client_ip[4] = {192, 168, 1, 100};
const uint16_t client_port = 1004;
const uint16_t client_local_port = 1004;

/* Transport layers */
eth_hal_phy_t eth_phy;
udp_tl_t mav_udp_tl;

/* UDP */
udp_t udp;

#endif /* MAVLINK_OR_CYPHAL */

/* FreeRTOS */
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
volatile uint32_t ulIdleCycleCount = 0UL;

// Timers
TimerHandle_t timer_blinky;

// Mutex
SemaphoreHandle_t tx_queue_mutex;
SemaphoreHandle_t mutex_mav;

// Mailboxes
QueueHandle_t mailbox_epos_ctrl;
// Queues
QueueHandle_t queue_mav_hb;
QueueHandle_t queue_mav_ack;
QueueHandle_t queue_mav_act_data;
QueueHandle_t queue_mav_manual_ctrl;
QueueHandle_t queue_epos_hb;
QueueHandle_t queue_epos_cmd;
QueueHandle_t queue_mav_ftp;
QueueHandle_t queue_command_long;
QueueHandle_t queue_cyphal_rx;
QueueHandle_t queue_status_text_report;

// QueueSets
QueueSetHandle_t queueset_mav;

TaskHandle_t task_tx_can_handle;
TaskHandle_t task_epos_handle;

flash_cfg_rec_t g_act_cfg;
actuator_modes_e act_mode = ACT_MODE_CYPHAL;

volatile node_mode_state_t current_node_mode = NODE_MODE_INITIALIZATION;

/* Servo variables */
servo_t g_servo_0 = {
	.gpio_port = SERVO_0_PORT,
	.gpio_pin = SERVO_0_PIN,
	.match_channel = SERVO_0_MATCH_CHANNEL,
	.position =
		{
			.pulse_width_min_us = SERVO_MIN_PULSE_WIDTH_US,
			.pulse_width_max_us = SERVO_MAX_PULSE_WIDTH_US,
			.pulse_width_us = SERVO_MIN_PULSE_WIDTH_US,
		},
};

servo_t g_servo_1 = {
	.gpio_port = SERVO_1_PORT,
	.gpio_pin = SERVO_1_PIN,
	.match_channel = SERVO_1_MATCH_CHANNEL,
	.position =
		{
			.pulse_width_min_us = SERVO_MIN_PULSE_WIDTH_US,
			.pulse_width_max_us = SERVO_MAX_PULSE_WIDTH_US,
			.pulse_width_us = SERVO_MIN_PULSE_WIDTH_US,
		},
};

servo_t g_servo_2 = {
	.gpio_port = SERVO_2_PORT,
	.gpio_pin = SERVO_2_PIN,
	.match_channel = SERVO_2_MATCH_CHANNEL,
	.position =
		{
			.pulse_width_min_us = SERVO_ONESHOT125_MIN_US,
			.pulse_width_max_us = SERVO_ONESHOT125_MAX_US,
			.pulse_width_us = SERVO_ONESHOT125_MIN_US,
		},
};

servo_group_t g_servo_group_0 = {0};
servo_group_t g_servo_group_1 = {0};

static uint32_t t_rise = 0;
static uint32_t pulse_ticks = 0;

static capture_ch_t g_capture = {0};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void timer_blinky_cb(TimerHandle_t xTimer);

void cyphal_cb(void *usr_arg, uint32_t int_status);

// For testing the latency in system
void timer0_init(void)
{
	LPC_SC->PCONP |= (1 << 1); // Power TIMER0

	LPC_SC->PCLKSEL0 &= ~(3 << 2);
	LPC_SC->PCLKSEL0 |= (1 << 2);
	// PCLK_TIMER0 = CCLK

	LPC_TIM0->TCR = 0x02; // reset

	LPC_TIM0->PR = (SystemCoreClock / 1000000) - 1;
	// 1 us tick

	LPC_TIM0->TC = 0;

	LPC_TIM0->TCR = 0x01; // enable
}

servo_return_value_t Servo_Init(void);

void capture_callback(tim_hal_ch_t ch, tim_hal_int_type_t type);
/*******************************************************************************
 * Code
 ******************************************************************************/
int main()
{

	// See how the actuator should work
	flash_cfg_load(&g_act_cfg);
	switch (g_act_cfg.mode) {
	case ACT_MODE_CYPHAL: {
		act_mode = ACT_MODE_CYPHAL;
		break;
	}
	case ACT_MODE_RC: {
		act_mode = ACT_MODE_RC;
		break;
	}
	case ACT_MODE_ONESHOOT125: {
		act_mode = ACT_MODE_ONESHOOT125;
		break;
	}
	case ACT_MODE_ONESHOOT42: {
		act_mode = ACT_MODE_ONESHOOT42;
		break;
	}
	case ACT_MODE_DSHOOT: {
		act_mode = ACT_MODE_DSHOOT;
		break;
	}
	default: {
		act_mode = ACT_MODE_CYPHAL;
		break;
	}
	}
	// flash_cfg_erase_debug();
	/*------------------------------- FreeRTOS -------------------------------*/

	/* Mutexes */
	tx_queue_mutex = xSemaphoreCreateMutex();

	/* Mutexes */
	mutex_mav = xSemaphoreCreateMutex();

#if MAVLINK_OR_CYPHAL

	queue_mav_hb = xQueueCreate(2, sizeof(mavlink_heartbeat_t));
	/* Mailboxes */
	mailbox_epos_ctrl =
		xQueueCreate(1, sizeof(mavlink_lisum_manual_ctrl_hornet_t));
	queue_mav_ack = xQueueCreate(4, sizeof(mavlink_command_ack_t));
	queue_mav_act_data =
		xQueueCreate(5, sizeof(mavlink_lisum_power_hornet_act_data_t));
	queue_mav_manual_ctrl =
		xQueueCreate(5, sizeof(mavlink_lisum_manual_ctrl_hornet_t));

	queue_epos_cmd = xQueueCreate(4, sizeof(mavlink_command_long_t));

#else

	queue_mav_hb = xQueueCreate(2, sizeof(mavlink_heartbeat_t));
	/* Mailboxes */
	mailbox_epos_ctrl = xQueueCreate(
		20, sizeof(messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0));
	queue_mav_ack =
		xQueueCreate(4, sizeof(messages_cyphal_uavcan_common_CommandAck_1_0));
	queue_mav_act_data = xQueueCreate(
		20, sizeof(messages_cyphal_uavcan_lisum_LisumPowerHornetActData_1_0));
	queue_mav_manual_ctrl = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0));

	queue_mav_ftp = xQueueCreate(5, sizeof(mavlink_file_transfer_protocol_t));

	queue_command_long = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_common_ComponentInformationBasic_1_0));

	queue_epos_cmd =
		xQueueCreate(4, sizeof(messages_cyphal_uavcan_common_CommandLong_1_0));

	queue_status_text_report =
		xQueueCreate(4, sizeof(messages_cyphal_uavcan_common_Statustext_1_0));

	queue_cyphal_rx = xQueueCreate(10, sizeof(can_hal_msg_t));

#endif /* MAVLINK_OR_CYPHAL */

	/*--------------------------------- NVIC ---------------------------------*/
	HAL_NVIC_SetPriority(SysTick_IRQn, 31);
	HAL_NVIC_SetPriority(PendSV_IRQn, 30);
	HAL_NVIC_EnableIRQ(TIMER0_IRQn);

	/*------------------------------- Dev Time -------------------------------*/
	HAL_DevTimeInit(TIM_HAL_INSTANCE_0);

	/*--------------------------------- GPIO ---------------------------------*/
	gpio_hal_cfg_type_t gpio_cfg;
	gpio_cfg.openDrain = GPIO_HAL_OPENDRAIN_NORMAL;
	gpio_cfg.pinMode = GPIO_HAL_PINMODE_PULLUP;
	gpio_cfg.pinDir = GPIO_HAL_OUTPUT;

	HAL_GPIO_Init(GPIO_HAL_INSTANCE_3, 25, &gpio_cfg);
	HAL_GPIO_Init(GPIO_HAL_INSTANCE_3, 26, &gpio_cfg);

	/*--------------------------------- CAN ----------------------------------*/

	HAL_CAN_Init(CAN_HAL_INSTANCE_0, 1000000);

	/* --------------------------------- CAN ------------------------------ */
	HAL_CAN_Init(CAN_HAL_INSTANCE_1, 1000000);

	HAL_CAN_EnableInterrupt(CAN_HAL_INSTANCE_1, cyphal_cb, NULL);

	HAL_CAN_IntCmd(CAN_HAL_INSTANCE_1, CAN_HAL_INT_RI, lFunctionalState_Enable);

	gpio_hal_cfg_type_t gpio_can_cfg;
	gpio_can_cfg.openDrain = GPIO_HAL_OPENDRAIN_NORMAL;
	gpio_can_cfg.pinMode = GPIO_HAL_PINMODE_PULLUP;
	gpio_can_cfg.pinDir = GPIO_HAL_OUTPUT;

	HAL_GPIO_Init(GPIO_HAL_INSTANCE_2, 13, &gpio_can_cfg); // P2.13 CAN1_RD
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_2, 13, 0);	   // P2.13 LOW

	// Init gpio for testing the spead of writing data in flash and erasing
	gpio_hal_cfg_type_t gpio_test_cfg;
	gpio_test_cfg.openDrain = GPIO_HAL_OPENDRAIN_NORMAL;
	gpio_test_cfg.pinMode = GPIO_HAL_PINMODE_PULLDOWN;
	gpio_test_cfg.pinDir = GPIO_HAL_OUTPUT;

	// /*------------------------------  Servo -------------------------------*/

	if (act_mode != ACT_MODE_CYPHAL) {

		LPC_PINCON->PINSEL3 &= ~(3U << 24);
		LPC_PINCON->PINSEL3 |= (3U << 24);

		// Init capture
		tim_hal_capture_cfg_t cap_cfg = {
			.captureChannel = TIM_HAL_CH_1, // CAP0.1 = P1.28
			.risingEdge = lFunctionalState_Enable,
			.fallingEdge = lFunctionalState_Enable,
			.intOnCaption = lFunctionalState_Enable,
		};

		// Timer bez prescalera — max rezolucija
		tim_hal_cfg_t timer_cfg = {
			.Prescale = TIM_HAL_PRESCALE_TICKS,
			.PrescaleValue = 1,
		};

		HAL_TIM_Init(TIM_HAL_INSTANCE_1, &timer_cfg);

		CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER1, CLKPWR_PCLKSEL_CCLK_DIV_1);
		LPC_TIM1->PR = 0; // PR=0 -> TC++ svaki takt = 10 ns @ 100 MHz
		HAL_TIM_ConfigCapture(TIM_HAL_INSTANCE_1, &cap_cfg);
		HAL_TIM_EnableInterrupt(TIM_HAL_INSTANCE_1, capture_callback);
	}

#if !ACTUATOR_CONTROL

	/*--------------------------------- ETH ----------------------------------*/
	HAL_ETH_Init(&eth_phy, dev_mac);

	/*--------------------------------- UDP ----------------------------------*/
	/* initialize UDP */
	udp_init(&udp, dev_mac, dev_ip, (udp_phy_t *)&eth_phy);

	/* add client MAC & IP to static ARP table */
	udp_arptab_add(&udp, client_mac, client_ip);

	/*-------------------------------- UDP TL --------------------------------*/
	udp_tl_init(&mav_udp_tl, &udp, (uint8_t *)client_ip, client_local_port,
				client_port);

#endif /* MAVLINK_OR_CYPHAL */

	/*------------------------------- FreeRTOS -------------------------------*/
	/* work task - lowest priority */
	xTaskCreate(task_work, "udp", 512, NULL, 2, NULL);

	/* MAVLink task - medium priority */
	xTaskCreate(task_mav, "mav", 512, NULL, 2, NULL);

	/* EPOS task - high priority */
	xTaskCreate(task_epos, "epos", 256, NULL, 3, &task_epos_handle);

#if !MAVLINK_OR_CYPHAL

	/* tx_can task - high priority */
	xTaskCreate(task_tx_can, "tx_can", 128, NULL, 4, &task_tx_can_handle);

#endif /* MAVLINK_OR_CYPHAL */

	// /* create blinky timer */
	timer_blinky = xTimerCreate("tim_blinky", pdMS_TO_TICKS(BLINKY_PERIOD_MS),
								pdTRUE, NULL, timer_blinky_cb);

	/* start heartbeat timer */
	xTimerStart(timer_blinky, 0);

	/* Time past from initialization (we will use as a boot time for MAV) */
	boot_time_ms = HAL_GetTimeUS() / 1000;

	vTaskStartScheduler();

	/* loop */
	while (1) {
	}
}

void capture_callback(tim_hal_ch_t ch, tim_hal_int_type_t type)
{
	if (ch != TIM_HAL_CH_1 || type != TIM_HAL_INT_TYPE_CAPTURE)
		return;

	uint32_t cap = HAL_TIM_GetCaptureValue(TIM_HAL_INSTANCE_1, TIM_HAL_CH_1);
	uint32_t pclk = HAL_TIM_GetPCLK(TIM_HAL_INSTANCE_1);

	uint32_t ticks_min = pclk / 1000U; // 1000 us
	uint32_t ticks_max = pclk / 500U;  // 2000 us
	uint32_t ticks_3ms = pclk / 333U;  // 3000 us — sanity limit

	if (LPC_GPIO1->FIOPIN & (1 << 19)) {
		/* Rising edge */
		if (g_capture.rise_valid)
			g_capture.period_us =
				(uint32_t)((uint64_t)(cap - g_capture.t_period_rise) *
						   1000000ULL / pclk);
		g_capture.t_period_rise = cap;
		g_capture.t_rise = cap;
		g_capture.rise_valid = true;
	} else {
		/* Falling edge */
		if (!g_capture.rise_valid)
			return;

		uint32_t ticks = cap - g_capture.t_rise;

		/* Sanity check — ako je van 3ms, nevalidno merenje */
		if (ticks > ticks_3ms) {
			g_capture.rise_valid = false;
			return;
		}

		/* Clamp na validni opseg */
		if (ticks < ticks_min)
			ticks = ticks_min;
		if (ticks > ticks_max)
			ticks = ticks_max;

		// float pos = min_pos + ratio * (max_pos - min_pos);
		float ratio =
			(float)(ticks - ticks_min) / (float)(ticks_max - ticks_min);
		float pos =
			g_act_cfg.min_pos + ratio * (g_act_cfg.max_pos - g_act_cfg.min_pos);

		/* Clamp pozicije */
		if (pos < g_act_cfg.min_pos)
			pos = g_act_cfg.min_pos;
		if (pos > g_act_cfg.max_pos)
			pos = g_act_cfg.max_pos;

		messages_cyphal_uavcan_lisum_LisumManualCtrlHornet_1_0 arr;
		arr.pos_sp_act = pos;
		arr.act_id = 0;
		task_epos_ctrl(&arr);

		g_capture.pulse_us = (uint32_t)((uint64_t)ticks * 1000000ULL / pclk);
		g_capture.percent = (uint8_t)(ratio * 100.0f);
		g_capture.valid = true;
	}
}

/* ================================== Tasks ================================= */

servo_return_value_t Servo_Init(void)
{
	servo_return_value_t ret;

	/* Registruj servoe u grupu */
	ret = ServoGroup_AddServo(&g_servo_group_0, &g_servo_0);
	if (ret != SERVO_OK)
		return ret;

	ret = ServoGroup_AddServo(&g_servo_group_0, &g_servo_1);
	if (ret != SERVO_OK)
		return ret;

	ret = ServoGroup_AddServo(&g_servo_group_1, &g_servo_2);
	if (ret != SERVO_OK)
		return ret;

	/* Konfiguriši timer i pokreni */
	const servo_group_cfg_t cfg_0 = {
		.timer_instance = SERVO_TIMER_INSTANCE,
		.period_us = SERVO_PERIOD_US,
		.period_channel = SERVO_ALL_MATCH_CHANNEL,
	};

	const servo_group_cfg_t cfg_1 = {
		.timer_instance = SERVO_TIMER_INSTANCE_2,
		.period_us = SERVO_ONESHOT125_MAX_US,
		.period_channel = SERVO_ALL_MATCH_CHANNEL_2,
	};

	ServoGroup_Init(&g_servo_group_0, &cfg_0);
	return ServoGroup_Init(&g_servo_group_1, &cfg_1);
}

/* ============================= User Callbacks ============================= */

static void timer_blinky_cb(TimerHandle_t xTimer)
{
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 26,
						 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 26));
}

/* ============================ Private Functions =========================== */

/* ============================= FreeRTOS Hooks ============================= */

/* Increment a counter while Application is in Idle. */
void vApplicationIdleHook(void) { ulIdleCycleCount++; }

/* Callback to trap FreeRTOS Misconfiguration. */
void vAssertCalled(const char *pcFile, uint32_t ulLine)
{
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

/* Callback to trap FreeRTOS stack overflow. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

/* Callback to trap FreeRTOS insufficient heap memory. */
void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}