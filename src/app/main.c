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
#include "util.h"

/* External hardware drivers */

/* Lib */

/* Middleware */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "udp.h"
#include "udp_tl.h"

#include "mav.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

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

volatile node_mode_state_t current_node_mode = NODE_MODE_INITIALIZATION;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void timer_blinky_cb(TimerHandle_t xTimer);

void cyphal_cb(void *usr_arg, uint32_t int_status);

/*******************************************************************************
 * Code
 ******************************************************************************/
int main()
{
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

	/*------------------------------- Dev Time -------------------------------*/
	HAL_DevTimeInit(TIM_HAL_INSTANCE_0);

	/*--------------------------------- GPIO ---------------------------------*/
	gpio_hal_cfg_type_t gpio_cfg;
	gpio_cfg.openDrain = GPIO_HAL_OPENDRAIN_NORMAL;
	gpio_cfg.pinMode = GPIO_HAL_PINMODE_PULLUP;
	gpio_cfg.pinDir = GPIO_HAL_OUTPUT;

	HAL_GPIO_Init(GPIO_HAL_INSTANCE_3, 25, &gpio_cfg);
	HAL_GPIO_Init(GPIO_HAL_INSTANCE_3, 26, &gpio_cfg);

	HAL_GPIO_Init(GPIO_HAL_INSTANCE_0, 4, &gpio_cfg);
	HAL_GPIO_Init(GPIO_HAL_INSTANCE_0, 5, &gpio_cfg);
	HAL_GPIO_Init(GPIO_HAL_INSTANCE_0, 6, &gpio_cfg);
	HAL_GPIO_Init(GPIO_HAL_INSTANCE_0, 7, &gpio_cfg);

	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_0, 4, 0);
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_0, 5, 0);
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_0, 6, 0);
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_0, 7, 0);
	/*--------------------------------- CAN ----------------------------------*/

	HAL_CAN_Init(CAN_HAL_INSTANCE_0, 1000000);

#if !MAVLINK_OR_CYPHAL

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

	HAL_GPIO_Init(GPIO_HAL_INSTANCE_0, 4, &gpio_test_cfg); // Time for writing
	HAL_GPIO_Init(GPIO_HAL_INSTANCE_0, 5, &gpio_test_cfg); // Time for erasing
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_0, 4, 0); // Time for writing LOW
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_0, 5, 0); // Time for erasing LOW
#else

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
	xTaskCreate(task_work, "udp", 256, NULL, 2, NULL);

	/* MAVLink task - medium priority */
	xTaskCreate(task_mav, "mav", 512, NULL, 2, NULL);

	/* EPOS task - high priority */
	xTaskCreate(task_epos, "epos", 256, NULL, 3, NULL);

#if !MAVLINK_OR_CYPHAL

	/* tx_can task - high priority */
	xTaskCreate(task_tx_can, "tx_can", 128, NULL, 4, NULL);

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

/* ================================== Tasks ================================= */

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