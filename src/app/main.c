/**
 * @file    main.c
 * @brief   Power Management project.
 * @details This project handles messages received from EDePro motor and
 *          batteries (main and standby). Received messages are later used to
 *          pack appropriate MAVLink messages.
 * @version 1.0.0
 * @date    07.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include "task_bms.h"
#include "task_mav.h"
#include "task_motor.h"
#include "task_tx_can.h"
#include "task_work.h"
#include "types.h"

/* Peripherals */
#include "can.h"
#include "common.h"
#include "eth.h"
#include "gpio.h"
#include "uart.h"
#include "util.h"

/* External hardware drivers */

/* Lib */

/* Middleware */
#include "FreeRTOS.h"
#include "queue.h"
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
const uint8_t dev_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x01};
const uint8_t dev_ip[4] = {192, 168, 1, 101};

// gateway sky UDP information
const uint8_t gw_sky_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x00};
const uint8_t gw_sky_ip[4] = {192, 168, 1, 100};
const uint16_t gw_sky_port = 1001;
const uint16_t gw_sky_local_port = 1001;

/* Transport layers */
eth_hal_phy_t eth_phy;
udp_tl_t mav_gw_sky_tl;

/* UDP */
udp_t udp;

#endif /* MAVLINK_OR_CYPHAL */

/* FreeRTOS */
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
volatile uint32_t ulIdleCycleCount = 0UL;

// Timers
TimerHandle_t timer_blinky;

// Mutexess
SemaphoreHandle_t tx_queue_mutex;

// Queues
QueueHandle_t queue_mav_hb;
QueueHandle_t queue_mav_motor_scaled;
QueueHandle_t queue_mav_batt_status1;
QueueHandle_t queue_mav_batt_status2;
QueueHandle_t queue_motor_rx;
QueueHandle_t queue_bms1_rx;
QueueHandle_t queue_bms2_rx;
QueueHandle_t queue_cyphal_rx;
QueueHandle_t queue_mav_ftp;
QueueHandle_t queue_command_long;

/* task bms */
task_bms_arg_t bms1_cfg;
task_bms_arg_t bms2_cfg;

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

	/* mutexes */
	tx_queue_mutex = xSemaphoreCreateMutex();

	queue_cyphal_rx = xQueueCreate(10, sizeof(can_hal_msg_t));

#if MAVLINK_OR_CYPHAL

	/* Queues */
	queue_mav_hb = xQueueCreate(2, sizeof(mavlink_heartbeat_t));
	queue_mav_motor_scaled =
		xQueueCreate(5, sizeof(mavlink_lisum_power_motor_scaled_data_t));
	queue_mav_batt_status1 = xQueueCreate(2, sizeof(mavlink_battery_status_t));
	queue_mav_batt_status2 = xQueueCreate(2, sizeof(mavlink_battery_status_t));
	queue_motor_rx = xQueueCreate(MOTOR_UART_QUEUE_LEN, sizeof(uint8_t));
	queue_bms1_rx = xQueueCreate(BMS1_UART_QUEUE_LEN, sizeof(uint8_t));
	queue_bms2_rx = xQueueCreate(BMS2_UART_QUEUE_LEN, sizeof(uint8_t));

#else
	/* Queues */
	queue_mav_hb = xQueueCreate(2, sizeof(mavlink_heartbeat_t));
	queue_mav_motor_scaled = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_lisum_LisumPowerMotorScaledData_1_0));
	queue_mav_batt_status1 = xQueueCreate(
		2, sizeof(messages_cyphal_uavcan_common_BatteryStatus_1_0));
	queue_mav_batt_status2 = xQueueCreate(
		2, sizeof(messages_cyphal_uavcan_common_BatteryStatus_1_0));
	queue_motor_rx = xQueueCreate(MOTOR_UART_QUEUE_LEN, sizeof(uint8_t));
	queue_bms1_rx = xQueueCreate(BMS1_UART_QUEUE_LEN, sizeof(uint8_t));
	queue_bms2_rx = xQueueCreate(BMS2_UART_QUEUE_LEN, sizeof(uint8_t));

	queue_command_long = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_common_ComponentInformationBasic_1_0));

	queue_mav_ftp = xQueueCreate(5, sizeof(mavlink_file_transfer_protocol_t));

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

	/*--------------------------------- UART ---------------------------------*/
	// bms1 UART init
	uart_hal_cfg_t uart_cfg;
	uart_cfg.baudrate = BMS1_UART_BAUD_RATE;
	uart_cfg.databits = UART_HAL_DATABIT_8;
	uart_cfg.parity = UART_HAL_PARITY_NONE;
	uart_cfg.stopbits = UART_HAL_STOPBIT_1;

	uart_hal_fifo_cfg_t uart_fifo_cfg;
	HAL_UART_FIFOConfigStructInit(&uart_fifo_cfg);

	HAL_UART_Init(BMS1_UART_INSTANCE, &uart_cfg);
	HAL_UART_ConfigureFIFO(BMS1_UART_INSTANCE, &uart_fifo_cfg);
	HAL_UART_EnableInterrupt(BMS1_UART_INSTANCE);

	// bms2 UART init
	uart_cfg.baudrate = BMS2_UART_BAUD_RATE;

	HAL_UART_Init(BMS2_UART_INSTANCE, &uart_cfg);
	HAL_UART_ConfigureFIFO(BMS2_UART_INSTANCE, &uart_fifo_cfg);
	HAL_UART_EnableInterrupt(BMS2_UART_INSTANCE);

	// motor UART init
	uart_cfg.baudrate = MOTOR_UART_BAUD_RATE;

	HAL_UART_RS232Init(MOTOR_UART_INSTANCE, &uart_cfg);
	HAL_UART_ConfigureFIFO(MOTOR_UART_INSTANCE, &uart_fifo_cfg);
	HAL_UART_EnableInterrupt(MOTOR_UART_INSTANCE);

#if !MAVLINK_OR_CYPHAL

	/*--------------------------------- CAN ----------------------------------*/
	HAL_CAN_Init(CAN_HAL_INSTANCE_0, 1000000);

	HAL_CAN_EnableInterrupt(CAN_HAL_INSTANCE_0, cyphal_cb, NULL);

	HAL_CAN_IntCmd(CAN_HAL_INSTANCE_0, CAN_HAL_INT_RI, lFunctionalState_Enable);

	gpio_hal_cfg_type_t gpio_can_cfg;
	gpio_can_cfg.openDrain = GPIO_HAL_OPENDRAIN_NORMAL;
	gpio_can_cfg.pinMode = GPIO_HAL_PINMODE_PULLUP;
	gpio_can_cfg.pinDir = GPIO_HAL_OUTPUT;

	HAL_GPIO_Init(GPIO_HAL_INSTANCE_2, 13, &gpio_can_cfg); // P2.13 CAN1_RD
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_2, 13, 0);	   // P2.13 LOW

#else

	/*--------------------------------- ETH ----------------------------------*/
	HAL_ETH_Init(&eth_phy, dev_mac);

	/*--------------------------------- UDP ----------------------------------*/
	/* initialize UDP */
	udp_init(&udp, dev_mac, dev_ip, (udp_phy_t *)&eth_phy);

	/* add client MAC & IP to static ARP table */
	udp_arptab_add(&udp, gw_sky_mac, gw_sky_ip);

	/*-------------------------------- UDP TL --------------------------------*/
	udp_tl_init(&mav_gw_sky_tl, &udp, (uint8_t *)gw_sky_ip, gw_sky_local_port,
				gw_sky_port);

#endif /* MAVLINK_OR_CYPHAL */

	/*------------------------------- FreeRTOS -------------------------------*/

	BaseType_t status;

	/* work task - lowest priority */
	xTaskCreate(task_work, "work", 256, NULL, 2, NULL);

	/* motor task - lowest priority */
	status = xTaskCreate(task_motor, "motor", 256, NULL, 2, NULL);

	if (status != pdPASS) {
		while (1)
			;
	}

	/* bms1 task - lowest priority */
	bms1_cfg.instance = BMS1_UART_INSTANCE;
	bms1_cfg.rx_queue = &queue_bms1_rx;
	bms1_cfg.tx_queue = &queue_mav_batt_status1;

	status = xTaskCreate(task_bms, "bms1", 256, &bms1_cfg, 2, NULL);

	if (status != pdPASS) {
		while (1)
			;
	}

	/* bms2 task - lowest priority */
	bms2_cfg.instance = BMS2_UART_INSTANCE;
	bms2_cfg.rx_queue = &queue_bms2_rx;
	bms2_cfg.tx_queue = &queue_mav_batt_status2;

	status = xTaskCreate(task_bms, "bms2", 128, &bms2_cfg, 2, NULL);

	if (status != pdPASS) {
		while (1)
			;
	}
	/* MAVLink task - medium priority */
	status = xTaskCreate(task_mav, "mav", 512, NULL, 2, NULL);

	if (status != pdPASS) {
		while (1)
			;
	}

#if !MAVLINK_OR_CYPHAL

	/* MAVLink task - medium priority */
	status = xTaskCreate(task_tx_can, "tx_can", 128, NULL, 3, NULL);

	if (status != pdPASS) {
		while (1)
			;
	}

#endif /* MAVLINK_OR_CYPHAL */

	/* blinky timer */
	timer_blinky = xTimerCreate("tim_blinky", pdMS_TO_TICKS(BLINKY_PERIOD_MS),
								pdTRUE, NULL, timer_blinky_cb);

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
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 25,
						 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 25));
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
