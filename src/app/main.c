/**
 * @file    main.c
 * @brief   Inertial Navigation System project.
 * @details This project acquires data from sensors (accelerometer, gyroscope,
 *          magnetometer, barometer, airspeed, GNSS), estimates aircraft
 *          attitude (using Madgwick filter) and altitude above mean sea level.
 *          These values are later packed into MAVLink messages and transmitted
 *          via UDP.
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_anpp.h"
#include "task_baro.h"
#include "task_fusion.h"
#include "task_gnss.h"
#include "task_imu.h"
#include "task_mav.h"
#include "task_tx_can.h"
#include "task_work.h"
#include "types.h"

/* Peripherals */
#include "can.h"
#include "common.h"
#include "eth.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"
#include "util.h"

/* External hardware drivers */
#include "icm42688p.h"
#include "icm42688p_spi.h"
#include "ms5611.h"
#include "ms5611_spi.h"

/* Lib */

/* Middleware */
#include "FreeRTOS.h"
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

#if MAVLINK_OR_CYPHAL
/* Device information */
// UDP
const uint8_t dev_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x03};
const uint8_t dev_ip[4] = {192, 168, 1, 103};

// gateway ground UDP information
const uint8_t gw_sky_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x00};
const uint8_t gw_sky_ip[4] = {192, 168, 1, 100};
const uint16_t gw_sky_port = 1003;
const uint16_t gw_sky_local_port = 1003;

/* trasnport layers */
eth_hal_phy_t eth_phy;
udp_tl_t mav_gw_sky_tl;

/* UDP */
udp_t udp;

#endif /* MAVLINK_OR_CYPHAL */

/* timers */
TimerHandle_t timer_blinky;

/* mutexes */
SemaphoreHandle_t mutex_spi;
SemaphoreHandle_t tx_queue_mutex;

/* semaphores */
SemaphoreHandle_t semphr_imu_drdy;

/* mailboxes */
QueueHandle_t mailbox_fusion_head;

/* queues */
QueueHandle_t queue_mav_hb;
QueueHandle_t queue_mav_scaled_imu;
QueueHandle_t queue_mav_scaled_pressure;
QueueHandle_t queue_mav_altitude;
QueueHandle_t queue_mav_attitude;
QueueHandle_t queue_mav_lisum_gnss_data;
QueueHandle_t queue_mav_lisum_airspeed_data;
QueueHandle_t queue_fusion_data;
QueueHandle_t queue_gnss;
QueueHandle_t queue_anpp;
QueueHandle_t queue_mav_ftp;
QueueHandle_t queue_command_long;

QueueHandle_t queue_cyphal_rx;

ms5611_t baro;
ms5611_spi_interface_t baro_spi_if;
icm42688p_t imu;
icm42688p_spi_interface_t imu_spi_if;

/* FreeRTOS */
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
volatile uint32_t ulIdleCycleCount = 0UL;

volatile node_mode_state_t current_node_mode = NODE_MODE_INITIALIZATION;

/* Temporary variables */
uint64_t boot_time_ms = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* blinky timer callback */
static void timer_blinky_cb(TimerHandle_t xTimer);

/* gpio callback */
void gpio_cb(uint16_t pinNum);

void cyphal_cb(void *usr_arg, uint32_t int_status);

/*******************************************************
 * ************************
 * Code
 ******************************************************************************/
int main()
{
	/* mutexes */
	mutex_spi = xSemaphoreCreateMutex();

	tx_queue_mutex = xSemaphoreCreateMutex();

	/* semaphores */
	semphr_imu_drdy = xSemaphoreCreateBinary();

	/* mailboxes */
	mailbox_fusion_head = xQueueCreate(1, sizeof(float));

	/* queues */
	queue_mav_hb = xQueueCreate(2, sizeof(mavlink_heartbeat_t));
	queue_fusion_data = xQueueCreate(5, sizeof(fusion_data_t));
	queue_gnss = xQueueCreate(256, sizeof(uint8_t));
	queue_anpp = xQueueCreate(256, sizeof(uint8_t));

#if MAVLINK_OR_CYPHAL

	queue_mav_scaled_imu = xQueueCreate(5, sizeof(mavlink_scaled_imu_t));
	queue_mav_scaled_pressure =
		xQueueCreate(5, sizeof(mavlink_scaled_pressure_t));
	queue_mav_altitude = xQueueCreate(5, sizeof(mavlink_altitude_t));
	queue_mav_attitude = xQueueCreate(5, sizeof(mavlink_attitude_t));
	queue_mav_lisum_gnss_data =
		xQueueCreate(5, sizeof(mavlink_lisum_gnss_recv_data_t));
	queue_mav_lisum_airspeed_data =
		xQueueCreate(5, sizeof(mavlink_lisum_sensor_airspeed_data_t));

#else
	queue_mav_scaled_imu =
		xQueueCreate(5, sizeof(messages_cyphal_uavcan_common_ScaledImu_1_0));
	queue_mav_scaled_pressure = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_common_ScaledPressure_1_0));
	queue_mav_altitude =
		xQueueCreate(5, sizeof(messages_cyphal_uavcan_common_Altitude_1_0));
	queue_mav_attitude =
		xQueueCreate(5, sizeof(messages_cyphal_uavcan_common_Attitude_1_0));
	queue_mav_lisum_gnss_data = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0));
	queue_mav_lisum_airspeed_data = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_lisum_LisumSensorAirspeedData_1_0));

	queue_mav_ftp = xQueueCreate(5, sizeof(mavlink_file_transfer_protocol_t));

	queue_command_long = xQueueCreate(
		5, sizeof(messages_cyphal_uavcan_common_ComponentInformationBasic_1_0));

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

	// icm42688p interrupt pin
	gpio_cfg.pinMode = GPIO_HAL_PINMODE_PULLUP;
	gpio_cfg.pinDir = GPIO_HAL_INPUT;

	HAL_GPIO_Init(ICM42688P_INT_PORT, ICM42688P_INT_PIN, &gpio_cfg);

	HAL_GPIO_ConfigureCallback(GPIO_HAL_INSTANCE_0, gpio_cb);

	/*--------------------------------- UART ---------------------------------*/
	uart_hal_cfg_t uart_cfg;
	uart_cfg.baudrate = GNSS_UART_BAUD_RATE;
	uart_cfg.databits = UART_HAL_DATABIT_8;
	uart_cfg.parity = UART_HAL_PARITY_NONE;
	uart_cfg.stopbits = UART_HAL_STOPBIT_1;

	uart_hal_fifo_cfg_t uart_fifo_cfg;
	HAL_UART_FIFOConfigStructInit(&uart_fifo_cfg);

	// gnss UART init
	HAL_UART_Init(GNSS_UART_INSTANCE, &uart_cfg);
	HAL_UART_ConfigureFIFO(GNSS_UART_INSTANCE, &uart_fifo_cfg);
	HAL_UART_EnableInterrupt(GNSS_UART_INSTANCE);

	// anpp UART init
	uart_cfg.baudrate = ANPP_UART_BAUD_RATE;
	uart_cfg.databits = UART_HAL_DATABIT_8;
	uart_cfg.parity = UART_HAL_PARITY_NONE;
	uart_cfg.stopbits = UART_HAL_STOPBIT_1;

	HAL_UART_RS232Init(ANPP_UART_INSTANCE, &uart_cfg);
	HAL_UART_ConfigureFIFO(ANPP_UART_INSTANCE, &uart_fifo_cfg);
	HAL_UART_EnableInterrupt(ANPP_UART_INSTANCE);

#if !MAVLINK_OR_CYPHAL

	/* --------------------------------- CAN ------------------------------ */
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

	/*---------------------------------- SPI ---------------------------------*/
	spi_hal_cfg_t spi_cfg = {.cpha = SPI_HAL_CPHA_FIRST,
							 .cpol = SPI_HAL_CPOL_HI,
							 .clockRate = 20000000,
							 .dataOrder = SPI_HAL_DATA_MSB_FIRST,
							 .databit = SPI_HAL_DATABIT_8,
							 .mode = SPI_HAL_MASTER_MODE};

	HAL_SPI_Init(SPI_HAL_INSTANCE_0, &spi_cfg);

	/*-------------------------------- MS5611 --------------------------------*/
	spi_hal_slave_id_t baro_slave_id;
	spi_hal_slave_t baro_slave = {.portNum = MS5611_CS_PORT,
								  .pinNum = MS5611_CS_PIN};

	baro_slave_id = HAL_SPI_SlaveInit(MS5611_SPI_INSTANCE, &baro_slave);
	ms5611_spi_init(&baro_spi_if, MS5611_SPI_INSTANCE, baro_slave_id);

	ms5611_init(&baro, (ms5611_interface_t *)&baro_spi_if);

	/*------------------------------ ICM42688P -------------------------------*/
	spi_hal_slave_id_t imu_slave_id;
	spi_hal_slave_t imu_slave = {.portNum = ICM42688P_CS_PORT,
								 .pinNum = ICM42688P_CS_PIN};

	imu_slave_id = HAL_SPI_SlaveInit(ICM42688P_SPI_INSTANCE, &imu_slave);
	icm42688p_spi_init(&imu_spi_if, ICM42688P_SPI_INSTANCE, imu_slave_id);

	icm42688p_cfg_t cfg = {
		.spi_mode = ICM42688P_SPI_MODE_0_3,
		.gyro_fs = ICM42688P_GYRO_FS_SEL_2000,
		.gyro_odr = ICM42688P_GYRO_ODR_200,
		.accel_fs = ICM42688P_ACCEL_FS_SEL_4,
		.accel_odr = ICM42688P_ACCEL_ODR_200,
	};

	icm42688p_init(&imu, (icm42688p_interface_t *)&imu_spi_if, &cfg);

	// Configure ICM42688P Interrupts
	icm42688p_int_cfg_t int1_cfg = {
		.int_mode = ICM42688P_INT_MODE_PULSED,
		.int_drive = ICM42688P_INT_DRIVE_CIRCUIT_OPEN_DRAIN,
		.int_polarity = ICM42688P_INT_POLARITY_ACTIVE_HIGH,
		.int_type = ICM42688P_INT_TYPE_DATA_RDY};

	icm42688p_int_cfg_t int2_cfg = {
		.int_mode = ICM42688P_INT_MODE_PULSED,
		.int_drive = ICM42688P_INT_DRIVE_CIRCUIT_OPEN_DRAIN,
		.int_polarity = ICM42688P_INT_POLARITY_ACTIVE_HIGH,
		.int_type = ICM42688P_INT_TYPE_DISABLE};

	icm42688p_int_cfg(&imu, &int1_cfg, &int2_cfg);

	HAL_GPIO_EnableInterrupt(ICM42688P_INT_PORT, ICM42688P_INT_PIN,
							 GPIO_HAL_EDGESTATE_RISING);

	/*------------------------------- FreeRTOS -------------------------------*/
	/* work task - lowest priority */
	xTaskCreate(task_work, "work", 256, NULL, 2, NULL);

	/* gnss task - lowest priority */
	xTaskCreate(task_gnss, "gnss", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	/* anpp task - lowest priority */
	xTaskCreate(task_anpp, "anpp", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* barometer task - medium priority */
	xTaskCreate(task_baro, "baro", configMINIMAL_STACK_SIZE, &baro, 2, NULL);

	/* MAVLink task - medium priority */
	xTaskCreate(task_mav, "mav", 512, NULL, 2, NULL);

	/* imu task - high priority */
	xTaskCreate(task_imu, "imu", 256, &imu, 3, NULL);

	/* fusion task - high priority */
	xTaskCreate(task_fusion, "fusion", 256, NULL, 3, NULL);

#if !MAVLINK_OR_CYPHAL

	/* tx_can task - high priority */
	xTaskCreate(task_tx_can, "tx_can", 256, NULL, 4, NULL);

#endif /* MAVLINK_OR_CYPHAL */

	// /* blinky timer */
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
	HAL_GPIO_SetPinValue(GPIO_HAL_INSTANCE_3, 26,
						 !HAL_GPIO_GetPinValue(GPIO_HAL_INSTANCE_3, 26));
}

void gpio_cb(uint16_t pinNum)
{
	BaseType_t taskWoken = pdFALSE;

	if (HAL_GPIO_GetIntStatus(ICM42688P_INT_PORT, ICM42688P_INT_PIN,
							  GPIO_HAL_EDGESTATE_RISING)) {
		xSemaphoreGiveFromISR(semphr_imu_drdy, &taskWoken);
	}

	portYIELD_FROM_ISR(taskWoken);
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