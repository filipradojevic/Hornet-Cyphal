/**
 * @file    main.c
 * @brief   Black Box project.
 * @details This project acquires data from GW_SKY via UDP and records this
 *          data on block device. Logged data is stored on external flash
 *          memory using the LittleFS file system, ensuring reliability and
 *          wear-leveling.
 *
 * @version 1.0.0
 * @date    24.04.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * System Includes
 ******************************************************************************/

#include "sdkconfig.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*******************************************************************************
 * Application Includes
 ******************************************************************************/

#include "main.h"
#include "protocol_utility.h"
#include "task_log.h"
#include "task_work.h"
#include "types.h"

/*******************************************************************************
 * ESP-IDF Core Includes
 ******************************************************************************/

#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

/*******************************************************************************
 * ESP-IDF Driver Includes
 ******************************************************************************/

#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "driver/twai.h"

/*******************************************************************************
 * FreeRTOS Includes
 ******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOSConfig_arch.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/*******************************************************************************
 * External Hardware Driver Includes
 ******************************************************************************/

#include "ds1302.h"
#include "flash.h"
#include "flash_common.h"
#include "psram_ring.h"
#include "sd.h"
#include "w5500.h"

/*******************************************************************************
 * Middleware/Protocol Includes
 ******************************************************************************/

/* LittleFS */
#include "lfs.h"
#include "lfs_def.h"

#if MAVLINK_OR_CYPHAL

/* MAVLink */
#include "mav.h"

/* UDP */
#include "udp.h"
#include "udp_tl.h"

#endif /* MAVLINK_OR_CYPHAL */

/*******************************************************************************
 * ULog Format Includes
 ******************************************************************************/

#include "ulog.h"

#include "ulog_adsb_vehicle.h"
#include "ulog_ais_vessel.h"
#include "ulog_altitude.h"
#include "ulog_attitude.h"
#include "ulog_battery_status.h"
#include "ulog_def.h"
#include "ulog_lisum_esc_status.h"
#include "ulog_lisum_gnss_recv_data.h"
#include "ulog_lisum_manual_ctrl_hornet.h"
#include "ulog_lisum_power_hornet_act_data.h"
#include "ulog_lisum_power_motor_scaled_data.h"
// #include "ulog_lisum_power_motor_vesc_data.h"
#include "ulog_lisum_propulsion_tb_status.h"
#include "ulog_lisum_sensor_airspeed_data.h"
#include "ulog_named_value_float.h"
#include "ulog_scaled_imu.h"
#include "ulog_scaled_pressure.h"

/*******************************************************************************
 * Cyphal messages
 ******************************************************************************/

#include "common/FileTransferProtocol_1_0.h"
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/Mode_1_0.h"
#include "uavcan/primitive/array/Integer8_1_0.h"

// Imu
#include "common/Altitude_1_0.h"
#include "common/Attitude_1_0.h"
#include "common/ScaledImu_1_0.h"
#include "common/ScaledPressure_1_0.h"
#include "lisum/LisumGnssRecvData_1_0.h"
#include "lisum/LisumSensorAirspeedData_1_0.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define ULOG_INFO_COUNT 1	/* Number of ULog information entries */
#define ULOG_PARAM_COUNT 17 /* Number of ULog parameter entries */

#define MAIN_BATTERY_STATUS_ID 0 /* Main battery status multi-ID */
#define STBY_BATTERY_STATUS_ID 1 /* Standby battery status multi-ID */

#define INS_SCALED_IMU_ID 0		 /* Scaled IMU data multi-ID */
#define INS_COTS_SCALED_IMU_ID 1 /* COTS scaled IMU data multi-ID */

#define INS_ATTITUDE_ID 0	   /* Attitude multi-ID */
#define INS_COTS_ATTITUDE_ID 1 /* COTS attitude multi-ID */

#define INS_GNSS_DATA_ID 0		   /* GNSS data multi-ID */
#define INS_COTS_HR_GNSS_DATA_ID 1 /* COTS high-rate GNSS data multi-ID */
#define INS_COTS_LR_GNSS_DATA_ID 2 /* COTS low-rate GNSS data multi-ID */

#define LISUM_POWER_MOTOR_SCALED_DATA_ID 0 /* Power motor scaled data ID */

/* Propulsion testbed status data ID */
#define LISUM_PROPULSION_TB_STATUS_DATA_ID 0

#define LISUM_ESC_STATUS_DATA_ID 0 /* ESC status data ID */

#define LISUM_VESC_CONTROL_DATA_ID 0 /* VESC control data ID */

#define NAMED_VALUE_FLOAT_ID 1 /* Named value (float) ID */

#define AIS_VESSEL_ID 0 /* AIS vessel data ID */

#define ADSB_VEHICLE_ID 0 /* ADS-B vehicle data ID */

#define ULOG_CYPHAL_PARAM_COUNT 3 /* Number of ULog parameter entries */

#define PRESSURE_TEMP_VARS_TS_0_1_ID 0 /* PressureTempVarsTs_0_1 parameter */

#define SI_SAMPLE_LENGTH_SCALAR_1_0_ID 0

#define LISUM_GNSS_RECV_DATA_1_0_ID 0

#define LISUM_SENSOR_AIRSPEED_DATA_1_0_ID 0

#define PAGE_SIZE_FLASH 2048U /* Flash page flash size in bytes */
#define PAGE_SIZE_SD 512U	  /* Flash page flash size in bytes */
#define LOOKAHEAD_SIZE 256U	  /* LittleFS lookahead buffer size in bytes */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/* ULOG STRUCTURES */

/* ULog information entry */
typedef struct ulog_info_entry_t {
	char* key;
	char* val;
} ulog_info_entry_t;

/* ULog parameter entry */
typedef struct ulog_param_entry_t {
	char* key;
	int32_t val;
} ulog_param_entry_t;

/* DS1302 RTC (Real Time Clock) instance */

static ds1302_t rtc = {.interface = {
						   .rst_port = 0,
						   .rst_pin = 45,
						   .clk_port = 0,
						   .clk_pin = 48,
						   .io_port = 0,
						   .io_pin = 47,
					   }};

static ds1302_time_t current_time = {0};

/* FSM State of the Node device */

volatile node_mode_state_t current_node_mode = NODE_MODE_INITIALIZATION;

rmt_item32_t led_data[24 * LED_NUM];
/*******************************************************************************
 * Variables
 ******************************************************************************/

#if MAVLINK_OR_CYPHAL

/* Network Configuration */

// Device (ESP32) UDP information
const uint8_t dev_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x02};
const uint8_t dev_ip[4] = {192, 168, 1, 102};
const uint8_t dev_gateway[4] = {0, 0, 0, 0};
const uint8_t dev_subnet[4] = {0, 0, 0, 0};
const uint16_t local_port = 1002;

// Gateway Sky UDP information
const uint8_t gw_sky_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x00};
const uint8_t gw_sky_ip[4] = {192, 168, 1, 100};
const uint16_t gw_sky_port = 1002;
const uint16_t gw_sky_local_port = 1002;

// INS UDP information
const uint8_t ins_mac[6] = {0x4C, 0x50, 0x43, 0x01, 0x00, 0x03};
const uint8_t ins_ip[4] = {192, 168, 1, 103};
const uint16_t ins_port = 1003;
const uint16_t ins_local_port = 1003;

// Motor Control
const uint8_t motor_control_mac[6] = {0x4C, 0x50, 0x43, 0x02, 0x00, 0x00};
const uint8_t motor_control_ip[4] = {192, 168, 1, 200};
const uint16_t motor_control_port = 1024;
const uint16_t motor_control_local_port = 1024;

// Vesc Control
const uint8_t vesc_control_mac[6] = {0x4C, 0x50, 0x43, 0x02, 0x00, 0x01};
const uint8_t vesc_control_ip[4] = {192, 168, 1, 201};
const uint16_t vesc_control_port = 1024;
const uint16_t vesc_control_local_port = 1024;

/* Transport Layer Objects */
udp_tl_t mav_pc_tl;
udp_tl_t mav_gw_heli_tl;
udp_tl_t mav_gw_sky_tl;
udp_tl_t mav_ins_tl;
udp_tl_t mav_motor_control_tl;
udp_tl_t mav_vesc_control_tl;
udp_t udp;

/* Hardware Device Objects */
static w5500_t w5500_dev;
static uint8_t rx_buffer[RX_BUFFER_SIZE];

#endif /* MAVLINK_OR_CYPHAL */

#if FLASH_OR_SD_LITTLEFS

flash_t flash;
spi_device_handle_t flash_spi_handle;

#else

sd_t sd;
spi_device_handle_t sd_spi_handle;

#endif

/* PSRAM Ring Buffer */
ring_t ring_buffer;
uint8_t* psram_buf = NULL;

/* FreeRTOS Objects */
// Task handles
TaskHandle_t task_work_handle;
TaskHandle_t task_log_global;

// Timers
TimerHandle_t timer_blinky;

// Synchronization objects
SemaphoreHandle_t mutex_mav_ftp;
SemaphoreHandle_t semaphore_logging_ready;
volatile uint32_t ulIdleCycleCount = 0UL;

/* LittleFS File System */
lfs_t lfs;
lfs_file_t lfs_file;
struct lfs_file_config file_cfg;
struct lfs_config lfs_cfg;
char file_name[LFS_NAME_MAX];

// LittleFS buffers
#if FLASH_OR_SD_LITTLEFS

static uint8_t read_buffer[PAGE_SIZE_FLASH];
static uint8_t prog_buffer[PAGE_SIZE_FLASH];
static uint8_t lookahead_buffer[LOOKAHEAD_SIZE];
static uint8_t file_buffer[PAGE_SIZE_FLASH];

#else

static uint8_t read_buffer[PAGE_SIZE_SD];
static uint8_t prog_buffer[PAGE_SIZE_SD];
static uint8_t lookahead_buffer[LOOKAHEAD_SIZE];
static uint8_t file_buffer[PAGE_SIZE_SD];

#endif

uint32_t boot_count;

ulog_t ulog;

// ULog message IDs
uint16_t batt_status1_id = 0;
uint16_t batt_status2_id = 0;
uint16_t motor_scaled_id = 0;
uint16_t scaled_imu1_id = 0;
uint16_t scaled_imu2_id = 0;
uint16_t scaled_pressure_id = 0;
uint16_t altitude_id = 0;
uint16_t attitude1_id = 0;
uint16_t attitude2_id = 0;
uint16_t lisum_gnss_data1_id = 0;
uint16_t lisum_gnss_data2_id = 0;
uint16_t lisum_gnss_data3_id = 0;
uint16_t lisum_airspeed_data_id = 0;
uint16_t lisum_act_data_id = 0;
uint16_t lisum_manual_ctrl_id = 0;
uint16_t lisum_propulsion_tb_status_id = 0;
uint16_t lisum_esc_status_id = 0;
uint16_t lisum_power_motor_vesc_id = 0;
uint16_t named_value_float_id = 0;
uint16_t adsb_vehicle_id = 0;
uint16_t ais_vessel_id = 0;
uint16_t pressuretempvarts_0_1_id = 0;
uint16_t uavcan_si_sample_length_scalar_1_0_id = 0;
uint16_t uavcan_lisum_lisum_gnss_recv_data_1_0_id = 0;

// ULog configuration data
const ulog_info_entry_t ulog_infos[ULOG_INFO_COUNT] = {
	{.key = "char[11] sys_name", .val = "Hornet X-01"}};

const ulog_param_entry_t ulog_params[ULOG_PARAM_COUNT] = {
	{.key = "int32_t NAMED_VALUE_FLOAT_ID", .val = NAMED_VALUE_FLOAT_ID},
	{.key = "int32_t MAIN_BATTERY_STATUS_ID", .val = MAIN_BATTERY_STATUS_ID},
	{.key = "int32_t STBY_BATTERY_STATUS_ID", .val = STBY_BATTERY_STATUS_ID},
	{.key = "int32_t INS_SCALED_IMU_ID", .val = INS_SCALED_IMU_ID},
	{.key = "int32_t INS_COTS_SCALED_IMU_ID", .val = INS_COTS_SCALED_IMU_ID},
	{.key = "int32_t INS_ATTITUDE_ID", .val = INS_ATTITUDE_ID},
	{.key = "int32_t LISUM_POWER_MOTOR_SCALED_DATA_ID",
	 .val = LISUM_POWER_MOTOR_SCALED_DATA_ID},
	{.key = "int32_t INS_COTS_ATTITUDE_ID", .val = INS_COTS_ATTITUDE_ID},
	{.key = "int32_t INS_GNSS_DATA_ID", .val = INS_GNSS_DATA_ID},
	{.key = "int32_t INS_COTS_HR_GNSS_DATA_ID", .val = INS_COTS_HR_GNSS_DATA_ID},
	{.key = "int32_t INS_COTS_LOW_RATE_GNSS_DATA_ID", .val = INS_COTS_LR_GNSS_DATA_ID},
	{.key = "int32_t LISUM_PROPULSION_TB_STATUS_DATA_ID",
	 .val = LISUM_PROPULSION_TB_STATUS_DATA_ID},
	{.key = "int32_t LISUM_ESC_STATUS_DATA_ID", .val = LISUM_ESC_STATUS_DATA_ID},
	{.key = "int32_t LISUM_VESC_CONTROL_DATA_ID", .val = LISUM_VESC_CONTROL_DATA_ID},
	{.key = "int32_t AIS_VESSEL_ID", .val = AIS_VESSEL_ID},
	{.key = "int32_t ADSB_VEHICLE_ID", .val = ADSB_VEHICLE_ID},
	{.key = "int32_t LISUM_GNSS_RECV_DATA", .val = LISUM_GNSS_RECV_DATA_1_0_ID}};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void ws2812_init(void)
{
	rmt_config_t config = {.rmt_mode = RMT_MODE_TX,
						   .channel = RMT_CHANNEL,
						   .gpio_num = WS2812_GPIO,
						   .clk_div = 2, // 80MHz / 2 = 40MHz
						   .mem_block_num = 1,
						   .tx_config = {
							   .loop_en = false,
							   .carrier_en = false,
							   .idle_output_en = true,
							   .idle_level = RMT_IDLE_LEVEL_LOW,
						   }};

	ESP_ERROR_CHECK(rmt_config(&config));
	ESP_ERROR_CHECK(rmt_driver_install(RMT_CHANNEL, 0, 0));
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Forward declarations for initialization functions */
static esp_err_t init_real_time_clock(void);
static esp_err_t init_psram_ring_buffer(void);
static esp_err_t init_freertos_objects(void);

#if MAVLINK_OR_CYPHAL

static esp_err_t init_ethernet_network(void);

#else

static esp_err_t twai_initialization(void);

#endif /* MAVLINK_OR_CYPHAL */

static esp_err_t init_storage_system(void);
static esp_err_t create_application_tasks(void);

/**
 * @brief Main application entry point
 * @details Initializes all system components in proper order and creates tasks
 */
void app_main(void)
{
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "===============================================");
	ESP_LOGI(TAG, "=== BlackBox ESP32 System Initialization ===");
	ESP_LOGI(TAG, "===============================================");
	ESP_LOGI(TAG, "Version: %s, Build Date: %s", FLIGHT_CUSTOM_VERSION, __DATE__);
	ESP_LOGI(TAG, "");

	ws2812_init();

	/* Initialize all subsystems in dependency order */
	ESP_ERROR_CHECK_WITHOUT_ABORT(init_real_time_clock());
	ESP_LOGI(TAG, "");

	ESP_ERROR_CHECK_WITHOUT_ABORT(init_psram_ring_buffer());
	ESP_LOGI(TAG, "");

	ESP_ERROR_CHECK_WITHOUT_ABORT(init_freertos_objects());
	ESP_LOGI(TAG, "");

#if MAVLINK_OR_CYPHAL

	ESP_ERROR_CHECK_WITHOUT_ABORT(init_ethernet_network());
	ESP_LOGI(TAG, "");

#else

	ESP_ERROR_CHECK_WITHOUT_ABORT(twai_initialization());
	ESP_LOGI(TAG, "");

#endif /* MAVLINK_OR_CYPHAL */

	ESP_ERROR_CHECK_WITHOUT_ABORT(init_storage_system());
	ESP_LOGI(TAG, "");

	ESP_ERROR_CHECK_WITHOUT_ABORT(init_littlefs_filesystem());
	ESP_LOGI(TAG, "");

	ESP_ERROR_CHECK_WITHOUT_ABORT(create_application_tasks());
	ESP_LOGI(TAG, "");

	ESP_LOGI(TAG, "===============================================");
	ESP_LOGI(TAG, "=== System Initialization Complete ===");
	ESP_LOGI(TAG, "===============================================");
	ESP_LOGI(TAG, "BlackBox ready for operation");
	ESP_LOGI(TAG, "");
}

static esp_err_t init_real_time_clock(void)
{
	ESP_LOGI(TAG, "--- Real-Time Clock Initialization ---");

	// 1. Initialize ds1302 RTC
	ds1302_init(&rtc);

	// ds1302_time_t default_time = {
	// 	.sec = 0, .min = 10, .hour = 11, .day = 3, .month = 2, .year = 2026};

	// ds1302_set_time(&rtc, &default_time);

	// 2. Read current time
	ds1302_get_time(&rtc, &current_time);

	ESP_LOGI(TAG, "Current RTC time: %02u:%02u:%02u %02u/%02u/%u", current_time.hour,
			 current_time.min, current_time.sec, current_time.day, current_time.month,
			 current_time.year);

	// 3. Validate time (simple check)
	if (current_time.year < 2025 || current_time.year > 2030) {
		// ESP_LOGI(TAG, "RTC time invalid, setting to default...");

		// ds1302_time_t default_time = {
		// 	.sec = 0, .min = 53, .hour = 16, .day = 8, .month = 1, .year = 2026};

		// ESP_LOGI(TAG, "Setting to default RTC time: %02u:%02u:%02u %02u/%02u/%u",
		// 		 default_time.hour, default_time.min, default_time.sec, default_time.day,
		// 		 default_time.month, default_time.year);

		// ds1302_set_time(&rtc, &default_time);

		// ds1302_get_time(&rtc, &current_time);

		// ESP_LOGI(TAG, "Current RTC time: %02u:%02u:%02u %02u/%02u/%u",
		// current_time.hour, 		 current_time.min, current_time.sec, current_time.day,
		// current_time.month, 		 current_time.year);

		ESP_LOGI(TAG, "RTC time is INVALID.");
	} else {
		ESP_LOGI(TAG, "RTC time VALID.");
	}

	return ESP_OK;
}

/**
 * @brief Initialize PSRAM and Ring Buffer
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t init_psram_ring_buffer(void)
{
	ESP_LOGI(TAG, "--- PSRAM & Ring Buffer Initialization ---");

	/* Check PSRAM availability */
	if (!esp_psram_is_initialized()) {
		ESP_LOGE(TAG, "PSRAM not initialized! Check menuconfig settings");
		return ESP_ERR_NOT_FOUND;
	}

	size_t total_psram = esp_psram_get_size();
	ESP_LOGI(TAG, "PSRAM Available: %u B (%.2f MB)", total_psram,
			 (float)total_psram / (1024.0 * 1024.0));

	/* Allocate ring buffer in PSRAM */
	ESP_LOGI(TAG, "Allocating ring buffer: %u MB", RING_BUFFER_SIZE / (1024 * 1024));

	psram_buf = (uint8_t*)heap_caps_malloc(RING_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
	if (!psram_buf) {
		ESP_LOGE(TAG, "Failed to allocate %u MB in PSRAM",
				 RING_BUFFER_SIZE / (1024 * 1024));
		ESP_LOGE(TAG, "Available PSRAM: %u bytes",
				 heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
		return ESP_ERR_NO_MEM;
	}

	ESP_LOGI(TAG, "Ring buffer allocated at: %p", psram_buf);
	ESP_LOGI(TAG, "Remaining PSRAM: %u bytes",
			 heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

	/* Initialize ring buffer structure */
	esp_err_t ret = ring_init(&ring_buffer, psram_buf, RING_BUFFER_SIZE, 0, 0);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Ring buffer initialization failed");
		heap_caps_free(psram_buf);
		return ret;
	}

	ESP_LOGI(TAG, "Ring buffer initialized successfully");
	return ESP_OK;
}

/**
 * @brief Initialize FreeRTOS objects (queues, semaphores)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t init_freertos_objects(void)
{
	ESP_LOGI(TAG, "--- FreeRTOS Objects Initialization ---");

	/* Create synchronization objects */
	mutex_mav_ftp = xSemaphoreCreateMutex();
	if (!mutex_mav_ftp) {
		ESP_LOGE(TAG, "Failed to create FTP mutex");
		return ESP_ERR_NO_MEM;
	}

	semaphore_logging_ready = xSemaphoreCreateBinary();
	if (!semaphore_logging_ready) {
		ESP_LOGE(TAG, "Failed to create logging semaphore");
		return ESP_ERR_NO_MEM;
	}

	ESP_LOGI(TAG, "FreeRTOS objects created successfully");
	return ESP_OK;
}

#if MAVLINK_OR_CYPHAL

/**
 * @brief Initialize Ethernet and network stack
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t init_ethernet_network(void)
{
	ESP_LOGI(TAG, "--- Ethernet & Network Initialization ---");

	/* Initialize SPI bus for W5500 */
	spi_bus_config_t buscfg = {
		.miso_io_num = W5500_ETH_MISO,
		.mosi_io_num = W5500_ETH_MOSI,
		.sclk_io_num = W5500_ETH_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 2048,
	};

	ESP_RETURN_ON_ERROR(spi_bus_initialize(W5500_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO),
						TAG, "SPI bus initialization failed");

	/* Add W5500 device */
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = 20 * 1000 * 1000,
		.mode = 0,
		.spics_io_num = W5500_ETH_CS,
		.queue_size = 20,
	};

	ESP_RETURN_ON_ERROR(spi_bus_add_device(W5500_ETH_SPI_HOST, &devcfg, &w5500_dev.spi),
						TAG, "Failed to add W5500 device");

	/* Initialize W5500 chip */
	W5500_ETH_Init(&w5500_dev, dev_mac, dev_ip, dev_subnet, dev_gateway, local_port,
				   W5500_ETH_CS, W5500_ETH_RST, W5500_ETH_MISO, W5500_ETH_MOSI,
				   W5500_ETH_CLK, W5500_ETH_INT);

	gpio_set_level(W5500_ETH_CS, 1);
	vTaskDelay(pdMS_TO_TICKS(100));

	/* Initialize UDP stack */
	udp_custom_init(&udp, dev_mac, dev_ip, (udp_phy_t*)&w5500_dev.phy);

	/* Setup ARP table */
	udp_arptab_add(&udp, gw_sky_mac, gw_sky_ip);
	udp_arptab_add(&udp, ins_mac, ins_ip);
	udp_arptab_add(&udp, motor_control_mac, motor_control_ip);
	udp_arptab_add(&udp, vesc_control_mac, vesc_control_ip);

	/* Initialize transport layers */
	udp_tl_init(&mav_gw_sky_tl, &udp, (uint8_t*)gw_sky_ip, gw_sky_local_port,
				gw_sky_port);
	udp_tl_init(&mav_ins_tl, &udp, (uint8_t*)ins_ip, ins_local_port, ins_port);

	udp_tl_init(&mav_motor_control_tl, &udp, (uint8_t*)motor_control_ip,
				motor_control_local_port, motor_control_port);

	udp_tl_init(&mav_vesc_control_tl, &udp, (uint8_t*)vesc_control_ip,
				vesc_control_local_port, vesc_control_port);

	ESP_LOGI(TAG, "Network initialized - Device IP: %d.%d.%d.%d", dev_ip[0], dev_ip[1],
			 dev_ip[2], dev_ip[3]);

	return ESP_OK;
}

#else

/**
 * @brief Initialize Twai (CAN) peripheral
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t twai_initialization(void)
{

	ESP_LOGI(TAG, "--- TWAI INITIALIZATION ---");

	/* Ažurirana konfiguracija za GPIO 16 (TX) i GPIO 18 (RX) */
	twai_general_config_t g_config =
		TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_16, GPIO_NUM_18, TWAI_MODE_NORMAL);

	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	// ... (ostatak koda)
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_FAIL) {
		return ESP_FAIL;
	}
	if (twai_start() == ESP_FAIL) {
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "TWAI Initialized Successfully");
	return ESP_OK;
}

#endif /* MAVLINK_OR_CYPHAL */

/**
 * @brief Initialize storage system (SD Card)
 * @return ESP_OK on success, error code otherwise
 */

static esp_err_t init_storage_system(void)
{
	ESP_LOGI(TAG, "--- Storage System Initialization ---");

#if FLASH_OR_SD_LITTLEFS

	spi_bus_config_t buscfg = {.miso_io_num = FLASH_MISO_PIN,
							   .mosi_io_num = FLASH_MOSI_PIN,
							   .sclk_io_num = FLASH_SCLK_PIN,
							   .quadwp_io_num = -1,
							   .quadhd_io_num = -1};

	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = FLASH_CLOCK_RATE_HZ, // 5 MHz
		.mode = 0,
		.spics_io_num = -1,
		.queue_size = 1,
	};

	esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "SPI bus init failed: %d", ret);

	ret = spi_bus_add_device(SPI2_HOST, &devcfg, &flash_spi_handle);
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "SPI bus init failed: %d", ret);

	if (FLASH_Init(&flash, flash_spi_handle, (uint64_t (*)(void))esp_timer_get_time,
				   FLASH_CS_PIN) == 0) {
		ESP_LOGI("FLASH", "Flash memory initialized successfully!");

		ESP_LOGI("FLASH", "Flash Geometry:");
		ESP_LOGI("FLASH", "  Blocks per device : %d", flash.geometry.blkCnt);
		ESP_LOGI("FLASH", "  Pages per block   : %d", flash.geometry.pageCnt);
		ESP_LOGI("FLASH", "  Page size         : %d bytes", flash.geometry.pageSize);
		ESP_LOGI("FLASH", "  Block size        : %lu bytes", flash.geometry.blkSize);
		ESP_LOGI("FLASH", "  Device size       : %lu bytes", flash.geometry.devSize);
		ESP_LOGI("FLASH", "  Spare area size   : %d bytes", flash.geometry.spareAreaSize);
	}

	else
		ESP_LOGE("FLASH", "Flash initialization failed!");

#else

	spi_bus_config_t buscfg = {.miso_io_num = SD_MISO_PIN,
							   .mosi_io_num = SD_MOSI_PIN,
							   .sclk_io_num = SD_SCLK_PIN,
							   .quadwp_io_num = -1,
							   .quadhd_io_num = -1};

	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = SD_CLOCK_RATE_HZ,
		.mode = 0,
		.spics_io_num = -1,
		.queue_size = 1,
	};

	spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	spi_bus_add_device(SPI2_HOST, &devcfg, &sd_spi_handle);

	if (sd_init(&sd, sd_spi_handle, SD_CS_PIN) == 0)
		ESP_LOGI("SD", "SD card initialized successfully!");
	else
		ESP_LOGE("SD", "SD card initialization failed!");

	ESP_LOGI(TAG, "SD Card Details:");

	switch (sd.info.type) {
	case SD_CARD_TYPE_V1_STANDARD_CAPACITY: {
		ESP_LOGI(TAG, "SD Card Type: SDSC (Standard Capacity SD)");
		break;
	}
	case SD_CARD_TYPE_V2_STANDARD_CAPACITY: {
		ESP_LOGI(TAG, "SD Card Type: SDSC (V2)");
		break;
	}
	case SD_CARD_TYPE_V2_HIGH_OR_EXTENDED_CAPACITY: {
		ESP_LOGI(TAG, "SD Card Type: SDHC (High Capacity, 2-32 GB) i SDXC "
					  "(eXtended Capacity, >32 GB)");
		break;
	}
	}

	ESP_LOGI(TAG, "SD Card Block Length %lu", (unsigned long)sd.info.blk_len);
	ESP_LOGI(TAG, "SD Card Block Count %lu", (unsigned long)sd.info.blk_cnt);

#endif

	return ESP_OK;
}

/**
 * @brief Create application tasks
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t create_application_tasks(void)
{
	ESP_LOGI(TAG, "--- Application Tasks Creation ---");

	uint8_t work_priority = 2;
	uint16_t work_stack_size = 8192;
	uint8_t log_priority = 1;
	uint16_t log_stack_size = 8192;

	/* Create work task */
	BaseType_t result = xTaskCreate(task_work, "work", work_stack_size, NULL,
									work_priority, &task_work_handle);
	if (result != pdPASS) {
		ESP_LOGE(TAG, "Failed to create work task");
		return ESP_ERR_NO_MEM;
	}

	/* Create logging task */
	result = xTaskCreate(task_log, "log", log_stack_size, NULL, log_priority,
						 &task_log_global);

	if (result != pdPASS) {
		ESP_LOGE(TAG, "Failed to create log task");
		return ESP_ERR_NO_MEM;
	}

	ESP_LOGI(TAG, "Application tasks created successfully");
	ESP_LOGI(TAG, "  - Work Task: Priority %d, Stack %dKB", work_priority,
			 work_stack_size / 1024);
	ESP_LOGI(TAG, "  - Log Task:  Priority %d, Stack %dKB", log_priority,
			 log_stack_size / 1024);

	return ESP_OK;
}

/**
 * @brief Initialize LittleFS filesystem
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t init_littlefs_filesystem(void)
{
	ESP_LOGI(TAG, "--- LittleFS Initialization ---");

	/* Configure LittleFS */
#if FLASH_OR_SD_LITTLEFS
	flash_lfs_init_config(&lfs_cfg, &file_cfg, &flash, read_buffer, PAGE_SIZE_FLASH,
						  prog_buffer, PAGE_SIZE_FLASH, lookahead_buffer, LOOKAHEAD_SIZE,
						  file_buffer, PAGE_SIZE_FLASH);
#else
	sd_lfs_init_config(&lfs_cfg, &file_cfg, &sd, read_buffer, PAGE_SIZE_SD, prog_buffer,
					   PAGE_SIZE_SD, lookahead_buffer, LOOKAHEAD_SIZE, file_buffer,
					   PAGE_SIZE_SD);
#endif

	/* Mount or format filesystem */
	int err = lfs_mount(&lfs, &lfs_cfg);

	if (err < 0) {
		ESP_LOGW(TAG, "Mount failed (%d), formatting filesystem", err);

		err = lfs_format(&lfs, &lfs_cfg);
		if (err < 0) {
			ESP_LOGE(TAG, "Format failed: %d", err);
			return ESP_FAIL;
		}

		err = lfs_mount(&lfs, &lfs_cfg);
		if (err < 0) {
			ESP_LOGE(TAG, "Mount after format failed: %d", err);
			return ESP_FAIL;
		}
	}

	ESP_LOGI(TAG, "LittleFS mounted successfully");
	return ESP_OK;
}

/**
 * @brief Initialize ULog logging system
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t init_ulog_system(void)
{
	ESP_LOGI(TAG, "--- ULog System Initialization ---");

	int err;

	/* Create log file using current RTC time */
	snprintf(file_name, sizeof(file_name), "log_%02u_%02u_%04u_%02u_%02u_%02u.ulg",
			 current_time.day, current_time.month, current_time.year, current_time.hour,
			 current_time.min, current_time.sec);

	ESP_LOGI(TAG, "Creating File name: %s", file_name);

	err = lfs_file_opencfg(&lfs, &lfs_file, file_name, LFS_O_WRONLY | LFS_O_CREAT,
						   &file_cfg);
	if (err < 0) {
		ESP_LOGE(TAG, "Failed to create log file: %s", file_name);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Log file created successfully");

	/* Initialize ULog */
	ulog_start(&ulog, &lfs, &lfs_file, HAL_GetTimeUS);

	/* Write system information */
	for (uint32_t i = 0; i < ULOG_INFO_COUNT; i++) {
		const ulog_info_entry_t* info = &ulog_infos[i];
		ulog_info(&ulog, info->key, strlen(info->key), info->val, strlen(info->val));
	}

	/* Write parameters */
	for (uint32_t i = 0; i < ULOG_PARAM_COUNT; i++) {
		const ulog_param_entry_t* param = &ulog_params[i];
		ulog_param(&ulog, param->key, strlen(param->key), &param->val);
	}

	/* ===================== REGISTER MESSAGE FORMATS ===================== */

	/* Generic / custom */
	ulog_format_battery_status(&ulog);

	ulog_format_lisum_power_motor_scaled_data(&ulog);
	ulog_format_lisum_gnss_recv_data(&ulog);
	ulog_format_lisum_sensor_airspeed_data(&ulog);
	ulog_format_lisum_power_hornet_act_data(&ulog);
	ulog_format_lisum_manual_ctrl_hornet(&ulog);
	ulog_format_lisum_propulsion_tb_status(&ulog);
	ulog_format_lisum_esc_status(&ulog);

	ulog_format_scaled_imu(&ulog);
	ulog_format_scaled_pressure(&ulog);

	ulog_format_altitude(&ulog);
	ulog_format_attitude(&ulog);

	ulog_format_named_value_float(&ulog);

	ulog_format_ais_vessel(&ulog);
	ulog_format_adsb_vehicle(&ulog);

	/* ===================== SUBSCRIPTIONS ===================== */

	ulog_subscribe_battery_status(&ulog, MAIN_BATTERY_STATUS_ID, &batt_status1_id);
	ulog_subscribe_battery_status(&ulog, STBY_BATTERY_STATUS_ID, &batt_status2_id);

	ulog_subscribe_lisum_power_motor_scaled_data(&ulog, 0x00, &motor_scaled_id);
	ulog_subscribe_lisum_gnss_recv_data(&ulog, INS_GNSS_DATA_ID, &lisum_gnss_data1_id);
	ulog_subscribe_lisum_gnss_recv_data(&ulog, INS_COTS_HR_GNSS_DATA_ID,
										&lisum_gnss_data2_id);
	ulog_subscribe_lisum_gnss_recv_data(&ulog, INS_COTS_LR_GNSS_DATA_ID,
										&lisum_gnss_data3_id);
	ulog_subscribe_lisum_gnss_recv_data(&ulog, LISUM_GNSS_RECV_DATA_1_0_ID,
										&uavcan_lisum_lisum_gnss_recv_data_1_0_id);

	ulog_subscribe_lisum_sensor_airspeed_data(&ulog, 0x00, &lisum_airspeed_data_id);
	ulog_subscribe_lisum_power_hornet_act_data(&ulog, 0x00, &lisum_act_data_id);
	ulog_subscribe_lisum_manual_ctrl_hornet(&ulog, 0x00, &lisum_manual_ctrl_id);

	ulog_subscribe_lisum_propulsion_tb_status(&ulog, LISUM_PROPULSION_TB_STATUS_DATA_ID,
											  &lisum_propulsion_tb_status_id);

	ulog_subscribe_lisum_esc_status(&ulog, LISUM_ESC_STATUS_DATA_ID,
									&lisum_esc_status_id);

	// ulog_subscribe_lisum_power_motor_vesc_data(&ulog, LISUM_VESC_CONTROL_DATA_ID,
	// 										   &lisum_power_motor_vesc_id);

	ulog_subscribe_scaled_imu(&ulog, INS_SCALED_IMU_ID, &scaled_imu1_id);
	ulog_subscribe_scaled_imu(&ulog, INS_COTS_SCALED_IMU_ID, &scaled_imu2_id);
	ulog_subscribe_scaled_pressure(&ulog, 0x00, &scaled_pressure_id);

	ulog_subscribe_altitude(&ulog, 0x00, &altitude_id);
	ulog_subscribe_attitude(&ulog, INS_ATTITUDE_ID, &attitude1_id);
	ulog_subscribe_attitude(&ulog, INS_COTS_ATTITUDE_ID, &attitude2_id);

	ulog_subscribe_named_value_float(&ulog, 0x00, &named_value_float_id);

	ulog_subscribe_ais_vessel(&ulog, AIS_VESSEL_ID, &ais_vessel_id);
	ulog_subscribe_adsb_vehicle(&ulog, ADSB_VEHICLE_ID, &adsb_vehicle_id);

	ESP_LOGI(TAG, "ULog system initialized successfully");
	return ESP_OK;
}
