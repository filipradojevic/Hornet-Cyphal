/**
 * @file    types.h
 * @brief   Project common defines and types.
 * @version 1.0.0
 * @date    24.04.2025
 * @author  BetaTehPro
 */

#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Driver */
#include "driver/rmt.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"

/* External hardware drivers */
#include "cyphal_mavlink_publishers.h"
#include "cyphal_reg_publishers.h"
#include "cyphal_uavcan_publishers.h"

/* Lib */
#include "canard.h"

/* Middleware*/
#include "cyphal_utility.h"

/* Cyphal COMMON */
#include "common/AdsbVehicle_1_0.h"
#include "common/AisVessel_1_0.h"
#include "common/Attitude_1_0.h"
#include "common/AutopilotVersion_1_0.h"
#include "common/FileTransferProtocol_1_0.h"
#include "common/NamedValueFloat_1_0.h"
#include "common/ScaledImu_1_0.h"

/* Cyphal LISUM */
#include "lisum/LisumGnssRecvData_1_0.h"
#include "lisum/LisumPowerMotorScaledData_1_0.h"

/* Cyphal UAVCAN */
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/primitive/array/Integer8_1_0.h"
#include "uavcan/si/sample/length/Scalar_1_0.h"
#include "uavcan/si/unit/duration/Scalar_1_0.h"
#include "uavcan/si/unit/pressure/Scalar_1_0.h"
#include "uavcan/si/unit/temperature/Scalar_1_0.h"

/* Cyphal REG */
#include "reg/udral/physics/thermodynamics/PressureTempVarTs_0_1.h"

#include "uavcan/primitive/scalar/Integer64_1_0.h"	 // all flags GNSS message
#include "uavcan/si/sample/angle/Scalar_1_0.h"		 // heading/heading_rtk GNSS message
#include "uavcan/si/sample/length/WideVector3_1_0.h" // Lat, Lon, Alt GNSS message
#include "uavcan/si/sample/velocity/Vector3_1_0.h"	 // vel_n/d/e GNSS message

#include "uavcan/primitive/scalar/Integer16_1_0.h" // ID AND FLAG ANPP message
#include "uavcan/si/unit/pressure/Scalar_1_0.h"	   // raw_press ANPP message
#include "uavcan/si/unit/temperature/Scalar_1_0.h" // temperature ANPP message
#include "uavcan/si/unit/velocity/Scalar_1_0.h"	   // Airspeed ANPP message

#include "uavcan/si/sample/angular_velocity/Vector3_1_0.h" // Roll, Pitch, Yaw speed Fusion message
#include "uavcan/si/unit/angle/Quaternion_1_0.h"		   // roll/pich/yaw time_boot Fusion message

#include "uavcan/si/sample/magnetic_field_strength/Vector3_1_0.h" // x,y,z mag IMU message
#include "uavcan/si/unit/angular_acceleration/Vector3_1_0.h"	  // x,y,z acc IMU message
#include "uavcan/si/unit/angular_velocity/Vector3_1_0.h"		  // x,y,z gyro IMU message
#include "uavcan/si/unit/temperature/Scalar_1_0.h"				  // temperature IMU message
#include "common/ComponentInformationBasic_1_0.h"

	/*******************************************************************************
	 * Defines
	 ******************************************************************************/

#define WS2812_GPIO 21
#define RMT_CHANNEL RMT_CHANNEL_0
#define LED_NUM 1

/* WS2812 timing (80 MHz APB clock → 12.5 ns tick) */
#define T0H 14 // 0.35us
#define T0L 34 // 0.85us
#define T1H 34 // 0.85us
#define T1L 14 // 0.35us

/* General */
#define QUEUE_NUMBERS 17

	/* Autopilot Version */

#define OS_CUSTOM_VERSION                                               \
	OS_CUSTOM_HASH		 /**< From src/middleware/FreeRTOS/manifest.yml \
						  */
#define BOARD_TYPE 20	 /**< Ethernet port marked by marker */
#define BOARD_REVISION 2 /**< Hardware Version V0.2 */
#define VENDOR_ID 0x4254 /**< 'B' 'T' - Company Identifier */
#define PRODUCT_ID 20	 /**< Model Number */

	/* Component information basic */

#define VENDOR_NAME "BetaTehPro"			   /**< Company name */
#define MODEL_NAME "HW_BLACK_BOX"			   /**< Model name */
#define SOFTWARE_VERSION FLIGHT_CUSTOM_VERSION /**< Software version */
#define HARDWARE_VERSION "DEV-Board V0.2"	   /**< Hardware version */
#define SERIAL_NUMBER "S_N: 020"			   /**< Serial number */
#define PAYLOAD_LENGTH 8					   /**< Length of custom version strings */

	/* Blinky Led */

#define BLINKY_PERIOD_MS 500	 /**< LED green blinky period [ms] */
#define LED_RGB_GPIO GPIO_NUM_21 /**< GPIO number for blinky LED */

	/* Flash configuration */

#define FLASH_CLOCK_RATE_HZ 2000000 /**< Flash clock (5 MHz) */
#define FLASH_INIT_MAX_RETRY_CNT 32 /**< Max retries during init */
#define FLASH_SPI_HOST SPI2_HOST	/**< SPI host */
#define FLASH_MISO_PIN GPIO_NUM_3	/**< FLASH_MISO pin */
#define FLASH_MOSI_PIN GPIO_NUM_0	/**< FLASH_MOSI pin */
#define FLASH_SCLK_PIN GPIO_NUM_2	/**< FLASH_SCLK pin */
#define FLASH_CS_PIN GPIO_NUM_15	/**< FLASH_CS pin */

	/* SD card configuration */

#define SD_CLOCK_RATE_HZ (10 * 1000 * 1000) /**< SD clock (10 MHz) */
#define SD_INIT_MAX_RETRY_CNT 100			/**< Max retries during init */
#define SD_SPI_HOST SPI2_HOST				/**< HSPI */
#define SD_MISO_PIN GPIO_NUM_5				/**< SD_MISO pin */
#define SD_MOSI_PIN GPIO_NUM_6				/**< SD_MOSI pin */
#define SD_SCLK_PIN GPIO_NUM_7				/**< SD_SCLK pin */
#define SD_CS_PIN GPIO_NUM_4				/**< SD_CS pin */

/* Ethernet configuration */
#define W5500_ETH_SPI_HOST SPI3_HOST
#define W5500_ETH_MOSI GPIO_NUM_11
#define W5500_ETH_MISO GPIO_NUM_12
#define W5500_ETH_CLK GPIO_NUM_13
#define W5500_ETH_CS GPIO_NUM_14
#define W5500_ETH_INT GPIO_NUM_10
#define W5500_ETH_RST GPIO_NUM_9
#define TAG "MAIN"
#define RX_BUFFER_SIZE 2048

	/* PSRAM configuration */

	/* Choose between FLASH & SD card for LittleFS storage */

#define FLASH_OR_SD_LITTLEFS 0 /**< 1 is for Flash, 0 is for SD Card */
#define MAVLINK_OR_CYPHAL 0	   /**< 1 is for MAVLink UDP, 0 is for Cyphal CAN*/

	/*******************************************************************************
	 * Typedefs
	 ******************************************************************************/

	typedef struct cyphal_subscription_messages_t
	{
		const enum CanardTransferKind kind;
		const CanardPortID port_id;
		const size_t serialization_buffer_size;
		struct CanardRxSubscription *subscription;
	} cyphal_subscription_messages_t;

	typedef enum diode_color_t
	{
		DIODE_COLOR_GREEN = 0,
		DIODE_COLOR_RED = 1,
		DIODE_COLOR_BLUE = 2,
		DIODE_COLOR_ORANGE = 3,
	} diode_color_t;

	/*******************************************************************************
	 * Variables
	 ******************************************************************************/
	extern rmt_item32_t led_data[24 * LED_NUM];

	/*******************************************************************************
	 * API
	 ******************************************************************************/

	static inline uint64_t HAL_GetTimeUS(void)
	{
		return (uint32_t)(esp_timer_get_time());
	}

	static inline void ws2812_set_pixel(uint8_t r, uint8_t g, uint8_t b)
	{
		uint32_t color = (g << 16) | (r << 8) | b; // GRB format

		for (int i = 0; i < 24; i++)
		{
			if (color & (1 << (23 - i)))
			{
				led_data[i].level0 = 1;
				led_data[i].duration0 = T1H;
				led_data[i].level1 = 0;
				led_data[i].duration1 = T1L;
			}
			else
			{
				led_data[i].level0 = 1;
				led_data[i].duration0 = T0H;
				led_data[i].level1 = 0;
				led_data[i].duration1 = T0L;
			}
		}
	}

	static inline void ws2812_show(void)
	{
		rmt_write_items(RMT_CHANNEL, led_data, 24, true);
		rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
	}

	static inline void Diode_Toggle(uint64_t *counter, uint8_t times, bool *toggle,
									diode_color_t color)
	{
		*counter += 1;

		if (*counter == times)
		{

			if (*toggle)
			{
				if (color == DIODE_COLOR_GREEN)
				{
					ws2812_set_pixel(0, 255, 0);
					*toggle = !(*toggle);
				}
				else if (color == DIODE_COLOR_RED)
				{
					ws2812_set_pixel(255, 0, 0);
					*toggle = !(*toggle);
				}
				else if (color == DIODE_COLOR_BLUE)
				{
					ws2812_set_pixel(0, 0, 255);
					*toggle = !(*toggle);
				}
				else if (color == DIODE_COLOR_ORANGE)
				{
					ws2812_set_pixel(255, 165, 0);
					*toggle = !(*toggle);
				}
			}
			else
			{
				ws2812_set_pixel(0, 0, 0);
				*toggle = !(*toggle);
			}

			ws2812_show();
			*counter = 0;
		}
	}

	esp_err_t init_littlefs_filesystem(void);
	esp_err_t init_ulog_system(void);

#ifdef __cplusplus
}
#endif

#endif /* TYPES_H */