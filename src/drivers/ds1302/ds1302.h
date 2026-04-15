/**
 * @file    ds1302.h
 * @brief   DS1302 driver definitions.
 * @version	1.0.0
 * @date    17.12.2025
 * @author  BetaTehPro
 */

#ifndef DS1302_H
#define DS1302_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "driver/gpio.h"
#include "ds1302_def.h"
#include <stdint.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef struct {
	uint8_t rst_port; /* GPIO port for RST */
	uint8_t rst_pin;  /* GPIO pin for RST */
	uint8_t clk_port; /* GPIO port for CLK */
	uint8_t clk_pin;  /* GPIO pin for CLK */
	uint8_t io_port;  /* GPIO port for IO */
	uint8_t io_pin;	  /* GPIO pin for IO */
} ds1302_pins_t;

typedef struct {
	ds1302_pins_t interface;
} ds1302_t;

typedef struct {
	uint8_t sec;   // 0–59
	uint8_t min;   // 0–59
	uint8_t hour;  // 0–23
	uint8_t day;   // 1–31
	uint8_t month; // 1–12
	uint16_t year; // fe. 2025
} ds1302_time_t;

typedef enum {
	DS1302_OK = 0,
	DS1302_ERROR_GPIO,
	DS1302_ERROR_INVALID_PARAM,
} ds1302_status_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

ds1302_status_t ds1302_init(ds1302_t* ctx);
ds1302_status_t ds1302_set_time(ds1302_t* ctx, const ds1302_time_t* t);
ds1302_status_t ds1302_get_time(ds1302_t* ctx, ds1302_time_t* t);

#ifdef __cplusplus
}
#endif

#endif /* DS1302_H */