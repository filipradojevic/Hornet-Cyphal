/**
 * @file    ds1302.c
 * @brief   DS1302 driver implementation.
 * @version	1.0.0
 * @date    17.12.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "ds1302.h"
#include <driver/gpio.h>
#include <esp_timer.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define RST_H(ctx) gpio_set_level((ctx)->interface.rst_pin, 1)
#define RST_L(ctx) gpio_set_level((ctx)->interface.rst_pin, 0)

#define CLK_H(ctx) gpio_set_level((ctx)->interface.clk_pin, 1)
#define CLK_L(ctx) gpio_set_level((ctx)->interface.clk_pin, 0)

#define IO_H(ctx) gpio_set_level((ctx)->interface.io_pin, 1)
#define IO_L(ctx) gpio_set_level((ctx)->interface.io_pin, 0)
#define IO_R(ctx) gpio_get_level((ctx)->interface.io_pin)

#define IO_OUT(ctx) ds1302_io_output(ctx)
#define IO_IN(ctx) ds1302_io_input(ctx)

/* Busy-wait delay using HAL_GetTimeUS() to avoid depending on ROM helper
   and to allow using the project's time wrapper. Delay value is in us. */
#define DS1302_DELAY_US(us)                                                              \
	do {                                                                                 \
		uint64_t __ds1302_delay_t0 = esp_timer_get_time();                               \
		while ((esp_timer_get_time() - __ds1302_delay_t0) < (uint64_t)(us)) {            \
			;                                                                            \
		}                                                                                \
	} while (0)

#define DS1302_DELAY() DS1302_DELAY_US(5) /* ~5 us delay between clock edges */

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* ================= UTILS ================= */
static uint8_t dec_to_bcd(uint8_t v)
{
	return ((v / 10) << 4) | (v % 10);
}
static uint8_t bcd_to_dec(uint8_t v)
{
	return ((v >> 4) * 10) + (v & 0x0F);
}

/* ================= LOW LEVEL ================= */
static void ds1302_io_output(ds1302_t* ctx)
{
	gpio_set_direction(ctx->interface.io_pin, GPIO_MODE_OUTPUT);
}

static void ds1302_io_input(ds1302_t* ctx)
{
	gpio_set_direction(ctx->interface.io_pin, GPIO_MODE_INPUT);
}

static void ds1302_write_byte(ds1302_t* ctx, uint8_t data)
{
	IO_OUT(ctx);
	for (int i = 0; i < 8; i++) {
		(data & 0x01) ? IO_H(ctx) : IO_L(ctx);
		DS1302_DELAY();
		CLK_H(ctx);
		DS1302_DELAY();
		CLK_L(ctx);
		data >>= 1;
	}
}

static uint8_t ds1302_read_byte(ds1302_t* ctx)
{
	uint8_t data = 0;
	IO_IN(ctx);
	for (int i = 0; i < 8; i++) {
		data >>= 1;
		if (IO_R(ctx))
			data |= 0x80;
		CLK_H(ctx);
		DS1302_DELAY();
		CLK_L(ctx);
		DS1302_DELAY();
	}
	return data;
}

static void ds1302_write_reg(ds1302_t* ctx, uint8_t reg, uint8_t val)
{
	RST_H(ctx);
	ds1302_write_byte(ctx, reg);
	ds1302_write_byte(ctx, val);
	RST_L(ctx);
}

static uint8_t ds1302_read_reg(ds1302_t* ctx, uint8_t reg)
{
	uint8_t v;
	RST_H(ctx);
	DS1302_DELAY();
	ds1302_write_byte(ctx, reg | 0x01);
	v = ds1302_read_byte(ctx);
	RST_L(ctx);
	return v;
}

/* ================= API ================= */
ds1302_status_t ds1302_init(ds1302_t* ctx)
{
	if (!ctx)
		return DS1302_ERROR_INVALID_PARAM;

	gpio_config_t cfg = {.pin_bit_mask = (1ULL << ctx->interface.rst_pin) |
										 (1ULL << ctx->interface.clk_pin),
						 .mode = GPIO_MODE_OUTPUT,
						 .pull_up_en = GPIO_PULLUP_DISABLE,
						 .pull_down_en = GPIO_PULLDOWN_DISABLE,
						 .intr_type = GPIO_INTR_DISABLE};
	gpio_config(&cfg);

	IO_OUT(ctx);
	RST_L(ctx);
	CLK_L(ctx);

	/* Disable write protect */
	ds1302_write_reg(ctx, DS1302_WP, 0x00);

	return DS1302_OK;
}

ds1302_status_t ds1302_set_time(ds1302_t* ctx, const ds1302_time_t* t)
{
	if (!ctx || !t)
		return DS1302_ERROR_INVALID_PARAM;

	ds1302_write_reg(ctx, DS1302_WP, 0x00);
	DS1302_DELAY();

	/* STOP clock */
	ds1302_write_reg(ctx, DS1302_SEC, 0x80);
	DS1302_DELAY();
	ds1302_write_reg(ctx, DS1302_MIN, dec_to_bcd(t->min));
	DS1302_DELAY();
	ds1302_write_reg(ctx, DS1302_HOUR, dec_to_bcd(t->hour));
	DS1302_DELAY();
	ds1302_write_reg(ctx, DS1302_DATE, dec_to_bcd(t->day));
	DS1302_DELAY();
	ds1302_write_reg(ctx, DS1302_MONTH, dec_to_bcd(t->month));
	DS1302_DELAY();
	ds1302_write_reg(ctx, DS1302_YEAR, dec_to_bcd(t->year % 100));
	DS1302_DELAY();

	/* START clock */
	ds1302_write_reg(ctx, DS1302_SEC, dec_to_bcd(t->sec) & 0x7F);
	DS1302_DELAY();

	return DS1302_OK;
}

ds1302_status_t ds1302_get_time(ds1302_t* ctx, ds1302_time_t* t)
{
	if (!ctx || !t)
		return DS1302_ERROR_INVALID_PARAM;

	t->sec = bcd_to_dec(ds1302_read_reg(ctx, DS1302_SEC) & 0x7F);
	t->min = bcd_to_dec(ds1302_read_reg(ctx, DS1302_MIN));
	t->hour = bcd_to_dec(ds1302_read_reg(ctx, DS1302_HOUR));
	t->day = bcd_to_dec(ds1302_read_reg(ctx, DS1302_DATE));
	t->month = bcd_to_dec(ds1302_read_reg(ctx, DS1302_MONTH));
	t->year = 2000 + bcd_to_dec(ds1302_read_reg(ctx, DS1302_YEAR));

	return DS1302_OK;
}