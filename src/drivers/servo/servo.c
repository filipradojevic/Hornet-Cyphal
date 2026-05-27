/**
 * @file    servo.c
 * @brief   Servo driver
 * @version	1.0.0
 * @date    26.05.2026
 * @author  Filip Radojevic
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "servo.h"
#include "gpio.h"
#include "timer.h"
#include <stddef.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static servo_group_t* g_active_groups[4] = {NULL};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static servo_return_value_t servo_gpio_init(servo_t* s);
static void servo_dispatch(servo_group_t* g, tim_hal_ch_t channel);
static void servo_timer_callback_0(tim_hal_ch_t ch, tim_hal_int_type_t type);
static void servo_timer_callback_1(tim_hal_ch_t ch, tim_hal_int_type_t type);
static void servo_timer_callback_2(tim_hal_ch_t ch, tim_hal_int_type_t type);
static void servo_timer_callback_3(tim_hal_ch_t ch, tim_hal_int_type_t type);

/*******************************************************************************
 * Code
 ******************************************************************************/

static servo_return_value_t servo_gpio_init(servo_t* s)
{
	gpio_hal_cfg_type_t cfg = {
		.pinDir = GPIO_HAL_OUTPUT,
		.pinMode = GPIO_HAL_PINMODE_TRISTATE,
		.openDrain = GPIO_HAL_OPENDRAIN_NORMAL,
	};

	if (HAL_GPIO_Init(s->gpio_port, s->gpio_pin, &cfg) != lStatus_Success)
		return SERVO_ERROR_GPIO;

	HAL_GPIO_SetPinValue(s->gpio_port, s->gpio_pin, 0);

	s->initialized = true;

	return SERVO_OK;
}

/* ─── Timer callback ────────────────────────────────────────────────────── */

static void servo_dispatch(servo_group_t* g, tim_hal_ch_t channel)
{
	/* Period interrupt: raise all pins, arm match channels */
	if (channel == TIM_HAL_CH_0) {
		for (uint8_t i = 0; i < g->count; i++) {
			servo_t* s = g->servos[i];
			if (!s->initialized)
				continue;

			HAL_GPIO_SetPinValue(s->gpio_port, s->gpio_pin, 1);
			HAL_TIM_MatchValue(g->timer_instance, s->match_channel, s->position.pulse_width_us);
			HAL_TIM_EnableMatchInterrupt(g->timer_instance, s->match_channel);
		}
		return;
	}

	/* Match interrupt: find the servo and pull pin low */
	for (uint8_t i = 0; i < g->count; i++) {
		servo_t* s = g->servos[i];
		if (channel == s->match_channel) {
			HAL_GPIO_SetPinValue(s->gpio_port, s->gpio_pin, 0);
			HAL_TIM_DisableInterrupt(g->timer_instance, s->match_channel);
			return;
		}
	}
}

static void servo_timer_callback_0(tim_hal_ch_t ch, tim_hal_int_type_t type)
{
	if (g_active_groups[0])
		servo_dispatch(g_active_groups[0], ch);
}

static void servo_timer_callback_1(tim_hal_ch_t ch, tim_hal_int_type_t type)
{
	if (g_active_groups[1])
		servo_dispatch(g_active_groups[1], ch);
}

static void servo_timer_callback_2(tim_hal_ch_t ch, tim_hal_int_type_t type)
{
	if (g_active_groups[2])
		servo_dispatch(g_active_groups[2], ch);
}

static void servo_timer_callback_3(tim_hal_ch_t ch, tim_hal_int_type_t type)
{
	if (g_active_groups[3])
		servo_dispatch(g_active_groups[3], ch);
}

static const tim_hal_callback_t g_servo_callbacks[4] = {
	servo_timer_callback_0,
	servo_timer_callback_1,
	servo_timer_callback_2,
	servo_timer_callback_3,
};

/* ─── Public API ────────────────────────────────────────────────────────── */

servo_return_value_t ServoGroup_AddServo(servo_group_t* group, servo_t* servo)
{
	if (!group || !servo)
		return SERVO_ERROR_INVALID_ARG;
	if (group->count >= SERVO_MAX_COUNT)
		return SERVO_ERROR_FULL;

	group->servos[group->count++] = servo;
	return SERVO_OK;
}

servo_return_value_t ServoGroup_Init(servo_group_t* group, const servo_group_cfg_t* cfg)
{
	if (!group || !cfg)
		return SERVO_ERROR_INVALID_ARG;
	if (group->count == 0)
		return SERVO_ERROR_INVALID_ARG;
	if (cfg->timer_instance >= 4)
		return SERVO_ERROR_INVALID_ARG;

	group->timer_instance = cfg->timer_instance;
	group->protocol = cfg->protocol;

	/* Init GPIO for each servo */
	for (uint8_t i = 0; i < group->count; i++) {
		servo_t* s = group->servos[i];

		servo_return_value_t ret = servo_gpio_init(s);
		if (ret != SERVO_OK)
			return ret;

		if (s->position.pulse_width_us < s->position.pulse_width_min_us)
			s->position.pulse_width_us = s->position.pulse_width_min_us;
		if (s->position.pulse_width_us > s->position.pulse_width_max_us)
			s->position.pulse_width_us = s->position.pulse_width_max_us;

		s->initialized = true;
	}

	/* Configure timer: 1 us resolution */
	tim_hal_cfg_t timer_cfg = {
		.Prescale = TIM_HAL_PRESCALE_US,
		.PrescaleValue = 1,
	};
	HAL_TIM_Init(cfg->timer_instance, &timer_cfg);

	/* Period match — samo za PWM */
	if (cfg->protocol == SERVO_PROTOCOL_PWM) {
		tim_hal_match_cfg_t period_match = {
			.MatchChannel = cfg->period_channel,
			.MatchValue = cfg->period_us,
			.StopOnMatch = lFunctionalState_Disable,
			.ResetOnMatch = lFunctionalState_Enable,
		};
		HAL_TIM_MatchCfg(cfg->timer_instance, &period_match);
	}

	/* Pulse match channels: one per servo, disabled initially */
	tim_hal_match_cfg_t pulse_match = {
		.StopOnMatch = lFunctionalState_Disable,
		.ResetOnMatch = lFunctionalState_Disable,
		.MatchValue = SERVO_MIN_PULSE_WIDTH_US,
	};
	for (uint8_t i = 0; i < group->count; i++) {
		pulse_match.MatchChannel = group->servos[i]->match_channel;
		HAL_TIM_MatchCfg(cfg->timer_instance, &pulse_match);
		HAL_TIM_DisableInterrupt(cfg->timer_instance, group->servos[i]->match_channel);
	}

	/* Register callback and store active group */
	g_active_groups[cfg->timer_instance] = group;
	HAL_TIM_EnableInterrupt(cfg->timer_instance, g_servo_callbacks[cfg->timer_instance]);

	return SERVO_OK;
}

servo_return_value_t Servo_SetPulse(servo_t* servo, uint16_t pulse_width_us)
{
	if (!servo)
		return SERVO_ERROR_INVALID_ARG;
	if (!servo->initialized)
		return SERVO_ERROR_NOT_INIT;

	if (pulse_width_us < servo->position.pulse_width_min_us ||
		pulse_width_us > servo->position.pulse_width_max_us)
		return SERVO_ERROR_INVALID_ARG;

	servo->position.pulse_width_us = pulse_width_us;
	return SERVO_OK;
}

servo_return_value_t Servo_SetPercent(servo_t* servo, uint8_t percent)
{
	if (!servo)
		return SERVO_ERROR_INVALID_ARG;
	if (!servo->initialized)
		return SERVO_ERROR_NOT_INIT;
	if (percent > 100)
		return SERVO_ERROR_INVALID_ARG;

	uint16_t range = servo->position.pulse_width_max_us - servo->position.pulse_width_min_us;

	servo->position.pulse_width_us =
		servo->position.pulse_width_min_us + (uint16_t)((range * percent) / 100U);

	return SERVO_OK;
}

servo_return_value_t Servo_GetPercent(const servo_t* servo, uint8_t* out_percent)
{
	if (!servo || !out_percent)
		return SERVO_ERROR_INVALID_ARG;
	if (!servo->initialized)
		return SERVO_ERROR_NOT_INIT;

	uint16_t range = servo->position.pulse_width_max_us - servo->position.pulse_width_min_us;

	if (range == 0) {
		*out_percent = 0;
		return SERVO_OK;
	}

	uint16_t offset = servo->position.pulse_width_us - servo->position.pulse_width_min_us;

	*out_percent = (uint8_t)((offset * 100U) / range);
	return SERVO_OK;
}

servo_return_value_t ServoGroup_SendOneshot(servo_group_t* group)
{
	if (!group)
		return SERVO_ERROR_INVALID_ARG;
	if (group->protocol == SERVO_PROTOCOL_PWM)
		return SERVO_ERROR_INVALID_ARG;

	for (uint8_t i = 0; i < group->count; i++) {
		servo_t* s = group->servos[i];
		if (!s->initialized)
			continue;

		HAL_GPIO_SetPinValue(s->gpio_port, s->gpio_pin, 1);
		HAL_TIM_MatchValue(group->timer_instance, s->match_channel, s->position.pulse_width_us);
		HAL_TIM_EnableMatchInterrupt(group->timer_instance, s->match_channel);
	}

	return SERVO_OK;
}