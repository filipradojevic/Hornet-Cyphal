/**
 * @file    servo.h
 * @brief   Servo driver common types and definitions
 * @version	1.0.0
 * @date    26.05.2026
 * @author  Filip Radojevic
 */

#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "gpio.h"
#include "timer.h"
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define SERVO_MAX_COUNT 8
#define SERVO_PERIOD_US 20000U
#define SERVO_MIN_PULSE_WIDTH_US 1000U
#define SERVO_MAX_PULSE_WIDTH_US 2000U
#define SERVO_MID_PULSE_WIDTH_US 1500U

/* Oneshot125 */
#define SERVO_ONESHOT125_MIN_US 125U
#define SERVO_ONESHOT125_MAX_US 250U

/* Oneshot42 */
#define SERVO_ONESHOT42_MIN_US 42U
#define SERVO_ONESHOT42_MAX_US 84U

#define SERVO_0_PORT GPIO_HAL_INSTANCE_1
#define SERVO_0_PIN 27

#define SERVO_1_PORT GPIO_HAL_INSTANCE_1
#define SERVO_1_PIN 28

#define SERVO_2_PORT GPIO_HAL_INSTANCE_1
#define SERVO_2_PIN 29

#define SERVO_TIMER_INSTANCE TIM_HAL_INSTANCE_0
#define SERVO_ALL_MATCH_CHANNEL TIM_HAL_CH_0
#define SERVO_0_MATCH_CHANNEL TIM_HAL_CH_1
#define SERVO_1_MATCH_CHANNEL TIM_HAL_CH_2
#define SERVO_2_MATCH_CHANNEL TIM_HAL_CH_3

#define SERVO_TIMER_INSTANCE_2 TIM_HAL_INSTANCE_2
#define SERVO_ALL_MATCH_CHANNEL_2 TIM_HAL_CH_0
#define SERVO_0_MATCH_CHANNEL_2 TIM_HAL_CH_1
#define SERVO_1_MATCH_CHANNEL_2 TIM_HAL_CH_2
#define SERVO_2_MATCH_CHANNEL_2 TIM_HAL_CH_3

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef enum {
	SERVO_OK = 0,
	SERVO_ERROR = -1,
	SERVO_ERROR_INVALID_ARG = -2,
	SERVO_ERROR_GPIO = -3,
	SERVO_ERROR_NOT_INIT = -4,
	SERVO_ERROR_FULL = -5,
} servo_return_value_t;

typedef enum {
	SERVO_PROTOCOL_PWM = 0,		   /* Standardni 50Hz PWM       */
	SERVO_PROTOCOL_ONESHOT125 = 1, /* Oneshot125 (125–250 µs)   */
	SERVO_PROTOCOL_ONESHOT42 = 2,  /* Oneshot42  (42–84 µs)     */
} servo_protocol_t;

typedef struct {
	uint16_t pulse_width_max_us;
	uint16_t pulse_width_min_us;
	uint16_t pulse_width_us;
} servo_position_t;

typedef struct {
	gpio_hal_instance_t gpio_port;
	uint8_t gpio_pin;
	tim_hal_ch_t match_channel;
	servo_position_t position;
	bool initialized;
} servo_t;

typedef struct {
	servo_t* servos[SERVO_MAX_COUNT];
	uint8_t count;
	tim_hal_instance_t timer_instance;
	servo_protocol_t protocol;
} servo_group_t;

typedef struct {
	tim_hal_instance_t timer_instance;
	uint16_t period_us;
	tim_hal_ch_t period_channel;
	servo_protocol_t protocol;
} servo_group_cfg_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/**
 * @brief  Initialize a servo group (timer + all registered servos).
 */
servo_return_value_t ServoGroup_Init(servo_group_t* group,
									 const servo_group_cfg_t* cfg);

/**
 * @brief  Register a servo into the group before calling ServoGroup_Init.
 *         Fill in gpio_port, gpio_pin, match_channel, and position limits
 *         before calling this.
 */
servo_return_value_t ServoGroup_AddServo(servo_group_t* group, servo_t* servo);

/**
 * @brief  Set servo position by pulse width in microseconds.
 */
servo_return_value_t Servo_SetPulse(servo_t* servo, uint16_t pulse_width_us);

/**
 * @brief  Set servo position as a percentage [0–100].
 */
servo_return_value_t Servo_SetPercent(servo_t* servo, uint8_t percent);

/**
 * @brief  Get current position as a percentage [0–100].
 */
servo_return_value_t Servo_GetPercent(const servo_t* servo,
									  uint8_t* out_percent);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */