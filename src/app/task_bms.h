/**
 * @file    task_bms.h
 * @brief   Task BMS - process BMS status
 * @version 1.0.0
 * @date    07.04.2025
 * @author  LisumLab
 */

#ifndef TASK_BMS_H
#define TASK_BMS_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "uart.h"

#include "FreeRTOS.h"
#include "queue.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief Task BMS arguments */
typedef struct task_bms_arg_t {
	uart_hal_instance_t instance; //!< UART instance
	QueueHandle_t *rx_queue;	  //!< battery status receive queue
	QueueHandle_t *tx_queue;	  //!< battery status transmit queue
} task_bms_arg_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

void task_bms(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* TASK_BMS_H */