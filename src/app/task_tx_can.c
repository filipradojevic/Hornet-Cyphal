/**
 * @file    task_tx_can.c
 * @brief   Task TX CAN - Drain queue and send CAN messages
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "main.h"
#include "task_mav.h"
#include "types.h"

/* Peripherals */
#include "gpio.h"

/* External hardware drivers */
#include "cyphal_mavlink_publishers.h"
#include "cyphal_uavcan_publishers.h"

/* Lib */
#include "can.h"
#include "canard.h"
#include "cyphal_utility.h"

/* Middleware */
#include "FreeRTOS.h"
#include "mav.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "udp_tl.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* CANARD DATA STRUCTURES */
extern struct CanardInstance canard;
extern struct CanardTxQueue tx_queue;

extern SemaphoreHandle_t tx_queue_mutex;

int drain_tx_queue_result = 0;
static uint8_t counter = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_tx_can(void *arg)
{
	for (;;) {
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));

		TX_QUEUE_MUTEX_TAKE
		{
			int32_t res;
			do {
				res = cyphal_drain_tx_queue(&canard, &tx_queue, 8);
			} while (res > 0);

			TX_QUEUE_MUTEX_GIVE;
		}
	}
}