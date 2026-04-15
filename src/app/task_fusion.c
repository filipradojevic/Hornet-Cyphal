/**
 * @file    task_fusion.c
 * @brief   Task fusion - fusion algorithm
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_fusion.h"
#include "types.h"

/* Peripherals */

/* External hardware drivers */

/* Lib */
#include "Fusion.h"

/* Middleware */
#include "FreeRTOS.h"
#include "queue.h"

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

extern QueueHandle_t mailbox_fusion_head;
extern QueueHandle_t queue_mav_attitude;
extern QueueHandle_t queue_fusion_data;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_fusion(void *arg)
{
	TickType_t time_sent = xTaskGetTickCount();

#if !MAVLINK_OR_CYPHAL

	messages_cyphal_uavcan_common_Attitude_1_0 attitude = {0};

#else

	mavlink_attitude_t attitude = {0};

#endif /* MAVLINK_OR_CYPHAL */

	fusion_data_t sample;
	FusionAhrs fusion;
	FusionOffset offset;

	FusionAhrsInitialise(&fusion);
	FusionOffsetInitialise(&offset, 200);

	/* set fusion algorithm settings */
	const FusionAhrsSettings settings = {
		.convention = FusionConventionNed,
		.gain = 0.5f,
		.gyroscopeRange = 500.0f,
		.accelerationRejection = 2.0f,
		.magneticRejection = 2.0f,
		.recoveryTriggerPeriod = 10 * 200, /* 10 seconds */
	};

	FusionAhrsSetSettings(&fusion, &settings);

	for (;;) {
		xQueueReceive(queue_fusion_data, &sample, portMAX_DELAY);

		FusionVector accel = FUSION_VECTOR_ZERO;
		FusionVector gyro = FUSION_VECTOR_ZERO;

		accel.axis.x = sample.accel[0];
		accel.axis.y = sample.accel[1];
		accel.axis.z = sample.accel[2];

		gyro.axis.x = sample.gyro[0];
		gyro.axis.y = sample.gyro[1];
		gyro.axis.z = sample.gyro[2];

		gyro = FusionOffsetUpdate(&offset, gyro);

		/* apply heading correction if it exists */
		float head_corr;
		if (pdPASS == xQueueReceive(mailbox_fusion_head, &head_corr, 0))
			FusionAhrsSetHeading(&fusion, head_corr);
		else
			FusionAhrsUpdateNoMagnetometer(&fusion, gyro, accel, sample.dt);

		/* pack attitude */
		if (xTaskGetTickCount() - time_sent >= ATTITUDE_PERIOD_MS) {
			time_sent = xTaskGetTickCount();

			FusionEuler euler = {0};

			euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusion));

			attitude.time_boot_ms = xTaskGetTickCount();
			attitude.roll = -FusionDegreesToRadians(euler.angle.roll);
			attitude.rollspeed = -FusionDegreesToRadians(gyro.axis.x);
			attitude.pitch = -FusionDegreesToRadians(euler.angle.pitch);
			attitude.pitchspeed = -FusionDegreesToRadians(gyro.axis.y);
			attitude.yaw = FusionDegreesToRadians(euler.angle.yaw);
			attitude.yawspeed = FusionDegreesToRadians(gyro.axis.z);

			xQueueSendToBack(queue_mav_attitude, &attitude, 0);
		}
	}
}
