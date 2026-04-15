/**
 * @file    task_gnss.c
 * @brief   Task GNSS - process GNSS data
 * @version 1.0.0
 * @date    09.04.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#include "task_gnss.h"
#include "types.h"

/* Peripherals */
#include "uart.h"

/* External hardware drivers */
#include "ubx_parser.h"

/* Lib */

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

typedef struct gnss_data_t {
	uint8_t fixtype;
	uint8_t carr_sol;
	uint8_t relposheading_valid;
	uint8_t numsv;
	double headmot;
	double relposheading;
	float relposheading_acc;
	double lon;
	double lat;
	double hmsl;
	double gspeed;
	double veld;
} gnss_data_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern QueueHandle_t mailbox_fusion_head;
extern QueueHandle_t queue_mav_lisum_gnss_data;
extern QueueHandle_t queue_gnss;

ubx_t ubx;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void uart_cb(uart_hal_instance_t instance, uart_hal_handle_t *handle,
					lStatus_t status, void *userData);

/*******************************************************************************
 * Code
 ******************************************************************************/

void task_gnss(void *arg)
{
	TickType_t head_corr_sent_ms = xTaskGetTickCount();

#if !MAVLINK_OR_CYPHAL

	messages_cyphal_uavcan_lisum_LisumGnssRecvData_1_0 gnss_data = {0};

#else

	mavlink_lisum_gnss_recv_data_t gnss_data = {0};

#endif /* MAVLINK_OR_CYPHAL */

	uart_hal_handle_t uart_handle;
	uint8_t process_byte;
	uint8_t receive_byte;
	int32_t ret = 0;
	gnss_data_t gnss_recv_data = {0};
	uint32_t recv_status = 0x00;
	uint32_t itow = 0;

	/* create new transfer handle */
	HAL_UART_TransferCreateHandle(GNSS_UART_INSTANCE, &uart_handle, uart_cb,
								  &queue_gnss);

	/* receive data from UART via ISR */
	HAL_UART_ReceiveISR(GNSS_UART_INSTANCE, &uart_handle, &receive_byte, 1);

	/* initialize UBX */
	ubx_init(&ubx);

	for (;;) {
		/* wait to receive data */
		xQueueReceive(queue_gnss, &process_byte, portMAX_DELAY);

		/* parse data */
		ret = ubx_parse(&ubx, process_byte);
		if (ret != 0)
			continue;

		/* process only packets from NAV class */
		if (ubx.packet_class != UBX_PACKET_CLASS_NAV)
			continue;

		if (ubx.packet_id == UBX_PACKET_ID_NAV_PVT) {
			/* process UBX Navigation position velocity time solution. */
			ubx_nav_pvt_t pvt;

			ubx_nav_pvt_decode(&ubx, &pvt);

			gnss_recv_data.headmot = pvt.headmot * 1e-5;
			gnss_recv_data.gspeed = pvt.gspeed * 1e-3;
			gnss_recv_data.veld = pvt.veld * 1e-3;
			gnss_recv_data.fixtype = pvt.fixtype;
			gnss_recv_data.carr_sol = (pvt.flags & 0xC0) >> 6;
			gnss_recv_data.numsv = pvt.numsv;

			if (itow != pvt.itow) {
				itow = pvt.itow;
				recv_status = 0x00;
			}

			recv_status |= 0x01;
		} else if (ubx.packet_id == UBX_PACKET_ID_NAV_HPPOSLLH) {
			/* process UBX High precision geodetic position solution. */
			ubx_nav_hpposllh_t hpposllh;

			ubx_nav_hpposllh_decode(&ubx, &hpposllh);

			int64_t tmp;

			tmp = (int64_t)hpposllh.lon * 100 + hpposllh.lonhp;
			gnss_recv_data.lon = tmp * 1e-9;
			tmp = (int64_t)hpposllh.lat * 100 + hpposllh.lathp;
			gnss_recv_data.lat = tmp * 1e-9;
			tmp = (int64_t)hpposllh.hmsl * 10 + hpposllh.hmslhp;
			gnss_recv_data.hmsl = tmp * 1e-4;

			if (itow != hpposllh.itow) {
				itow = hpposllh.itow;
				recv_status = 0x00;
			}

			recv_status |= 0x02;
		} else if (ubx.packet_id == UBX_PACKET_ID_NAV_RELPOSNED) {
			/* process UBX Relative positioning information in NED frame. */
			ubx_nav_relposned_t relposned;

			ubx_nav_relposned_decode(&ubx, &relposned);

			gnss_recv_data.relposheading = relposned.relposheading * 1e-5;
			gnss_recv_data.relposheading_valid =
				(relposned.flags & 0x0100) >> 8;
			gnss_recv_data.relposheading_acc = relposned.accheading * 1e-5;

			if (itow != relposned.itow) {
				itow = relposned.itow;
				recv_status = 0x00;
			}

			recv_status |= 0x04;
		}

		if (recv_status == 0x07) {
			/* all expected data is received */
			recv_status = 0x00;

			/* send gnss receiver data */

			gnss_data.dev_id = LISUM_GNSS_RECV_INTERNAL;
			gnss_data.lon = gnss_recv_data.lon;
			gnss_data.lat = gnss_recv_data.lat;
			gnss_data.alt = gnss_recv_data.hmsl;

			if (gnss_recv_data.headmot > 180.0)
				gnss_data.head = -(360.0 - gnss_recv_data.headmot);
			else
				gnss_data.head = gnss_recv_data.headmot;

			if (gnss_recv_data.relposheading > 180.0)
				gnss_data.head_rtk = -(360.0 - gnss_recv_data.relposheading);
			else
				gnss_data.head_rtk = gnss_recv_data.relposheading;

			gnss_data.vel_ne = gnss_recv_data.gspeed;
			gnss_data.vel_d = gnss_recv_data.veld;
			gnss_data.fix_type = gnss_recv_data.fixtype;
			gnss_data.carr_sol = gnss_recv_data.carr_sol;
			gnss_data.rtk_head_valid = gnss_recv_data.relposheading_valid;
			gnss_data.sv_num = gnss_recv_data.numsv;

			xQueueSendToBack(queue_mav_lisum_gnss_data, &gnss_data, 0);

			/* send rtk heading corrections */
			if (xTaskGetTickCount() - head_corr_sent_ms >= 1000 &&
				gnss_recv_data.carr_sol == LISUM_GNSS_RECV_CARR_SOL_FIXED &&
				gnss_recv_data.relposheading_valid &&
				gnss_recv_data.relposheading_acc < 1.0f) {
				head_corr_sent_ms = xTaskGetTickCount();
				float head = -gnss_data.head_rtk;
				xQueueOverwrite(mailbox_fusion_head, &head);
			}
		}
	}
}

static void uart_cb(uart_hal_instance_t instance, uart_hal_handle_t *handle,
					lStatus_t status, void *userData)
{
	BaseType_t taskWoken = pdFALSE;
	QueueHandle_t *queue = (QueueHandle_t *)userData;

	if (handle->rxDataSize && status == lStatus_Success) {
		/* send received byte to the back of the queue */
		xQueueSendToBackFromISR(*queue, handle->rxData, &taskWoken);
		/* receive another byte via interrupt */
		HAL_UART_ReceiveISR(instance, handle, handle->rxData, 1);
	}

	portYIELD_FROM_ISR(taskWoken);
}
