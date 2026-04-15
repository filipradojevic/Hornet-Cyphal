/**
 * @file    cyphal_utility.c
 * @brief   Cyphal utility functions and definitions
 * @version 1.0.0
 * @date    20.11.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "cyphal_utility.h"
// #include "can.h"
// #include "timer.h"
#include "driver/twai.h"
#include "types.h"
#include <stdbool.h>
#include <string.h>

/*******************************************************************************
 * Private Prototypes
 ******************************************************************************/

static void* instance_allocate(void* const user_reference, const size_t size);
static void instance_deallocate(void* const user_reference, const size_t size,
								void* const pointer);
static void* txq_allocate(void* const user_reference, const size_t size);
static void txq_deallocate(void* const user_reference, const size_t size,
						   void* const pointer);

/*******************************************************************************
 * Global Cyphal Memory Resources
 ******************************************************************************/

const struct CanardMemoryResource cyphal_instance_memory = {
	.user_reference = NULL,
	.allocate = instance_allocate,
	.deallocate = instance_deallocate,
};

const struct CanardMemoryResource cyphal_txq_memory = {
	.user_reference = NULL,
	.allocate = txq_allocate,
	.deallocate = txq_deallocate,
};

/* Default list of known nodes */
const uint8_t CYPHAL_NODES_ID_ARRAY[CYPHAL_NODES_IDX_MAX] = {
	CYPHAL_INS_ID,	  CYPHAL_BLACK_BOX_ID,		  CYPHAL_GW_GND_ID,
	CYPHAL_GW_SKY_ID, CYPHAL_POWER_MANAGEMENT_ID, CYPHAL_ACTUATOR_MASTER_ID};

static uint8_t counter = 0;

/*******************************************************************************
 * Memory Pools (AHB SRAM)
 ******************************************************************************/

/* --- Instance Pool (RX sessions, payloads, etc.) --- */
static uint8_t cyphal_instance_pool[CYPAHL_INSTANCE_BLOCK_CAPACITY]
								   [CYPAHL_INSTANCE_BLOCK_SIZE];

static uint16_t instance_free_stack[CYPAHL_INSTANCE_BLOCK_CAPACITY];
static uint16_t instance_free_top;

/* --- TX Queue Pool (CAN frames) --- */
static uint8_t cyphal_txq_pool[CYPAHL_TXQ_BLOCK_CAPACITY][CYPAHL_TXQ_BLOCK_SIZE];

static uint16_t txq_free_stack[CYPAHL_TXQ_BLOCK_CAPACITY];
static uint16_t txq_free_top;

extern struct CanardInstance canard;

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void cyphal_pool_init(void)
{
	/* Instance pool */
	for (uint16_t i = 0; i < CYPAHL_INSTANCE_BLOCK_CAPACITY; i++) {
		instance_free_stack[i] = i;
	}
	instance_free_top = CYPAHL_INSTANCE_BLOCK_CAPACITY;

	/* TXQ pool */
	for (uint16_t i = 0; i < CYPAHL_TXQ_BLOCK_CAPACITY; i++) {
		txq_free_stack[i] = i;
	}
	txq_free_top = CYPAHL_TXQ_BLOCK_CAPACITY;
}

/*******************************************************************************
 * Memory Management (Freelist, O(1))
 ******************************************************************************/

float cyphal_get_txq_usage_pct(void)
{
	return 100.0f * (1.0f - ((float)txq_free_top / (float)CYPAHL_TXQ_BLOCK_CAPACITY));
}

/* ---------------- Instance Allocator ---------------- */

static inline void* instance_allocate(void* const user_reference, const size_t size)
{
	(void)user_reference;

	if (size > CYPAHL_INSTANCE_BLOCK_SIZE || instance_free_top == 0)
		return NULL;

	const uint16_t idx = instance_free_stack[--instance_free_top];
	return (void*)cyphal_instance_pool[idx];
}

static inline void instance_deallocate(void* const user_reference, const size_t size,
									   void* const pointer)
{
	(void)user_reference;
	(void)size;

	if (pointer == NULL)
		return;

	const uintptr_t base = (uintptr_t)cyphal_instance_pool;
	const uintptr_t p = (uintptr_t)pointer;

	const uint16_t idx = (uint16_t)((p - base) / CYPAHL_INSTANCE_BLOCK_SIZE);

	if (idx < CYPAHL_INSTANCE_BLOCK_CAPACITY) {
		instance_free_stack[instance_free_top++] = idx;
	}
}

/* ---------------- TXQ Allocator ---------------- */

static inline void* txq_allocate(void* const user_reference, const size_t size)
{
	(void)user_reference;

	if (size > CYPAHL_TXQ_BLOCK_SIZE || txq_free_top == 0)
		return NULL;

	const uint16_t idx = txq_free_stack[--txq_free_top];
	return (void*)cyphal_txq_pool[idx];
}

static inline void txq_deallocate(void* const user_reference, const size_t size,
								  void* const pointer)
{
	(void)user_reference;
	(void)size;

	if (pointer == NULL)
		return;

	const uintptr_t base = (uintptr_t)cyphal_txq_pool;
	const uintptr_t p = (uintptr_t)pointer;

	const uint16_t idx = (uint16_t)((p - base) / CYPAHL_TXQ_BLOCK_SIZE);

	if (idx < CYPAHL_TXQ_BLOCK_CAPACITY) {
		txq_free_stack[txq_free_top++] = idx;
	}
}

/*******************************************************************************
 * TX / RX Handlers
 ******************************************************************************/

int8_t cyphal_default_tx_handler(void* user_ref, CanardMicrosecond deadline_usec,
								 struct CanardMutableFrame* frame)
{
	(void)user_ref;
	(void)deadline_usec;
	twai_message_t msg;
	memset(&msg, 0, sizeof(msg));
	msg.identifier = frame->extended_can_id;
	msg.extd = 1; /* extended ID */
	msg.data_length_code = (uint8_t)frame->payload.size;
	memcpy(msg.data, frame->payload.data, msg.data_length_code);
	const esp_err_t st = twai_transmit(&msg, pdMS_TO_TICKS(200));
	if (st == ESP_OK) {
		return 1; /* transmitted */
	}

	return 0; /* retry later */
}

int cyphal_drain_tx_queue(struct CanardInstance* ins, struct CanardTxQueue* txq,
						  uint32_t max_frames)
{
	if (!ins || !txq)
		return -CANARD_ERROR_INVALID_ARGUMENT;

	uint32_t sent = 0;

	while (sent < max_frames) {
		const CanardMicrosecond now = (CanardMicrosecond)HAL_GetTimeUS();

		const int8_t r =
			canardTxPoll(txq, ins, now, NULL, cyphal_default_tx_handler, NULL, NULL);

		if (r > 0) {
			sent++;
		} else if (r == 0) {
			break;
		} else {
			return (int)r;
		}
	}

	return (int)sent;
}

/*******************************************************************************
 * RX Processing
 ******************************************************************************/

void cyphal_process(void)
{
	twai_message_t rx_msg;

	while (twai_receive(&rx_msg, 0) == ESP_OK) {
		if (!(rx_msg.extd))
			continue;

		struct CanardFrame frame = {
			.extended_can_id = rx_msg.identifier,
			.payload = {.size = rx_msg.data_length_code, .data = rx_msg.data}};

		const CanardMicrosecond ts = (CanardMicrosecond)esp_timer_get_time();
		struct CanardRxTransfer tr = {0};
		struct CanardRxSubscription* sub = NULL;

		int8_t acc = canardRxAccept(&canard, ts, &frame, 0, &tr, &sub);

		if (acc > 0) {

			handle_cyphal_message(&tr);

			if (tr.payload.allocated_size > 0) {
				canard.memory.deallocate(canard.memory.user_reference,
										 tr.payload.allocated_size, tr.payload.data);
			}
		}
	}
}

/*******************************************************************************
 * Publisher Functions
 ******************************************************************************/

static int32_t push_to_txq(struct CanardInstance* ins, struct CanardTxQueue* txq,
						   const struct CanardTransferMetadata* meta, const void* payload,
						   size_t size, CanardMicrosecond timeout)
{
	const CanardMicrosecond now = (CanardMicrosecond)HAL_GetTimeUS();

	const struct CanardPayload pl = {.size = size, .data = (void*)payload};

	return canardTxPush(txq, ins, now + timeout, meta, pl, now, NULL);
}

int cyphal_pub_message(struct CanardInstance* ins, struct CanardTxQueue* txq,
					   CanardPortID subject_id, enum CanardPriority priority,
					   const void* payload, size_t payload_size, CanardTransferID* tid,
					   CanardMicrosecond timeout_usec)
{
	if (!ins || !txq)
		return -CANARD_ERROR_INVALID_ARGUMENT;

	static CanardTransferID fallback_tid = 0;
	CanardTransferID id = tid ? *tid : fallback_tid;

	const struct CanardTransferMetadata meta = {
		.priority = priority,
		.transfer_kind = CanardTransferKindMessage,
		.port_id = subject_id,
		.remote_node_id = CANARD_NODE_ID_UNSET,
		.transfer_id = id,
	};

	if (tid)
		*tid = (CanardTransferID)((*tid + 1U) & CANARD_TRANSFER_ID_MAX);
	else
		fallback_tid = (CanardTransferID)((fallback_tid + 1U) & CANARD_TRANSFER_ID_MAX);

	return push_to_txq(ins, txq, &meta, payload, payload_size, timeout_usec);
}

int cyphal_pub_service(struct CanardInstance* ins, struct CanardTxQueue* txq,
					   enum CanardTransferKind kind, CanardPortID service_id,
					   CanardNodeID dst_node_id, enum CanardPriority priority,
					   const void* payload, size_t payload_size, CanardTransferID* tid,
					   CanardMicrosecond timeout_usec)
{
	if (!ins || !txq || dst_node_id > CANARD_NODE_ID_MAX)
		return -CANARD_ERROR_INVALID_ARGUMENT;

	static CanardTransferID fallback_tid = 0;
	CanardTransferID id = tid ? *tid : fallback_tid;

	const struct CanardTransferMetadata meta = {
		.priority = priority,
		.transfer_kind = kind,
		.port_id = service_id,
		.remote_node_id = dst_node_id,
		.transfer_id = id,
	};

	if (tid)
		*tid = (CanardTransferID)((*tid + 1U) & CANARD_TRANSFER_ID_MAX);
	else
		fallback_tid = (CanardTransferID)((fallback_tid + 1U) & CANARD_TRANSFER_ID_MAX);

	return push_to_txq(ins, txq, &meta, payload, payload_size, timeout_usec);
}

int cyphal_pub_message_to(struct CanardInstance* ins, struct CanardTxQueue* txq,
						  CanardPortID subject_id, enum CanardPriority priority,
						  CanardNodeID dst_node_id, const void* payload,
						  size_t payload_size, CanardTransferID* tid,
						  CanardMicrosecond timeout_usec)
{
	if ((ins == NULL) || (txq == NULL))
		return -CANARD_ERROR_INVALID_ARGUMENT;
	if (dst_node_id > CANARD_NODE_ID_MAX)
		return -CANARD_ERROR_INVALID_ARGUMENT;
	if ((payload == NULL) && (payload_size != 0))
		return -CANARD_ERROR_INVALID_ARGUMENT;

	CanardTransferID id;
	static CanardTransferID fallback_tid = 0;
	if (tid != NULL) {
		id = *tid;
		*tid = (CanardTransferID)((id + 1U) & CANARD_TRANSFER_ID_MAX);
	} else {
		id = fallback_tid;
		fallback_tid = (CanardTransferID)((id + 1U) & CANARD_TRANSFER_ID_MAX);
	}

	const CanardMicrosecond now = (CanardMicrosecond)HAL_GetTimeUS();
	const CanardMicrosecond deadline = now + timeout_usec;

	const struct CanardTransferMetadata meta = {
		.priority = priority,
		.transfer_kind = CanardTransferKindMessage,
		.port_id = subject_id,
		.remote_node_id = dst_node_id,
		.transfer_id = id,
	};
	const struct CanardPayload pl = {.size = payload_size, .data = (void*)payload};
	return canardTxPush(txq, ins, deadline, &meta, pl, now, NULL);
}
