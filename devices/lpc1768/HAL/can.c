/**
 *	@file     can.c
 *  @brief    CAN Hardware Abstraction Layer.
 *  @details  v1.1
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "can.h"

#include <assert.h>

#include "lpc17xx_can.h"
#include "lpc17xx_pinsel.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define HAL_CAN_INSTANCE_CNT 2

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

can_hal_callback_t prvCAN1_CALLBACK; //!< CAN1 Callback
void *prvCAN1_USR_ARG;				 //!< CAN1 Callback User data
can_hal_callback_t prvCAN2_CALLBACK; //!< CAN2 Callback
void *prvCAN2_USR_ARG;				 //!< CAN2 CAllback User data

typedef struct {
	uint32_t busOffCount;
	uint32_t txBufferFullCount;
	uint8_t lastTEC; // Transmit Error Counter
	uint8_t lastREC; // Receive Error Counter
} CAN_Debug_T;

CAN_Debug_T canDebug = {0};
volatile int counter = 0;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* Initialize pins. */
static void prvCAN_InitPins(can_hal_instance_t instance);

/* Get LPC Instance. */
static inline LPC_CAN_TypeDef *prvCAN_GetInstance(can_hal_instance_t instance);

/* Send message on CAN bus. */
static lStatus_t prvCAN_SendMsg(LPC_CAN_TypeDef *lpcInstance,
								CAN_MSG_Type *msg);

/*******************************************************************************
 * Code
 ******************************************************************************/

can_hal_status_t prvCAN_CheckStatus(LPC_CAN_TypeDef *lpcInstance)
{
	if (lpcInstance->SR & (1 << 6))
		return CAN_HAL_BUS_OFF;
	if (lpcInstance->SR & (1 << 5))
		return CAN_HAL_ERROR_PASSIVE;

	// Opcionalno: proveri da li postoji ACK (nije striktno potrebano za
	// LPC17xx) Ako nema ACK više puta: return CAN_HAL_NO_ACK;

	return CAN_HAL_OK;
}

void HAL_CAN_Init(can_hal_instance_t instance, uint32_t baudrate)
{
	assert(instance < HAL_CAN_INSTANCE_CNT);

	prvCAN_InitPins(instance);

	LPC_CAN_TypeDef *lpcInstance = prvCAN_GetInstance(instance);

	CAN_Init(lpcInstance, baudrate);

	CAN_ModeConfig(lpcInstance, CAN_HAL_MODE_OPERATING, ENABLE);

	// Put CAN Controller in Reset State.
	lpcInstance->MOD |= CAN_MOD_RM;
	// Set Transmit Priority Mode bit.
	lpcInstance->MOD |= CAN_MOD_TPM;
	// Put CAN Controller back in Operational Mode.
	lpcInstance->MOD &= ~CAN_MOD_RM;

	CAN_SetAFMode(LPC_CANAF, CAN_AccBP); /** @note: Bypass Acceptance filter. */
}

void HAL_CAN_ModeConfig(can_hal_instance_t instance, can_hal_mode_type_t mode,
						lFunctionalState_t state)
{
	assert(instance < HAL_CAN_INSTANCE_CNT);

	LPC_CAN_TypeDef *lpcInstance;

	lpcInstance = prvCAN_GetInstance(instance);

	CAN_ModeConfig(lpcInstance, mode, state);
}

void HAL_CAN_DeInit(can_hal_instance_t instance)
{
	assert(instance < HAL_CAN_INSTANCE_CNT);

	LPC_CAN_TypeDef *lpcInstance = prvCAN_GetInstance(instance);

	CAN_DeInit(lpcInstance);
}

lStatus_t HAL_CAN_SendMessage(can_hal_instance_t instance,
							  can_hal_msg_t *canMessage, uint64_t timeoutUs)
{
	(void)timeoutUs; // Cyphal NE koristi timeout

	assert(instance < HAL_CAN_INSTANCE_CNT);

	CAN_MSG_Type canMsg;

	canMsg.id = canMessage->messageId;
	canMsg.len = canMessage->length;
	canMsg.format = canMessage->idFormat;
	canMsg.type = canMessage->frameType;

	memcpy(canMsg.dataA, canMessage->data, 4);
	memcpy(canMsg.dataB, canMessage->data + 4, 4);

	LPC_CAN_TypeDef *lpcInstance = prvCAN_GetInstance(instance);

	// JEDAN pokušaj, bez čekanja
	return prvCAN_SendMsg(lpcInstance, &canMsg);
}

lStatus_t HAL_CAN_ReceiveMessage(can_hal_instance_t instance,
								 can_hal_msg_t *canMessage, uint64_t timeoutUs)
{
	assert(instance < HAL_CAN_INSTANCE_CNT);

	Status status;
	uint64_t time0;
	CAN_MSG_Type canMsg;
	LPC_CAN_TypeDef *lpcInstance = prvCAN_GetInstance(instance);

	time0 = HAL_GetTimeUS();

	do {
		status = CAN_ReceiveMsg(lpcInstance, &canMsg);

		if ((HAL_GetTimeUS() - time0) >= timeoutUs)
			break;

	} while (status != SUCCESS);

	if (status == SUCCESS) {
		canMessage->messageId = canMsg.id;
		memcpy(canMessage->data, &canMsg.dataA, 4);
		memcpy(canMessage->data + 4, &canMsg.dataB, 4);
		canMessage->length = canMsg.len;
		canMessage->idFormat = canMsg.format;
		canMessage->frameType = canMsg.type;

		return lStatus_Success;
	} else {
		return lStatus_Fail;
	}
}

void HAL_CAN_EnableInterrupt(can_hal_instance_t instance,
							 can_hal_callback_t callback, void *usr_arg)
{
	assert(instance < HAL_CAN_INSTANCE_CNT);

	switch (instance) {
	case CAN_HAL_INSTANCE_0:
		prvCAN1_CALLBACK = callback;
		prvCAN1_USR_ARG = usr_arg;

		break;
	case CAN_HAL_INSTANCE_1:
		prvCAN2_CALLBACK = callback;
		prvCAN2_USR_ARG = usr_arg;

		break;
	}

	NVIC_SetPriority(CAN_IRQn, CAN_HAL_DEF_INT_PRIO);
	NVIC_EnableIRQ(CAN_IRQn);
}

void HAL_CAN_IntCmd(can_hal_instance_t instance, can_hal_int_t intType,
					lFunctionalState_t state)
{
	assert(instance < HAL_CAN_INSTANCE_CNT);

	LPC_CAN_TypeDef *lpcInstance = prvCAN_GetInstance(instance);

	CAN_IRQCmd(lpcInstance, intType, state);
}

uint32_t HAL_CAN_GetIntStatus(can_hal_instance_t instance)
{
	assert(instance < HAL_CAN_INSTANCE_CNT);

	LPC_CAN_TypeDef *lpcInstance = prvCAN_GetInstance(instance);

	return CAN_IntGetStatus(lpcInstance);
}

/************************************ IRQs ************************************/

void CAN_IRQHandler(void)
{
	uint32_t intStatus;

	intStatus = CAN_IntGetStatus(LPC_CAN1);
	if (intStatus) {
		if (prvCAN1_CALLBACK != NULL)
			prvCAN1_CALLBACK(prvCAN1_USR_ARG, intStatus & CAN_HAL_INT_MASK_FC);
	}

	/** @note: CAN2 is not considered for now!
	 *         Reading CAN2 Interrupt Status causes Hard Fault. */
	// intStatus = CAN_IntGetStatus(LPC_CAN2);
	// if (intStatus)
	// {
	// 	if(prvCAN2_CALLBACK != NULL)
	//     	prvCAN2_CALLBACK(prvCAN2_USR_ARG, intStatus & CAN_HAL_INT_MASK_FC);

	// }
}

/****************************** static functions ******************************/

static void prvCAN_InitPins(can_hal_instance_t instance)
{
	PINSEL_CFG_Type PinCfg;

	uint8_t funcNum = PINSEL_FUNC_0;
	uint8_t portNum = PINSEL_PORT_0;
	uint8_t tdPin = PINSEL_PIN_0;
	uint8_t rdPin = PINSEL_PIN_0;

	switch (instance) {
	case CAN_HAL_INSTANCE_0:
		if (HAL_CAN_1_PIN_OPTION == 0) {
			funcNum = PINSEL_FUNC_1;
			portNum = PINSEL_PORT_0;
			rdPin = PINSEL_PIN_0;
			tdPin = PINSEL_PIN_1;
		} else {
			funcNum = PINSEL_FUNC_3;
			portNum = PINSEL_PORT_0;
			rdPin = PINSEL_PIN_21;
			tdPin = PINSEL_PIN_22;
		}
		break;
	case CAN_HAL_INSTANCE_1:
		if (HAL_CAN_2_PIN_OPTION == 0) {
			funcNum = PINSEL_FUNC_2;
			portNum = PINSEL_PORT_0;
			rdPin = PINSEL_PIN_4;
			tdPin = PINSEL_PIN_5;
		} else {
			funcNum = PINSEL_FUNC_1;
			portNum = PINSEL_PORT_2;
			rdPin = PINSEL_PIN_7;
			tdPin = PINSEL_PIN_8;
		}
		break;
	}

	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.Funcnum = funcNum;
	PinCfg.Portnum = portNum;
	PinCfg.Pinnum = rdPin;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = tdPin;
	PINSEL_ConfigPin(&PinCfg);
}

static inline LPC_CAN_TypeDef *prvCAN_GetInstance(can_hal_instance_t instance)
{
	LPC_CAN_TypeDef *lpcInstance;
	switch (instance) {
	case CAN_HAL_INSTANCE_0:
		lpcInstance = (LPC_CAN_TypeDef *)LPC_CAN1;
		break;
	case CAN_HAL_INSTANCE_1:
		lpcInstance = (LPC_CAN_TypeDef *)LPC_CAN2;
		break;
	}

	return lpcInstance;
}

static uint8_t prvCAN_GetPriority(LPC_CAN_TypeDef *lpcInstance)
{
	uint8_t prio = 0;
	uint8_t tmp;

	if ((lpcInstance->SR & (1 << 2)) == 0) {
		tmp = lpcInstance->TFI1 & 0xFFU;
		if (tmp >= prio)
			prio = tmp + 1;
	}

	if ((lpcInstance->SR & (1 << 10)) == 0) {
		tmp = lpcInstance->TFI2 & 0xFFU;
		if (tmp >= prio)
			prio = tmp + 1;
	}

	if ((lpcInstance->SR & (1 << 18)) == 0) {
		tmp = lpcInstance->TFI3 & 0xFFU;
		if (tmp >= prio)
			prio = tmp + 1;
	}

	return prio;
}

static lStatus_t prvCAN_SendMsg(LPC_CAN_TypeDef *lpcInstance, CAN_MSG_Type *msg)
{
	// --- 1. PROVERA GREŠAKA (DIJAGNOSTIKA) ---
	// Čitamo Error Counters: TEC je na bitovima 16-23, REC na 0-7 registra
	// CANICR (ako ga tvoj MCU ima) Ili čitamo direktno iz registra za greške
	uint32_t errorStatus = lpcInstance->GSR;
	canDebug.lastTEC = (errorStatus >> 16) & 0xFF;
	canDebug.lastREC = (errorStatus >> 0) & 0xFF;

	// Bus-Off provera (Bit 6)
	if (lpcInstance->SR & (1 << 6)) {

		canDebug.busOffCount++;

		// 1. Uđi u Reset mod
		lpcInstance->MOD = 1;

		// 2. Abortuj sve prenose (Bitovi 1, 2 i 3 u CMR registru za Abort)
		// Na nekim LPC modelima, bit 1 je 'Abort Transmission'
		lpcInstance->CMR = (1 << 1);

		// 3. Opciono: Očisti TX bafer u softveru ako imaš FIFO

		// 4. Vrati u normalan rad
		lpcInstance->MOD = 0;

		return lStatus_Fail;
	}

	if (LPC_CAN1->GSR & (1 << 7)) {
		counter++;
		// Kontroler je u BUS-OFF stanju (kabl je verovatno otkačen)
		return lStatus_Fail;
	}

	// --- 2. PROVERA CENTRALNOG STATUSA (BRZA PROVERA) ---
	// Koristimo Central Transmit Status Register (CANTxSR)
	// On se nalazi na adresi 0x40040000.
	// Bit 8 (za CAN1) ili Bit 9 (za CAN2) je TBS (Transmit Buffer Status)
	// Ako je taj bit 1, SVA tri bafera su slobodna.

	uint32_t *TFI = NULL, *TID = NULL, *TDA = NULL, *TDB = NULL;
	uint32_t cmr = 0;

	// Provera pojedinačnih bafera preko SR registra
	if (lpcInstance->SR & (1 << 2)) {
		cmr = 0x21;
		TFI = &lpcInstance->TFI1;
		TID = &lpcInstance->TID1;
		TDA = &lpcInstance->TDA1;
		TDB = &lpcInstance->TDB1;
	} else if (lpcInstance->SR & (1 << 10)) {
		cmr = 0x41;
		TFI = &lpcInstance->TFI2;
		TID = &lpcInstance->TID2;
		TDA = &lpcInstance->TDA2;
		TDB = &lpcInstance->TDB2;
	} else if (lpcInstance->SR & (1 << 18)) {
		cmr = 0x81;
		TFI = &lpcInstance->TFI3;
		TID = &lpcInstance->TID3;
		TDA = &lpcInstance->TDA3;
		TDB = &lpcInstance->TDB3;
	} else {
		// Svi baferi su zauzeti
		canDebug.txBufferFullCount++;
		return lStatus_Fail;
	}

	// --- 3. SLANJE PODATAKA ---
	uint8_t prio = prvCAN_GetPriority(lpcInstance);

	// TFI setup (Length, Priority, Format)
	*TFI = (prio & 0xFF) | ((msg->len & 0x0F) << 16) |
		   ((msg->type == CAN_HAL_REMOTE_FRAME) ? (1U << 30) : 0) |
		   ((msg->format == CAN_HAL_ID_FORMAT_EXT) ? (1U << 31) : 0);

	*TID = msg->id;
	*TDA = msg->dataA[0] | (msg->dataA[1] << 8) | (msg->dataA[2] << 16) |
		   (msg->dataA[3] << 24);
	*TDB = msg->dataB[0] | (msg->dataB[1] << 8) | (msg->dataB[2] << 16) |
		   (msg->dataB[3] << 24);

	// Komanda za slanje
	lpcInstance->CMR = cmr;

	return lStatus_Success;
}

/********************************* End Of File ********************************/