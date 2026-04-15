/**
 *	@file     iap.c
 *  @brief    HAL IAP library.
 *  @details  v1.1
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "iap.h"
#include "lpc17xx_iap.h"
/*******************************************************************************
 * Defines
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
iap_hal_sector_num_t HAL_IAP_GetSectorNumber(iap_hal_sector_addr_t address)
{
	return GetSecNum(address);
}

iap_hal_sector_addr_t HAL_IAP_GetSectorAddress(iap_hal_sector_num_t sector_num)
{
	switch (sector_num) {
	case IAP_HAL_SECTOR_NUM_0:
		return IAP_HAL_SECTOR_0_ADDR;
	case IAP_HAL_SECTOR_NUM_1:
		return IAP_HAL_SECTOR_1_ADDR;
	case IAP_HAL_SECTOR_NUM_2:
		return IAP_HAL_SECTOR_2_ADDR;
	case IAP_HAL_SECTOR_NUM_3:
		return IAP_HAL_SECTOR_3_ADDR;
	case IAP_HAL_SECTOR_NUM_4:
		return IAP_HAL_SECTOR_4_ADDR;
	case IAP_HAL_SECTOR_NUM_5:
		return IAP_HAL_SECTOR_5_ADDR;
	case IAP_HAL_SECTOR_NUM_6:
		return IAP_HAL_SECTOR_6_ADDR;
	case IAP_HAL_SECTOR_NUM_7:
		return IAP_HAL_SECTOR_7_ADDR;
	case IAP_HAL_SECTOR_NUM_8:
		return IAP_HAL_SECTOR_8_ADDR;
	case IAP_HAL_SECTOR_NUM_9:
		return IAP_HAL_SECTOR_9_ADDR;
	case IAP_HAL_SECTOR_NUM_10:
		return IAP_HAL_SECTOR_10_ADDR;
	case IAP_HAL_SECTOR_NUM_11:
		return IAP_HAL_SECTOR_11_ADDR;
	case IAP_HAL_SECTOR_NUM_12:
		return IAP_HAL_SECTOR_12_ADDR;
	case IAP_HAL_SECTOR_NUM_13:
		return IAP_HAL_SECTOR_13_ADDR;
	case IAP_HAL_SECTOR_NUM_14:
		return IAP_HAL_SECTOR_14_ADDR;
	case IAP_HAL_SECTOR_NUM_15:
		return IAP_HAL_SECTOR_15_ADDR;
	case IAP_HAL_SECTOR_NUM_16:
		return IAP_HAL_SECTOR_16_ADDR;
	case IAP_HAL_SECTOR_NUM_17:
		return IAP_HAL_SECTOR_17_ADDR;
	case IAP_HAL_SECTOR_NUM_18:
		return IAP_HAL_SECTOR_18_ADDR;
	case IAP_HAL_SECTOR_NUM_19:
		return IAP_HAL_SECTOR_19_ADDR;
	case IAP_HAL_SECTOR_NUM_20:
		return IAP_HAL_SECTOR_20_ADDR;
	case IAP_HAL_SECTOR_NUM_21:
		return IAP_HAL_SECTOR_21_ADDR;
	case IAP_HAL_SECTOR_NUM_22:
		return IAP_HAL_SECTOR_22_ADDR;
	case IAP_HAL_SECTOR_NUM_23:
		return IAP_HAL_SECTOR_23_ADDR;
	case IAP_HAL_SECTOR_NUM_24:
		return IAP_HAL_SECTOR_24_ADDR;
	case IAP_HAL_SECTOR_NUM_25:
		return IAP_HAL_SECTOR_25_ADDR;
	case IAP_HAL_SECTOR_NUM_26:
		return IAP_HAL_SECTOR_26_ADDR;
	case IAP_HAL_SECTOR_NUM_27:
		return IAP_HAL_SECTOR_27_ADDR;
	case IAP_HAL_SECTOR_NUM_28:
		return IAP_HAL_SECTOR_28_ADDR;
	case IAP_HAL_SECTOR_NUM_29:
		return IAP_HAL_SECTOR_29_ADDR;
	default:
		return 0; // or handle error
	}
}

iap_hal_status_code_t
HAL_IAP_PrepareSector(const iap_hal_sector_num_t startSector,
					  const iap_hal_sector_num_t endSector)
{
	return PrepareSector(startSector, endSector);
}

iap_hal_status_code_t HAL_IAP_CopyRAM2Flash(uint8_t* dest, uint8_t* source,
											const iap_hal_write_size_t size)
{
	return CopyRAM2Flash(dest, source, size);
}

iap_hal_status_code_t
HAL_IAP_EraseSector(const iap_hal_sector_num_t startSector,
					const iap_hal_sector_num_t endSector)
{
	return EraseSector(startSector, endSector);
}

iap_hal_status_code_t
HAL_IAP_BlankCheckSector(const iap_hal_sector_num_t startSector,
						 const iap_hal_sector_num_t endSector,
						 uint32_t* firstNotBlankLoc, uint32_t* firstNotBlankVal)
{
	return BlankCheckSector(startSector, endSector, firstNotBlankLoc,
							firstNotBlankVal);
}

iap_hal_status_code_t HAL_IAP_ReadPartID(uint32_t* partID)
{
	return ReadPartID(partID);
}

iap_hal_status_code_t HAL_IAP_ReadBootCodeVer(uint8_t* major, uint8_t* minor)
{
	return ReadBootCodeVer(major, minor);
}

iap_hal_status_code_t HAL_IAP_ReadDeviceSerialNum(uint32_t* uid)
{
	return ReadDeviceSerialNum(uid);
}

iap_hal_status_code_t HAL_IAP_Compare(uint8_t* addr1, uint8_t* addr2,
									  const uint32_t size)
{
	return Compare(addr1, addr2, size);
}

void HAL_IAP_ReInvokeISP()
{
	InvokeISP();
}

/************************************ IRQs ************************************/

/****************************** static functions ******************************/

/* --------------------------------- End Of File -----------------------------*/