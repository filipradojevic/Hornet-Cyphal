/**
 *	@file     flash.c
 *  @brief    Library used for communication with flash memory.
 *  @version  v1.1.
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "flash.h"
#include "flashconf.h"

#ifdef FLASH_USE_W25N01GV
#include "w25n01gv/w25n01gv.h"
#endif

#ifdef FLASH_USE_GD5F2GQ5UE
#include "gd5f2gq5ue/gd5f2gq5ue.h"
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* Flash SSP DMA Transfer Callback */
// void prvFLASH_SSP_CALLBACK(ssp_hal_instance_t instance,
//                           ssp_hal_handle_t *handle,
//                           ssp_hal_dma_tran_type_t tranType, 
//                           esp_err_t status,
//                           void *userData);

/*******************************************************************************
 * Code
 ******************************************************************************/

esp_err_t FLASH_Init(flash_t *dev,
                     spi_device_handle_t spi_handle,
                     uint64_t (*time_us)(void),
                     gpio_num_t cs_pin)
{
  esp_err_t status;

  dev->handle = spi_handle;
  dev->cs_pin = cs_pin;

  uint8_t dev_identified = 0;
  uint8_t jedec_id;
  uint8_t dev_id;
  uint8_t resp[2];
  uint8_t cmd[2] = {FLASH_GET_JEDEC_ID_CMD, 0x00U};

  /* postavi CS pin */
  gpio_set_direction(dev->cs_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(dev->cs_pin, 1);  // deassert CS

  status = FLASH_Transfer(dev, cmd, sizeof(cmd), resp, sizeof(resp));
  if (status != ESP_OK)
    return ESP_FAIL;

  // Get ids
  jedec_id = resp[0];
  dev_id = resp[1];

  #ifdef FLASH_USE_W25N01GV
  if(!dev_identified) {
    status = W25N01GV_Identify(dev, jedec_id, dev_id);
    if(status == ESP_OK)
      dev_identified = 1;
  }
  #endif

  #ifdef FLASH_USE_GD5F2GQ5UE
  if(!dev_identified) {
    status = GD5F2GQ5UE_Identify(dev, jedec_id, dev_id);
    if(status == ESP_OK)
      dev_identified = 1;
  }
  #endif

  if(!dev_identified)
    return ESP_FAIL;

  dev->time_us = time_us;

  FLASH_Reset(dev);

  return ESP_OK;
}

uint8_t FLASH_Busy(flash_t *dev)
{
  return dev->vtable->busy(dev);
}

void FLASH_ConfigureDMA(flash_t *dev, 
                        flash_dma_ch_type_t chType, 
                        uint8_t dmaChannel)
{
  if (chType == FLASH_DMA_CH_TYPE_RX)
    dev->dmaChannelRX = dmaChannel;
  else if (chType == FLASH_DMA_CH_TYPE_TX)
    dev->dmaChannelTX = dmaChannel;
}

void FLASH_ConfigureCallback(flash_t *dev, 
                             flash_tran_callback_t callback, 
                             void *userData)
{
  dev->callback = callback;
  dev->userData = userData;
}

esp_err_t FLASH_BlockErase(flash_t *dev, uint32_t blkAddr, uint8_t exeType)
{
  return dev->vtable->blockErase(dev, blkAddr, exeType);
}

esp_err_t FLASH_AreaErase(flash_t *dev, uint32_t start, uint32_t end)
{
  return dev->vtable->areaErase(dev, start, end);
}

esp_err_t FLASH_CacheLoad(flash_t *dev,
                          uint32_t blkAddr,
                          uint32_t pageAddr,
                          uint8_t exeType)
{
  return dev->vtable->cacheLoad(dev, blkAddr, pageAddr, exeType);
}

esp_err_t FLASH_CacheRead(flash_t *dev,
                          uint16_t column,
                          uint8_t *recv,
                          uint16_t rSize,
                          flash_tran_e tranType)
{
  return dev->vtable->cacheRead(dev, column, recv, rSize, tranType);
}

esp_err_t FLASH_PageRead(flash_t *dev,
                         uint32_t blkAddr,
                         uint32_t pageAddr,
                         uint8_t *recv,
                         uint16_t rSize)
{
  return dev->vtable->pageRead(dev, blkAddr, pageAddr, recv, rSize);
}

esp_err_t FLASH_SpareAreaRead(flash_t *dev,
                              uint32_t blkAddr,
                              uint32_t pageAddr,
                              uint8_t *recv,
                              uint16_t rSize)
{
  return dev->vtable->spareAreaRead(dev, blkAddr, pageAddr, recv, rSize);
}

esp_err_t FLASH_CacheWrite(flash_t *dev,
                           uint16_t column,
                           uint8_t *data,
                           uint16_t dataLen,
                           uint8_t tranType)
{
  return dev->vtable->cacheWrite(dev, column, data, dataLen, tranType);
}

esp_err_t FLASH_CacheFlush(flash_t *dev,
                           uint32_t blkAddr,
                           uint32_t pageAddr,
                           uint8_t exeType)
{
  return dev->vtable->cacheFlush(dev, blkAddr, pageAddr, exeType);
}

esp_err_t FLASH_PageWrite(flash_t *dev,
                          uint32_t blkAddr,
                          uint32_t pageAddr,
                          uint8_t *data,
                          uint16_t dataLen)
{
  return dev->vtable->pageWrite(dev, blkAddr, pageAddr, data, dataLen);
}

esp_err_t FLASH_Reset(flash_t *dev)
{
  return dev->vtable->reset(dev);
}

/****************************** static functions ******************************/


/********************************* End Of File ********************************/