/**
 *	@file     flash_common.c
 *  @brief    Library used for common terms between different NAND flash devices
 *  @version  v1.1.
 *  @author   LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "flash_common.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define SPI_BUFFER_MAX_SIZE   4096

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint8_t spi_buffer[SPI_BUFFER_MAX_SIZE];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/


esp_err_t FLASH_Transfer(flash_t *dev,
                         uint8_t *tx, uint32_t txSize,
                         uint8_t *rx, uint32_t rxSize)
{
    esp_err_t status = ESP_FAIL;

    // Total length in bits
    uint32_t totalBits = (txSize + rxSize) * 8;

    // Copy tx in buffer
    memcpy(spi_buffer, tx, txSize);

    spi_transaction_t t = {0};
    t.tx_buffer = spi_buffer;
    t.rx_buffer = spi_buffer;
    t.length = totalBits;
    t.flags = 0;

    // Assert CS
    gpio_set_level(dev->cs_pin, 0);

    // Transmit
    status = spi_device_transmit(dev->handle, &t);

    // Deassert CS
    gpio_set_level(dev->cs_pin, 1);

    // Kopiramo RX deo u prosledjeni buffer
    if (rx && rxSize > 0) {
        memcpy(rx, spi_buffer + txSize, rxSize);
    }

    return status;
}


esp_err_t FLASH_Transmit(flash_t *dev, uint8_t *tx, uint32_t txSize)
{
  esp_err_t status = ESP_FAIL;

  spi_transaction_t t = {0};
  t.tx_buffer = tx;
  t.length = txSize * 8;  // length in bits

  // SPI
  gpio_set_level(dev->cs_pin, 0);                 // DEASSERT
  status = spi_device_transmit(dev->handle, &t);  // TRANSMIT
  gpio_set_level(dev->cs_pin, 1);                 // ASSERT

  if (status != ESP_OK) {
    return ESP_FAIL;
  }

  return ESP_OK;
}

/****************************** static functions ******************************/

/********************************* End Of File ********************************/