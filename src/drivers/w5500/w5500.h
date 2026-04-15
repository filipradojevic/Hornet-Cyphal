/**
 * @file    w5500.h
 * @brief   W5500 Ethernet driver for ESP32 (UDP/PHY control)
 * @version 1.0.0
 * @date    05.10.2025
 * @author  BetaTehPro
 */

#ifndef W5500_H
#define W5500_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define W5500_VERSIONR         0x0039
#define W5500_PHYCFGR          0x002E
#define W5500_WRITE_OPCODE     0x04
#define W5500_READ_OPCODE      0x00
#define W5500_S0_REG_BLOCK     (0x01 << 3) // 0x08
#define W5500_S0_TX_BUFFER     (0x02 << 3) // 0x10
#define W5500_S0_RX_BUFFER     (0x03 << 3) // 0x18
#define W5500_UDP_HEADER_LEN   8
#define W5500_LINK_STATUS_BIT  0x01 
#define MAX_BUFFER_SIZE        2048U
#define SOCKET0_BLOCK          0x08U
#define SOCKET0_TX_BUF         0x10U
#define SOCKET0_RX_BUF         0x18U
#define SOCKET0_RX_SIZE 0x800  // 2048 bajtova za 2KB bafer
#define SOCKET0_RX_MASK 0x07FF // 2048 - 1 = 2047
#define UDP_HEADER_LEN         8U

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief Ethernet HAL Physical Layer - used by higher level protocols */
typedef struct eth_hal_phy_t {
    uint16_t (*recv)(void *phy, uint8_t *data, uint16_t size);
    uint16_t (*send)(void *phy, const uint8_t *data, uint16_t size);
    void *parent; /* Pointer to parent w5500_t structure */
} eth_hal_phy_t;

/*! @brief W5500 device structure */
typedef struct w5500_t{
    spi_device_handle_t spi;
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t subnet[4];
    uint8_t gateway[4];
    uint16_t local_port;

    int pin_cs;
    int pin_rst;
    int pin_miso;
    int pin_mosi;
    int pin_sclk;
    int pin_int;

    eth_hal_phy_t phy;
} w5500_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
// Nema globalnih promenljivih u ovom trenutku

/*******************************************************************************
 * API / Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize W5500 device with network config and SPI pins.
 */
void w5500_init(w5500_t *dev, 
                const uint8_t *mac, 
                const uint8_t *ip,
                const uint8_t *subnet, 
                const uint8_t *gateway, 
                uint16_t local_port,
                int pin_cs, 
                int pin_rst,
                int pin_miso, 
                int pin_mosi, 
                int pin_sclk, 
                int pin_int);

int W5500_ETH_Init(w5500_t *dev,
                     const uint8_t *mac,
                     const uint8_t *ip,
                     const uint8_t *subnet,
                     const uint8_t *gateway,
                     uint16_t local_port,
                     int pin_cs, int pin_rst, int pin_miso, int pin_mosi, int pin_sclk, int pin_int);

/**
 * @brief Read W5500 version register.
 */
uint8_t w5500_read_version(w5500_t *dev);

/**
 * @brief Read PHYCFGR register.
 */
uint8_t w5500_read_phycfgr(w5500_t *dev);

/**
 * @brief Write PHYCFGR register.
 */
void w5500_write_phycfgr(w5500_t *dev, uint8_t value);

/**
 * @brief Configure PHY to all-capable mode.
 */
void w5500_set_phy_allcapable(w5500_t *dev);

/**
 * @brief Receive UDP data from W5500.
 */
uint16_t w5500_recv(w5500_t *dev, uint8_t *buffer, uint16_t max_len);

/**
 * @brief Send raw data over W5500.
 */
uint16_t w5500_send(w5500_t *dev, const uint8_t *buffer, uint16_t len);

/**
 * @brief UDP send function for PHY layer.
 */
uint16_t w5500_phy_send(void *phy, const uint8_t *data, uint16_t size);

/**
 * @brief UDP receive function for PHY layer.
 */
uint16_t w5500_phy_recv(void *phy, uint8_t *data, uint16_t size);

/**
 * @brief Open UDP socket0 on given port.
 */
void w5500_udp_open(w5500_t *dev, uint16_t port);

/**
 * @brief Send UDP packet to destination IP and port.
 */
uint16_t w5500_sendto(w5500_t *dev, const uint8_t *buffer, uint16_t len, uint8_t *dst_ip, uint16_t dst_port);

/**
 * @brief Set network configuration (gateway + subnet).
 */
void w5500_set_network_config(w5500_t *dev, const uint8_t *gateway, const uint8_t *subnet);

/**
 * @brief Check if PHY link is up.
 */
int w5500_is_link_up(w5500_t *dev);

/**
 * @brief Reset W5500 and reinitialize device.
 */
int32_t W5500_ResetAndInit(w5500_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* W5500_H */
