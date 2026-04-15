/**
 * @file    w5500.c
 * @brief   W5500 Ethernet driver implementation for ESP32 (UDP/PHY control)
 * @version 1.0.1
 * @date    05.10.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "w5500.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define TAG "W5500_SPI"


/*******************************************************************************
 * Typedefs
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
extern uint8_t gw_sky_ip[4];
extern uint16_t gw_sky_port;

/*******************************************************************************
 * Prototypes (static/internal)
 ******************************************************************************/
static void w5500_select(w5500_t *dev);
static void w5500_deselect(w5500_t *dev);
static esp_err_t w5500_write_buffer(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, const uint8_t *data, size_t len);
static esp_err_t w5500_read_buffer(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t *buffer, size_t len);
static esp_err_t w5500_write_reg(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t data);
static esp_err_t w5500_read_reg(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t *data);
static void w5500_write_ip_addr(w5500_t *dev, uint16_t addr, const uint8_t *data);
uint16_t w5500_phy_send(void *phy, const uint8_t *data, uint16_t size);
uint16_t w5500_phy_recv(void *phy, uint8_t *buffer, uint16_t size);

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Select/deselect W5500 SPI */
static void w5500_select(w5500_t *dev)
{
    (void)gpio_set_level(dev->pin_cs, 0);
}

static void w5500_deselect(w5500_t *dev)
{
    (void)gpio_set_level(dev->pin_cs, 1);
}

/* Initialize W5500 device */
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
                int pin_int)
{
    if ((dev == NULL) || (mac == NULL) || (ip == NULL) || (subnet == NULL) || (gateway == NULL))
    {
        ESP_LOGE(TAG, "Invalid parameters in w5500_init \n");
        return;
    }

    ESP_LOGI(TAG, "Start W5500 initialization \n");

    memcpy(dev->mac, mac, 6U);
    memcpy(dev->ip, ip, 4U);
    memcpy(dev->subnet, subnet, 4U);
    memcpy(dev->gateway, gateway, 4U);
    dev->local_port = local_port;

    dev->pin_cs   = pin_cs;
    dev->pin_rst  = pin_rst;
    dev->pin_miso = pin_miso;
    dev->pin_mosi = pin_mosi;
    dev->pin_sclk = pin_sclk;
    dev->pin_int  = pin_int;

    gpio_set_direction(dev->pin_cs, GPIO_MODE_OUTPUT);
    w5500_deselect(dev);

    gpio_set_direction(dev->pin_rst, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    dev->phy.recv = w5500_phy_recv;
    dev->phy.send = w5500_phy_send;
    dev->phy.parent = dev;

    ESP_LOGI(TAG, "W5500 initialization successfully \n");
}


int W5500_ETH_Init(w5500_t *dev,
                     const uint8_t *mac,
                     const uint8_t *ip,
                     const uint8_t *subnet,
                     const uint8_t *gateway,
                     uint16_t local_port,
                     int pin_cs, int pin_rst, int pin_miso, int pin_mosi, int pin_sclk, int pin_int)
{
    ESP_LOGI(TAG, "ETH Init START \n");

    if (!dev || !mac || !ip || !subnet || !gateway) return -1;

    // 1. Initialize W5500 chip
    w5500_init(dev, mac, ip, subnet, gateway, local_port, pin_cs, pin_rst, pin_miso, pin_mosi, pin_sclk, pin_int);

    // 2. Configure PHY and network
    w5500_set_phy_allcapable(dev);
    w5500_set_network_config(dev, gateway, subnet);
    w5500_set_phy_allcapable(dev);

    // 3. Open UDP socket on specified port
    w5500_udp_open(dev, local_port);

    ESP_LOGI(TAG, "ETH Init DONE \n");

    return 0; // Success
}


/* Set network configuration */
void w5500_set_network_config(w5500_t *dev, const uint8_t *gateway, const uint8_t *subnet)
{
    if ((dev == NULL) || (gateway == NULL) || (subnet == NULL))
    {
        ESP_LOGE(TAG, "Invalid parameters in w5500_set_network_config \n");
        return;
    }

    ESP_LOGI(TAG, "Start network configuration \n");

    w5500_write_ip_addr(dev, 0x0001U, gateway);   /* GAR */
    w5500_write_ip_addr(dev, 0x0005U, subnet);    /* SUBR */
    w5500_write_buffer(dev, 0x0009U, 0x00U, dev->mac, 6U); /* SHAR */
    w5500_write_ip_addr(dev, 0x000FU, dev->ip);   /* SIPR */
    w5500_write_reg(dev, 0x001AU, 0x00U, 0x02U);  /* RX buffer 2KB */
    w5500_write_reg(dev, 0x001BU, 0x00U, 0x02U);  /* TX buffer 2KB */

    ESP_LOGI(TAG, "Network configuration successfully applied \n");
}


/* Open UDP socket0 */
void w5500_udp_open(w5500_t *dev, uint16_t port)
{
    uint8_t sr = 0U;
    uint16_t i = 0U;

    if (dev == NULL)
    {
        ESP_LOGE(TAG, "Invalid device pointer in w5500_udp_open");
        return;
    }

    ESP_LOGI(TAG, "Opening UDP socket0 on port %u", port);

    // 1) Zatvori ako je već otvoren
    w5500_write_reg(dev, 0x0001U, SOCKET0_BLOCK, 0x10U); /* CLOSE */

    // 2) Sačekaj da se zatvori
    for (i = 0U; i < 50U; ++i)
    {
        w5500_read_reg(dev, 0x0003U, SOCKET0_BLOCK, &sr);
        if (sr == 0x00U)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 3) Podesi režim UDP = 0x02
    w5500_write_reg(dev, 0x0000U, SOCKET0_BLOCK, 0x02U); /* Sn_MR = UDP */

    // 4) Postavi lokalni port (Sn_PORT)
    w5500_write_reg(dev, 0x0004U, SOCKET0_BLOCK, (port >> 8) & 0xFF); /* PORT high */
    w5500_write_reg(dev, 0x0005U, SOCKET0_BLOCK, port & 0xFF);        /* PORT low */

    // 5) Otvori socket
    w5500_write_reg(dev, 0x0001U, SOCKET0_BLOCK, 0x01U); /* OPEN */

    // 6) Sačekaj status UDP = 0x22
    for (i = 0U; i < 100U; ++i)
    {
        w5500_read_reg(dev, 0x0003U, SOCKET0_BLOCK, &sr);
        if (sr == 0x22U)
        {
            ESP_LOGI(TAG, "UDP socket0 READY (port %u)", port);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (sr != 0x22U)
        ESP_LOGE(TAG, "Failed to open UDP socket0!");
}


/* Set PHY all-capable mode */
void w5500_set_phy_allcapable(w5500_t *dev)
{
    uint8_t phy = 0U;
    uint16_t i = 0U;

    if (dev == NULL)
    {
        ESP_LOGE(TAG, "Invalid device in w5500_set_phy_allcapable \n");
        return;
    }

    ESP_LOGI(TAG, "Start PHY all-capable mode \n");

    phy = w5500_read_phycfgr(dev);
    phy &= ~(0x78U);           /* Clear capability bits */
    phy |= (1U << 6U) | (0x7U << 3U); /* Enable all capabilities */
    w5500_write_phycfgr(dev, phy);

    phy &= ~(1U << 7U);        /* Clear reset bit */
    w5500_write_phycfgr(dev, phy);
    vTaskDelay(pdMS_TO_TICKS(50));
    phy |= (1U << 7U);         /* Set reset bit */
    w5500_write_phycfgr(dev, phy);

    ESP_LOGI(TAG, "Waiting for PHY link... \n");
    for (i = 0U; i < 100U; ++i)
    {
        phy = w5500_read_phycfgr(dev);
        if ((phy & 0x01U) != 0U)
        {
            ESP_LOGI(TAG, "PHY link active \n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/********************************* static SPI/Reg functions ********************************/

static esp_err_t w5500_write_buffer(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, const uint8_t *data, size_t len)
{
    esp_err_t ret = ESP_FAIL;
    if ((dev == NULL) || (data == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_len = 3U + len;
    uint8_t *tx_buf = alloca(total_len);
    if (tx_buf == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    tx_buf[0] = (uint8_t)((addr >> 8U) & 0xFFU);
    tx_buf[1] = (uint8_t)(addr & 0xFFU);
    tx_buf[2] = W5500_WRITE_OPCODE | block_ctrl;
    (void)memcpy(&tx_buf[3], data, len);

    w5500_select(dev);
    ret = spi_device_transmit(dev->spi, &(spi_transaction_t){.length = 8U * total_len, .tx_buffer = tx_buf, .rx_buffer = NULL});
    w5500_deselect(dev);

    return ret;
}

static esp_err_t w5500_read_buffer(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t *buffer, size_t len)
{
    esp_err_t ret = ESP_FAIL;
    if ((dev == NULL) || (buffer == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_len = 3U + len;
    uint8_t *tx_buf = alloca(total_len);
    uint8_t *rx_buf = alloca(total_len);
    if ((tx_buf == NULL) || (rx_buf == NULL))
    {
        return ESP_ERR_NO_MEM;
    }

    memset(tx_buf, 0, total_len);
    tx_buf[0] = (uint8_t)((addr >> 8U) & 0xFFU);
    tx_buf[1] = (uint8_t)(addr & 0xFFU);
    tx_buf[2] = W5500_READ_OPCODE | block_ctrl;

    w5500_select(dev);
    ret = spi_device_transmit(dev->spi, &(spi_transaction_t){.length = 8U * total_len, .tx_buffer = tx_buf, .rx_buffer = rx_buf});
    w5500_deselect(dev);

    if (ret == ESP_OK)
    {
        (void)memcpy(buffer, &rx_buf[3], len);
    }

    return ret;
}

uint16_t w5500_sendto(w5500_t *dev,
                      const uint8_t *buffer,
                      uint16_t len,
                      uint8_t *dst_ip,
                      uint16_t dst_port)
{
    const uint8_t S0_BLOCK = 0x08U;
    uint16_t tx_ptr;
    uint8_t ptr_h;
    uint8_t ptr_l;

    /* Check if the link is active */
    if (!w5500_is_link_up(dev))
    {
        ESP_LOGW(TAG, "Link is not active in sendto! \n");
        return 0U;
    }

    /* Limit length to maximum buffer size */
    if (len > 2048U)
    {
        len = 2048U;
    }

    /* Set destination IP and port */
    (void)w5500_write_buffer(dev, 0x000CU, S0_BLOCK, dst_ip, 4U);  /* Sn_DIPR */
    (void)w5500_write_reg(dev, 0x0010U, S0_BLOCK, (uint8_t)(dst_port >> 8U)); /* Sn_DPORTH */
    (void)w5500_write_reg(dev, 0x0011U, S0_BLOCK, (uint8_t)(dst_port & 0xFFU)); /* Sn_DPORTL */

    /* Read current TX write pointer */
    (void)w5500_read_reg(dev, 0x0024U, S0_BLOCK, &ptr_h); /* Sn_TX_WRH */
    (void)w5500_read_reg(dev, 0x0025U, S0_BLOCK, &ptr_l); /* Sn_TX_WRL */
    tx_ptr = ((uint16_t)ptr_h << 8U) | (uint16_t)ptr_l;

    /* Write the full packet directly from user buffer */
    (void)w5500_write_buffer(dev, tx_ptr, 0x10U, buffer, len);

    /* Update TX write pointer */
    tx_ptr += len;
    (void)w5500_write_reg(dev, 0x0024U, S0_BLOCK, (uint8_t)(tx_ptr >> 8U));
    (void)w5500_write_reg(dev, 0x0025U, S0_BLOCK, (uint8_t)(tx_ptr & 0xFFU));

    /* Issue SEND command */
    (void)w5500_write_reg(dev, 0x0001U, S0_BLOCK, 0x20U);

    /* Wait until send completes (SEND_OK flag in IR) */
    uint8_t ir;
    do
    {
        (void)w5500_read_reg(dev, 0x0002U, S0_BLOCK, &ir);
    } while ((ir & 0x10U) == 0U);

    /* Clear SEND_OK interrupt */
    (void)w5500_write_reg(dev, 0x0002U, S0_BLOCK, 0x10U);

    return len;
}

uint16_t w5500_recv(w5500_t *dev, uint8_t *buffer, uint16_t max_len)
{
    const uint8_t S0_BLOCK = 0x08;
    const uint8_t S0_RX_BUF = 0x18; 
    const uint16_t UDP_HDR = 8; // 8 bajtova zaglavlja (IP/Port/Length)
    
    uint8_t size_h, size_l;
    uint8_t ptr_h, ptr_l;
    uint16_t rx_ptr;
    uint16_t rx_size;    // Ukupna veličina u RX baferu (Zaglavlje + Payload)
    uint16_t payload_len = 0;

     /* Check if the link is active */
    if (!w5500_is_link_up(dev))
    {
        ESP_LOGW(TAG, "Link is not active in sendto! \n");
        
        W5500_ResetAndInit(dev);

        return 0U;
    }

    // 1) Čitanje Sn_RX_RSR (dostupni bajtovi)
    w5500_read_reg(dev, 0x0026, S0_BLOCK, &size_h);
    w5500_read_reg(dev, 0x0027, S0_BLOCK, &size_l);
    rx_size = (size_h << 8) | size_l;
    
    // Ako nema dovoljno za zaglavlje (min 8 bajtova), vrati 0.
    if (rx_size < UDP_HDR) return 0; 
    
    // 2) Čitanje Sn_RX_RD (adresa početka bloka)
    w5500_read_reg(dev, 0x0028, S0_BLOCK, &ptr_h);
    w5500_read_reg(dev, 0x0029, S0_BLOCK, &ptr_l);
    rx_ptr = (ptr_h << 8) | ptr_l;

    // 3) PRORAČUN DUŽINE I OFSETA
    payload_len = rx_size - UDP_HDR;

    // Odredišni bafer ne sme biti prevelik
    if (payload_len > max_len) {
        payload_len = max_len;
    }

    // 4) ⭐ KRITIČNA OPTIMIZACIJA: Čitaj direktno u bafer, PRESKAČUĆI UDP_HDR
    // Novi RX pointer počinje nakon 8 bajtova zaglavlja
    uint16_t data_start_ptr = rx_ptr + UDP_HDR;
    
    // Čitaj samo payload (payload_len) direktno u odredišni bafer
    // Ova operacija je najbrži mogući SPI prenos.
    w5500_read_buffer(dev, data_start_ptr, S0_RX_BUF, buffer, payload_len);
    
    // 5) Ažuriraj Sn_RX_RD za CEO blok (zaglavlje + payload)
    uint16_t new_rx_ptr = rx_ptr + rx_size;
    w5500_write_reg(dev, 0x0028, S0_BLOCK, (new_rx_ptr >> 8) & 0xFF);
    w5500_write_reg(dev, 0x0029, S0_BLOCK, new_rx_ptr & 0xFF);

    // 6) Izdaj RECV komandu
    w5500_write_reg(dev, 0x0001, S0_BLOCK, 0x40); 
    
    return payload_len; // Vraća dužinu čistog payload-a
}


int32_t W5500_ResetAndInit(w5500_t *dev)
{
    int32_t i;

    /* 1. Hard reset W5500 */
    gpio_set_level(dev->pin_rst, 0U);
    vTaskDelay(pdMS_TO_TICKS(50U));
    gpio_set_level(dev->pin_rst, 1U);
    vTaskDelay(pdMS_TO_TICKS(500U));

    /* 2. Re-initialize W5500 with existing device parameters */
    w5500_init(dev,
               dev->mac,
               dev->ip,
               dev->subnet,
               dev->gateway,
               dev->local_port,
               dev->pin_cs,
               dev->pin_rst,
               dev->pin_miso,
               dev->pin_mosi,
               dev->pin_sclk,
               dev->pin_int);

    /* 3. Configure PHY and network parameters */
    w5500_set_phy_allcapable(dev);
    w5500_set_network_config(dev, dev->gateway, dev->subnet);
    w5500_set_phy_allcapable(dev);  /* Ensure PHY is ready */

    /* 4. Open UDP socket on predefined port */
    ESP_LOGI(TAG, "Opening UDP socket0... \n");
    w5500_udp_open(dev, dev->local_port);

    /* 5. Wait until PHY link is active */
    for (i = 0; i < 50; i++)
    {
        if (w5500_is_link_up(dev) != 0)
        {
            ESP_LOGI(TAG, "PHY link active before sending \n");
            break;
        }
        ESP_LOGW(TAG, "Waiting for PHY link... \n");
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    ESP_LOGI(TAG, "W5500 re-initialization complete \n");

    if (!w5500_is_link_up(dev))
    {
        ESP_LOGE(TAG, "Link is not active! \n");
        return 0U;
    }
    else{
        ESP_LOGI(TAG, "Link is active! \n");
    }
    
    return 1; /* Successful */
}

int w5500_is_link_up(w5500_t *dev) {
    uint8_t phy_cfg;
    
    w5500_read_reg(dev, W5500_PHYCFGR, 0x00, &phy_cfg);

    return (phy_cfg & W5500_LINK_STATUS_BIT) ? 1 : 0;
}

uint8_t w5500_read_version(w5500_t *dev) { 
    uint8_t ver = 0; 
    
    w5500_read_reg(dev, W5500_VERSIONR, 0, &ver); 
    
    return ver; 
}

uint8_t w5500_read_phycfgr(w5500_t *dev)
{
    uint8_t val = 0;
    w5500_read_reg(dev, W5500_PHYCFGR, 0, &val);
    return val;
}

void w5500_write_phycfgr(w5500_t *dev, uint8_t value)
{
    w5500_write_reg(dev, W5500_PHYCFGR, 0, value);
}

static esp_err_t w5500_write_reg(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t data)
{
    return w5500_write_buffer(dev, addr, block_ctrl, &data, 1U);
}

static esp_err_t w5500_read_reg(w5500_t *dev, uint16_t addr, uint8_t block_ctrl, uint8_t *data)
{
    return w5500_read_buffer(dev, addr, block_ctrl, data, 1U);
}

static void w5500_write_ip_addr(w5500_t *dev, uint16_t addr, const uint8_t *data)
{
    if ((dev != NULL) && (data != NULL))
    {
        (void)w5500_write_buffer(dev, addr, 0x00U, data, 4U);
    }
}

/* PHY send/recv wrappers */
uint16_t w5500_phy_send(void *phy, const uint8_t *data, uint16_t size)
{
    eth_hal_phy_t *phy_layer = (eth_hal_phy_t *)phy;
    w5500_t *dev = (w5500_t *)phy_layer->parent;
    if (dev == NULL)
    {
        return 0U;
    }
    return w5500_sendto(dev, 
                        data, 
                        size, 
                        (uint8_t *)gw_sky_ip,   // Fiksna IP adresa
                        gw_sky_port);          // Fiksni port
}

uint16_t w5500_phy_recv(void *phy, uint8_t *buffer, uint16_t size)
{
    eth_hal_phy_t *phy_layer = (eth_hal_phy_t *)phy;

    if (phy_layer == NULL || phy_layer->parent == NULL)
    {
        ESP_LOGE(TAG, "Invalid PHY or parent pointer!");
        return 0U;
    }
    w5500_t *dev = (w5500_t *)phy_layer->parent;
    return w5500_recv(dev, buffer, size);
}

/********************************* End Of File ********************************/
