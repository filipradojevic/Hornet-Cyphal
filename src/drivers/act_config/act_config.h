/**
 * @file    flash_cfg.h
 * @brief   Actuator configuration - wear-leveled Flash storage (LPC17xx IAP)
 *
 * Strategija:
 *   - Sektor na 0xF000 (4 KB) = 16 slotova po 256 B (IAP minimum)
 *   - Svaki slot sadrzi jedan flash_cfg_rec_t (16 B) + 240 B padding (0xFF)
 *   - Upis: read 256 B -> izmeni slot u RAM-u -> write 256 B nazad
 *   - Aktivna konfiguracija = slot sa najvecim seq i validnim CRC
 *   - Kad se popune svi slotovi -> EraseSector -> kreni od slota 0
 */

#ifndef FLASH_CFG_H
#define FLASH_CFG_H

#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define FLASH_CFG_SECTOR_ADDR 0x0000F000UL /* adresa sektora              */
#define FLASH_CFG_SLOT_SIZE 256U		   /* IAP minimum write = 256 B   */
#define FLASH_CFG_SECTOR_SIZE 4096U		   /* velicina sektora            */
#define FLASH_CFG_NUM_SLOTS                                                    \
	(FLASH_CFG_SECTOR_SIZE / FLASH_CFG_SLOT_SIZE) /* 16 */

/*******************************************************************************
 * Types
 ******************************************************************************/

/**
 * Jedan zapis u Flash-u. Smestena na pocetak svakog 256 B slota.
 * sizeof = 4+1+4+4+2+1 = 16 B (packed).
 *
 * seq = 0           -> rezervisan (nikad ne upisuj)
 * seq = 0xFFFFFFFF  -> neobrisan Flash (prazan slot)
 * Sve ostalo        -> validan zapis ako CRC prolazi
 */
typedef struct __attribute__((packed)) {
	uint32_t seq;	  /*  4 B */
	uint8_t mode;	  /*  1 B */
	float min_pos;	  /*  4 B */
	float center_pos; /*  4 B */
	float max_pos;	  /*  4 B */
	uint16_t crc;	  /*  2 B */
	uint8_t _pad[13]; /* 13 B */
} flash_cfg_rec_t;	  /* = 32 B, 256/32 = 8 tačno */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void flash_cfg_erase_debug(void);

/**
 * @brief  Ucitaj konfiguraciju pri bootu.
 *         Skenira svih 16 slotova, vraca onaj sa najvecim seq i validnim
 * CRC. Ako nema nijednog validnog, puni out sa default vrednostima.
 *
 * @param  out   pokazivac koji ce biti popunjen
 * @return true  nadjen validan zapis
 * @return false nije nadjen, koriscene default vrednosti
 */
bool flash_cfg_load(flash_cfg_rec_t *out);

/**
 * @brief  Sacuvaj novu konfiguraciju u Flash.
 *         Read-modify-write: cita 256 B, upisuje novi slot, vraca 256 B.
 *         Ako su svi slotovi puni, brise sektor pa pise na slot 0.
 *
 * @param  cfg   konfiguracija za upis (seq i crc se popunjavaju interno)
 * @return true  uspesno upisano
 * @return false IAP greska
 */
bool flash_cfg_save(const flash_cfg_rec_t *cfg);

#endif /* FLASH_CFG_H */
