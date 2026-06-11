/**
 * @file    flash_cfg.c
 * @brief   Actuator configuration - wear-leveled Flash storage (LPC17xx IAP)
 *
 * Wear-leveling strategija (sub-slot packing):
 *
 *   Sektor (4 KB) je podeljen na 16 slotova po 256 B (IAP minimum za upis).
 *   Svaki slot sadrzi 8 zapisa od 32 B (RECS_PER_SLOT = 256/32 = 8).
 *
 *   Redosled punjenja:
 *     Slot 0, rec 0  -> rec 1  -> ... -> rec 7
 *     Slot 1, rec 0  -> rec 1  -> ... -> rec 7
 *     ...
 *     Slot 15, rec 0 -> ... -> rec 7 -> EraseSector -> kreni od pocetka
 *
 *   Svaki upis (flash_cfg_save):
 *     1. Procitaj 256 B tekuceg slota u RAM buffer
 *     2. Upiši novi zapis na sledeci slobodan offset unutar buffera
 *     3. Vrati 256 B u Flash (CopyRAM2Flash)
 *
 *   Na NOR Flash bit moze ici samo 1->0 bez brisanja.
 *   Slobodni (jos neupisani) bajtovi unutar slota su 0xFF.
 *   Svaki sledeci zapis pise na visi offset — nikad ne dira vec upisane
 * bajtove. Tek kad je slot pun (svih 8 zapisa upisano), prelazimo na sledeci
 * slot. Kad su svi slotovi puni -> EraseSector -> slot 0, rec 0.
 *
 *   Ukupan kapacitet pre brisanja: 16 slotova * 8 zapisa = 128 upisa.
 */

#include "act_config.h"
#include "iap.h"

#include "LPC17xx.h"
#include "core_cmFunc.h"
#include <stddef.h>
#include <string.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define CFG_DEFAULT_MODE 0
#define CFG_DEFAULT_MIN_POS 0.0f
#define CFG_DEFAULT_CENTER_POS 50.0f
#define CFG_DEFAULT_MAX_POS 100.0f

#define REC_SIZE ((uint32_t)sizeof(flash_cfg_rec_t))   /* 32 B         */
#define RECS_PER_SLOT (FLASH_CFG_SLOT_SIZE / REC_SIZE) /* 256/32 = 8   */

#ifndef SystemCoreClock
extern uint32_t SystemCoreClock;
#endif

/*******************************************************************************
 * Private: CRC16-CCITT (poly 0x1021, init 0xFFFF)
 ******************************************************************************/

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (uint8_t j = 0; j < 8; j++) {
			crc = (crc & 0x8000U) ? ((crc << 1) ^ 0x1021U) : (crc << 1);
		}
	}
	return crc;
}

/*******************************************************************************
 * Private helpers
 ******************************************************************************/

/**
 * CRC pokriva prvih 17 B zapisa (sve do polja crc, ne ukljucujuci ga):
 *   seq(4) + mode(1) + min_pos(4) + center_pos(4) + max_pos(4) = 17 B
 */
static uint16_t record_crc(const flash_cfg_rec_t *r)
{
	return crc16_ccitt((const uint8_t *)r, offsetof(flash_cfg_rec_t, crc));
}

static bool record_is_valid(const flash_cfg_rec_t *r)
{
	if (r->seq == 0xFFFFFFFFU || r->seq == 0U)
		return false;
	return record_crc(r) == r->crc;
}

/**
 * Flash adresa pocetka slota slot_idx (0..FLASH_CFG_NUM_SLOTS-1).
 */
static inline uint32_t slot_addr(uint8_t slot_idx)
{
	return FLASH_CFG_SECTOR_ADDR + (uint32_t)slot_idx * FLASH_CFG_SLOT_SIZE;
}

/**
 * Flash adresa zapisa rec_idx unutar slota slot_idx.
 */
static inline uint32_t rec_addr(uint8_t slot_idx, uint8_t rec_idx)
{
	return slot_addr(slot_idx) + (uint32_t)rec_idx * REC_SIZE;
}

/**
 * Procitaj 256 B slota u RAM buffer.
 * Na LPC17xx Flash je direktno mapiran u memorijski prostor.
 */
static void slot_read(uint8_t slot_idx, uint8_t buf[FLASH_CFG_SLOT_SIZE])
{
	memcpy(buf, (const void *)slot_addr(slot_idx), FLASH_CFG_SLOT_SIZE);
}

/**
 * Upiši RAM buffer (256 B) na Flash slot.
 * Sekvenca: PrepareSector -> CopyRAM2Flash
 *
 * NAPOMENA: buf mora biti __attribute__((aligned(4))) u pozivacu!
 */
static bool slot_write(uint8_t slot_idx, uint8_t buf[FLASH_CFG_SLOT_SIZE])
{
	__disable_irq();

	iap_hal_sector_num_t sec = HAL_IAP_GetSectorNumber(slot_addr(slot_idx));

	if (HAL_IAP_PrepareSector(sec, sec) != IAP_HAL_CMD_SUCCESS) {
		__enable_irq();
		return false;
	}

	if (HAL_IAP_CopyRAM2Flash((uint8_t *)slot_addr(slot_idx), buf,
							  IAP_HAL_WRITE_256) != IAP_HAL_CMD_SUCCESS) {
		__enable_irq();
		return false;
	}

	__enable_irq();
	return true;
}

/**
 * Obrisi ceo Flash sektor.
 */
static bool sector_erase(void)
{
	iap_hal_sector_num_t sec = HAL_IAP_GetSectorNumber(FLASH_CFG_SECTOR_ADDR);

	__disable_irq();

	if (HAL_IAP_PrepareSector(sec, sec) != IAP_HAL_CMD_SUCCESS) {
		__enable_irq();
		return false;
	}

	if (HAL_IAP_EraseSector(sec, sec) != IAP_HAL_CMD_SUCCESS) {
		__enable_irq();
		return false;
	}

	__enable_irq();
	return true;
}

/**
 * Skeniraj ceo sektor i pronadji:
 *   out_best     - zapis sa najvecim seq i validnim CRC
 *   out_slot_idx - slot sledece slobodne pozicije (-1 ako nema slobodnog mesta)
 *   out_rec_idx  - rec  sledece slobodne pozicije (-1 ako nema slobodnog mesta)
 *
 * Slobodna pozicija = prvi rec gde seq == 0xFFFFFFFF (jos neupisan).
 * Kad je rec slobodan, svi rec-ovi iza njega u istom slotu su takodje slobodni
 * (punimo uvek od manjeg prema vecem indeksu).
 *
 * Vraca indeks slota najboljeg zapisa, ili -1 ako nema ni jednog validnog.
 */
static int scan_all(flash_cfg_rec_t *out_best, int *out_slot_idx,
					int *out_rec_idx)
{
	__attribute__((aligned(4))) uint8_t best_buf[REC_SIZE];
	memset(best_buf, 0x00, REC_SIZE);
	flash_cfg_rec_t *best = (flash_cfg_rec_t *)best_buf;
	best->seq = 0U;

	int best_slot = -1;
	int free_slot = -1;
	int free_rec = -1;

	__attribute__((aligned(4))) uint8_t rec_buf[REC_SIZE];

	for (uint8_t s = 0; s < FLASH_CFG_NUM_SLOTS; s++) {
		for (uint8_t r = 0; r < (uint8_t)RECS_PER_SLOT; r++) {
			flash_cfg_rec_t *rec = (flash_cfg_rec_t *)rec_buf;
			memcpy(rec_buf, (const void *)rec_addr(s, r), REC_SIZE);

			if (rec->seq == 0xFFFFFFFFU) {
				if (free_slot < 0) {
					free_slot = (int)s;
					free_rec = (int)r;
				}
				break;
			}

			if (record_is_valid(rec) && rec->seq > best->seq) {
				memcpy(best_buf, rec_buf, REC_SIZE);
				best_slot = (int)s;
			}
		}
	}

	if (out_best)
		memcpy(out_best, best_buf, REC_SIZE);
	if (out_slot_idx)
		*out_slot_idx = free_slot;
	if (out_rec_idx)
		*out_rec_idx = free_rec;

	return best_slot;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

bool flash_cfg_load(flash_cfg_rec_t *out)
{
	if (!out)
		return false;

	__attribute__((aligned(4))) uint8_t best_buf[REC_SIZE];
	flash_cfg_rec_t *best = (flash_cfg_rec_t *)best_buf;

	int free_slot, free_rec;
	int best_slot = scan_all(best, &free_slot, &free_rec);

	if (best_slot >= 0) {
		memcpy(out, best_buf, REC_SIZE);
		return true;
	}

	/* Nema validnog zapisa — vrati default vrednosti */
	out->seq = 0U;
	out->mode = CFG_DEFAULT_MODE;
	out->min_pos = CFG_DEFAULT_MIN_POS;
	out->center_pos = CFG_DEFAULT_CENTER_POS;
	out->max_pos = CFG_DEFAULT_MAX_POS;
	out->crc = 0U;
	memset(out->_pad, 0x00, sizeof(out->_pad));
	return false;
}

bool flash_cfg_save(const flash_cfg_rec_t *cfg)
{
	if (!cfg)
		return false;

	/* 1. Skeniraj sektor */
	__attribute__((aligned(4))) uint8_t last_buf[REC_SIZE];
	flash_cfg_rec_t *last = (flash_cfg_rec_t *)last_buf;
	int free_slot, free_rec;
	int last_slot = scan_all(last, &free_slot, &free_rec);

	uint32_t next_seq = (last_slot >= 0) ? (last->seq + 1U) : 1U;

	/* 2. Ako nema slobodnog mesta -> obrisi sektor, kreni od pocetka */
	if (free_slot < 0) {
		if (!sector_erase())
			return false;
		free_slot = 0;
		free_rec = 0;
	}

	/* 3. Procitaj 256 B tekuceg slota u RAM buffer */
	__attribute__((aligned(4))) uint8_t buf[FLASH_CFG_SLOT_SIZE];
	slot_read((uint8_t)free_slot, buf);

	/* 4. Pripremi novi zapis */
	__attribute__((aligned(4))) uint8_t rec_buf[REC_SIZE];
	memset(rec_buf, 0x00, REC_SIZE);

	flash_cfg_rec_t *rec = (flash_cfg_rec_t *)rec_buf;
	rec->seq = next_seq;
	rec->mode = cfg->mode;
	rec->min_pos = cfg->min_pos;
	rec->center_pos = cfg->center_pos;
	rec->max_pos = cfg->max_pos;
	rec->crc = record_crc(rec);

	/* 5. Upiši zapis na odgovarajuci offset unutar buffera */
	memcpy(buf + (uint32_t)free_rec * REC_SIZE, rec_buf, REC_SIZE);

	/* 6. Vrati 256 B u Flash */
	return slot_write((uint8_t)free_slot, buf);
}

void flash_cfg_erase_debug(void) { sector_erase(); }