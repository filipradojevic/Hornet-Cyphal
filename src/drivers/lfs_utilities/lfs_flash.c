/**
 * @file    lfs_flash.c
 * @brief   .
 * @version	1.0.0
 * @date    19.09.2025
 * @author  BetaTehPro  
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "lfs_flash.h"

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

/*******************************************************************************
 * Code
 ******************************************************************************/

void flash_lfs_init_config(struct lfs_config *cfg,
                           struct lfs_file_config *fcfg,
                           flash_t *flash_dev,
                           uint8_t* read_buffer,
                           size_t read_buffer_size,
                           uint8_t* prog_buffer,
                           size_t prog_buffer_size,
                           uint8_t* lookahead_buffer,
                           size_t lookahead_buffer_size,
                           uint8_t* file_buffer,
                           size_t file_buffer_size)
{
    memset(cfg, 0, sizeof(*cfg));
    memset(fcfg, 0, sizeof(*fcfg));

    cfg->context        = flash_dev;
    cfg->read           = _flash_lfs_bd_read;
    cfg->prog           = _flash_lfs_bd_prog;
    cfg->erase          = _flash_lfs_bd_erase;
    cfg->sync           = _flash_lfs_bd_sync;

    /* use sizes passed in, not sizeof(pointer) */
    cfg->read_size      = read_buffer_size;
    cfg->prog_size      = prog_buffer_size;
    cfg->block_size     = flash_dev->geometry.blkSize;
    cfg->block_count    = flash_dev->geometry.blkCnt;
    cfg->block_cycles   = -1;
    cfg->cache_size     = file_buffer_size;
    cfg->lookahead_size = lookahead_buffer_size;
    cfg->compact_thresh = -1;

    cfg->read_buffer      = read_buffer;
    cfg->prog_buffer      = prog_buffer;
    cfg->lookahead_buffer = lookahead_buffer;

    fcfg->attrs      = NULL;
    fcfg->attr_count = 0;
    fcfg->buffer     = file_buffer;
}


int _flash_lfs_bd_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 void *buffer, lfs_size_t size)
{
    flash_t *flash = (flash_t *)c->context;

    uint32_t block_addr = block;
    uint32_t page_offset = off; // off je unutar bloka

    // Izračunaj offset u bajtovima unutar bloka
    uint32_t page = page_offset / c->read_size;

    // Pretpostavljamo da se čita u jednom page-u (ako ne, moraš petlju)
    if (FLASH_PageRead(flash, block_addr, page, (uint8_t *)buffer, size) != ESP_OK) {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}



int _flash_lfs_bd_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 const void *buffer, lfs_size_t size)
{
    flash_t *flash = (flash_t *)c->context;

    uint32_t block_addr = block;
    uint32_t page_offset = off; // off je unutar bloka

    // Izračunaj offset u bajtovima unutar bloka
    uint32_t page = page_offset / c->prog_size;


    if (FLASH_PageWrite(flash, block_addr, page, (uint8_t *)buffer, size)  != ESP_OK){
        return LFS_ERR_IO;
	}

    return LFS_ERR_OK;
}

int _flash_lfs_bd_erase(const struct lfs_config *c, lfs_block_t block)
{
    flash_t *flash = (flash_t *)c->context;

    uint32_t block_addr = block;

    if (FLASH_BlockErase(flash, block_addr, 0)  != ESP_OK)
        return LFS_ERR_IO;

    return LFS_ERR_OK;
}

int _flash_lfs_bd_sync(const struct lfs_config *c)
{
    return LFS_ERR_OK;
}