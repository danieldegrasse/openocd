/***************************************************************************
 *   Copyright (c) 2024 Tenstorrent AI ULC                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/* This driver implements support for interfacing with external flash memories
 * that are connected to an instance of the Synopsis SPI DW IP on the target
 * device.
 */

#include "imp.h"
#include "spi.h"

#include <stdint.h>
#include <helper/bits.h>

/* SPI DW register offset and bitmasks */
#define CTRLR0 0x0
#define CTRLR0_SECONV_MASK BIT(25)
#define CTRLR0_SSTE_MASK BIT(24)
#define CTRLR0_SPI_FRF(x) (((x) & 0x3) << 21)
#define CTRLR0_SPI_FRF_MASK CTRLR0_SPI_FRF(0x3)
#define CTRLR0_DFS_32(x) (((x) & 0x1f) << 16)
#define CTRLR0_DFS_32_MASK CTRLR0_DFS_32(0x1f)
#define CTRLR0_CFS(x) (((x) & 0xf) << 12)
#define CTRLR0_CFS_MASK CTRLR0_CFS(0xf)
#define CTRLR0_SRL_MASK BIT(11)
#define CTRLR0_SLV_OE_MASK BIT(10)
#define CTRLR0_TMOD(x) (((x) & 0x3) << 8)
#define CTRLR0_TMOD_MASK CTRLR0_TMOD(0x3)
#define CTRLR0_SCPOL_MASK BIT(7)
#define CTRLR0_SCPH_MASK BIT(6)
#define CTRLR0_FRF(x) (((x) & 0x3) << 4)
#define CTRLR0_FRF_MASK CTRLR0_FRF(0x3)
#define CTRLR0_DFS(x) ((x) & 0xf)
#define CTRLR0_DFS_MASK CTRLR0_DFS(0xf)

#define CTRLR1 0x4
#define CTRLR1_NDF(x) ((x) & 0xFFFF)
#define CTRLR1_NDF_MASK CTRLR1_NDF(0xFFFF)

#define SSIENR 0x8
#define SSIENR_SSI_EN_MASK BIT(0)

#define SER 0x10

#define BAUDR 0x14
#define BAUDR_SCKDV(x) ((x) & 0xFFFF)

#define TXFLR 0x20

#define RXFLR 0x24

#define SR 0x28
#define SR_BUSY_MASK BIT(0)

#define IMR 0x2c
#define IMR_MSTIM_MASK BIT(5)
#define IMR_RXFIM_MASK BIT(4)
#define IMR_RXOIM_MASK BIT(3)
#define IMR_RXIUM_MASK BIT(2)
#define IMR_TXOIM_MASK BIT(1)
#define IMR_TXEIM_MASK BIT(0)

#define DMACR 0x4c
#define DMACR_TDMAE_MASK BIT(1)
#define DMACR_RDMAE_MASK BIT(1)

#define SSI_VERSION_ID 0x5c

#define DR0 0x60

#define RX_SAMPLE_DLY 0xf0

struct spi_dw_info {
	uint32_t regs_base;
	uint16_t sclk_div;
	bool probed;
};

/* Helper function to write to IP register */
static int spi_dw_write_reg(struct flash_bank *bank, uint32_t offset,
			    uint32_t val)
{
	struct target *target = bank->target;
	struct spi_dw_info *spi_dw_info = bank->driver_priv;

	return target_write_u32(target, spi_dw_info->regs_base + offset, val);
}

/* Helper function to read from IP register */
static int spi_dw_read_reg(struct flash_bank *bank, uint32_t offset,
			   uint32_t *val)
{
	struct target *target = bank->target;
	struct spi_dw_info *spi_dw_info = bank->driver_priv;

	return target_read_u32(target, spi_dw_info->regs_base + offset, val);
}

/* Helper to write command and address, then read data */
static int spi_dw_read_addr(struct flash_bank *bank, uint8_t cmd,
			    uint8_t *addr_buf, size_t addr_len,
			    uint8_t *data_buf, size_t data_len)
{
	int rc;
	uint32_t reg;

	/*
	 * Configure SPI into eeprom mode, and configure length of data to
	 * receive
	 */
	rc = spi_dw_read_reg(bank, CTRLR0, &reg);
	if (rc != ERROR_OK)
		return rc;
	reg &= ~(CTRLR0_TMOD_MASK);
	reg |= CTRLR0_TMOD(0x3); /* EEPROM read mode */
	rc = spi_dw_write_reg(bank, CTRLR0, reg);
	if (rc != ERROR_OK)
		return rc;
	rc = spi_dw_read_reg(bank, CTRLR1, &reg);
	if (rc != ERROR_OK)
		return rc;
	reg &= ~(CTRLR1_NDF_MASK);
	reg |= CTRLR1_NDF(data_len - 1);
	rc = spi_dw_write_reg(bank, CTRLR1, reg);
	if (rc != ERROR_OK)
		return rc;

	/* Enable SPI */
	rc = spi_dw_write_reg(bank, SSIENR, 0x1);
	if (rc != ERROR_OK)
		return rc;

	/* Fill TX FIFO */
	rc = spi_dw_write_reg(bank, DR0, cmd);
	if (rc != ERROR_OK)
		return rc;
	for (size_t i = 0; i < addr_len; i++) {
		reg = addr_buf[i] & 0xff;
		/* Write to DR to push to FIFO */
		rc = spi_dw_write_reg(bank, DR0, reg);
		if (rc != ERROR_OK)
			return rc;
	}

	/* Set SER to enable chip select and start transfer */
	rc = spi_dw_write_reg(bank, SER, 0x1);
	if (rc != ERROR_OK)
		return rc;

	do {
		/* Poll the status register busy bit */
		rc = spi_dw_read_reg(bank, SR, &reg);
		if (rc != ERROR_OK)
			return rc;
	} while (reg & SR_BUSY_MASK);

	/* Wait for data in the RX FIFO */
	do {
		rc = spi_dw_read_reg(bank, RXFLR, &reg);
		if (rc != ERROR_OK)
			return rc;
	} while (reg == 0x0);


	uint32_t *int_buf = malloc(data_len * 4);
	if (!int_buf)
		return ERROR_FAIL;
	/* Read data clocked in */
	rc = target_read_memory(bank->target, 0x80070060, 0x4, data_len,
				(uint8_t *)int_buf);
	if (rc != ERROR_OK)
		return rc;
	for (size_t i = 0; i < data_len; i++) {
		data_buf[i] = int_buf[i] & 0xFF;
	}

	free(int_buf);

	/* Disable SPI */
	rc = spi_dw_write_reg(bank, SSIENR, 0x0);
	if (rc != ERROR_OK)

	/* Clear SER */
	rc = spi_dw_write_reg(bank, SER, 0x0);
	if (rc != ERROR_OK)
		return rc;

	return ERROR_OK;
}

// /* Helper to simulanously transmit and receive from SPI*/
// static int spi_dw_txrx(struct flash_bank *bank, uint8_t *wr_buf,
// 		       uint8_t *rd_buf, size_t len)
// {
// 	int rc;
// 	uint32_t reg;
//
// 	/* Enable SPI */
// 	rc = spi_dw_write_reg(bank, SSIENR, 0x1);
// 	if (rc != ERROR_OK)
// 		return rc;
//
// 	/* Flush any data in the RX FIFO */
// 	do {
// 		rc = spi_dw_read_reg(bank, DR0, &reg);
// 		if (rc != ERROR_OK)
// 			return rc;
// 		rc = spi_dw_read_reg(bank, RXFLR, &reg);
// 		if (rc != ERROR_OK)
// 			return rc;
// 	} while (reg != 0x0);
//
// 	/* Fill TX FIFO */
// 	for (size_t i = 0; i < len; i++) {
// 		/* Write to DR to push to FIFO */
// 		rc = spi_dw_write_reg(bank, DR0, ((uint32_t) wr_buf[i]));
// 		if (rc != ERROR_OK)
// 			return rc;
// 	}
//
// 	/* Set SER to enable chip select and start transfer */
// 	rc = spi_dw_write_reg(bank, SER, 0x1);
// 	if (rc != ERROR_OK)
// 		return rc;
//
// 	do {
// 		/* Poll the status register busy bit */
// 		rc = spi_dw_read_reg(bank, SR, &reg);
// 		if (rc != ERROR_OK)
// 			return rc;
// 	} while (reg & SR_BUSY_MASK);
//
// 	/* Wait for data in the RX FIFO */
// 	do {
// 		rc = spi_dw_read_reg(bank, RXFLR, &reg);
// 		if (rc != ERROR_OK)
// 			return rc;
// 	} while (reg == 0x0);
//
//
// 	uint32_t *int_buf = malloc(len * 4);
// 	if (!int_buf)
// 		return ERROR_FAIL;
// 	/* Read data clocked in */
// 	rc = target_read_memory(bank->target, 0x80070060, 0x4, len, (uint8_t *)int_buf);
// 	if (rc != ERROR_OK)
// 		return rc;
// 	for (size_t i = 0; i < len; i++) {
// 		rd_buf[i] = int_buf[i] & 0xFF;
// 	}
//
// 	free(int_buf);
//
// 	/* Disable SPI */
// 	rc = spi_dw_write_reg(bank, SSIENR, 0x0);
// 	if (rc != ERROR_OK)
//
// 	/* Clear SER */
// 	rc = spi_dw_write_reg(bank, SER, 0x0);
// 	if (rc != ERROR_OK)
// 		return rc;
//
// 	return ERROR_OK;
// }

static int spi_dw_read_id(struct flash_bank *bank, uint32_t *id)
{
	int rc;
	uint8_t tmp[3];
	/* Read JEDEC ID from flash */
	rc = spi_dw_read_addr(bank, SPIFLASH_READ_ID, NULL, 0,tmp, 3);
	if (rc != ERROR_OK)
		return rc;
	/* Swap bytes */
	*id = tmp[0] << 16;
	*id |= tmp[1] << 8;
	*id |= tmp[2];
	return ERROR_OK;
}

static int spi_dw_probe(struct flash_bank *bank)
{
	struct spi_dw_info *spi_dw_info = bank->driver_priv;
	int rc;
	uint32_t reg, dev_id;
	char *version_str = (char *)&reg;

	/* Read SPI version */
	rc = spi_dw_read_reg(bank, SSI_VERSION_ID, &reg);
	if (rc != ERROR_OK)
		return rc;
	LOG_DEBUG("Found SPI DW IP revision %c.%c.%c%c", version_str[0],
			  version_str[1], version_str[2], version_str[3]);

	/*
	 * Configure SPI DW instance for polling access, using serial
	 * SPI mode. Dual/Quad mode not currently supported.
	 */

	/* Disable SPI */
	rc = spi_dw_write_reg(bank, SSIENR, 0x0);
	if (rc != ERROR_OK)
		return rc;

	/* Clear RX sample delay */
	rc = spi_dw_write_reg(bank, RX_SAMPLE_DLY, 0x0);
	if (rc != ERROR_OK)
		return rc;

	/* Mask interrupts */
	rc = spi_dw_read_reg(bank, IMR, &reg);
	if (rc != ERROR_OK)
		return rc;
	reg |= (IMR_MSTIM_MASK | IMR_RXFIM_MASK | IMR_RXOIM_MASK |
		IMR_RXIUM_MASK | IMR_TXOIM_MASK | IMR_TXEIM_MASK);
	rc = spi_dw_write_reg(bank, IMR, reg);
	if (rc != ERROR_OK)
		return rc;
	/* Configure the SPI */
	rc = spi_dw_read_reg(bank, CTRLR0, &reg);
	if (rc != ERROR_OK)
		return rc;
	reg &= ~(CTRLR0_SECONV_MASK | /* Disable endian conversion */
		CTRLR0_SSTE_MASK | /* Keep SCLK running for entire transfer */
		CTRLR0_SRL_MASK | /* Disable loop mode */
		CTRLR0_SPI_FRF_MASK |
		CTRLR0_DFS_32_MASK |
		CTRLR0_CFS_MASK |
		CTRLR0_SRL_MASK | /* Disable shift register loop */
		CTRLR0_SLV_OE_MASK | /* Disable slave output */
		CTRLR0_TMOD_MASK |
		CTRLR0_SCPOL_MASK | /* Serial clock is active high */
		CTRLR0_SCPH_MASK | /* Serial clock toggles in middle of bit */
		CTRLR0_FRF_MASK |
		CTRLR0_DFS_MASK);
	reg |= (CTRLR0_SPI_FRF(0x0) | /* Use standard SPI frame format */
	       CTRLR0_DFS_32(0x7) | /* 8 bit data frame size (32 bit transfer) */
	       CTRLR0_CFS(0x0) | /* 0 bit control word */
	       CTRLR0_FRF(0x0) | /* Motorola SPI mode */
	       CTRLR0_SECONV_MASK |
	       CTRLR0_DFS(0x7)); /* 8 bit data frame size */
	rc = spi_dw_write_reg(bank, CTRLR0, reg);
	if (rc != ERROR_OK)
		return rc;

	/* Make sure DMA is disabled, we use polling mode */
	rc = spi_dw_read_reg(bank, DMACR, &reg);
	if (rc != ERROR_OK)
		return rc;
	reg &= ~(DMACR_TDMAE_MASK | DMACR_RDMAE_MASK);
	rc = spi_dw_write_reg(bank, DMACR, reg);
	if (rc != ERROR_OK)
		return rc;

	/* Configure SPI serial clock divider */
	rc = spi_dw_read_reg(bank, BAUDR, &reg);
	if (rc != ERROR_OK)
		return rc;
	reg = BAUDR_SCKDV(spi_dw_info->sclk_div);
	rc = spi_dw_write_reg(bank, BAUDR, reg);
	if (rc != ERROR_OK)
		return rc;

	rc = spi_dw_read_id(bank, &dev_id);
	if (rc != ERROR_OK)
		return rc;
	LOG_DEBUG("SPI Flash ID: 0x%03X", dev_id);



	spi_dw_info->probed = true;

	return ERROR_OK;
}

static int spi_dw_auto_probe(struct flash_bank *bank)
{
	struct spi_dw_info *spi_dw_info = bank->driver_priv;

	if (spi_dw_info->probed)
		return ERROR_OK;
	spi_dw_probe(bank);
	return ERROR_OK;
}

static int spi_dw_read(struct flash_bank *bank, uint8_t *buffer,
		       uint32_t offset, uint32_t count)
{
	return ERROR_FAIL;
}

FLASH_BANK_COMMAND_HANDLER(spi_dw_flash_bank_command)
{
	struct spi_dw_info *spi_dw_info;
	uint32_t regs_base;
	uint16_t sclk_div;

	if (CMD_ARGC < 8)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], regs_base);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[7], sclk_div);

	spi_dw_info = malloc(sizeof(struct spi_dw_info));
	if (spi_dw_info == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = spi_dw_info;

	spi_dw_info->regs_base = regs_base;
	spi_dw_info->sclk_div = sclk_div;
	spi_dw_info->probed = false;


	return ERROR_OK;
}

struct flash_driver spi_dw_flash = {
	.name = "spi_dw",
	.flash_bank_command = spi_dw_flash_bank_command,
	.probe = spi_dw_probe,
	.auto_probe = spi_dw_auto_probe,
	.read = spi_dw_read,
	.free_driver_priv = default_flash_free_driver_priv,
};
