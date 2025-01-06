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

#define TXFTLR 0x18

#define RXFTLR 0x1c

#define TXFLR 0x20

#define RXFLR 0x24

#define SR 0x28
#define SR_BUSY_MASK BIT(0)
#define SR_RFF_MASK BIT(4)

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

#define SPI_DW_DR_SIZE 36

struct spi_dw_info {
	uint32_t regs_base;
	uint16_t sclk_div;
	bool probed;
	struct flash_device fdev;
	uint32_t tx_abw;
	uint32_t rx_abw;
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

/* Helper function to byte swap flash address */
static void swap_addr(uint32_t addr, unsigned int addr_len, uint8_t *buffer)
{
	for (buffer += addr_len; addr_len > 0; --addr_len) {
		*--buffer = addr;
		addr >>= 8;
	}
}

/* Helper to write command and address, then read data */
static int spi_dw_read_addr(struct flash_bank *bank, uint8_t cmd,
			    uint8_t *addr_buf, size_t addr_len,
			    uint8_t *data_buf, size_t data_len)
{
	struct spi_dw_info *spi_dw_info = bank->driver_priv;
	uint32_t int_buf[SPI_DW_DR_SIZE];
	int rc, exit_ret;
	uint32_t reg;
	size_t rd_offset = 0;
	size_t rd_len = 0;

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
	if (data_len > (CTRLR1_NDF_MASK + 1)) {
		LOG_ERROR("DW SPI cannot support read of %ld bytes", data_len);
		return ERROR_FAIL;
	}
	reg |= CTRLR1_NDF(data_len - 1);
	rc = spi_dw_write_reg(bank, CTRLR1, reg);
	if (rc != ERROR_OK)
		return rc;

	/* Enable SPI */
	rc = spi_dw_write_reg(bank, SSIENR, 0x1);
	if (rc != ERROR_OK)
		goto out;

	if (spi_dw_info->tx_abw < (1 + addr_len)) {
		/* Can't perform this transfer, TX FIFO is too small */
		LOG_ERROR("Cannot perform transfer, HW TX FIFO is too small");
		return ERROR_FAIL;
	}

	/* Fill TX FIFO */
	rc = spi_dw_write_reg(bank, DR0, cmd);
	if (rc != ERROR_OK)
		goto out;
	for (size_t i = 0; i < addr_len; i++) {
		reg = addr_buf[i] & 0xff;
		/* Write to DR to push to FIFO */
		rc = spi_dw_write_reg(bank, DR0, reg);
		if (rc != ERROR_OK)
			goto out;
	}

	/* Set SER to enable chip select and start transfer */
	rc = spi_dw_write_reg(bank, SER, 0x1);
	if (rc != ERROR_OK)
		goto out;

	while (data_len > 0) {
		rc = spi_dw_read_reg(bank, RXFLR, &reg); if (rc != ERROR_OK)
			goto out;
		/*
		 * This is a ugly hack- the JTAG read on the ARC is buggy, and
		 * will issue a second AXI read at the next address when
		 * accessing the data register. We work around this by always
		 * reading all 36 data registers, so the extra read just goes
		 * to an invalid address and is ignored.
		 *
		 * Therefore, we *must* wait for the RX FIFO to have at least
		 * 36 entries if we want all 36 bytes to be valid.
		 */
		rd_len = MIN(SPI_DW_DR_SIZE, reg);

		rc = spi_dw_read_reg(bank, SR, &reg);
		if (rc != ERROR_OK)
			goto out;
		if (reg & SR_RFF_MASK) {
			LOG_ERROR("RX FIFO overflowed. You likely need to "
				  "increase your baud rate divider");
			rc = ERROR_FAIL;
			goto out;
		}

		if ((rd_len != data_len) && (rd_len != SPI_DW_DR_SIZE)) {
			continue;
		}

		/* Read data clocked in */
		rc = target_read_memory(bank->target, 0x80070060, 0x4,
					SPI_DW_DR_SIZE, (uint8_t *)int_buf);
		if (rc != ERROR_OK)
			goto out;
		for (size_t i = 0; i < rd_len; i++) {
			data_buf[rd_offset++] = int_buf[i] & 0xFF;
		}

		data_len -= rd_len;
	}

out:
	exit_ret = rc;

	/* Clear RX FIFO */
	while (1) {
		rc = spi_dw_read_reg(bank, RXFLR, &reg);
		if (rc != ERROR_OK)
			return rc;
		if (reg == 0)
			break;
		rc = spi_dw_read_reg(bank, DR0, &reg);
		if (rc != ERROR_OK)
			return rc;
	}


	/* Disable SPI */
	rc = spi_dw_write_reg(bank, SSIENR, 0x0);
	if (rc != ERROR_OK)
		return rc;

	/* Clear SER */
	rc = spi_dw_write_reg(bank, SER, 0x0);
	if (rc != ERROR_OK)
		return rc;

	return exit_ret;
}

static int spi_dw_read_id(struct flash_bank *bank, uint32_t *id)
{
	/* Read JEDEC ID from flash */
	return spi_dw_read_addr(bank, SPIFLASH_READ_ID, NULL, 0, (uint8_t *)id, 3);
}

static int spi_dw_probe(struct flash_bank *bank)
{
	struct spi_dw_info *spi_dw_info = bank->driver_priv;
	const struct flash_device *fdev;
	int rc;
	uint32_t reg, dev_id;
	char *version_str = (char *)&reg;

	/* Read SPI version */
	rc = spi_dw_read_reg(bank, SSI_VERSION_ID, &reg);
	if (rc != ERROR_OK)
		return rc;
	LOG_DEBUG("Found SPI DW IP revision %c.%c.%c%c", version_str[0],
			  version_str[1], version_str[2], version_str[3]);

	/* Probe the SPI flash to determine the RX and TX fifo size */
	spi_dw_info->tx_abw = 512;
	spi_dw_info->rx_abw = 512;
	do {
		spi_dw_info->rx_abw >>= 1;
		reg = spi_dw_info->rx_abw - 1;
		rc = spi_dw_write_reg(bank, RXFTLR, reg);
		if (rc != ERROR_OK)
			return rc;
		/* Read back value */
		rc = spi_dw_read_reg(bank, RXFTLR, &reg);
		if (rc != ERROR_OK)
			return rc;
	} while (reg != (spi_dw_info->rx_abw - 1));

	do {
		spi_dw_info->tx_abw >>= 1;
		reg = spi_dw_info->tx_abw - 1;
		rc = spi_dw_write_reg(bank, TXFTLR, reg);
		if (rc != ERROR_OK)
			return rc;
		/* Read back value */
		rc = spi_dw_read_reg(bank, TXFTLR, &reg);
		if (rc != ERROR_OK)
			return rc;
	} while (reg != (spi_dw_info->tx_abw - 1));

	LOG_DEBUG("SPI TX_ABW: %d, RX_ABW: %d", spi_dw_info->tx_abw,
		  spi_dw_info->rx_abw);

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

	for (fdev = flash_devices; fdev->name != NULL; fdev++) {
		if (fdev->device_id == dev_id) {
			memcpy(&spi_dw_info->fdev, fdev, sizeof(spi_dw_info->fdev));
			LOG_INFO("flash \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
				 " kbytes", fdev->name, dev_id, fdev->size_in_bytes / 1024);
			break;
		}
	}

	if (fdev->name == NULL) {
		LOG_ERROR("Flash device was not recognized");
		return ERROR_FAIL;
	}

	bank->size = fdev->size_in_bytes;
	bank->write_start_alignment = FLASH_WRITE_ALIGN_SECTOR;
	bank->write_end_alignment = FLASH_WRITE_ALIGN_SECTOR;

	/* Create sectors array */
	bank->num_sectors = fdev->size_in_bytes / fdev->sectorsize;
	if (bank->sectors)
		free(bank->sectors);
	bank->sectors = malloc(bank->num_sectors * sizeof(struct flash_sector));
	if (!bank->sectors) {
		LOG_ERROR("Not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		bank->sectors[sector].offset = sector * fdev->sectorsize;
		bank->sectors[sector].size = fdev->sectorsize;
		bank->sectors[sector].is_erased = -1;
		bank->sectors[sector].is_protected = 0;
	}

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
	struct spi_dw_info *spi_dw_info = bank->driver_priv;
	struct flash_device *fdev = &spi_dw_info->fdev;
	const uint8_t addr_len = (fdev->read_cmd == 0x13) ? 4 : 3;
	size_t rd_count, rd_offset = 0;
	int rc;
	uint8_t addr[4];


	if (count > CTRLR1_NDF_MASK) {
		/*
		 * Can't support a read this large in one call. Split the
		 * SPI access into multiple reads
		 */
		while (count > 0) {
			rd_count = MIN(count, CTRLR1_NDF_MASK + 1);
			swap_addr(offset, addr_len, addr);

			rc = spi_dw_read_addr(bank, fdev->read_cmd, addr,
					      addr_len, &buffer[rd_offset], rd_count);
			if (rc != ERROR_OK)
				return rc;
			count -= rd_count;
			rd_offset += rd_count;
			offset += rd_count;
		}
	} else {
		swap_addr(offset, addr_len, addr);
		rc = spi_dw_read_addr(bank, fdev->read_cmd, addr,
				      addr_len, buffer, count);
	}
	return rc;
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
