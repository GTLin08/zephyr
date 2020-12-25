/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#include <device.h>
#include <drivers/flash.h>
#include <init.h>
#include <kernel.h>
#include <linker/linker-defs.h>
#include <soc.h>
#include <string.h>

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(flash_ite_it8xxx2);


extern char _flash_dma_start;
#define FLASH_DMA_START ((uint32_t)&_flash_dma_start)
#define FLASH_DMA_CODE __attribute__((section(".flash_direct_map")))
#define __ram_code __attribute__((section(".ram_code")))

#define CONFIG_FLASH_WRITE_IDEAL_SIZE 256

/* erase size of sector is 1KB or 4KB */
#define FLASH_SECTOR_ERASE_SIZE CONFIG_FLASH_ERASE_SIZE

/* page program command  */
#define FLASH_CMD_PAGE_WRITE    0x2
/* ector erase command (erase size is 4KB) */
#define FLASH_CMD_SECTOR_ERASE  0x20
/* command for flash write */
#define FLASH_CMD_WRITE         FLASH_CMD_PAGE_WRITE

/* Write status register */
#define FLASH_CMD_WRSR         0x01
/* Write disable */
#define FLASH_CMD_WRDI         0x04
/* Write enable */
#define FLASH_CMD_WREN         0x06
/* Read status register */
#define FLASH_CMD_RS           0x05

static int flash_dma_code_enabled;
static int all_protected;

#define FWP_REG(bank) (bank / 8)
#define FWP_MASK(bank) (1 << (bank % 8))

enum flash_wp_interface {
	FLASH_WP_HOST = 0x01,
	FLASH_WP_DBGR = 0x02,
	FLASH_WP_EC = 0x04,
};

enum flash_status_mask {
	FLASH_SR_NO_BUSY = 0,
	/* Internal write operation is in progress */
	FLASH_SR_BUSY = 0x01,
	/* Device is memory Write enabled */
	FLASH_SR_WEL = 0x02,

	FLASH_SR_ALL = (FLASH_SR_BUSY | FLASH_SR_WEL),
};

struct flash_it8xxx2_dev_data {
	struct k_sem sem;
};

static const struct flash_parameters flash_it8xxx2_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

#define DEV_CFG(dev) \
	((const struct flash_it8xxx2_dev_cfg *const)(dev)->config)

#define DEV_DATA(dev) \
	((struct flash_it8xxx2_dev_data *const)(dev)->data)


void __ram_code interrupt_disable(void)
{
	/* bit11: disable MEIE */
	__asm__ volatile ("li t0, 0x800");
	__asm__ volatile ("csrc mie, t0");
}

void __ram_code interrupt_enable(void)
{
	/* bit11: enable MEIE */
	__asm__ volatile ("li t0, 0x800");
	__asm__ volatile ("csrs  mie, t0");
}

void FLASH_DMA_CODE dma_reset_immu(void)
{
	/* Immu tag sram reset */
	IT83XX_GCTRL_MCCR |= 0x10;

	IT83XX_GCTRL_MCCR &= ~0x10;

}

void FLASH_DMA_CODE dma_flash_follow_mode(void)
{
	/*
	 * ECINDAR3-0 are EC-indirect memory address registers.
	 *
	 * Enter follow mode by writing 0xf to low nibble of ECINDAR3 register,
	 * and set high nibble as 0x4 to select internal flash.
	 */
	IT83XX_SMFI_ECINDAR3 = (EC_INDIRECT_READ_INTERNAL_FLASH | 0x0F);

	/* Set FSCE# as high level by writing 0 to address xfff_fe00h */
	IT83XX_SMFI_ECINDAR2 = 0xFF;
	IT83XX_SMFI_ECINDAR1 = 0xFE;
	IT83XX_SMFI_ECINDAR0 = 0x00;

	/* EC-indirect memory data register */
	IT83XX_SMFI_ECINDDR = 0x00;
}

void FLASH_DMA_CODE dma_flash_follow_mode_exit(void)
{
	/* Exit follow mode, and keep the setting of selecting internal flash */
	IT83XX_SMFI_ECINDAR3 = EC_INDIRECT_READ_INTERNAL_FLASH;
	IT83XX_SMFI_ECINDAR2 = 0x00;
}

void FLASH_DMA_CODE dma_flash_fsce_high(void)
{
	/* FSCE# high level */
	IT83XX_SMFI_ECINDAR1 = 0xFE;
	IT83XX_SMFI_ECINDDR = 0x00;
}

void FLASH_DMA_CODE dma_flash_write_dat(uint8_t wdata)
{
	/* Write data to FMOSI */
	IT83XX_SMFI_ECINDDR = wdata;
}

void FLASH_DMA_CODE dma_flash_transaction(int wlen, uint8_t *wbuf,
					int rlen, uint8_t *rbuf, int cmd_end)
{
	int i;

	/*  FSCE# with low level */
	IT83XX_SMFI_ECINDAR1 = 0xFD;
	/* Write data to FMOSI */
	for (i = 0; i < wlen; i++)
		IT83XX_SMFI_ECINDDR = wbuf[i];
	/* Read data from FMISO */
	for (i = 0; i < rlen; i++)
		rbuf[i] = IT83XX_SMFI_ECINDDR;

	/* FSCE# high level if transaction done */
	if (cmd_end)
		dma_flash_fsce_high();
}

void FLASH_DMA_CODE dma_flash_cmd_read_status(enum flash_status_mask mask,
						enum flash_status_mask target)
{
	uint8_t status[1];
	uint8_t cmd_rs[] = {FLASH_CMD_RS};

	/*
	 * We prefer no timeout here. We can always get the status
	 * we want, or wait for watchdog triggered to check
	 * e-flash's status instead of breaking loop.
	 * This will avoid fetching unknown instruction from e-flash
	 * and causing exception.
	 */
	while (1) {
		/* read status */
		dma_flash_transaction(sizeof(cmd_rs), cmd_rs, 1, status, 1);
		/* only bit[1:0] valid */
		if ((status[0] & mask) == target)
			break;
	}

}

void FLASH_DMA_CODE dma_flash_cmd_write_enable(void)
{
	uint8_t cmd_we[] = {FLASH_CMD_WREN};

	/* enter EC-indirect follow mode */
	dma_flash_follow_mode();
	/* send write enable command */
	dma_flash_transaction(sizeof(cmd_we), cmd_we, 0, NULL, 1);
	/* read status and make sure busy bit cleared and write enabled. */
	dma_flash_cmd_read_status(FLASH_SR_ALL, FLASH_SR_WEL);
	/* exit EC-indirect follow mode */
	dma_flash_follow_mode_exit();
}

void FLASH_DMA_CODE dma_flash_cmd_write_disable(void)
{
	uint8_t cmd_wd[] = {FLASH_CMD_WRDI};

	/* enter EC-indirect follow mode */
	dma_flash_follow_mode();
	/* send write disable command */
	dma_flash_transaction(sizeof(cmd_wd), cmd_wd, 0, NULL, 1);
	/* make sure busy bit cleared. */
	dma_flash_cmd_read_status(FLASH_SR_ALL, FLASH_SR_NO_BUSY);
	/* exit EC-indirect follow mode */
	dma_flash_follow_mode_exit();
}

int FLASH_DMA_CODE dma_flash_verify(int addr, int size,
					const char *data)
{
	int i;
	uint8_t *wbuf = (uint8_t *)data;
	uint8_t *flash = (uint8_t *)addr;

	/* verify for erase */
	if (data == NULL) {
		for (i = 0; i < size; i++) {
			if (flash[i] != 0xFF)
				return EINVAL;
		}
	/* verify for write */
	} else {
		for (i = 0; i < size; i++) {
			if (flash[i] != wbuf[i])
				return EINVAL;
		}
	}

	return 0;
}

void FLASH_DMA_CODE dma_flash_cmd_write(int addr, int wlen, uint8_t *wbuf)
{
	int i;
	uint8_t flash_write[] = {FLASH_CMD_WRITE, ((addr >> 16) & 0xFF),
				((addr >> 8) & 0xFF), (addr & 0xFF)};

	/* enter EC-indirect follow mode */
	dma_flash_follow_mode();
	/* send flash write command (aai word or page program) */
	dma_flash_transaction(sizeof(flash_write), flash_write, 0, NULL, 0);

	for (i = 0; i < wlen; i++) {
		/* send data byte */
		dma_flash_write_dat(wbuf[i]);

		/*
		 * we want to restart the write sequence every IDEAL_SIZE
		 * chunk worth of data.
		 */
		if (!(++addr % CONFIG_FLASH_WRITE_IDEAL_SIZE)) {
			uint8_t w_en[] = {FLASH_CMD_WREN};

			dma_flash_fsce_high();
			/* make sure busy bit cleared. */
			dma_flash_cmd_read_status(FLASH_SR_BUSY, FLASH_SR_NO_BUSY);
			/* send write enable command */
			dma_flash_transaction(sizeof(w_en), w_en, 0, NULL, 1);
			/* make sure busy bit cleared and write enabled. */
			dma_flash_cmd_read_status(FLASH_SR_ALL, FLASH_SR_WEL);
			/* re-send write command */
			flash_write[1] = (addr >> 16) & 0xff;
			flash_write[2] = (addr >> 8) & 0xff;
			flash_write[3] = addr & 0xff;
			dma_flash_transaction(sizeof(flash_write), flash_write,
				0, NULL, 0);
		}
	}
	dma_flash_fsce_high();
	/* make sure busy bit cleared. */
	dma_flash_cmd_read_status(FLASH_SR_BUSY, FLASH_SR_NO_BUSY);
	/* exit EC-indirect follow mode */
	dma_flash_follow_mode_exit();
}

void FLASH_DMA_CODE dma_flash_write(int addr, int wlen,
						const char *wbuf)
{
	dma_flash_cmd_write_enable();
	dma_flash_cmd_write(addr, wlen, (uint8_t *)wbuf);
	dma_flash_cmd_write_disable();
}

void FLASH_DMA_CODE dma_flash_cmd_erase(int addr, int cmd)
{
	uint8_t cmd_erase[] = {cmd, ((addr >> 16) & 0xFF),
				((addr >> 8) & 0xFF), (addr & 0xFF)};

	/* enter EC-indirect follow mode */
	dma_flash_follow_mode();
	/* send erase command */
	dma_flash_transaction(sizeof(cmd_erase), cmd_erase, 0, NULL, 1);
	/* make sure busy bit cleared. */
	dma_flash_cmd_read_status(FLASH_SR_BUSY, FLASH_SR_NO_BUSY);
	/* exit EC-indirect follow mode */
	dma_flash_follow_mode_exit();
}

void FLASH_DMA_CODE dma_flash_erase(int addr, int cmd)
{
	dma_flash_cmd_write_enable();
	dma_flash_cmd_erase(addr, cmd);
	dma_flash_cmd_write_disable();
}

/**
 * Protect flash banks until reboot.
 *
 * @param start_bank    Start bank to protect
 * @param bank_count    Number of banks to protect
 */
static void flash_protect_banks(int start_bank, int bank_count,
				enum flash_wp_interface wp_if)
{
	int bank;

	for (bank = start_bank; bank < start_bank + bank_count; bank++) {
		if (wp_if & FLASH_WP_EC)
			IT83XX_GCTRL_EWPR0PFEC(FWP_REG(bank)) |= FWP_MASK(bank);
		if (wp_if & FLASH_WP_HOST)
			IT83XX_GCTRL_EWPR0PFH(FWP_REG(bank)) |= FWP_MASK(bank);
		if (wp_if & FLASH_WP_DBGR)
			IT83XX_GCTRL_EWPR0PFD(FWP_REG(bank)) |= FWP_MASK(bank);
	}
}

/* Read data from flash */
static int FLASH_DMA_CODE flash_it8xxx2_read(const struct device *dev,
					off_t offset, void *data, size_t len)
{
	int i;
	uint8_t *data_t = data;

	//printk("read - offset = 0x%lx, len = %zu\n", (long)offset, len);

	for (i = 0; i < len; i++) {
		IT83XX_SMFI_ECINDAR3 = EC_INDIRECT_READ_INTERNAL_FLASH;
		IT83XX_SMFI_ECINDAR2 = (offset >> 16) & 0xFF;
		IT83XX_SMFI_ECINDAR1 = (offset >> 8) & 0xFF;
		IT83XX_SMFI_ECINDAR0 = (offset & 0xFF);

		/*
		 * Read/Write to this register will access one byte on the
		 * flash with the 32-bit flash address defined in ECINDAR3-0
		 */
		data_t[i] = IT83XX_SMFI_ECINDDR;

		offset++;
	}

	return 0;
}

/* Write data to the flash, page by page */
static int FLASH_DMA_CODE flash_it8xxx2_write(const struct device *dev,
					off_t offset, const void *data, size_t len)
{
	int ret = EINVAL;
	struct flash_it8xxx2_dev_data *data_dev = DEV_DATA(dev);

	/*
	 * Check that the offset and length are multiples of the write
	 * block size.
	 */
	if ((offset % FLASH_WRITE_BLK_SZ) != 0) {
		return -EINVAL;
	}
	if ((len % FLASH_WRITE_BLK_SZ) != 0) {
		return -EINVAL;
	}

	if (flash_dma_code_enabled == 0)
		return EACCES;

	if (all_protected)
		return EACCES;

	/*
	 * CPU can't fetch instruction from flash while use
	 * EC-indirect follow mode to access flash, interrupts need to be
	 * disabled.
	 */
	interrupt_disable();

	k_sem_take(&data_dev->sem, K_FOREVER);

	dma_flash_write(offset, len, data);
	dma_reset_immu();
	/*
	 * Internal flash of N8 or RISC-V core is ILM(Instruction Local Memory)
	 * mapped, but RISC-V's ILM base address is 0x80000000.
	 *
	 * Ensure that we will get the ILM address of a flash offset.
	 */
	offset |= CONFIG_MAPPED_STORAGE_BASE;
	ret = dma_flash_verify(offset, len, data);

	interrupt_enable();

	k_sem_give(&data_dev->sem);

	return ret;
}

/* Erase multiple blocks */
static int FLASH_DMA_CODE flash_it8xxx2_erase(const struct device *dev,
					off_t offset, size_t len)
{
	int v_size = len, v_addr = offset, ret = EINVAL;
	struct flash_it8xxx2_dev_data *data = DEV_DATA(dev);

	/*
	 * Check that the offset and length are multiples of the write
	 * erase block size.
	 */
	if ((offset % FLASH_ERASE_BLK_SZ) != 0) {
		return -EINVAL;
	}
	if ((len % FLASH_ERASE_BLK_SZ) != 0) {
		return -EINVAL;
	}

	if (flash_dma_code_enabled == 0)
		return EACCES;

	if (all_protected)
		return EACCES;

	/* Make sure no interrupt while enable static DMA */
	interrupt_disable();

	k_sem_take(&data->sem, K_FOREVER);

	/* Always use sector erase command (1K or 4K bytes) */
	for (; len > 0; len -= FLASH_SECTOR_ERASE_SIZE) {
		dma_flash_erase(offset, FLASH_CMD_SECTOR_ERASE);
		offset += FLASH_SECTOR_ERASE_SIZE;
	}
	dma_reset_immu();
	/* get the ILM address of a flash offset. */
	v_addr |= CONFIG_MAPPED_STORAGE_BASE;
	ret = dma_flash_verify(v_addr, v_size, NULL);

	k_sem_give(&data->sem);

	interrupt_enable();

	return ret;
}

/* Enable or disable the write protection */
static int flash_it8xxx2_write_protection(const struct device *dev,
					bool enable)
{
	if (enable) {
		/* Protect the entire flash */
		flash_protect_banks(0, CONFIG_FLASH_SIZE / CONFIG_FLASH_BANK_SIZE,
			FLASH_WP_EC);
		all_protected = 1;
	} else {
		all_protected = 0;
	}

	/*
	 * bit[0], eflash protect lock register which can only be write 1 and
	 * only be cleared by power-on reset.
	 */
	IT83XX_GCTRL_EPLR |= 0x01;

	return 0;
}

static const struct flash_parameters *
flash_it8xxx2_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_it8xxx2_parameters;
}

static void flash_code_static_dma(void)
{
	/* Make sure no interrupt while enable static DMA */
	interrupt_disable();

	/* invalid static DMA first */
	IT83XX_GCTRL_RVILMCR0 &= ~ILMCR_ILM2_ENABLE;

	IT83XX_SMFI_SCAR2H = 0x08;

	memcpy((void *)CHIP_RAMCODE_BASE, (const void *)FLASH_DMA_START,
		IT83XX_ILM_BLOCK_SIZE);

	/* RISCV ILM 2 Enable */
	IT83XX_GCTRL_RVILMCR0 |= ILMCR_ILM2_ENABLE;

	/*
	 * Enable ILM
	 * Set the logic memory address(flash code of RO/RW) in eflash
	 * by programming the register SCARx bit19-bit0.
	 */
	IT83XX_SMFI_SCAR2L = FLASH_DMA_START & 0xFF;
	IT83XX_SMFI_SCAR2M = (FLASH_DMA_START >> 8) & 0xFF;
#ifdef IT83XX_DAM_ADDR_BIT19_AT_REG_SCARXH_BIT7
	IT83XX_SMFI_SCAR2H = (FLASH_DMA_START >> 16) & 0x7;
	if (FLASH_DMA_START & BIT(19))
		IT83XX_SMFI_SCAR2H |= BIT(7);
	else
		IT83XX_SMFI_SCAR2H &= ~BIT(7);
#else
	IT83XX_SMFI_SCAR2H = (FLASH_DMA_START >> 16) & 0x0F;
#endif
	/*
	 * Validate Direct-map SRAM function by programming
	 * register SCARx bit20=0
	 */
	IT83XX_SMFI_SCAR2H &= ~0x10;

	flash_dma_code_enabled = 0x01;

	interrupt_enable();

}

static int flash_it8xxx2_init(const struct device *dev)
{
	struct flash_it8xxx2_dev_data *data = DEV_DATA(dev);

	/* By default, select internal flash for indirect fast read. */
	IT83XX_SMFI_ECINDAR3 = EC_INDIRECT_READ_INTERNAL_FLASH;

	/*
	 * If the embedded flash's size of this part number is larger
	 * than 256K-byte, enable the page program cycle constructed
	 * by EC-Indirect Follow Mode.
	 */
	IT83XX_SMFI_FLHCTRL6R |= IT83XX_SMFI_MASK_ECINDPP;

	k_sem_init(&data->sem, 1, 1);

	flash_code_static_dma();

	return 0;
}

static const struct flash_driver_api flash_it8xxx2_api = {
	.write_protection = flash_it8xxx2_write_protection,
	.erase = flash_it8xxx2_erase,
	.write = flash_it8xxx2_write,
	.read = flash_it8xxx2_read,
	.get_parameters = flash_it8xxx2_get_parameters,

};

static struct flash_it8xxx2_dev_data flash_it8xxx2_data;

DEVICE_DT_INST_DEFINE(0,
                    flash_it8xxx2_init,
                    device_pm_control_nop,
                    &flash_it8xxx2_data,
                    NULL,
                    POST_KERNEL,
                    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                    &flash_it8xxx2_api);
