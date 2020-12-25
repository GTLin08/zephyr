#ifndef __RISCV_ITE_SOC_H_
#define __RISCV_ITE_SOC_H_
/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <soc_common.h>
#include <devicetree.h>

#define UART_REG_ADDR_INTERVAL 1

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               CONFIG_SRAM_BASE_ADDRESS
#define RISCV_RAM_SIZE               KB(CONFIG_SRAM_SIZE)

#define ite_write(reg, reg_size, val) \
			((*((volatile unsigned char *)(reg))) = val)
#define ite_read(reg, reg_size) \
			(*((volatile unsigned char *)(reg)))

/* PINMUX config */
#define IT8XXX2_PINMUX_IOF0		0x00
#define IT8XXX2_PINMUX_IOF1		0x01
#define IT8XXX2_PINMUX_PINS		128

/* Memory mapping */
#define CHIP_ILM_BASE               0x80000000
#define CONFIG_PROGRAM_MEMORY_BASE  (CHIP_ILM_BASE)
/* Program is run directly from storage */
#define CONFIG_MAPPED_STORAGE_BASE CONFIG_PROGRAM_MEMORY_BASE

#define CONFIG_RAM_BASE             0x80100000
#define CHIP_RAMCODE_BASE (CONFIG_RAM_BASE + 0x2000) /* base+2000h~base+2FFF */
#define IT83XX_ILM_BLOCK_SIZE       0x00001000

/*
 * The bit19 of ram code base address is controlled by bit7 of register SCARxH
 * instead of bit3.
 */
#define IT83XX_DAM_ADDR_BIT19_AT_REG_SCARXH_BIT7

#define CONFIG_FLASH_SIZE           0x00100000
#define CONFIG_FLASH_BANK_SIZE      0x00001000  /* protect bank size */

#define CONFIG_FLASH_ERASE_SIZE     0x00001000  /* erase bank size */

#endif /* __RISCV_ITE_SOC_H_ */
