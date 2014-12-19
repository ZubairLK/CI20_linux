/*
 * JZ4780 MTD NAND setup
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * Based on arch/mips/xburst/soc-4780/board/hdmi-80/hdmi-80-nand-mtd.c
 * Copyright (C) 2013 Fighter Sun <wanmyqawdr@126.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <gpio.h>

#include <mach/jz4780_nand.h>

#define GPIO_BUSY0	GPIO_PA(20)
#define GPIO_WP		GPIO_PF(22)

#define SZ_KB		1024
#define SZ_MB		(1024 * SZ_KB)
#define SZ_GB		(1024 * SZ_MB)

#define SIZE_NAND	(4ull * SZ_GB)

#define START_UBOOT	0ull
#define SIZE_UBOOT	(8ull * SZ_MB)

#define START_SYSTEM	(START_UBOOT + SIZE_UBOOT)
#define SIZE_SYSTEM	(SIZE_NAND - START_SYSTEM)

static struct mtd_partition parts[] = {
	{
		.name = "u-boot",
		.offset = START_UBOOT,
		.size = SIZE_UBOOT,
	},
	{
		.name = "system",
		.offset = START_SYSTEM,
		.size = SIZE_SYSTEM,
	},
};

static nand_flash_if_t nand_interfaces[] = {
	{ COMMON_NAND_INTERFACE(1, GPIO_BUSY0, 1, GPIO_WP, 1) },
};

#define NAND_FLASH_MT29F32G08CBACAWP_NAME    "MT29F32G08CBACAWP"
#define NAND_FLASH_MT29F32G08CBACAWP_ID      0x68

static nand_flash_info_t board_support_nand_info_adjust_table[] = {
	{
		COMMON_NAND_CHIP_INFO(
			NAND_FLASH_MT29F32G08CBACAWP_NAME,
			NAND_MFR_MICRON, NAND_FLASH_MT29F32G08CBACAWP_ID,
			1024, 24, 0,
			10, 5, 10, 5, 15, 5, 7, 5, 10, 7,
			20, 20, 70, 200, 100, 60, 200, 10, 20, 0, 100,
			100, 100 * 1000, 0, 0, 0, 5, BUS_WIDTH_8,
			NAND_OUTPUT_NORMAL_DRIVER,
			NAND_RB_DOWN_FULL_DRIVER,
			micron_nand_pre_init)
	},
};

static struct jz4780_nand_platform_data nand_pdata = {
	.part_table = parts,
	.num_part = ARRAY_SIZE(parts),

	.nand_flash_if_table = nand_interfaces,
	.num_nand_flash_if = ARRAY_SIZE(nand_interfaces),

	.ecc_type = NAND_ECC_TYPE_HW,
	.xfer_type = NAND_XFER_DMA_POLL,
	.flash_bbt = 1,

	.nand_flash_info_table = board_support_nand_info_adjust_table,
	.num_nand_flash_info = ARRAY_SIZE(board_support_nand_info_adjust_table),
};

static struct platform_device nand_dev = {
	.name = "jz4780-nand",
	.id = 0,
};

static __init int nand_mtd_device_register(void)
{
	platform_device_add_data(&nand_dev, &nand_pdata, sizeof(nand_pdata));
	return platform_device_register(&nand_dev);
}
arch_initcall(nand_mtd_device_register);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Burton <paul.burton@imgtec.com>");
MODULE_DESCRIPTION("CI20 MTD NAND setup");
