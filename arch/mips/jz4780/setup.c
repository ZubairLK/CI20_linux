/*
 * JZ4780 setup
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
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

#include <linux/bootmem.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>

#include <asm/bootinfo.h>
#include <asm/fw/fw.h>
#include <asm/mach-jz4780/jz4780-smp.h>
#include <asm/prom.h>

void __init plat_mem_setup(void)
{
	__dt_setup_arch(__dtb_start);
	strlcpy(boot_command_line, arcs_cmdline, COMMAND_LINE_SIZE);
}

void __init prom_init(void)
{
	jz4780_smp_init();
	fw_init_cmdline();
}

void prom_free_prom_memory(void)
{
}

void __init plat_time_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
}

void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();
}

const char *get_system_type(void)
{
	return "JZ4780 based system";
}

static struct of_device_id __initdata jz4780_ids[] = {
	{ .compatible = "simple-bus", },
	{},
};

int __init jz4780_publish_devices(void)
{
	return of_platform_bus_probe(NULL, jz4780_ids, NULL);
}
device_initcall(jz4780_publish_devices);
