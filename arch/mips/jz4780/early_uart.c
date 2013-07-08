/*
 * JZ4780 early UART access
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

#include <asm/io.h>
#include <jz4780.h>

static uintptr_t early_uart_base;

#define JZ4780_CPM_CLKGR0	0x20
#define JZ4780_CPM_CLKGR0_UART0_BIT 15

static void detect_early_uart(void)
{
	uint32_t clkgr0;
	int i;

	clkgr0 = readl((void *)(JZ4780_CPM_BASE + JZ4780_CPM_CLKGR0));

	for (i = 0; i < 4; i++) {
		/* skip UARTs with gated clocks */
		if (clkgr0 & (1 << (JZ4780_CPM_CLKGR0_UART0_BIT + i)))
			continue;

		/*
		 * use this UART - note we assume that if the bootloader ungated
		 * the clock then it also initialized the UART to a sane
		 * configuration.
		 */
		early_uart_base = JZ4780_UARTn_BASE(i);
		return;
	}

	/* no UART is available */
	early_uart_base = (uintptr_t)~0;
}

int prom_putchar(char c)
{
	void __iomem *thr, __iomem *lsr;

	if (!early_uart_base)
		detect_early_uart();

	if (early_uart_base == (uintptr_t)~0)
		return -1;

	thr = (void __iomem *)(early_uart_base + 0x00);
	lsr = (void __iomem *)(early_uart_base + 0x14);

	while ((readb(lsr) & 0x60) != 0x60)
		;
	writeb(c, thr);
	return 1;
}
