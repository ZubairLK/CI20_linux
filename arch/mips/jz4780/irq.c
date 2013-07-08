/*
 * JZ4780 IRQ handling
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

#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/linkage.h>
#include <asm/irq.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/mach-jz4780/jz4780-smp.h>

asmlinkage void plat_irq_dispatch(void)
{
	uint32_t pending = read_c0_cause() & read_c0_status() & CAUSEF_IP;

	if (pending & CAUSEF_IP4) {
		/* from OS timer */
		do_IRQ(4);
#ifdef CONFIG_SMP
	} else if (pending & CAUSEF_IP3) {
		/* from a mailbox write */
		do_IRQ(3);
#endif
	} else if (pending & CAUSEF_IP2) {
		/* from interrupt controller */
		do_IRQ(2);
	} else {
		spurious_interrupt();
	}
}

void __init arch_init_irq(void)
{
	mips_cpu_irq_init();
	irqchip_init();
}
