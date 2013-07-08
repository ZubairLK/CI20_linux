/*
 * JZ4780 IRQ controller
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
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <asm/irq_cpu.h>
#include "irqchip.h"

struct jz4780_intc {
	void __iomem *base;
	struct irq_domain *domain;
};

#define INTC_ICMSR0	0x08
#define INTC_ICMCR0	0x0c
#define INTC_ICPR0	0x10
#define INTC_ICMSR1	0x28
#define INTC_ICMCR1	0x2c
#define INTC_ICPR1	0x30

static void jz4780_intc_irq_unmask(struct irq_data *d)
{
	struct jz4780_intc *intc = irq_data_get_irq_handler_data(d);

	if (d->hwirq < 32)
		writel(BIT(d->hwirq), intc->base + INTC_ICMCR0);
	else
		writel(BIT(d->hwirq - 32), intc->base + INTC_ICMCR1);
}

static void jz4780_intc_irq_mask(struct irq_data *d)
{
	struct jz4780_intc *intc = irq_data_get_irq_handler_data(d);

	if (d->hwirq < 32)
		writel(BIT(d->hwirq), intc->base + INTC_ICMSR0);
	else
		writel(BIT(d->hwirq - 32), intc->base + INTC_ICMSR1);
}

static struct irq_chip jz4780_intc_irq_chip = {
	.name		= "INTC",
	.irq_unmask	= jz4780_intc_irq_unmask,
	.irq_mask	= jz4780_intc_irq_mask,
	.irq_mask_ack	= jz4780_intc_irq_mask,
};

static void jz4780_intc_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct jz4780_intc *intc = irq_get_handler_data(irq);
	uint32_t pending;
	unsigned mapped;

	pending = readl(intc->base + INTC_ICPR0);
	if (pending) {
		mapped = irq_find_mapping(intc->domain, __ffs(pending));
		generic_handle_irq(mapped);
		return;
	}

	pending = readl(intc->base + INTC_ICPR1);
	if (pending) {
		mapped = irq_find_mapping(intc->domain, 32 + __ffs(pending));
		generic_handle_irq(mapped);
		return;
	}

	spurious_interrupt();
}

static int jz4780_intc_map(struct irq_domain *d, unsigned int irq,
			   irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &jz4780_intc_irq_chip, handle_level_irq);
	irq_set_handler_data(irq, d->host_data);
	return 0;
}

static struct irq_domain_ops jz4780_irq_domain_ops = {
	.map = jz4780_intc_map,
};

static int __init jz4780_intc_of_init(struct device_node *node,
	struct device_node *parent)
{
	struct jz4780_intc *intc;
	int irq;

	intc = kcalloc(1, sizeof(*intc), GFP_NOWAIT | __GFP_NOFAIL);

	intc->base = of_iomap(node, 0);
	if (!intc->base)
		panic("%s: unable to map registers\n", node->full_name);

	intc->domain = irq_domain_add_linear(node, 64,
		&jz4780_irq_domain_ops, intc);
	if (!intc->domain)
		panic("%s: unable to create IRQ domain\n", node->full_name);

	irq = irq_of_parse_and_map(node, 0);
	if (!irq)
		panic("%s: failed to get parent IRQ\n", node->full_name);

	irq_set_chained_handler(irq, jz4780_intc_irq_handler);
	irq_set_handler_data(irq, intc);

	return 0;
}

IRQCHIP_DECLARE(cpu_intc, "mti,cpu-interrupt-controller", mips_cpu_intc_init);
IRQCHIP_DECLARE(jz4780_intc, "ingenic,jz4780-intc", jz4780_intc_of_init);
