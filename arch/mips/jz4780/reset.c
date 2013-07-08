/*
 * JZ4780 poweroff/reset
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
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

#include <linux/io.h>
#include <linux/pm.h>

#include <asm/reboot.h>

#include <asm/mach-jz4780/jz4780.h>

#define CPM_RSR		0x8

#define RTC_RTCCR	0x0
#define RTC_HCR		0x20
#define RTC_HWFCR	0x24
#define RTC_HRCR	0x28
#define RTC_HWCR	0x2c
#define RTC_HWRSR	0x30
#define RTC_WENR	0x3c

#define RTC_RTCCR_WRDY	BIT(7)

#define RTC_HCR_PD	BIT(0)

#define RTC_HWCR_EPDET	BIT(3)

#define RTC_WENR_PAT	0xa55a
#define RTC_WENR_WEN	BIT(31)

#define WDT_TDR		0x0
#define WDT_TCER	0x4
#define WDT_TCNT	0x8
#define WDT_TCSR	0xc
#define TCU_TSCR	0x3c

#define WDT_TCER_RTC_EN	BIT(1)

#define TCU_TSR_WDTS	BIT(16)

#define RTCLK_RATE	32768

static void jz4780_halt(void)
{
	local_irq_disable();

	while (1) {
		__asm__(".set	push\n"
			".set	mips3\n"
			"wait\n"
			".set	pop\n");
	}
}

static void jz4780_restart(char *command)
{
	void __iomem *cpm_base = (void __iomem *)JZ4780_CPM_BASE;
	void __iomem *tcu_base = (void __iomem *)JZ4780_TCU_BASE;

	local_irq_disable();

	/* Clear reset status. */
	writel(0, cpm_base + CPM_RSR);

	/* Disable watchdog timer. */
	writel(0, tcu_base + WDT_TCER);

	/* Enable clock to WDT. */
	writel(TCU_TSR_WDTS, tcu_base + TCU_TSCR);

	/* Configure watchdog timer to count up to 4 milliseconds. */
	writel(0, tcu_base + WDT_TCNT);
	writel((RTCLK_RATE * 4) / 1000, tcu_base + WDT_TDR);
	writel(WDT_TCER_RTC_EN, tcu_base + WDT_TCSR);

	/* Enable watchdog timer. */
	writel(1, tcu_base + WDT_TCER);

	jz4780_halt();
}

static inline void jz4780_rtc_wait_ready(void)
{
	void __iomem *rtccr = (void __iomem *)JZ4780_RTC_BASE + RTC_RTCCR;

	while (!(readl(rtccr) & RTC_RTCCR_WRDY))
		;
}

static inline void jz4780_rtc_write_reg(int reg, unsigned long val)
{
	void __iomem *wenr = (void __iomem *)JZ4780_RTC_BASE + RTC_WENR;
	void __iomem *addr = (void __iomem *)JZ4780_RTC_BASE + reg;

	/*
	 * Ensure that registers are writeable. Doing this once doesn't seem
	 * to be sufficient, writing certain registers causes us to hang waiting
	 * for WRDY afterwards unless this is done before each write.
	 */
	jz4780_rtc_wait_ready();
	writel(RTC_WENR_PAT, wenr);
	jz4780_rtc_wait_ready();
	while (!(readl(wenr) & RTC_WENR_WEN))
		;

	jz4780_rtc_wait_ready();
	writel(val, addr);
}

static void jz4780_power_off(void)
{
	local_irq_disable();

	/* Set minimum wakeup pin assertion time to 1 second. */
	jz4780_rtc_write_reg(RTC_HWFCR, RTCLK_RATE);

	/* Set reset pin assertion time to 60 ms. */
	jz4780_rtc_write_reg(RTC_HRCR, (60 * RTCLK_RATE / 1000));

	/* Clear wakeup status register. */
	jz4780_rtc_write_reg(RTC_HWRSR, 0);

	jz4780_rtc_write_reg(RTC_HWCR, RTC_HWCR_EPDET);

	/* Power down the chip. */
	jz4780_rtc_write_reg(RTC_HCR, RTC_HCR_PD);

	jz4780_halt();
}

static int __init jz4780_reset_setup(void)
{
	_machine_halt = jz4780_halt;
	_machine_restart = jz4780_restart;
	pm_power_off = jz4780_power_off;

	return 0;
}
arch_initcall(jz4780_reset_setup);
