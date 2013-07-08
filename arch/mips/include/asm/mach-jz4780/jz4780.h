/*
 * JZ4780 SoC definitions
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

#ifndef __MIPS_ASM_MACH_JZ4780_JZ4780_H__
#define __MIPS_ASM_MACH_JZ4780_JZ4780_H__

/* APB bus devices */
#define JZ4780_CPM_BASE		0xb0000000
#define JZ4780_TCU_BASE		0xb0002000
#define JZ4780_RTC_BASE		0xb0003000
#define JZ4780_GPIO_BASE	0xb0010000
#define JZ4780_UART0_BASE	0xb0030000
#define JZ4780_UART1_BASE	0xb0031000
#define JZ4780_UART2_BASE	0xb0032000
#define JZ4780_UART3_BASE	0xb0033000
#define JZ4780_UARTn_BASE(n)	(JZ4780_UART0_BASE + (n * 0x1000))

#endif /* __MIPS_ASM_MACH_JZ4780_JZ4780_H__ */
