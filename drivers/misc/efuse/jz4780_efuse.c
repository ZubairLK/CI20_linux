/*
 * jz4780_efuse.c - Driver to read/write from the JZ4780 one time programmable
 * efuse 8K memory
 *
 * The rom itself is accessed using a 9 bit address line and an 8 word wide bus
 * which reads/writes based on strobes. The strobe is configured in the config
 * register and is based on number of cycles of the AHB2 clock.
 *
 * Copyright (C) 2014 Imagination Technologies
 *
 * Based on work by Ingenic Semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "jz4780_efuse.h"

void dump_jz_efuse(struct jz_efuse *efuse)
{
	dev_info(efuse->dev, "max_program_length = %x\n",
		 efuse->max_program_length);
	dev_info(efuse->dev, "use_count = %x\n", efuse->use_count);
	dev_info(efuse->dev, "is_timer_on = %x\n", efuse->is_timer_on);
	dev_info(efuse->dev, "gpio_vddq_en_n = %x\n", efuse->gpio_vddq_en_n);

	dev_info(efuse->dev, "rd_adj = %x\n", efuse->efucfg_info.rd_adj);
	dev_info(efuse->dev, "rd_strobe = %x\n", efuse->efucfg_info.rd_strobe);
	dev_info(efuse->dev, "wr_adj = %x\n", efuse->efucfg_info.wr_adj);
	dev_info(efuse->dev, "wr_strobe = %x\n", efuse->efucfg_info.wr_strobe);

	dev_info(efuse->dev, "min_rd_adj = %x\n",
		 efuse->efucfg_info.strict.min_rd_adj);
	dev_info(efuse->dev, "min_rd_adj_strobe = %x\n",
			efuse->efucfg_info.strict.min_rd_adj_strobe);
	dev_info(efuse->dev, "min_wr_adj = %x\n",
		 efuse->efucfg_info.strict.min_wr_adj);
	dev_info(efuse->dev, "min_wr_adj_strobe = %x\n",
			efuse->efucfg_info.strict.min_wr_adj_strobe);
	dev_info(efuse->dev, "max_wr_adj_strobe = %x\n",
			efuse->efucfg_info.strict.max_wr_adj_strobe);
}

static void jz_efuse_vddq_set(unsigned long efuse_ptr)
{
	struct jz_efuse *efuse = (struct jz_efuse *)efuse_ptr;

	dev_info(efuse->dev, "JZ4780-EFUSE: vddq_set %d\n",
		 (int)efuse->is_timer_on);

	if (efuse->is_timer_on)
		mod_timer(&efuse->vddq_protect_timer, jiffies + HZ);

	gpio_set_value(efuse->gpio_vddq_en_n, !efuse->is_timer_on);
}

static inline int jz_efuse_get_skip(size_t size)
{
	if (size >= 32)
		return 32;
	else if ((size / 4) > 0)
		return (size / 4) * 4;
	else
		return size % 4;
}

static int calculate_efuse_strict(struct jz_efuse *efuse,
				  struct jz_efuse_strict *t, unsigned long rate)
{
	unsigned int tmp;

	/* The EFUSE read/write strobes need to be within these values for it
	 * to function properly. Based on programmers manual.
	 */

	tmp = (((6500 * (rate / 1000000)) / 1000000) + 1) - 1;
	if (tmp > 0xf) {
		dev_err(efuse->dev, "Cannot calculate min RD_ADJ\n");
		return -EINVAL;
	} else {
		t->min_rd_adj = tmp;
	}

	tmp = ((((35000 * (rate / 1000000)) / 1000000) + 1) - 5);
	if (tmp > (0xf + 0xf)) {
		dev_err(efuse->dev, "Cannot calculate min RD_STROBE\n");
		return -EINVAL;
	} else {
		t->min_rd_adj_strobe = tmp;
	}

	tmp = (((6500 * (rate / 1000000)) / 1000000) + 1) - 1;
	if (tmp > 0xf) {
		dev_err(efuse->dev, "Cannot calculate min WR_ADJ\n");
		return -EINVAL;
	} else {
		t->min_wr_adj = tmp;
	}

	tmp = ((((9000000 / 1000000) * (rate / 1000000))) + 1) - 1666;
	if (tmp > (0xfff + 0xf)) {
		dev_err(efuse->dev, "Cannot calculate min WR_STROBE\n");
		return -EINVAL;
	} else {
		t->min_wr_adj_strobe = tmp;
	}

	tmp = ((((11000000 / 1000000) * (rate / 1000000))) + 1) - 1666;
	if (tmp > (0xfff + 0xf)) {
		dev_err(efuse->dev, "Cannot calculate max WR_STROBE\n");
		return -EINVAL;
	} else {
		t->max_wr_adj_strobe = tmp;
	}

	return 0;
}

static int jz_init_efuse_cfginfo(struct jz_efuse *efuse, unsigned long clk_rate)
{
	struct jz_efucfg_info *info = &efuse->efucfg_info;
	struct jz_efuse_strict *s = &info->strict;
	unsigned int tmp = 0;

	if (calculate_efuse_strict(efuse, s, clk_rate) < 0) {
		dev_err(efuse->dev, "can't calculate strobe parameters\n");
		return -EINVAL;
	}

	info->rd_adj = (s->min_rd_adj + 0xf) / 2;
	tmp = s->min_rd_adj_strobe - info->rd_adj;
	info->rd_strobe = ((tmp + 0xf) / 2 < 7) ? 7 : (tmp + 0xf) / 2;
	if (info->rd_strobe > 0xf) {
		dev_err(efuse->dev, "can't calculate read strobe\n");
		return -EINVAL;
	}

	tmp = (s->min_wr_adj_strobe + s->max_wr_adj_strobe) / 2;
	info->wr_adj = tmp < 0xf ? tmp : 0xf;
	info->wr_strobe = tmp - info->wr_adj;
	if (info->wr_strobe > 0xfff) {
		dev_err(efuse->dev, "can't calculate write strobe\n");
		return -EINVAL;
	}

	return 0;
}

static int jz_efuse_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_efuse *efuse = container_of(dev, struct jz_efuse, mdev);

	spin_lock(&efuse->lock);
	efuse->use_count++;
	spin_unlock(&efuse->lock);

	return 0;
}

static int jz_efuse_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_efuse *efuse = container_of(dev, struct jz_efuse, mdev);

	spin_lock(&efuse->lock);
	efuse->use_count--;
	spin_unlock(&efuse->lock);

	return 0;
}

static ssize_t _jz_efuse_read_bytes(struct jz_efuse *efuse, char *buf,
		unsigned int addr, int skip)
{
	unsigned int tmp = 0;
	int i = 0;
	unsigned long flag;
	int timeout = 1000;

	/* 1. Set config register */
	spin_lock_irqsave(&efuse->lock, flag);
	tmp = readl(efuse->iomem + JZ_EFUCFG);
	tmp &= ~((JZ_EFUSE_EFUCFG_RD_ADJ_MASK << JZ_EFUSE_EFUCFG_RD_ADJ_SHIFT)
	       | (JZ_EFUSE_EFUCFG_RD_STR_MASK << JZ_EFUSE_EFUCFG_RD_STR_SHIFT));
	tmp |= (efuse->efucfg_info.rd_adj << JZ_EFUSE_EFUCFG_RD_ADJ_SHIFT)
	       | (efuse->efucfg_info.rd_strobe << JZ_EFUSE_EFUCFG_RD_STR_SHIFT);
	writel(tmp, efuse->iomem + JZ_EFUCFG);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/*
	 * 2. Set control register to indicate what to read data address,
	 * read data numbers and read enable.
	 */
	spin_lock_irqsave(&efuse->lock, flag);
	tmp = readl(efuse->iomem + JZ_EFUCTRL);
	tmp &= ~(JZ_EFUSE_EFUCFG_RD_STR_SHIFT
		| (JZ_EFUSE_EFUCTRL_ADDR_MASK << JZ_EFUSE_EFUCTRL_ADDR_SHIFT)
		| JZ_EFUSE_EFUCTRL_PG_EN | JZ_EFUSE_EFUCTRL_WR_EN
		| JZ_EFUSE_EFUCTRL_WR_EN);

	if (addr >= (JZ_EFUSE_START_ADDR + 0x200))
		tmp |= JZ_EFUSE_EFUCTRL_CS;

	tmp |= (addr << JZ_EFUSE_EFUCTRL_ADDR_SHIFT)
		| ((skip - 1) << JZ_EFUSE_EFUCTRL_LEN_SHIFT)
		| JZ_EFUSE_EFUCTRL_RD_EN;
	writel(tmp, efuse->iomem + JZ_EFUCTRL);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/*
	 * 3. Wait status register RD_DONE set to 1 or EFUSE interrupted,
	 * software can read EFUSE data buffer 0 – 8 registers.
	 */
	do {
		tmp = readl(efuse->iomem + JZ_EFUSTATE);
		usleep_range(100, 200);
		if (timeout--)
			break;
	} while (!(tmp & JZ_EFUSE_EFUSTATE_RD_DONE));

	if (timeout <= 0) {
		dev_err(efuse->dev, "Timed out while reading\n");
		return -EAGAIN;
	}

	if ((skip % 4) == 0) {
		for (i = 0; i < (skip / 4); i++) {
			*((unsigned int *)(buf + i * 4))
				= readl(efuse->iomem + JZ_EFUDATA(i));
		}
	} else {
		*((unsigned int *)buf)
			= readl(efuse->iomem + JZ_EFUDATA(0)) & BYTEMASK(skip);
	}

	return 0;
}

static ssize_t _jz_efuse_read(struct jz_efuse *efuse, char *buf,
		size_t size, loff_t *l)
{
	int bytes = 0;
	unsigned int addr = 0, start = *l;
	int skip = jz_efuse_get_skip(size - bytes);
	int org_skip = 0;

	for (addr = start; addr < start + size; addr += org_skip) {
		if (_jz_efuse_read_bytes(efuse, buf + bytes, addr,
					skip) < 0) {
			dev_err(efuse->dev, "Can't read addr=%x\n", addr);
			return -1;
		}

		*l += skip;
		bytes += skip;

		org_skip = skip;
		skip = jz_efuse_get_skip(size - bytes);
	}

	return bytes;
}


static ssize_t jz_efuse_read(struct file *filp, char *buf, size_t size,
		loff_t *lpos)
{
	struct miscdevice *dev = filp->private_data;
	struct jz_efuse *efuse = container_of(dev, struct jz_efuse, mdev);
	int ret = -1;
	char *tmp_buf = NULL;

	if (size > (JZ_EFUSE_END_ADDR - (JZ_EFUSE_START_ADDR + *lpos) + 1)) {
		dev_err(efuse->dev, "Trying to read beyond efuse\n");
		return -EINVAL;
	}

	tmp_buf = devm_kzalloc(efuse->dev, size, GFP_KERNEL);
	if (!tmp_buf)
		ret = -ENOMEM;

	ret = _jz_efuse_read(efuse, tmp_buf, size, lpos);
	if (ret < 0) {
		dev_err(efuse->dev, "Could not read efuse\n");
		return -1;
	}

	copy_to_user(buf, tmp_buf, ret);

	return ret;
}

static int is_space_written(const char *tmp, const char *buf, int skip)
{
	int i = 0;

	for (i = 0; i < skip; i++)
		if ((tmp[i] & buf[i]) > 0)
			return 1;

	return 0;
}

static ssize_t _jz_efuse_write_bytes(struct jz_efuse *efuse,
		const char *buf, unsigned int addr, int skip)
{
	unsigned int tmp_buf[8];
	unsigned int tmp = 0;
	unsigned long flag = 0;
	int i = 0;
	int timeout = 1000;

	if (_jz_efuse_read_bytes(efuse, (char *)&tmp_buf, addr, skip) < 0) {
		dev_err(efuse->dev, "read efuse at addr = %x failed\n", addr);
		return -EINVAL;
	}

	if (is_space_written((char *)tmp_buf, (char *)buf, skip)) {
		dev_err(efuse->dev,
			"ERROR: the write spaced has already been written\n");
		return -EINVAL;
	}

	/* 1. Set config register */
	spin_lock_irqsave(&efuse->lock, flag);
	tmp = readl(efuse->iomem + JZ_EFUCFG);
	tmp &= ~((JZ_EFUSE_EFUCFG_WR_ADJ_MASK << JZ_EFUSE_EFUCFG_WR_ADJ_SHIFT)
	       | (JZ_EFUSE_EFUCFG_WR_STR_MASK << JZ_EFUSE_EFUCFG_WR_STR_SHIFT));
	tmp |= (efuse->efucfg_info.wr_adj << JZ_EFUSE_EFUCFG_WR_ADJ_SHIFT)
	       | (efuse->efucfg_info.wr_strobe << JZ_EFUSE_EFUCFG_WR_STR_SHIFT);
	writel(tmp, efuse->iomem + JZ_EFUCFG);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/* 2. Write want program data to EFUSE data buffer 0-7 registers */
	if (skip % 4 == 0) {
		for (i = 0; i < skip / 4; i++) {
			writel(*((unsigned int *)(buf + i * 4)),
					efuse->iomem + JZ_EFUDATA(i));
		}
	} else {
		writel(*((unsigned int *)buf) & BYTEMASK(skip),
				efuse->iomem + JZ_EFUDATA(0));
	}

	/*
	 * 3. Set control register, indicate want to program address,
	 * data length.
	 */
	spin_lock_irqsave(&efuse->lock, flag);
	tmp = readl(efuse->iomem + JZ_EFUCTRL);
	tmp &= ~((JZ_EFUSE_EFUCTRL_CS)
		 | (JZ_EFUSE_EFUCTRL_ADDR_MASK << JZ_EFUSE_EFUCTRL_ADDR_SHIFT)
		 | (JZ_EFUSE_EFUCTRL_PG_EN)
		 | JZ_EFUSE_EFUCTRL_WR_EN | JZ_EFUSE_EFUCTRL_RD_EN);

	if (addr >= (JZ_EFUSE_START_ADDR + 0x200))
		tmp |= JZ_EFUSE_EFUCTRL_CS;

	tmp |= (addr << JZ_EFUSE_EFUCTRL_ADDR_SHIFT)
		| ((skip - 1) << JZ_EFUSE_EFUCTRL_LEN_SHIFT);
	writel(tmp, efuse->iomem + JZ_EFUCTRL);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/* 4. Write control register PG_EN bit to 1 */
	spin_lock_irqsave(&efuse->lock, flag);
	tmp = readl(efuse->iomem + JZ_EFUCTRL);
	tmp |= JZ_EFUSE_EFUCTRL_PG_EN;
	writel(tmp, efuse->iomem + JZ_EFUCTRL);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/* 5. Connect VDDQ pin to 2.5V */
	spin_lock_irqsave(&efuse->lock, flag);
	efuse->is_timer_on = 1;
	jz_efuse_vddq_set((unsigned long)efuse);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/* 6. Write control register WR_EN bit */
	spin_lock_irqsave(&efuse->lock, flag);
	tmp = readl(efuse->iomem + JZ_EFUCTRL);
	tmp |= JZ_EFUSE_EFUCTRL_WR_EN;
	writel(tmp, efuse->iomem + JZ_EFUCTRL);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/* 7. Wait status register WR_DONE set to 1. */
	do {
		tmp = readl(efuse->iomem + JZ_EFUSTATE);
		usleep_range(100, 200);
		if (timeout--)
			break;
	} while (!(tmp & JZ_EFUSE_EFUSTATE_WR_DONE));

	/* 8. Disconnect VDDQ pin from 2.5V. */
	spin_lock_irqsave(&efuse->lock, flag);
	efuse->is_timer_on = 0;
	jz_efuse_vddq_set((unsigned long)efuse);
	spin_unlock_irqrestore(&efuse->lock, flag);

	/* 9. Write control register PG_EN bit to 0. */
	spin_lock_irqsave(&efuse->lock, flag);
	tmp = readl(efuse->iomem + JZ_EFUCTRL);
	tmp &= ~JZ_EFUSE_EFUCTRL_PG_EN;
	writel(tmp, efuse->iomem + JZ_EFUCTRL);
	spin_unlock_irqrestore(&efuse->lock, flag);

	if (timeout <= 0) {
		dev_err(efuse->dev, "Timed out while reading\n");
		return -EAGAIN;
	}

	return 0;
}

static ssize_t _jz_efuse_write(struct jz_efuse *efuse, const char *buf,
		size_t size, loff_t *l)
{
	int bytes = 0;
	unsigned int addr = 0, start = *l;
	int skip = jz_efuse_get_skip(size - bytes);
	int org_skip = 0;

	for (addr = start; addr < (start + size); addr += org_skip) {
		if (_jz_efuse_write_bytes(efuse, buf + bytes, addr,
					skip) < 0) {
			dev_err(efuse->dev,
				"error to write efuse byte at addr=%x\n", addr);
			return -1;
		}

		*l += skip;
		bytes += skip;

		org_skip = skip;
		skip = jz_efuse_get_skip(size - bytes);
	}

	return bytes;
}

static ssize_t jz_efuse_write(struct file *filp, const char *buf, size_t size,
		loff_t *lpos)
{
	int ret = -1;
	char *tmp_buf = NULL;
	struct miscdevice *dev = filp->private_data;
	struct jz_efuse *efuse = container_of(dev, struct jz_efuse, mdev);

	if (size > (JZ_EFUSE_END_ADDR - (JZ_EFUSE_START_ADDR + *lpos) + 1)) {
		dev_err(efuse->dev, "Trying to write beyond efuse\n");
		return -EINVAL;
	}

	tmp_buf = devm_kzalloc(efuse->dev, size, GFP_KERNEL);
	if (!tmp_buf)
		return -ENOMEM;

	copy_from_user(tmp_buf, buf, size);

	ret = _jz_efuse_write(efuse, tmp_buf, size, lpos);
	if (ret < 0) {
		dev_err(efuse->dev, "Could not write efuse\n");
		return -1;
	}

	return ret;
}

static const struct file_operations efuse_misc_fops = {
	.open		= jz_efuse_open,
	.release	= jz_efuse_release,
	.llseek		= default_llseek,
	.read		= jz_efuse_read,
	.write		= jz_efuse_write,
};

static ssize_t jz_efuse_id_show(struct device *dev,
		struct device_attribute *attr, char *buf, loff_t lpos)
{
	struct jz_efuse *efuse = dev_get_drvdata(dev);
	unsigned int data[16];
	char *tmp_buf = (char *) &data[0];
	int ret = -1;

	ret = _jz_efuse_read(efuse, tmp_buf, 16, &lpos);
	if (ret < 0) {
		dev_err(dev, "Cannot read efuse\n");
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%08x %08x %08x %08x\n",
				data[0], data[1], data[2], data[3]);
}

static ssize_t jz_efuse_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count,
		loff_t lpos)
{
	struct jz_efuse *efuse = dev_get_drvdata(dev);
	unsigned int data[16];
	char *tmp_buf = (char *) &data[0];
	int ret = -1;

	ret = sscanf(buf, "%08x %08x %08x %08x",
			&data[0], &data[1], &data[2], &data[3]);

	ret = _jz_efuse_write(efuse, tmp_buf, 16, &lpos);
	if (ret < 0) {
		dev_err(dev, "Could not write to efuse\n");
		return -EINVAL;
	}

	return ret;
}

static ssize_t jz_efuse_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return jz_efuse_id_show(dev, attr, buf,	JZ_EFUSE_SEG2_OFF);
}

static ssize_t jz_efuse_chip_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return jz_efuse_id_store(dev, attr, buf, count,	JZ_EFUSE_SEG2_OFF);
}

static ssize_t jz_efuse_user_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return jz_efuse_id_show(dev, attr, buf,	JZ_EFUSE_SEG3_OFF);
}

static ssize_t jz_efuse_user_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return jz_efuse_id_store(dev, attr, buf, count, JZ_EFUSE_SEG3_OFF);
}

static struct device_attribute jz_efuse_sysfs_attrs[] = {
	__ATTR(chip_id, S_IRUGO | S_IWUSR, jz_efuse_chip_id_show,
	       jz_efuse_chip_id_store),
	__ATTR(user_id, S_IRUGO | S_IWUSR, jz_efuse_user_id_show,
	       jz_efuse_user_id_store),
};

static struct of_device_id jz_efuse_of_match[] = {
{	.compatible = "ingenic,jz4780-efuse", },
	{ },
};

static int jz_efuse_probe(struct platform_device *pdev)
{
	int ret, i;
	struct resource	*regs;
	struct jz_efuse	*efuse = NULL;
	unsigned long	clk_rate;
	struct device *dev = &pdev->dev;
	enum of_gpio_flags flags;

	efuse = devm_kzalloc(dev, sizeof(struct jz_efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	efuse->iomem = devm_ioremap(dev, regs->start, resource_size(regs));
	if (IS_ERR(efuse->iomem))
		return PTR_ERR(efuse->iomem);

	efuse->clk = devm_clk_get(dev, "ahb2");
	if (IS_ERR(efuse->clk))
		return PTR_ERR(efuse->clk);

	clk_rate = clk_get_rate(efuse->clk);
	/* Based on maximum time for read/write strobe of 11us */
	if ((clk_rate < (185 * 1000000LL)) || (clk_rate > (512 * 1000000LL))) {
		dev_err(dev, "clock rate not between 185M-512M\n");
		return -1;
	}

	if (jz_init_efuse_cfginfo(efuse, clk_rate) < 0) {
		dev_err(dev, "Cannot set clock configuration\n");
		return -1;
	}

	efuse->dev = dev;
	efuse->mdev.minor = MISC_DYNAMIC_MINOR;
	efuse->mdev.name =  "jz-efuse";
	efuse->mdev.fops = &efuse_misc_fops;

	efuse->gpio_vddq_en_n = of_get_named_gpio_flags(dev->of_node,
							"vddq-gpio", 0, &flags);
	if (gpio_is_valid(efuse->gpio_vddq_en_n)) {
		ret = devm_gpio_request_one(dev, efuse->gpio_vddq_en_n, flags,
					    dev_name(&pdev->dev));
		if (ret) {
			dev_err(dev, "Failed to request vddq gpio pin: %d\n",
				ret);
			return -ret;
		}

		/* power off by default */
		ret = gpio_direction_output(efuse->gpio_vddq_en_n, 1);
		if (ret) {
			dev_err(dev, "Failed to set gpio as output: %d\n", ret);
			return -ret;
		}

		efuse->is_timer_on = 0;
		setup_timer(&efuse->vddq_protect_timer, jz_efuse_vddq_set,
				(unsigned long)efuse);
	} else {
		dev_err(dev, "can't find gpio vddq.\n");
		return -1;
	}

	spin_lock_init(&efuse->lock);

	for (i = 0; i < ARRAY_SIZE(jz_efuse_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &jz_efuse_sysfs_attrs[i]);
		if (ret) {
			dev_err(dev, "Cannot make sysfs device files\n");
			return -ret;
		}
	}

	ret = misc_register(&efuse->mdev);
	if (ret < 0) {
		dev_err(dev, "misc_register failed\n");
		return ret;
	} else
		dev_info(dev, "misc_register done!\n");

	platform_set_drvdata(pdev, efuse);

	dump_jz_efuse(efuse);

	return 0;
}

static int jz_efuse_remove(struct platform_device *pdev)
{
	struct jz_efuse *efuse = platform_get_drvdata(pdev);

	misc_deregister(&efuse->mdev);
	del_timer(&efuse->vddq_protect_timer);

	return 0;
}

static struct platform_driver jz_efuse_driver = {
	.probe		= jz_efuse_probe,
	.remove		= jz_efuse_remove,
	.driver		= {
		.name	= "jz-efuse",
		.of_match_table = jz_efuse_of_match,
		.owner	= THIS_MODULE,
	}
};

module_platform_driver(jz_efuse_driver);

MODULE_AUTHOR("Zubair Lutfullah Kakakhel <Zubair.Kakakhel@imgtec.com>");
MODULE_LICENSE("GPL");

