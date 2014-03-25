/*
 * Ingenic IW8101/8103 WiFi driver
 *
 * Copyright (c) 2014 Imagination Technologies
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
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/platform_data/brcmfmac-sdio.h>

static int iw810x_reset_gpio;
static int iw810x_power_gpio;
static struct clk *iw810x_32k_clk;

static void iw810x_brcmfmac_power_on(void)
{
	clk_prepare_enable(iw810x_32k_clk);
	msleep(200);

	gpio_set_value(iw810x_power_gpio, 1);
	msleep(200);

	gpio_set_value(iw810x_reset_gpio, 0);
	msleep(200);

	gpio_set_value(iw810x_reset_gpio, 1);
	msleep(200);
}

static void iw810x_brcmfmac_power_off(void)
{
	gpio_set_value(iw810x_reset_gpio, 0);
	gpio_set_value(iw810x_power_gpio, 0);
	clk_disable_unprepare(iw810x_32k_clk);
}

static void iw810x_brcmfmac_reset(void)
{
	gpio_set_value(iw810x_reset_gpio, 0);
	msleep(200);

	gpio_set_value(iw810x_reset_gpio, 1);
	msleep(200);
}

static struct brcmfmac_sdio_platform_data iw810x_brcmfmac_sdio_pdata = {
	.power_on = iw810x_brcmfmac_power_on,
	.power_off = iw810x_brcmfmac_power_off,
	.reset = iw810x_brcmfmac_reset,
};

static struct platform_device iw810x_brcmfmac_device = {
	.name = BRCMFMAC_SDIO_PDATA_NAME,
	.id = PLATFORM_DEVID_NONE,
	.dev.platform_data = &iw810x_brcmfmac_sdio_pdata,
};

static int iw810x_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *str;
	int ret;

	iw810x_reset_gpio = of_get_named_gpio(dev->of_node,
					      "ingenic,reset-gpio", 0);
	if (!gpio_is_valid(iw810x_reset_gpio)) {
		dev_err(dev, "no reset GPIO specified\n");
		return -ENODEV;
	}

	ret = devm_gpio_request_one(dev, iw810x_reset_gpio,
				    GPIOF_OUT_INIT_HIGH, "IW810x reset");
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to request reset GPIO: %d\n", ret);
		return ret;
	}

	iw810x_power_gpio = of_get_named_gpio(dev->of_node,
					      "ingenic,power-gpio", 0);
	if (!gpio_is_valid(iw810x_power_gpio)) {
		dev_err(dev, "no power GPIO specified\n");
		return -ENODEV;
	}

	ret = devm_gpio_request_one(dev, iw810x_power_gpio, GPIOF_OUT_INIT_LOW,
				    "IW810x power");
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to request power GPIO: %d\n", ret);
		return ret;
	}

	ret = of_property_read_string(dev->of_node, "ingenic,iw810x_32k", &str);
	iw810x_32k_clk = devm_clk_get(dev, str);
	if (IS_ERR(iw810x_32k_clk)) {
		dev_err(dev,"Failed to get clk \n");
		return PTR_ERR(iw810x_32k_clk);
	}

	platform_device_register(&iw810x_brcmfmac_device);
	dev_info(dev, "IW810x WiFi registered\n");
	return 0;
}

static const struct of_device_id iw810x_dt_match[] = {
	{ .compatible = "ingenic,iw810x", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, iw810x_dt_match);

static struct platform_driver iw810x_driver = {
	.probe		= iw810x_probe,
	.driver	= {
		.name	= "iw810x",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(iw810x_dt_match),
	},
};

static int __init iw810x_init(void)
{
	return platform_driver_register(&iw810x_driver);
}

static void __exit iw810x_exit(void)
{
	platform_driver_unregister(&iw810x_driver);
}
subsys_initcall(iw810x_init);
module_exit(iw810x_exit);

MODULE_AUTHOR("Alex Smith <alex@alex-smith.me.uk>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IW810X driver for Ingenic rebranded BRCM4330\n");
