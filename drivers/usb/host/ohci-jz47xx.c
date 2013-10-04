/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-jz4780/jz4780-cgu.h>

struct jz47xx_ohci_hcd {
	struct ohci_hcd ohci_hcd;

	struct regulator *vbus;
	bool vbus_enabled;
	struct clk *clk;
};

static inline struct jz47xx_ohci_hcd *hcd_to_jz47xx_hcd(struct usb_hcd *hcd)
{
	return (struct jz47xx_ohci_hcd *)(hcd->hcd_priv);
}

static inline struct usb_hcd *jz47xx_hcd_to_hcd(struct jz47xx_ohci_hcd *jz47xx_ohci)
{
	return container_of((void *)jz47xx_ohci, struct usb_hcd, hcd_priv);
}

static int ohci_jz47xx_start(struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	int	ret;

	ret = ohci_init(ohci);
	if (ret < 0)
		return ret;

	ohci->num_ports = 1;

	ret = ohci_run(ohci);
	if (ret < 0) {
		dev_err(hcd->self.controller, "Can not start %s",
			hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}
	return 0;
}

static int ohci_jz47xx_set_vbus_power(struct jz47xx_ohci_hcd *jz47xx_ohci,
	bool enabled)
{
	int ret = 0;

	if (!jz47xx_ohci->vbus)
		return 0;

	if (enabled && !jz47xx_ohci->vbus_enabled) {
		ret = regulator_enable(jz47xx_ohci->vbus);
		if (ret)
			dev_err(jz47xx_hcd_to_hcd(jz47xx_ohci)->self.controller,
				"Could not power vbus\n");
	} else if (!enabled && jz47xx_ohci->vbus_enabled) {
		ret = regulator_disable(jz47xx_ohci->vbus);
	}

	if (ret == 0)
		jz47xx_ohci->vbus_enabled = enabled;

	return ret;
}

static int ohci_jz47xx_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
	u16 wIndex, char *buf, u16 wLength)
{
	struct jz47xx_ohci_hcd *jz47xx_ohci = hcd_to_jz47xx_hcd(hcd);
	int ret = 0;

	switch (typeReq) {
	case SetPortFeature:
		if (wValue == USB_PORT_FEAT_POWER)
			ret = ohci_jz47xx_set_vbus_power(jz47xx_ohci, true);
		break;
	case ClearPortFeature:
		if (wValue == USB_PORT_FEAT_POWER)
			ret = ohci_jz47xx_set_vbus_power(jz47xx_ohci, false);
		break;
	}

	if (ret)
		return ret;

	return ohci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);
}


static const struct hc_driver ohci_jz47xx_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"jz47xx OHCI",
	.hcd_priv_size =	sizeof(struct jz47xx_ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_jz47xx_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_jz47xx_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};


static int jz47xx_ohci_probe(struct platform_device *pdev)
{
	int ret;
	struct usb_hcd *hcd;
	struct jz47xx_ohci_hcd *jz47xx_ohci;
	struct resource *res;
	int irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		dev_err(&pdev->dev, "Failed to get platform resource\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get platform irq\n");
		return irq;
	}

	hcd = usb_create_hcd(&ohci_jz47xx_hc_driver, &pdev->dev, "jz47xx");
	if (!hcd) {
		dev_err(&pdev->dev, "Failed to create hcd.\n");
		return -ENOMEM;
	}

	jz47xx_ohci = hcd_to_jz47xx_hcd(hcd);

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto err_free;
	}

	jz47xx_ohci->clk = devm_clk_get(&pdev->dev, "uhc");
	if (IS_ERR(jz47xx_ohci->clk)) {
		ret = PTR_ERR(jz47xx_ohci->clk);
		dev_err(&pdev->dev, "Failed to get clock: %d\n", ret);
		goto err_free;
	}

	jz47xx_ohci->vbus = devm_regulator_get(&pdev->dev, "vbus");
	if (IS_ERR(jz47xx_ohci->vbus))
		jz47xx_ohci->vbus = NULL;


	clk_set_rate(jz47xx_ohci->clk, 48000000);
	clk_enable(jz47xx_ohci->clk);
	if (jz47xx_ohci->vbus)
		ohci_jz47xx_set_vbus_power(jz47xx_ohci, true);

	if (of_machine_is_compatible("ingenic,jz4780-ohci")) {
		ret = jz4780_cgu_set_usb_suspend(USB_PORT_HOST, false);
		if (ret) {
			dev_err(&pdev->dev, "failed to unsuspend port\n");
			goto err_disable;
		}
	}

	platform_set_drvdata(pdev, hcd);

	ohci_hcd_init(hcd_to_ohci(hcd));

	ret = usb_add_hcd(hcd, irq, 0);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add hcd: %d\n", ret);
		goto err_disable;
	}
	device_wakeup_enable(hcd->self.controller);

	return 0;

err_disable:
	if (jz47xx_ohci->vbus)
		regulator_disable(jz47xx_ohci->vbus);
	clk_disable(jz47xx_ohci->clk);

err_free:
	usb_put_hcd(hcd);

	return ret;
}

static int jz47xx_ohci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct jz47xx_ohci_hcd *jz47xx_ohci = hcd_to_jz47xx_hcd(hcd);

	usb_remove_hcd(hcd);

	if (jz47xx_ohci->vbus)
		regulator_disable(jz47xx_ohci->vbus);

	clk_disable(jz47xx_ohci->clk);

	usb_put_hcd(hcd);

	return 0;
}

static struct of_device_id jz47xx_ohci_of_match[] = {
	{ .compatible = "ingenic,jz4740-ohci", },
	{ .compatible = "ingenic,jz4780-ohci", },
	{ },
};

static struct platform_driver ohci_hcd_jz47xx_driver = {
	.probe = jz47xx_ohci_probe,
	.remove = jz47xx_ohci_remove,
	.driver = {
		.name = "jz47xx-ohci",
		.of_match_table = jz47xx_ohci_of_match,
		.owner = THIS_MODULE,
	},
};

MODULE_ALIAS("platform:jz47xx-ohci");
MODULE_DEVICE_TABLE(of, jz47xx_ohci_of_match);
