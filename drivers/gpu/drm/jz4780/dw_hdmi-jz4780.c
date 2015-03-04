/* Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2015 Imagination Technologies Ltd.
 *
 * derived from dw_hdmi-imx.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <drm/bridge/dw_hdmi.h>
#include <linux/regmap.h>
#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

struct jz4780_hdmi {
	struct device *dev;
	struct drm_encoder encoder;
	struct regmap *regmap;
};

static const struct dw_hdmi_mpll_config jz4780_mpll_cfg[] = {
	{
		45250000, {
			{ 0x01e0, 0x0000 },
			{ 0x21e1, 0x0000 },
			{ 0x41e2, 0x0000 }
		},
	}, {
		92500000, {
			{ 0x0140, 0x0005 },
			{ 0x2141, 0x0005 },
			{ 0x4142, 0x0005 },
	},
	}, {
		148500000, {
			{ 0x00a0, 0x000a },
			{ 0x20a1, 0x000a },
			{ 0x40a2, 0x000a },
		},
	}, {
		~0UL, {
			{ 0x00a0, 0x000a },
			{ 0x2001, 0x000f },
			{ 0x4002, 0x000f },
		},
	}
};

static const struct dw_hdmi_curr_ctrl jz4780_cur_ctr[] = {
	/*      pixelclk     bpp8    bpp10   bpp12 */
	{
		54000000, { 0x091c, 0x091c, 0x06dc },
	}, {
		58400000, { 0x091c, 0x06dc, 0x06dc },
	}, {
		72000000, { 0x06dc, 0x06dc, 0x091c },
	}, {
		74250000, { 0x06dc, 0x0b5c, 0x091c },
	}, {
		118800000, { 0x091c, 0x091c, 0x06dc },
	}, {
		216000000, { 0x06dc, 0x0b5c, 0x091c },
	}
};

static const struct dw_hdmi_sym_term jz4780_sym_term[] = {
	/*pixelclk   symbol   term*/
	{ 148500000, 0x800d, 0x0005 },
	{ ~0UL,      0x0000, 0x0000 }
};

static void dw_hdmi_jz4780_encoder_disable(struct drm_encoder *encoder)
{
}

static bool dw_hdmi_jz4780_encoder_mode_fixup(struct drm_encoder *encoder,
					   const struct drm_display_mode *mode,
					   struct drm_display_mode *adj_mode)
{
	return true;
}

static void dw_hdmi_jz4780_encoder_mode_set(struct drm_encoder *encoder,
					 struct drm_display_mode *mode,
					 struct drm_display_mode *adj_mode)
{
}

static void dw_hdmi_jz4780_encoder_commit(struct drm_encoder *encoder)
{
}

static void dw_hdmi_jz4780_encoder_prepare(struct drm_encoder *encoder)
{
}

static struct drm_encoder_helper_funcs dw_hdmi_jz4780_encoder_helper_funcs = {
	.mode_fixup = dw_hdmi_jz4780_encoder_mode_fixup,
	.mode_set   = dw_hdmi_jz4780_encoder_mode_set,
	.prepare    = dw_hdmi_jz4780_encoder_prepare,
	.commit     = dw_hdmi_jz4780_encoder_commit,
	.disable    = dw_hdmi_jz4780_encoder_disable,
};

static struct drm_encoder_funcs dw_hdmi_jz4780_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static struct dw_hdmi_plat_data jz4780_hdmi_drv_data = {
	.mpll_cfg = jz4780_mpll_cfg,
	.cur_ctr  = jz4780_cur_ctr,
	.sym_term = jz4780_sym_term,
	.dev_type = JZ4780_HDMI,
};

static const struct of_device_id dw_hdmi_jz4780_dt_ids[] = {
	{ .compatible = "ingenic,jz4780-hdmi",
	  .data = &jz4780_hdmi_drv_data
	},
	{},
};
MODULE_DEVICE_TABLE(of, dw_hdmi_jz4780_dt_ids);

static int dw_hdmi_jz4780_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct dw_hdmi_plat_data *plat_data;
	const struct of_device_id *match;
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct jz4780_hdmi *hdmi;
	struct resource *iores;
	struct clk *isfr_clk;
	int irq;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	match = of_match_node(dw_hdmi_jz4780_dt_ids, pdev->dev.of_node);
	plat_data = match->data;
	hdmi->dev = &pdev->dev;
	encoder = &hdmi->encoder;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iores)
		return -ENXIO;

	isfr_clk = devm_clk_get(hdmi->dev, "isfr");
	if (IS_ERR(isfr_clk)) {
		ret = PTR_ERR(isfr_clk);
		dev_err(hdmi->dev, "Unable to get HDMI isfr clk: %d\n", ret);
		return ret;
	}

	clk_set_rate(isfr_clk, 27000000);

	platform_set_drvdata(pdev, hdmi);

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_helper_add(encoder, &dw_hdmi_jz4780_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &dw_hdmi_jz4780_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS);

	return dw_hdmi_bind(dev, master, data, encoder, iores, irq, plat_data);
}

static void dw_hdmi_jz4780_unbind(struct device *dev, struct device *master,
			       void *data)
{

	return dw_hdmi_unbind(dev, master, data);
}

static const struct component_ops dw_hdmi_jz4780_ops = {
	.bind	= dw_hdmi_jz4780_bind,
	.unbind	= dw_hdmi_jz4780_unbind,
};

static int dw_hdmi_jz4780_probe(struct platform_device *pdev)
{

	return component_add(&pdev->dev, &dw_hdmi_jz4780_ops);
}

static int dw_hdmi_jz4780_remove(struct platform_device *pdev)
{

	component_del(&pdev->dev, &dw_hdmi_jz4780_ops);

	return 0;
}

static struct platform_driver dw_hdmi_jz4780_platform_driver = {
	.probe  = dw_hdmi_jz4780_probe,
	.remove = dw_hdmi_jz4780_remove,
	.driver = {
		.name = "dwhdmi-jz4780",
		.of_match_table = dw_hdmi_jz4780_dt_ids,
	},
};

module_platform_driver(dw_hdmi_jz4780_platform_driver);

MODULE_AUTHOR("Zubair Lutfullah Kakakhel <Zubair.Kakakhel@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 Specific DW-HDMI Driver Extension");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dwhdmi-jz4780");
