/*
 * Copyright (C) 2015 Imagination Technologies
 * Author: Zubair Lutfullah Kakakhel <Zubair.Kakakhel@imgtec.com>
 *
 * DRM driver for Ingenic JZ4780
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <linux/of_graph.h>
#include <linux/component.h>

#include "jz4780_drv.h"
#include "jz4780_regs.h"

static void jz4780_fb_output_poll_changed(struct drm_device *drm_dev)
{

	struct jz4780_drm_private *priv = drm_dev->dev_private;

	if (priv->fbdev)
		drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs jz4780_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = jz4780_fb_output_poll_changed,
};

void jz4780_drm_mode_config_init(struct drm_device *drm_dev)
{

	drm_dev->mode_config.min_width = 0;
	drm_dev->mode_config.min_height = 0;
	drm_dev->mode_config.max_width = 2048;
	drm_dev->mode_config.max_height = 2048;
	drm_dev->mode_config.funcs = &jz4780_mode_config_funcs;
}

/*
 * DRM operations:
 */

static int jz4780_unload(struct drm_device *drm_dev)
{

	struct jz4780_drm_private *priv = drm_dev->dev_private;
	struct device *dev = drm_dev->dev;

	drm_kms_helper_poll_fini(drm_dev);
	drm_mode_config_cleanup(drm_dev);
	drm_vblank_cleanup(drm_dev);

	component_unbind_all(dev, drm_dev);

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	drm_dev->dev_private = NULL;
	pm_runtime_disable(drm_dev->dev);

	return 0;
}

static int jz4780_load(struct drm_device *drm_dev, unsigned long flags)
{
	struct device *dev = drm_dev->dev;
	struct jz4780_drm_private *priv;
	int ret;

	priv = devm_kzalloc(drm_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		DRM_DEBUG_DRIVER("failed to allocate private data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(drm_dev->platformdev, drm_dev);
	drm_dev->dev_private = priv;

	priv->wq = alloc_ordered_workqueue("jz4780", 0);

	drm_mode_config_init(drm_dev);

	jz4780_drm_mode_config_init(drm_dev);

	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev, drm_dev);
	if (ret)
		goto err_config_cleanup;

	ret = drm_vblank_init(drm_dev, 1);
	if (ret < 0) {
		DRM_DEBUG_DRIVER("failed to initialize vblank\n");
		goto err_vblank_cleanup;
	}

	priv->fbdev = drm_fbdev_cma_init(drm_dev, 32,
			drm_dev->mode_config.num_crtc,
			drm_dev->mode_config.num_connector);

	drm_kms_helper_poll_init(drm_dev);

	return 0;

err_vblank_cleanup:
	drm_vblank_cleanup(drm_dev);
	component_unbind_all(dev, drm_dev);
err_config_cleanup:
	drm_mode_config_cleanup(drm_dev);
	return ret;
}

static void jz4780_preclose(struct drm_device *drm_dev, struct drm_file *file)
{

	struct jz4780_drm_private *priv = drm_dev->dev_private;

	jz4780_crtc_cancel_page_flip(priv->crtc, file);
}

static void jz4780_lastclose(struct drm_device *drm_dev)
{

	struct jz4780_drm_private *priv = drm_dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static irqreturn_t jz4780_irq(int irq, void *arg)
{

	struct drm_device *drm_dev = arg;
	struct jz4780_drm_private *priv = drm_dev->dev_private;

	return jz4780_crtc_irq(priv->crtc);
}

static void jz4780_enable_disable_vblank(struct drm_device *drm_dev,
					 bool enable)
{

	u32 tmp;

	/* clear previous EOF flag */
	tmp = jz4780_read(drm_dev, LCDC_STATE);
	jz4780_write(drm_dev, LCDC_STATE, tmp & ~LCDC_STATE_EOF);

	/* enable end of frame interrupt */
	tmp = jz4780_read(drm_dev, LCDC_CTRL);
	if (enable)
		jz4780_write(drm_dev, LCDC_CTRL, tmp | LCDC_CTRL_EOFM);
	else
		jz4780_write(drm_dev, LCDC_CTRL, tmp & ~LCDC_CTRL_EOFM);

}

static int jz4780_enable_vblank(struct drm_device *drm_dev, int crtc)
{

	jz4780_enable_disable_vblank(drm_dev, true);
	return 0;
}

static void jz4780_disable_vblank(struct drm_device *drm_dev, int crtc)
{

	jz4780_enable_disable_vblank(drm_dev, false);
}

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = drm_gem_cma_mmap,
};

static struct drm_driver jz4780_driver = {
	.driver_features    = DRIVER_HAVE_IRQ | DRIVER_GEM | DRIVER_MODESET,
	.load               = jz4780_load,
	.unload             = jz4780_unload,
	.preclose           = jz4780_preclose,
	.lastclose          = jz4780_lastclose,
	.set_busid          = drm_platform_set_busid,
	.irq_handler        = jz4780_irq,
	.get_vblank_counter = drm_vblank_count,
	.enable_vblank      = jz4780_enable_vblank,
	.disable_vblank     = jz4780_disable_vblank,
	.gem_free_object    = drm_gem_cma_free_object,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.dumb_create        = drm_gem_cma_dumb_create,
	.dumb_map_offset    = drm_gem_cma_dumb_map_offset,
	.dumb_destroy       = drm_gem_dumb_destroy,
	.fops               = &fops,
	.name               = "jz4780",
	.desc               = "Ingenic LCD Controller DRM",
	.date               = "20140623",
	.major              = 1,
	.minor              = 0,
};

static int compare_of(struct device *dev, void *data)
{

	struct device_node *np = data;

	return dev->of_node == np;
}

static void jz4780_add_endpoints(struct device *dev,
				   struct component_match **match,
				   struct device_node *port)
{

	struct device_node *ep, *remote;

	for_each_child_of_node(port, ep) {
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote || !of_device_is_available(remote)) {
			of_node_put(remote);
			continue;
		} else if (!of_device_is_available(remote->parent)) {
			dev_warn(dev, "parent device of %s is not available\n",
				 remote->full_name);
			of_node_put(remote);
			continue;
		}

		component_match_add(dev, match, compare_of, remote);
		of_node_put(remote);
	}
}

static int jz4780_drm_bind(struct device *dev)
{

	struct drm_device *drm;
	struct platform_device *pdev = dev_get_drvdata(dev);
	int ret;

	drm = drm_dev_alloc(&jz4780_driver, dev);
	if (!drm)
		return -ENOMEM;

	ret = drm_dev_set_unique(drm, "%s", dev_name(dev));
	if (ret)
		goto err_free;

	drm->platformdev = pdev;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_free;

	dev_set_drvdata(dev, drm);

	return 0;

err_free:
	drm_dev_unref(drm);
	return ret;
}

static void jz4780_drm_unbind(struct device *dev)
{

	struct drm_device *drm = dev_get_drvdata(dev);

	drm_dev_unregister(drm);
	drm_dev_unref(drm);
	dev_set_drvdata(dev, NULL);
}

static const struct component_master_ops jz4780_drm_ops = {
	.bind = jz4780_drm_bind,
	.unbind = jz4780_drm_unbind,
};

/*
 * Platform driver:
 */
static int jz4780_pdev_probe(struct platform_device *pdev)
{

	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct device_node *np = dev->of_node;
	struct device_node *port;
	int i;

	if (!np)
		return -ENODEV;

	/*
	 * Bind the crtc ports first, so that
	 * drm_of_find_possible_crtcs called from encoder .bind callbacks
	 * works as expected.
	 */
	for (i = 0;; i++) {
		port = of_parse_phandle(np, "ports", i);
		if (!port)
			break;

		if (!of_device_is_available(port->parent)) {
			of_node_put(port);
			continue;
		}

		component_match_add(dev, &match, compare_of, port->parent);
		of_node_put(port);
	}

	if (i == 0) {
		dev_err(dev, "missing 'ports' property\n");
		return -ENODEV;
	}

	if (!match) {
		dev_err(dev, "No available crtc found for display-subsystem.\n");
		return -ENODEV;
	}
	/*
	 * For each bound crtc, bind the encoders attached to its
	 * remote endpoint.
	 */
	for (i = 0;; i++) {
		port = of_parse_phandle(np, "ports", i);
		if (!port)
			break;

		if (!of_device_is_available(port->parent)) {
			of_node_put(port);
			continue;
		}

		jz4780_add_endpoints(dev, &match, port);
		of_node_put(port);
	}

	dev_set_drvdata(dev, pdev);
	return component_master_add_with_match(dev, &jz4780_drm_ops, match);
}

static int jz4780_pdev_remove(struct platform_device *pdev)
{

	component_master_del(&pdev->dev, &jz4780_drm_ops);
	return 0;
}

static const struct of_device_id jz4780_of_match[] = {
		{ .compatible = "ingenic,jz4780-display-subsystem", },
		{ },
};
MODULE_DEVICE_TABLE(of, jz4780_of_match);

static struct platform_driver jz4780_platform_driver = {
	.probe      = jz4780_pdev_probe,
	.remove     = jz4780_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = "ingenic-jz4780-drm",
		.of_match_table = jz4780_of_match,
	},
};

module_platform_driver(jz4780_platform_driver);

MODULE_AUTHOR("Zubair Lutfullah Kakakhel <Zubair.Kakakhel@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 LCD/HDMI Driver");
MODULE_LICENSE("GPL v2");
