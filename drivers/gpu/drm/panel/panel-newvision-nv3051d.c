// SPDX-License-Identifier: GPL-2.0
/*
 * NewVision NV3051D MIPI-DSI panel driver
 * Clean 5.10-native implementation
 * Target: RK3566 (Anbernic RG353V)
 *
 * Uses Rockchip-style panel-init-sequence property from DT:
 *   panel-init-sequence = [
 *      <dtype> <delay_ms> <len> <len bytes of payload> ...
 *   ];
 *
 * Typical entry for this panel looks like:
 *   15 00 02 FF 30
 *   15 00 02 FF 52
 *   ...
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>

struct nv3051d {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	bool prepared;
};

static inline struct nv3051d *panel_to_nv3051d(struct drm_panel *panel)
{
	return container_of(panel, struct nv3051d, panel);
}

/* 640x480 @60Hz — NV3051D timings from upstream driver */
static const struct drm_display_mode rg353v_mode = {
	.clock = 24150,

	.hdisplay    = 640,
	.hsync_start = 640 + 40,
	.hsync_end   = 640 + 40 + 2,
	.htotal      = 640 + 40 + 2 + 80,

	.vdisplay    = 480,
	.vsync_start = 480 + 18,
	.vsync_end   = 480 + 18 + 2,
	.vtotal      = 480 + 18 + 2 + 28,

	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

/*
 * Parse and execute Rockchip-style panel-init-sequence DT property.
 *
 * Format:
 *   [ dtype delay_ms len payload... ] repeated
 *
 * Common dtype values:
 *   0x05 - DCS short write, no parameter   (command only)
 *   0x15 - DCS short write, 1+ parameters (cmd + params)
 *   0x39 - DCS long write (len bytes payload)
 */
static int nv3051d_init_from_dts(struct nv3051d *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	const u8 *buf;
	int len, i = 0;
	int ret;

	buf = of_get_property(dev->of_node, "panel-init-sequence", &len);
	if (!buf || len < 3) {
		dev_warn(dev, "no panel-init-sequence in DT, skipping\n");
		return -ENOENT;
	}

	dev_info(dev, "running panel-init-sequence (%d bytes)\n", len);

	while (i + 3 <= len) {
		u8 dtype = buf[i++];
		u8 delay_ms = buf[i++];
		u8 size = buf[i++];

		if (!size) {
			if (delay_ms)
				msleep(delay_ms);
			continue;
		}

		if (i + size > len) {
			dev_warn(dev, "truncated init-sequence at %d\n", i);
			break;
		}

		switch (dtype) {
		case 0x05: { /* DCS short write, no param */
			u8 cmd = buf[i];
			ret = mipi_dsi_dcs_write(ctx->dsi, cmd, NULL, 0);
			dev_dbg(dev, "DCS 0x05 cmd=0x%02x ret=%d\n", cmd, ret);
			break;
		}
		case 0x15: { /* DCS short write, 1+ params */
			u8 cmd = buf[i];
			const u8 *params = &buf[i + 1];
			int param_len = size - 1;

			ret = mipi_dsi_dcs_write(ctx->dsi, cmd, params, param_len);
			dev_dbg(dev,
				"DCS 0x15 cmd=0x%02x len=%d ret=%d\n",
				cmd, param_len, ret);
			break;
		}
		case 0x39: /* DCS long write */
			ret = mipi_dsi_dcs_write_buffer(ctx->dsi, &buf[i], size);
			dev_dbg(dev, "DCS 0x39 len=%d ret=%d\n", size, ret);
			break;
		default:
			dev_warn(dev, "unknown init dtype 0x%02x, skipping\n",
				 dtype);
			ret = 0;
			break;
		}

		if (ret < 0)
			dev_warn(dev,
				 "init cmd type 0x%02x at %d failed: %d\n",
				 dtype, i, ret);

		i += size;

		if (delay_ms)
			msleep(delay_ms);
	}

	/* Just to be safe, make sure we end in a sane state. These may
	 * already be included in panel-init-sequence, but double-calling
	 * is harmless.
	 */
	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret < 0)
		dev_warn(dev, "exit_sleep_mode failed: %d\n", ret);
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	if (ret < 0)
		dev_warn(dev, "set_display_on failed: %d\n", ret);
	msleep(20);

	return 0;
}

static int nv3051d_prepare(struct drm_panel *panel)
{
	struct nv3051d *ctx = panel_to_nv3051d(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	dev_info(dev, "nv3051d prepare()\n");

	if (!IS_ERR_OR_NULL(ctx->vdd)) {
		ret = regulator_enable(ctx->vdd);
		if (ret) {
			dev_err(dev, "failed to enable vdd: %d\n", ret);
			return ret;
		}
	}

	if (ctx->reset_gpio) {
		/* Reset pulse */
		gpiod_set_value(ctx->reset_gpio, 0);
		msleep(20);
		gpiod_set_value(ctx->reset_gpio, 1);
		msleep(120);
	}

	ret = nv3051d_init_from_dts(ctx);
	if (ret && ret != -ENOENT) {
		dev_err(dev, "panel init from DTS failed: %d\n", ret);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int nv3051d_unprepare(struct drm_panel *panel)
{
	struct nv3051d *ctx = panel_to_nv3051d(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	dev_info(dev, "nv3051d unprepare()\n");

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		dev_warn(dev, "set_display_off failed: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	if (ret < 0)
		dev_warn(dev, "enter_sleep_mode failed: %d\n", ret);

	msleep(100);

	if (!IS_ERR_OR_NULL(ctx->vdd))
		regulator_disable(ctx->vdd);

	ctx->prepared = false;
	return 0;
}

static int nv3051d_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &rg353v_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 57;

	connector->display_info.bus_flags =
		DRM_BUS_FLAG_DE_LOW |
		DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;
	connector->display_info.bpc = 8;

	return 1;
}

static const struct drm_panel_funcs nv3051d_funcs = {
	.prepare = nv3051d_prepare,
	.unprepare = nv3051d_unprepare,
	.get_modes = nv3051d_get_modes,
};

static int nv3051d_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nv3051d *ctx;
	int ret;

	dev_info(dev, "nv3051d panel probe()\n");

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dsi = dsi;

	/* Regulator is optional: if not present, we run without it. */
	ctx->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctx->vdd)) {
		if (PTR_ERR(ctx->vdd) == -ENODEV) {
			dev_info(dev, "no vdd regulator, proceeding without\n");
			ctx->vdd = NULL;
		} else {
			dev_err(dev, "failed to get vdd: %ld\n",
				PTR_ERR(ctx->vdd));
			return PTR_ERR(ctx->vdd);
		}
	}

	ctx->reset_gpio =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =
		MIPI_DSI_MODE_VIDEO |
		MIPI_DSI_MODE_VIDEO_BURST |
		MIPI_DSI_MODE_LPM;
		
	drm_panel_init(&ctx->panel, dev, &nv3051d_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	mipi_dsi_set_drvdata(dsi, ctx);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		dev_err(dev, "mipi_dsi_attach failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	dev_info(dev, "nv3051d panel attached to DSI\n");
	return 0;
}

static int nv3051d_remove(struct mipi_dsi_device *dsi)
{
	struct nv3051d *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id nv3051d_of_match[] = {
	{ .compatible = "newvision,nv3051d" },
	{ .compatible = "anbernic,rg353p-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, nv3051d_of_match);

static struct mipi_dsi_driver nv3051d_driver = {
	.probe = nv3051d_probe,
	.remove = nv3051d_remove,
	.driver = {
		.name = "panel-newvision-nv3051d",
		.of_match_table = nv3051d_of_match,
	},
};

module_mipi_dsi_driver(nv3051d_driver);

MODULE_DESCRIPTION("NV3051D Panel Driver (5.10 Native, DT init-sequence)");
MODULE_LICENSE("GPL");