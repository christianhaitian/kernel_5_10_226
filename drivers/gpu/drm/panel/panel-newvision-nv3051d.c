// SPDX-License-Identifier: GPL-2.0
/*
 * NewVision NV3051D MIPI-DSI panel driver
 * Clean 5.10-native implementation
 * Target: RK3566 (Anbernic RG353V / RG353P)
 *
 * Primary init path:
 *   - Parse and execute Rockchip-style panel-init-sequence from DT:
 *
 *       panel-init-sequence = [
 *         dtype delay_ms len payload...
 *         ...
 *       ];
 *
 *   - If the property is missing, we fall back to a built-in
 *     vendor init sequence used for the NV3051D panel in these devices.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#ifndef mipi_dsi_dcs_write_seq
#define mipi_dsi_dcs_write_seq(dsi, seq...)				\
({									\
	static const u8 d[] = { seq };					\
	int ret__;							\
	ret__ = mipi_dsi_dcs_write_buffer(dsi, d, sizeof(d));		\
	ret__;								\
})
#endif

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
	.clock = 23040,

	.hdisplay    = 640,
	.hsync_start = 640 + 64,
	.hsync_end   = 640 + 64 + 2,
	.htotal      = 640 + 64 + 2 + 80,

	.vdisplay    = 480,
	.vsync_start = 480 + 2,
	.vsync_end   = 480 + 2 + 4,
	.vtotal      = 480 + 2 + 4 + 3,

	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

/*
 * Built-in vendor NV3051D init sequence
 *
 * This is effectively the same sequence shipped in vendor kernels for
 * the RG353x series, expressed as a series of mipi_dsi_dcs_write_seq()
 * calls. It is only used if the DT does NOT provide a panel-init-sequence
 * property.
 */
static int nv3051d_hw_init(struct nv3051d *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

#define dsi_dcs_write_seq_checked(_dsi, seq...)			\
	do {							\
		ret = mipi_dsi_dcs_write_seq(_dsi, seq);	\
		if (ret < 0)					\
			dev_warn(dev,				\
				 "init cmd failed (%d)\n", ret);	\
	} while (0)

	/* Page 1 */
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x30);
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x52);
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0xE3, 0x00);
	dsi_dcs_write_seq_checked(dsi, 0x03, 0x40);
	dsi_dcs_write_seq_checked(dsi, 0x04, 0x00);
	dsi_dcs_write_seq_checked(dsi, 0x05, 0x03);
	dsi_dcs_write_seq_checked(dsi, 0x24, 0x12);
	dsi_dcs_write_seq_checked(dsi, 0x25, 0x1E);
	dsi_dcs_write_seq_checked(dsi, 0x26, 0x28);
	dsi_dcs_write_seq_checked(dsi, 0x27, 0x52);
	dsi_dcs_write_seq_checked(dsi, 0x28, 0x57);
	dsi_dcs_write_seq_checked(dsi, 0x29, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0x2A, 0xDF);
	dsi_dcs_write_seq_checked(dsi, 0x38, 0x9C);
	dsi_dcs_write_seq_checked(dsi, 0x39, 0xA7);
	dsi_dcs_write_seq_checked(dsi, 0x3A, 0x53);
	dsi_dcs_write_seq_checked(dsi, 0x44, 0x00);
	dsi_dcs_write_seq_checked(dsi, 0x49, 0x3C);
	dsi_dcs_write_seq_checked(dsi, 0x59, 0xFE);
	dsi_dcs_write_seq_checked(dsi, 0x5C, 0x00);
	dsi_dcs_write_seq_checked(dsi, 0x91, 0x77);
	dsi_dcs_write_seq_checked(dsi, 0x92, 0x77);
	dsi_dcs_write_seq_checked(dsi, 0xA0, 0x55);
	dsi_dcs_write_seq_checked(dsi, 0xA1, 0x50);
	dsi_dcs_write_seq_checked(dsi, 0xA4, 0x9C);
	dsi_dcs_write_seq_checked(dsi, 0xA7, 0x02);
	dsi_dcs_write_seq_checked(dsi, 0xA8, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0xA9, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0xAA, 0xFC);
	dsi_dcs_write_seq_checked(dsi, 0xAB, 0x28);
	dsi_dcs_write_seq_checked(dsi, 0xAC, 0x06);
	dsi_dcs_write_seq_checked(dsi, 0xAD, 0x06);
	dsi_dcs_write_seq_checked(dsi, 0xAE, 0x06);
	dsi_dcs_write_seq_checked(dsi, 0xAF, 0x03);
	dsi_dcs_write_seq_checked(dsi, 0xB0, 0x08);
	dsi_dcs_write_seq_checked(dsi, 0xB1, 0x26);
	dsi_dcs_write_seq_checked(dsi, 0xB2, 0x28);
	dsi_dcs_write_seq_checked(dsi, 0xB3, 0x28);
	dsi_dcs_write_seq_checked(dsi, 0xB4, 0x33);
	dsi_dcs_write_seq_checked(dsi, 0xB5, 0x08);
	dsi_dcs_write_seq_checked(dsi, 0xB6, 0x26);
	dsi_dcs_write_seq_checked(dsi, 0xB7, 0x08);
	dsi_dcs_write_seq_checked(dsi, 0xB8, 0x26);

	/* Page 2 */
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x30);
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x52);
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x02);
	dsi_dcs_write_seq_checked(dsi, 0xB1, 0x0E);
	dsi_dcs_write_seq_checked(dsi, 0xD1, 0x0E);
	dsi_dcs_write_seq_checked(dsi, 0xB4, 0x29);
	dsi_dcs_write_seq_checked(dsi, 0xD4, 0x2B);
	dsi_dcs_write_seq_checked(dsi, 0xB2, 0x0C);
	dsi_dcs_write_seq_checked(dsi, 0xD2, 0x0A);
	dsi_dcs_write_seq_checked(dsi, 0xB3, 0x28);
	dsi_dcs_write_seq_checked(dsi, 0xD3, 0x28);
	dsi_dcs_write_seq_checked(dsi, 0xB6, 0x11);
	dsi_dcs_write_seq_checked(dsi, 0xD6, 0x0D);
	dsi_dcs_write_seq_checked(dsi, 0xB7, 0x32);
	dsi_dcs_write_seq_checked(dsi, 0xD7, 0x30);
	dsi_dcs_write_seq_checked(dsi, 0xC1, 0x04);
	dsi_dcs_write_seq_checked(dsi, 0xE1, 0x06);
	dsi_dcs_write_seq_checked(dsi, 0xB8, 0x0A);
	dsi_dcs_write_seq_checked(dsi, 0xD8, 0x0A);
	dsi_dcs_write_seq_checked(dsi, 0xB9, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0xD9, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0xBD, 0x13);
	dsi_dcs_write_seq_checked(dsi, 0xDD, 0x13);
	dsi_dcs_write_seq_checked(dsi, 0xBC, 0x11);
	dsi_dcs_write_seq_checked(dsi, 0xDC, 0x11);
	dsi_dcs_write_seq_checked(dsi, 0xBB, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xDB, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xBA, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xDA, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xBE, 0x18);
	dsi_dcs_write_seq_checked(dsi, 0xDE, 0x18);
	dsi_dcs_write_seq_checked(dsi, 0xBF, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xDF, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xC0, 0x17);
	dsi_dcs_write_seq_checked(dsi, 0xE0, 0x17);
	dsi_dcs_write_seq_checked(dsi, 0xB5, 0x3B);
	dsi_dcs_write_seq_checked(dsi, 0xD5, 0x3C);
	dsi_dcs_write_seq_checked(dsi, 0xB0, 0x0B);
	dsi_dcs_write_seq_checked(dsi, 0xD0, 0x0C);

	/* Page 3 */
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x30);
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x52);
	dsi_dcs_write_seq_checked(dsi, 0xFF, 0x03);
	dsi_dcs_write_seq_checked(dsi, 0x00, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x01, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x02, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x03, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x04, 0x61);
	dsi_dcs_write_seq_checked(dsi, 0x05, 0x80);
	dsi_dcs_write_seq_checked(dsi, 0x06, 0xC7);
	dsi_dcs_write_seq_checked(dsi, 0x07, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0x08, 0x82);
	dsi_dcs_write_seq_checked(dsi, 0x09, 0x83);
	dsi_dcs_write_seq_checked(dsi, 0x30, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x31, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x32, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x33, 0x2A);
	dsi_dcs_write_seq_checked(dsi, 0x34, 0x61);
	dsi_dcs_write_seq_checked(dsi, 0x35, 0xC5);
	dsi_dcs_write_seq_checked(dsi, 0x36, 0x80);
	dsi_dcs_write_seq_checked(dsi, 0x37, 0x23);
	dsi_dcs_write_seq_checked(dsi, 0x40, 0x82);
	dsi_dcs_write_seq_checked(dsi, 0x41, 0x83);
	dsi_dcs_write_seq_checked(dsi, 0x42, 0x80);
	dsi_dcs_write_seq_checked(dsi, 0x43, 0x81);
	dsi_dcs_write_seq_checked(dsi, 0x44, 0x11);
	dsi_dcs_write_seq_checked(dsi, 0x45, 0xF2);
	dsi_dcs_write_seq_checked(dsi, 0x46, 0xF1);
	dsi_dcs_write_seq_checked(dsi, 0x47, 0x11);
	dsi_dcs_write_seq_checked(dsi, 0x48, 0xF4);
	dsi_dcs_write_seq_checked(dsi, 0x49, 0xF3);
	dsi_dcs_write_seq_checked(dsi, 0x50, 0x02);
	dsi_dcs_write_seq_checked(dsi, 0x51, 0x01);
	dsi_dcs_write_seq_checked(dsi, 0x52, 0x04);
	dsi_dcs_write_seq_checked(dsi, 0x53, 0x03);
	dsi_dcs_write_seq_checked(dsi, 0x54, 0x11);
	dsi_dcs_write_seq_checked(dsi, 0x55, 0xF6);
	dsi_dcs_write_seq_checked(dsi, 0x56, 0xF5);
	dsi_dcs_write_seq_checked(dsi, 0x57, 0x11);
	dsi_dcs_write_seq_checked(dsi, 0x58, 0xF8);
	dsi_dcs_write_seq_checked(dsi, 0x59, 0xF7);
	dsi_dcs_write_seq_checked(dsi, 0x7E, 0x02);
	dsi_dcs_write_seq_checked(dsi, 0x7F, 0x80);
	dsi_dcs_write_seq_checked(dsi, 0xE0, 0x5A);
	dsi_dcs_write_seq_checked(dsi, 0xB1, 0x00);
	dsi_dcs_write_seq_checked(dsi, 0xB4, 0x0E);
	dsi_dcs_write_seq_checked(dsi, 0xB5, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xB6, 0x04);
	dsi_dcs_write_seq_checked(dsi, 0xB7, 0x07);
	dsi_dcs_write_seq_checked(dsi, 0xB8, 0x06);
	dsi_dcs_write_seq_checked(dsi, 0xB9, 0x05);
	dsi_dcs_write_seq_checked(dsi, 0xBA, 0x0F);
	dsi_dcs_write_seq_checked(dsi, 0xC7, 0x00);

	/* Exit sleep and turn display on */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		dev_warn(dev, "exit_sleep_mode failed: %d\n", ret);

	msleep(200);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		dev_warn(dev, "set_display_on failed: %d\n", ret);

	msleep(20);

#undef dsi_dcs_write_seq_checked

	dev_info(dev, "NV3051D built-in init sequence done\n");

	return 0;
}

/*
 * Parse and execute Rockchip-style panel-init-sequence DT property.
 *
 * Format:
 *   [ dtype delay_ms len payload... ] repeated
 *
 * Common dtype values:
 *   0x05 - DCS short write, no parameter (command only)
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
		case 0x05: {
			/* DCS short write, no param */
			u8 cmd = buf[i];

			ret = mipi_dsi_dcs_write(ctx->dsi, cmd, NULL, 0);
			dev_dbg(dev, "DCS 0x05 cmd=0x%02x ret=%d\n",
				cmd, ret);
			break;
		}
		case 0x15: {
			/* DCS short write, 1+ params */
			u8 cmd = buf[i];
			const u8 *params = &buf[i + 1];
			int param_len = size - 1;

			ret = mipi_dsi_dcs_write(ctx->dsi, cmd,
						 params, param_len);
			dev_dbg(dev,
				"DCS 0x15 cmd=0x%02x len=%d ret=%d\n",
				cmd, param_len, ret);
			break;
		}
		case 0x39:
			/* DCS long write */
			ret = mipi_dsi_dcs_write_buffer(ctx->dsi,
							&buf[i], size);
			dev_dbg(dev, "DCS 0x39 len=%d ret=%d\n",
				size, ret);
			break;
		default:
			dev_warn(dev,
				 "unknown init dtype 0x%02x, skipping\n",
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

	/*
	 * Just to be safe, make sure we end in a sane state.
	 * These may already be included in panel-init-sequence, but
	 * double-calling is harmless.
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

	/*
	 * First try DT-driven init (panel-init-sequence). If that property
	 * is missing, fall back to the built-in vendor init sequence.
	 */
	ret = nv3051d_init_from_dts(ctx);
	if (ret == -ENOENT) {
		dev_info(dev,
			 "no panel-init-sequence in DT, "
			 "using built-in NV3051D init\n");
		ret = nv3051d_hw_init(ctx);
	}

	if (ret) {
		dev_err(dev, "panel init failed: %d\n", ret);
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
	.prepare   = nv3051d_prepare,
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
			dev_info(dev,
				 "no vdd regulator, proceeding without\n");
			ctx->vdd = NULL;
		} else {
			dev_err(dev, "failed to get vdd: %ld\n",
				PTR_ERR(ctx->vdd));
			return PTR_ERR(ctx->vdd);
		}
	}

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						  GPIOD_OUT_HIGH);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
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
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nv3051d_of_match);

static struct mipi_dsi_driver nv3051d_driver = {
	.probe  = nv3051d_probe,
	.remove = nv3051d_remove,
	.driver = {
		.name           = "panel-newvision-nv3051d",
		.of_match_table = nv3051d_of_match,
	},
};
module_mipi_dsi_driver(nv3051d_driver);

MODULE_DESCRIPTION("NV3051D Panel Driver (5.10 Native, DT + built-in init)");
MODULE_LICENSE("GPL");
