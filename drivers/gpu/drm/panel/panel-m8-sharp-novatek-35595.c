// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct m8_sharp_novatek_35595 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *backlight_gpio;
};

static inline
struct m8_sharp_novatek_35595 *to_m8_sharp_novatek_35595(struct drm_panel *panel)
{
	return container_of(panel, struct m8_sharp_novatek_35595, panel);
}

static int m8_sharp_novatek_35595_on(struct m8_sharp_novatek_35595 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x01, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x46, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x17, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x18, 0xfa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x19, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0xf5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1b, 0xee);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0xe1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1d, 0xd5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1e, 0xcd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1f, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x20, 0xb4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x21, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22, 0xfa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x23, 0xf5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x24, 0xeb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x25, 0xe1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_GAMMA_CURVE, 0xc8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x27, 0xaa);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x96);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x73);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x66);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0b, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0e, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0f, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x10, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x12);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x22);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x01, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x02, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x03, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x04, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x06, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0a, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0b, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0c, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0d, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0e, 0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x0f, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x10, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x13, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x32, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x38, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0x10);
	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_VSYNC_TIMING, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x41, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x42, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x43, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x44, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_GET_SCANLINE, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x46, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x47, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4a, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4b, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4c, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1a, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x54, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x56, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x68, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4d, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x69, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x82);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS,
				     0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb8, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x00);

	return dsi_ctx.accum_err;
}

static int m8_sharp_novatek_35595_off(struct m8_sharp_novatek_35595 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x28, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x10, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 74);

	return dsi_ctx.accum_err;
}

static int m8_sharp_novatek_35595_prepare(struct drm_panel *panel)
{
	struct m8_sharp_novatek_35595 *ctx = to_m8_sharp_novatek_35595(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	ret = m8_sharp_novatek_35595_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		regulator_disable(ctx->supply);
		return ret;
	}

	return 0;
}

static int m8_sharp_novatek_35595_unprepare(struct drm_panel *panel)
{
	struct m8_sharp_novatek_35595 *ctx = to_m8_sharp_novatek_35595(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = m8_sharp_novatek_35595_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	regulator_disable(ctx->supply);

	return 0;
}

static const struct drm_display_mode m8_sharp_novatek_35595_mode = {
	.clock = (1080 + 100 + 10 + 50) * (1920 + 4 + 2 + 4) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 100,
	.hsync_end = 1080 + 100 + 10,
	.htotal = 1080 + 100 + 10 + 50,
	.vdisplay = 1920,
	.vsync_start = 1920 + 4,
	.vsync_end = 1920 + 4 + 2,
	.vtotal = 1920 + 4 + 2 + 4,
	.width_mm = 61,
	.height_mm = 109,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int m8_sharp_novatek_35595_get_modes(struct drm_panel *panel,
					    struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &m8_sharp_novatek_35595_mode);
}

static const struct drm_panel_funcs m8_sharp_novatek_35595_panel_funcs = {
	.prepare = m8_sharp_novatek_35595_prepare,
	.unprepare = m8_sharp_novatek_35595_unprepare,
	.get_modes = m8_sharp_novatek_35595_get_modes,
};

static int m8_sharp_novatek_35595_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct m8_sharp_novatek_35595 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	gpiod_set_value_cansleep(ctx->backlight_gpio, !!brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops m8_sharp_novatek_35595_bl_ops = {
	.update_status = m8_sharp_novatek_35595_bl_update_status,
};

static struct backlight_device *
m8_sharp_novatek_35595_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &m8_sharp_novatek_35595_bl_ops, &props);
}

static int m8_sharp_novatek_35595_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct m8_sharp_novatek_35595 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "Failed to get power regulator\n");

	ctx->backlight_gpio = devm_gpiod_get(dev, "backlight", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->backlight_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->backlight_gpio),
				     "Failed to get backlight-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &m8_sharp_novatek_35595_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = m8_sharp_novatek_35595_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void m8_sharp_novatek_35595_remove(struct mipi_dsi_device *dsi)
{
	struct m8_sharp_novatek_35595 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id m8_sharp_novatek_35595_of_match[] = {
	{ .compatible = "mdss,m8-sharp-novatek-35595" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, m8_sharp_novatek_35595_of_match);

static struct mipi_dsi_driver m8_sharp_novatek_35595_driver = {
	.probe = m8_sharp_novatek_35595_probe,
	.remove = m8_sharp_novatek_35595_remove,
	.driver = {
		.name = "panel-m8-sharp-novatek-35595",
		.of_match_table = m8_sharp_novatek_35595_of_match,
	},
};
module_mipi_dsi_driver(m8_sharp_novatek_35595_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for m8 1080p sharp/NT35595 cmd mode dsi panel");
MODULE_LICENSE("GPL");
