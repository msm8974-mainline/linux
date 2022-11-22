// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct s6d6fa1 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline struct s6d6fa1 *to_s6d6fa1(struct drm_panel *panel)
{
	return container_of(panel, struct s6d6fa1, panel);
}

static void s6d6fa1_reset(struct s6d6fa1 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int s6d6fa1_on(struct s6d6fa1 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf1, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf5,
					 0x10, 0x18, 0x00, 0xd1, 0xa7, 0x11,
					 0x08);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3,
					 0x10, 0xf0, 0x00, 0xbb, 0x04, 0x08);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb6,
					 0x29, 0x10, 0x2c, 0x64, 0x64, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb7,
					 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
					 0x00, 0x09, 0x03, 0xf5, 0x00, 0x00,
					 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb8,
					 0x3b, 0x00, 0x18, 0x18, 0x03, 0x21,
					 0x11, 0x00, 0x83, 0xf4, 0xbb, 0x00,
					 0x03, 0x1e, 0x21, 0x00, 0xbe, 0xbb,
					 0x0b, 0xe0, 0x70, 0xff, 0xff, 0x00,
					 0x20, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb9,
					 0x1d, 0x00, 0x18, 0x18, 0x03, 0x00,
					 0x11, 0x00, 0x02, 0xf4, 0xbb, 0x00,
					 0x03, 0x00, 0x00, 0x11, 0x86, 0x83,
					 0x18, 0x61, 0x20, 0xff, 0xff, 0x00,
					 0x40, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba,
					 0x3b, 0x00, 0x18, 0x18, 0x03, 0x21,
					 0x11, 0x00, 0x83, 0xf4, 0xbb, 0x00,
					 0x03, 0x1e, 0x21, 0x00, 0xbe, 0xbb,
					 0x0b, 0xe0, 0x70, 0xff, 0xff, 0x00,
					 0x20, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbb,
					 0x3b, 0x00, 0x18, 0x18, 0x03, 0x21,
					 0x11, 0x00, 0x83, 0xf4, 0xbb, 0x00,
					 0x03, 0x1e, 0x21, 0x00, 0xbe, 0xbb,
					 0x0b, 0xe0, 0x70, 0xff, 0xff, 0x00,
					 0x20, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc,
					 0x20, 0x80, 0x3c, 0x19, 0x01, 0x02,
					 0x06, 0x00, 0x00, 0x2d, 0x06, 0x0f,
					 0x13, 0x0b, 0x1e, 0x04, 0x02, 0x2a,
					 0x00, 0x00, 0x02, 0x00, 0x99);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd,
					 0x01, 0x07, 0x07, 0x07, 0x00, 0x00,
					 0x00, 0x00, 0x05, 0x01, 0x00, 0x00,
					 0x00, 0x40, 0x00, 0x05, 0x2d, 0x05,
					 0x05, 0x05);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbe,
					 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					 0x02, 0x0b, 0x0a, 0x0b, 0x07, 0x0b,
					 0x04, 0x06, 0x02, 0x01, 0x00, 0x09,
					 0x0b, 0x0e, 0x0d, 0x0c, 0x11, 0x10,
					 0x0f, 0x0b, 0x0b, 0x0b, 0x0b, 0x12,
					 0x03, 0x05, 0x02, 0x01, 0x00, 0x08,
					 0x0b, 0x0e, 0x0d, 0x0c, 0x11, 0x10,
					 0x0f);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbf,
					 0x00, 0x00, 0x00, 0x00, 0x0c, 0xcc,
					 0xcc, 0xcc, 0xcc, 0x0c, 0xc6, 0xcc,
					 0xc7, 0xcc);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd0, 0x08);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe3, 0x22);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe8,
					 0x00, 0x03, 0x10, 0x04, 0x02, 0x06,
					 0xd0);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf2,
					 0x46, 0x43, 0x13, 0x33, 0xc1, 0x16);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf4,
					 0x50, 0x50, 0x70, 0x14, 0x14, 0x14,
					 0x14, 0x06, 0x16, 0x26, 0x00, 0x6e,
					 0x1f, 0x14, 0x0e, 0xc9, 0x02, 0x55,
					 0x35, 0x54, 0xaa, 0x55, 0x05, 0x04,
					 0x44, 0x48, 0x30);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf7,
					 0x00, 0x00, 0x3f, 0xff);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfa,
					 0x05, 0x7c, 0x19, 0x21, 0x2d, 0x39,
					 0x3f, 0x45, 0x48, 0x48, 0x56, 0x5b,
					 0x3c, 0x3d, 0x3d, 0x46, 0x43, 0x3e,
					 0x41, 0x3d, 0x3a, 0x3a, 0x23, 0x23,
					 0x26, 0x2b, 0x1a, 0x05, 0x7c, 0x19,
					 0x22, 0x2e, 0x3a, 0x40, 0x45, 0x48,
					 0x48, 0x56, 0x5b, 0x3c, 0x3d, 0x3d,
					 0x46, 0x44, 0x3f, 0x42, 0x3e, 0x3b,
					 0x3b, 0x25, 0x24, 0x27, 0x2c, 0x1a,
					 0x05, 0x7c, 0x22, 0x2f, 0x39, 0x43,
					 0x48, 0x4c, 0x4d, 0x4c, 0x5a, 0x5e,
					 0x3e, 0x3f, 0x3e, 0x46, 0x44, 0x3e,
					 0x41, 0x3d, 0x3a, 0x3a, 0x23, 0x22,
					 0x25, 0x2b, 0x1a, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfb,
					 0x05, 0x7c, 0x19, 0x21, 0x2d, 0x39,
					 0x3f, 0x45, 0x48, 0x48, 0x56, 0x5b,
					 0x3c, 0x3d, 0x3d, 0x46, 0x43, 0x3e,
					 0x41, 0x3d, 0x3a, 0x3a, 0x23, 0x23,
					 0x26, 0x2b, 0x1a, 0x05, 0x7c, 0x19,
					 0x22, 0x2e, 0x3a, 0x40, 0x45, 0x48,
					 0x48, 0x56, 0x5b, 0x3c, 0x3d, 0x3d,
					 0x46, 0x44, 0x3f, 0x42, 0x3e, 0x3b,
					 0x3b, 0x25, 0x24, 0x27, 0x2c, 0x1a,
					 0x05, 0x7c, 0x22, 0x2f, 0x39, 0x43,
					 0x48, 0x4c, 0x4d, 0x4c, 0x5a, 0x5e,
					 0x3e, 0x3f, 0x3e, 0x46, 0x44, 0x3e,
					 0x41, 0x3d, 0x3a, 0x3a, 0x23, 0x22,
					 0x25, 0x2b, 0x1a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf1, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x00ff);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x2c);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 40);

	return dsi_ctx.accum_err;
}

static int s6d6fa1_off(struct s6d6fa1 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 80);

	return dsi_ctx.accum_err;
}

static int s6d6fa1_prepare(struct drm_panel *panel)
{
	struct s6d6fa1 *ctx = to_s6d6fa1(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	s6d6fa1_reset(ctx);

	ret = s6d6fa1_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int s6d6fa1_unprepare(struct drm_panel *panel)
{
	struct s6d6fa1 *ctx = to_s6d6fa1(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = s6d6fa1_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode s6d6fa1_mode = {
	.clock = (1080 + 216 + 16 + 52) * (1920 + 4 + 1 + 3) * 57 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 216,
	.hsync_end = 1080 + 216 + 16,
	.htotal = 1080 + 216 + 16 + 52,
	.vdisplay = 1920,
	.vsync_start = 1920 + 4,
	.vsync_end = 1920 + 4 + 1,
	.vtotal = 1920 + 4 + 1 + 3,
	.width_mm = 62,
	.height_mm = 110,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int s6d6fa1_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &s6d6fa1_mode);
}

static const struct drm_panel_funcs s6d6fa1_panel_funcs = {
	.prepare = s6d6fa1_prepare,
	.unprepare = s6d6fa1_unprepare,
	.get_modes = s6d6fa1_get_modes,
};

static int s6d6fa1_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

// TODO: Check if /sys/class/backlight/.../actual_brightness actually returns
// correct values. If not, remove this function.
static int s6d6fa1_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness & 0xff;
}

static const struct backlight_ops s6d6fa1_bl_ops = {
	.update_status = s6d6fa1_bl_update_status,
	.get_brightness = s6d6fa1_bl_get_brightness,
};

static struct backlight_device *
s6d6fa1_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &s6d6fa1_bl_ops, &props);
}

static int s6d6fa1_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6d6fa1 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6d6fa1, panel,
				   &s6d6fa1_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = s6d6fa1_create_backlight(dsi);
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

static void s6d6fa1_remove(struct mipi_dsi_device *dsi)
{
	struct s6d6fa1 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6d6fa1_of_match[] = {
	{ .compatible = "fairphone,fp2-panel-s6d6fa1" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6d6fa1_of_match);

static struct mipi_dsi_driver s6d6fa1_driver = {
	.probe = s6d6fa1_probe,
	.remove = s6d6fa1_remove,
	.driver = {
		.name = "panel-s6d6fa1",
		.of_match_table = s6d6fa1_of_match,
	},
};
module_mipi_dsi_driver(s6d6fa1_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for S6D6FA1 1080p video mode dsi panel");
MODULE_LICENSE("GPL");
