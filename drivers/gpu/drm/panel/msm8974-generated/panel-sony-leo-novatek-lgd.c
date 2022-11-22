// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct novatek_lgd {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
};

static inline struct novatek_lgd *to_novatek_lgd(struct drm_panel *panel)
{
	return container_of(panel, struct novatek_lgd, panel);
}

static int novatek_lgd_on(struct novatek_lgd *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x94);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb8, 0xad);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x86);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6, 0x77);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x10, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_set_tear_scanline_multi(&dsi_ctx, 0x0300);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 30);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int novatek_lgd_off(struct novatek_lgd *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	return dsi_ctx.accum_err;
}

static int novatek_lgd_prepare(struct drm_panel *panel)
{
	struct novatek_lgd *ctx = to_novatek_lgd(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = novatek_lgd_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		return ret;
	}

	return 0;
}

static int novatek_lgd_unprepare(struct drm_panel *panel)
{
	struct novatek_lgd *ctx = to_novatek_lgd(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = novatek_lgd_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);


	return 0;
}

static const struct drm_display_mode novatek_lgd_mode = {
	.clock = (1080 + 56 + 8 + 8) * (1920 + 233 + 2 + 8) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 56,
	.hsync_end = 1080 + 56 + 8,
	.htotal = 1080 + 56 + 8 + 8,
	.vdisplay = 1920,
	.vsync_start = 1920 + 233,
	.vsync_end = 1920 + 233 + 2,
	.vtotal = 1920 + 233 + 2 + 8,
	.width_mm = 64,
	.height_mm = 114,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int novatek_lgd_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &novatek_lgd_mode);
}

static const struct drm_panel_funcs novatek_lgd_panel_funcs = {
	.prepare = novatek_lgd_prepare,
	.unprepare = novatek_lgd_unprepare,
	.get_modes = novatek_lgd_get_modes,
};

static int novatek_lgd_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct novatek_lgd *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct novatek_lgd, panel,
				   &novatek_lgd_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void novatek_lgd_remove(struct mipi_dsi_device *dsi)
{
	struct novatek_lgd *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id novatek_lgd_of_match[] = {
	{ .compatible = "sony,leo-panel-novatek-lgd" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, novatek_lgd_of_match);

static struct mipi_dsi_driver novatek_lgd_driver = {
	.probe = novatek_lgd_probe,
	.remove = novatek_lgd_remove,
	.driver = {
		.name = "panel-novatek-lgd",
		.of_match_table = novatek_lgd_of_match,
	},
};
module_mipi_dsi_driver(novatek_lgd_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for lgd novatek 1080p cmd");
MODULE_LICENSE("GPL");
