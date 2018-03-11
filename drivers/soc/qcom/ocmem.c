/*
 * Copyright (C) 2015 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
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
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/qcom_scm.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/soc/qcom/ocmem.h>
#include "ocmem.xml.h"

enum region_mode {
	WIDE_MODE = 0x0,
	THIN_MODE,
	MODE_DEFAULT = WIDE_MODE,
};

struct ocmem_region {
	unsigned psgsc_ctrl;
	bool interleaved;
	enum region_mode mode;
	unsigned int num_macros;
	enum ocmem_macro_state macro_state[4];
	unsigned long macro_size;
	unsigned long region_size;
};

struct ocmem_config {
	uint8_t  num_regions;
	uint32_t macro_size;
};

struct ocmem {
	struct device *dev;
	const struct ocmem_config *config;
	struct resource *ocmem_mem;
	struct clk *core_clk;
	struct clk *iface_clk;
	void __iomem *mmio;

	unsigned num_ports;
	unsigned num_macros;
	bool interleaved;

	struct ocmem_region *regions;
};

#define FIELD(val, name) (((val) & name ## __MASK) >> name ## __SHIFT)

static struct ocmem *ocmem;

static bool ocmem_exists(void);

static inline void ocmem_write(struct ocmem *ocmem, u32 reg, u32 data)
{
	writel(data, ocmem->mmio + reg);
}

static inline u32 ocmem_read(struct ocmem *ocmem, u32 reg)
{
	return readl(ocmem->mmio + reg);
}

static int ocmem_clk_enable(struct ocmem *ocmem)
{
	int ret;

	ret = clk_prepare_enable(ocmem->core_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ocmem->iface_clk);
	if (ret)
		return ret;

	return 0;
}

static void ocmem_clk_disable(struct ocmem *ocmem)
{
	clk_disable_unprepare(ocmem->iface_clk);
	clk_disable_unprepare(ocmem->core_clk);
}

static int ocmem_dev_remove(struct platform_device *pdev)
{
	ocmem_clk_disable(ocmem);
	return 0;
}

static void update_ocmem(struct ocmem *ocmem)
{
	uint32_t region_mode_ctrl = 0x0;
	unsigned pos = 0;
	unsigned i = 0;

	if (!qcom_scm_ocmem_lock_available()) {
		for (i = 0; i < ocmem->config->num_regions; i++) {
			struct ocmem_region *region = &ocmem->regions[i];
			pos = i << 2;
			if (region->mode == THIN_MODE)
				region_mode_ctrl |= BIT(pos);
		}
		dev_dbg(ocmem->dev, "ocmem_region_mode_control %x\n", region_mode_ctrl);
		ocmem_write(ocmem, REG_OCMEM_REGION_MODE_CTL, region_mode_ctrl);
	}

	for (i = 0; i < ocmem->config->num_regions; i++) {
		struct ocmem_region *region = &ocmem->regions[i];

		ocmem_write(ocmem, REG_OCMEM_PSGSC_CTL(i),
				OCMEM_PSGSC_CTL_MACRO0_MODE(region->macro_state[0]) |
				OCMEM_PSGSC_CTL_MACRO1_MODE(region->macro_state[1]) |
				OCMEM_PSGSC_CTL_MACRO2_MODE(region->macro_state[2]) |
				OCMEM_PSGSC_CTL_MACRO3_MODE(region->macro_state[3]));
	}
}

static unsigned long phys_to_offset(unsigned long addr)
{
	if ((addr < ocmem->ocmem_mem->start) ||
		(addr >= ocmem->ocmem_mem->end))
		return 0;
	return addr - ocmem->ocmem_mem->start;
}

static unsigned long device_address(enum ocmem_client client, unsigned long addr)
{
	/* TODO, gpu uses phys_to_offset, but others do not.. */
	return phys_to_offset(addr);
}

static void update_range(struct ocmem *ocmem, struct ocmem_buf *buf,
		enum ocmem_macro_state mstate, enum region_mode rmode)
{
	unsigned long offset = 0;
	int i, j;

	/*
	 * TODO probably should assert somewhere that range is aligned
	 * to macro boundaries..
	 */

	for (i = 0; i < ocmem->config->num_regions; i++) {
		struct ocmem_region *region = &ocmem->regions[i];
		if ((buf->offset <= offset) && (offset < (buf->offset + buf->len)))
			region->mode = rmode;
		for (j = 0; j < region->num_macros; j++) {
			if ((buf->offset <= offset) && (offset < (buf->offset + buf->len)))
				region->macro_state[j] = mstate;
			offset += region->macro_size;
		}
	}

	update_ocmem(ocmem);
}

struct ocmem_buf *ocmem_allocate(enum ocmem_client client, unsigned long size)
{
	struct ocmem_buf *buf;

	if (!ocmem) {
		if (ocmem_exists())
			return ERR_PTR(-EPROBE_DEFER);
		return ERR_PTR(-ENXIO);
	}

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	/*
	 * TODO less hard-coded allocation that works for more than
	 * one user:
	 */

	buf->offset = 0;
	buf->addr = device_address(client, buf->offset);
	buf->len = size;

	update_range(ocmem, buf, CORE_ON, WIDE_MODE);

	if (qcom_scm_ocmem_lock_available()) {
		int ret;
		ret = qcom_scm_ocmem_lock(QCOM_SCM_OCMEM_GRAPHICS_ID,
				buf->offset, buf->len, WIDE_MODE);
		if (ret)
			dev_err(ocmem->dev, "could not lock: %d\n", ret);
	} else {
		if (client == OCMEM_GRAPHICS) {
			ocmem_write(ocmem, REG_OCMEM_GFX_MPU_START, buf->offset);
			ocmem_write(ocmem, REG_OCMEM_GFX_MPU_END, buf->offset + buf->len);
		}
	}

	return buf;
}
EXPORT_SYMBOL(ocmem_allocate);

void ocmem_free(enum ocmem_client client, struct ocmem_buf *buf)
{
	update_range(ocmem, buf, CLK_OFF, MODE_DEFAULT);

	if (qcom_scm_ocmem_lock_available()) {
		int ret;
		ret = qcom_scm_ocmem_unlock(QCOM_SCM_OCMEM_GRAPHICS_ID,
				buf->offset, buf->len);
		if (ret)
			dev_err(ocmem->dev, "could not unlock: %d\n", ret);
	} else {
		if (client == OCMEM_GRAPHICS) {
			ocmem_write(ocmem, REG_OCMEM_GFX_MPU_START, 0x0);
			ocmem_write(ocmem, REG_OCMEM_GFX_MPU_END, 0x0);
		}
	}

	kfree(buf);
}
EXPORT_SYMBOL(ocmem_free);

static const struct ocmem_config ocmem_8974_config = {
	.num_regions = 3, .macro_size = SZ_128K,
};

static const struct of_device_id ocmem_dt_match[] = {
	{ .compatible = "qcom,ocmem-msm8974", .data = &ocmem_8974_config },
	{}
};

static int ocmem_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct ocmem_config *config = NULL;
	const struct of_device_id *match;
	struct resource *res;
	uint32_t reg, num_banks, region_size;
	int i, j, ret;

	/* we need scm to be available: */
	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	match = of_match_device(ocmem_dt_match, dev);
	if (match)
		config = match->data;

	if (!config) {
		dev_err(dev, "unknown config: %s\n", dev->of_node->name);
		return -ENXIO;
	}

	ocmem = devm_kzalloc(dev, sizeof(*ocmem), GFP_KERNEL);
	if (!ocmem)
		return -ENOMEM;

	ocmem->dev = dev;
	ocmem->config = config;

	ocmem->core_clk = devm_clk_get(dev, "core_clk");
	if (IS_ERR(ocmem->core_clk)) {
		dev_info(dev, "Unable to get the core clock\n");
		return PTR_ERR(ocmem->core_clk);
	}

	ocmem->iface_clk = devm_clk_get(dev, "iface_clk");
	if (IS_ERR(ocmem->iface_clk)) {
		ret = PTR_ERR(ocmem->iface_clk);
		ocmem->iface_clk = NULL;
		/* in probe-defer case, propagate error up and try again later: */
		if (ret == -EPROBE_DEFER)
			goto fail;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"ocmem_ctrl_physical");
	ocmem->mmio = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ocmem->mmio)) {
		dev_err(&pdev->dev, "failed to ioremap memory resource\n");
		ret = -EINVAL;
		goto fail;
	}

	ocmem->ocmem_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"ocmem_physical");
	if (!ocmem->ocmem_mem) {
		dev_err(dev, "could not get OCMEM region\n");
		return -ENXIO;
	}

	/* The core clock is synchronous with graphics */
	WARN_ON(clk_set_rate(ocmem->core_clk, 1000) < 0);
	WARN_ON(clk_set_rate(ocmem->iface_clk, 291750000) < 0);

	ret = ocmem_clk_enable(ocmem);
	if (ret)
		goto fail;

	//clk_set_rate(ocmem->iface_clk, 150000000);

	dev_dbg(dev, "configuring scm\n");
	ret = qcom_scm_restore_sec_cfg(5, 0);
	if (ret)
		goto fail;

	reg = ocmem_read(ocmem, REG_OCMEM_HW_PROFILE);
	ocmem->num_ports = FIELD(reg, OCMEM_HW_PROFILE_NUM_PORTS);
	ocmem->num_macros = FIELD(reg, OCMEM_HW_PROFILE_NUM_MACROS);
	ocmem->interleaved = !!(reg & OCMEM_HW_PROFILE_INTERLEAVING);

	num_banks = ocmem->num_ports / 2;
	region_size = config->macro_size * num_banks;

	dev_info(dev, "%u ports, %u regions, %u macros, %sinterleaved\n",
			ocmem->num_ports, config->num_regions, ocmem->num_macros,
			ocmem->interleaved ? "" : "not ");

	ocmem->regions = devm_kcalloc(dev, config->num_regions,
			sizeof(struct ocmem_region), GFP_KERNEL);
	if (!ocmem->regions) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < config->num_regions; i++) {
		struct ocmem_region *region = &ocmem->regions[i];

		if (WARN_ON(num_banks > ARRAY_SIZE(region->macro_state))) {
			ret = -EINVAL;
			goto fail;
		}

		region->mode = MODE_DEFAULT;
		region->num_macros = num_banks;

		if ((i == (config->num_regions - 1)) &&
				(reg & OCMEM_HW_PROFILE_LAST_REGN_HALFSIZE)) {
			region->macro_size = config->macro_size / 2;
			region->region_size = region_size / 2;
		} else {
			region->macro_size = config->macro_size;
			region->region_size = region_size;
		}

		for (j = 0; j < ARRAY_SIZE(region->macro_state); j++)
			region->macro_state[j] = CLK_OFF;
	}

	return 0;

fail:
	dev_err(dev, "probe failed\n");
	ocmem_dev_remove(pdev);
	return ret;
}

MODULE_DEVICE_TABLE(of, ocmem_dt_match);

static struct platform_driver ocmem_driver = {
	.probe = ocmem_dev_probe,
	.remove = ocmem_dev_remove,
	.driver = {
		.name = "ocmem",
		.of_match_table = ocmem_dt_match,
	},
};

static bool ocmem_exists(void)
{
	struct device_driver *drv = &ocmem_driver.driver;
	struct device *d;

	d = bus_find_device(&platform_bus_type, NULL, drv,
			(void *)platform_bus_type.match);
	if (d) {
		put_device(d);
		return true;
	}

	return false;
}

module_platform_driver(ocmem_driver);
