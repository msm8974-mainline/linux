// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
// Copyright (c) 2017-2018, Linaro Limited

#include <linux/slab.h>
#include <sound/soc.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include "wcd9320.h"
#include "wcd-clsh.h"

struct wcd_clsh_ctrl {
	int state;
	int clsh_users;
	int cp_users;
	struct snd_soc_component *comp;
};

struct wcd9xxx_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

static inline void wcd_clsh_common_init(struct snd_soc_component *comp, bool hph)
{
	const struct wcd9xxx_reg_mask_val reg_set[] = {
		// wcd9xxx_cfg_clsh_buck
		{WCD9320_BUCK_CTRL_CCL_4, 0x0B, 0x00},
		{WCD9320_BUCK_CTRL_CCL_1, 0xF0, 0x50},
		{WCD9320_BUCK_CTRL_CCL_3, 0x03, 0x00},
		{WCD9320_BUCK_CTRL_CCL_3, 0x0B, 0x00},
		// wcd9xxx_cfg_clsh_param_common
		{WCD9320_CDC_CLSH_BUCK_NCP_VARS, 0x3 << 0, 0},
		{WCD9320_CDC_CLSH_BUCK_NCP_VARS, 0x3 << 2, 1 << 2},
		{WCD9320_CDC_CLSH_BUCK_NCP_VARS, (0x1 << 4), 0},
		{WCD9320_CDC_CLSH_B2_CTL, (0x3 << 0), 0x01},
		{WCD9320_CDC_CLSH_B2_CTL, (0x3 << 2), (0x01 << 2)},
		{WCD9320_CDC_CLSH_B2_CTL, (0xf << 4), (0x03 << 4)},
		{WCD9320_CDC_CLSH_B3_CTL, (0xf << 4), (0x03 << 4)},
		{WCD9320_CDC_CLSH_B3_CTL, (0xf << 0), (0x0B)},
		{WCD9320_CDC_CLSH_B1_CTL, (0x1 << 5), (0x01 << 5)},
		{WCD9320_CDC_CLSH_B1_CTL, (0x1 << 1), (0x01 << 1)},
	};

	const struct wcd9xxx_reg_mask_val reg_ear[] = {
		{WCD9320_CDC_CLSH_B1_CTL, (0x1 << 7), 0},
		{WCD9320_CDC_CLSH_V_PA_HD_EAR, (0x3f << 0), 0x0D},
		{WCD9320_CDC_CLSH_V_PA_MIN_EAR, (0x3f << 0), 0x3A},

		/* Under assumption that EAR load is 10.7ohm */
		{WCD9320_CDC_CLSH_IDLE_EAR_THSD, (0x3f << 0), 0x26},
		{WCD9320_CDC_CLSH_FCLKONLY_EAR_THSD, (0x3f << 0), 0x2C},
		{WCD9320_CDC_CLSH_I_PA_FACT_EAR_L, 0xff, 0xA9},
		{WCD9320_CDC_CLSH_I_PA_FACT_EAR_U, 0xff, 0x07},
		{WCD9320_CDC_CLSH_K_ADDR, (0x1 << 7), 0},
		{WCD9320_CDC_CLSH_K_ADDR, (0xf << 0), 0x08},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x1b},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x00},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x2d},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x00},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x36},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x00},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x37},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x00},
	};

	const struct wcd9xxx_reg_mask_val reg_hph[] = {
		{WCD9320_CDC_CLSH_B1_CTL, (0x1 << 6), 0},
		{WCD9320_CDC_CLSH_V_PA_HD_HPH, 0x3f, 0x0D},
		{WCD9320_CDC_CLSH_V_PA_MIN_HPH, 0x3f, 0x1D},

		/* Under assumption that HPH load is 16ohm per channel */
		{WCD9320_CDC_CLSH_IDLE_HPH_THSD, 0x3f, 0x13},
		{WCD9320_CDC_CLSH_FCLKONLY_HPH_THSD, 0x1f, 0x19},
		{WCD9320_CDC_CLSH_I_PA_FACT_HPH_L, 0xff, 0x97},
		{WCD9320_CDC_CLSH_I_PA_FACT_HPH_U, 0xff, 0x05},
		{WCD9320_CDC_CLSH_K_ADDR, (0x1 << 7), 0},
		{WCD9320_CDC_CLSH_K_ADDR, 0x0f, 0},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0xAE},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x01},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x1C},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x00},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x24},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x00},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x25},
		{WCD9320_CDC_CLSH_K_DATA, 0xff, 0x00},
	};

	int i;
	for (i = 0; i < ARRAY_SIZE(reg_set); i++)
		snd_soc_component_update_bits(comp, reg_set[i].reg, reg_set[i].mask, reg_set[i].val);

	if (hph) {
		for (i = 0; i < ARRAY_SIZE(reg_hph); i++)
			snd_soc_component_update_bits(comp, reg_hph[i].reg, reg_hph[i].mask, reg_hph[i].val);
	} else {
		for (i = 0; i < ARRAY_SIZE(reg_ear); i++)
			snd_soc_component_update_bits(comp, reg_ear[i].reg, reg_ear[i].mask, reg_ear[i].val);
	}
}

static inline void wcd_enable_clsh_block(struct wcd_clsh_ctrl *ctrl,
					 bool enable)
{
	struct snd_soc_component *comp = ctrl->comp;

	if ((enable && ++ctrl->clsh_users == 1) ||
	    (!enable && --ctrl->clsh_users == 0))
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLSH_B1_CTL, 0x01, enable);
	if (ctrl->clsh_users < 0)
		ctrl->clsh_users = 0;
}

static inline void wcd_enable_chargepump(struct wcd_clsh_ctrl *ctrl, bool enable)
{
	struct snd_soc_component *comp = ctrl->comp;

	if ((enable && ++ctrl->cp_users == 1) ||
	    (!enable && --ctrl->cp_users == 0))
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLK_OTHR_CTL, 0x01, enable);
	if (ctrl->cp_users < 0)
		ctrl->cp_users = 0;
}

#define BUCK_VREF_2V 0xFF
#define BUCK_VREF_1P8V 0xE6
#define BUCK_SETTLE_TIME_US 50

static void wcd_enable_buck_mode(struct snd_soc_component *comp)
{
	int i;
	const struct wcd9xxx_reg_mask_val reg_set[] = {
		{WCD9320_BUCK_MODE_5, 0x02, 0x03},
		{WCD9320_BUCK_MODE_4, 0xFF, BUCK_VREF_2V},
		{WCD9320_BUCK_MODE_1, 0x04, 0x04},
		{WCD9320_BUCK_MODE_1, 0x08, 0x00},
		{WCD9320_BUCK_MODE_3, 0x04, 0x00},
		{WCD9320_BUCK_MODE_3, 0x08, 0x00},
		{WCD9320_BUCK_MODE_1, 0x80, 0x80},
	};

	for (i = 0; i < ARRAY_SIZE(reg_set); i++)
		snd_soc_component_update_bits(comp, reg_set[i].reg, reg_set[i].mask, reg_set[i].val);

	usleep_range(BUCK_SETTLE_TIME_US, BUCK_SETTLE_TIME_US);
}

#define NCP_FCLK_LEVEL_8 0x08
#define NCP_FCLK_LEVEL_5 0x05
#define NCP_SETTLE_TIME_US 50

static void wcd_set_fclk_enable_ncp(struct snd_soc_component *comp)
{
	int i;
	const struct wcd9xxx_reg_mask_val reg_set[] = {
		{WCD9320_NCP_STATIC, 0x20, 0x20},
		{WCD9320_NCP_EN, 0x01, 0x01},
	};
	snd_soc_component_update_bits(comp, WCD9320_NCP_STATIC,
						0x010, 0x00);
	snd_soc_component_update_bits(comp, WCD9320_NCP_STATIC,
						0x0F, NCP_FCLK_LEVEL_8);
	for (i = 0; i < ARRAY_SIZE(reg_set); i++)
		snd_soc_component_update_bits(comp, reg_set[i].reg,
					reg_set[i].mask, reg_set[i].val);

	usleep_range(NCP_SETTLE_TIME_US, NCP_SETTLE_TIME_US);
}

static void wcd_clsh_state_lo(struct wcd_clsh_ctrl *ctrl, int req_state, bool is_enable)
{
	printk("%s unimplemented\n", __FUNCTION__);
}

static void wcd_clsh_state_hph_r(struct wcd_clsh_ctrl *ctrl, int req_state, bool is_enable)
{
	struct snd_soc_component *comp = ctrl->comp;

	if (is_enable) {
		wcd_clsh_common_init(comp, true);
		wcd_enable_clsh_block(ctrl, true);
		wcd_enable_chargepump(ctrl, true);
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLSH_B1_CTL, 0x4, 0x4); // 8 for left
		wcd_enable_buck_mode(comp);
		wcd_set_fclk_enable_ncp(comp);
	} else {
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLSH_B1_CTL, 0x4, 0x0);
	}
}

static void wcd_clsh_state_hph_l(struct wcd_clsh_ctrl *ctrl, int req_state, bool is_enable)
{
	struct snd_soc_component *comp = ctrl->comp;

	if (is_enable) {
		wcd_clsh_common_init(comp, true);
		wcd_enable_clsh_block(ctrl, true);
		wcd_enable_chargepump(ctrl, true);
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLSH_B1_CTL, 0x8, 0x8); // 8 for left
		wcd_enable_buck_mode(comp);
		wcd_set_fclk_enable_ncp(comp);
	} else {
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLSH_B1_CTL, 0x8, 0x0);
	}
}

static void wcd_clsh_state_ear(struct wcd_clsh_ctrl *ctrl, int req_state, bool is_enable)
{
	printk("%s unimplemented\n", __FUNCTION__);
}

static int _wcd_clsh_ctrl_set_state(struct wcd_clsh_ctrl *ctrl, int req_state, bool is_enable)
{
	switch (req_state) {
	case WCD_CLSH_STATE_EAR:
		wcd_clsh_state_ear(ctrl, req_state, is_enable);
		break;
	case WCD_CLSH_STATE_HPHL:
		wcd_clsh_state_hph_l(ctrl, req_state, is_enable);
		break;
	case WCD_CLSH_STATE_HPHR:
		wcd_clsh_state_hph_r(ctrl, req_state, is_enable);
		break;
		break;
	case WCD_CLSH_STATE_LO:
		wcd_clsh_state_lo(ctrl, req_state, is_enable);
		break;
	default:
		break;
	}

	return 0;
}

/*
 * Function: wcd_clsh_is_state_valid
 * Params: state
 * Description:
 * Provides information on valid states of Class H configuration
 */
static bool wcd_clsh_is_state_valid(int state)
{
	switch (state) {
	case WCD_CLSH_STATE_IDLE:
	case WCD_CLSH_STATE_EAR:
	case WCD_CLSH_STATE_HPHL:
	case WCD_CLSH_STATE_HPHR:
	case WCD_CLSH_STATE_LO:
		return true;
	default:
		return false;
	};
}

/*
 * Function: wcd_clsh_fsm
 * Params: ctrl, req_state, req_type, clsh_event
 * Description:
 * This function handles PRE DAC and POST DAC conditions of different devices
 * and updates class H configuration of different combination of devices
 * based on validity of their states. ctrl will contain current
 * class h state information
 */
int wcd_clsh_ctrl_set_state(struct wcd_clsh_ctrl *ctrl,
			    enum wcd_clsh_event clsh_event, int nstate)
{
	struct snd_soc_component *comp = ctrl->comp;

	if (nstate == ctrl->state)
		return 0;

	if (!wcd_clsh_is_state_valid(nstate)) {
		dev_err(comp->dev, "Class-H not a valid new state:\n");
		return -EINVAL;
	}

	switch (clsh_event) {
	case WCD_CLSH_EVENT_PRE_DAC:
		_wcd_clsh_ctrl_set_state(ctrl, nstate, true);
		break;
	case WCD_CLSH_EVENT_POST_PA:
		_wcd_clsh_ctrl_set_state(ctrl, nstate, false);
		break;
	}

	ctrl->state = nstate;
	return 0;
}

int wcd_clsh_ctrl_get_state(struct wcd_clsh_ctrl *ctrl)
{
	return ctrl->state;
}

struct wcd_clsh_ctrl *wcd_clsh_ctrl_alloc(struct snd_soc_component *comp,
					  int version)
{
	struct wcd_clsh_ctrl *ctrl;

	ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return ERR_PTR(-ENOMEM);

	ctrl->state = WCD_CLSH_STATE_IDLE;
	ctrl->comp = comp;

	return ctrl;
}

void wcd_clsh_ctrl_free(struct wcd_clsh_ctrl *ctrl)
{
	kfree(ctrl);
}
