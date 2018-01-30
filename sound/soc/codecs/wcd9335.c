/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/info.h>

#include "wcd9335_registers.h"
#include "wcd-clsh.h"
#include "wcd9335.h"
#include "wcd-slim.h"

#define WCD9335_RX_PORT_START_NUMBER  16

/* Number of input and output Slimbus port */
enum {
	WCD9335_RX0 = 0,
	WCD9335_RX1,
	WCD9335_RX2,
	WCD9335_RX3,
	WCD9335_RX4,
	WCD9335_RX5,
	WCD9335_RX6,
	WCD9335_RX7,
	WCD9335_RX8,
	WCD9335_RX9,
	WCD9335_RX10,
	WCD9335_RX11,
	WCD9335_RX12,
	WCD9335_RX_MAX,
};

/*
 * Rx path gain offsets
 */
enum {
	RX_GAIN_OFFSET_M1P5_DB,
	RX_GAIN_OFFSET_0_DB,
};

struct wcd9335_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

#define WCD9335_RATES_MASK (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			    SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)
/* Fractional Rates */
#define WCD9335_FRAC_RATES_MASK (SNDRV_PCM_RATE_44100)

#define WCD9335_MIX_RATES_MASK (SNDRV_PCM_RATE_48000 |\
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define wcd9335_FORMATS_S16_S24_LE (SNDRV_PCM_FMTBIT_S16_LE | \
				  SNDRV_PCM_FMTBIT_S24_LE)

#define wcd9335_FORMATS_S16_S24_S32_LE (SNDRV_PCM_FMTBIT_S16_LE | \
				  SNDRV_PCM_FMTBIT_S24_LE | \
				  SNDRV_PCM_FMTBIT_S32_LE)

#define wcd9335_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

/*
 * Timeout in milli seconds and it is the wait time for
 * slim channel removal interrupt to receive.
 */
#define WCD9335_SLIM_CLOSE_TIMEOUT 1000
#define WCD9335_SLIM_IRQ_OVERFLOW (1 << 0)
#define WCD9335_SLIM_IRQ_UNDERFLOW (1 << 1)
#define WCD9335_SLIM_IRQ_PORT_CLOSED (1 << 2)
#define wcd9335_MCLK_CLK_12P288MHZ 12288000
#define wcd9335_MCLK_CLK_9P6MHZ 9600000

#define WCD9335_SLIM_NUM_PORT_REG 3
#define WCD9335_SLIM_PGD_PORT_INT_TX_EN0 (WCD9335_SLIM_PGD_PORT_INT_EN0 + 2)
#define BYTE_BIT_MASK(nr) (1 << ((nr) % BITS_PER_BYTE))

#define SLIM_BW_CLK_GEAR_9 6200000
#define SLIM_BW_UNVOTE 0

#define wcd9335_DIG_CORE_REG_MIN  WCD9335_CDC_ANC0_CLK_RESET_CTL
#define wcd9335_DIG_CORE_REG_MAX  0xDFF

#define CALCULATE_VOUT_D(req_mv) (((req_mv - 650) * 10) / 25)

/* SVS Scaling enable/disable */
static int svs_scaling_enabled = 1;
/* SVS buck setting */
static int sido_buck_svs_voltage = SIDO_VOLTAGE_SVS_MV;

enum {
	VI_SENSE_1,
	VI_SENSE_2,
	AIF4_SWITCH_VALUE,
	AUDIO_NOMINAL,
	CPE_NOMINAL,
	HPH_PA_DELAY,
	SB_CLK_GEAR,
	ANC_MIC_AMIC1,
	ANC_MIC_AMIC2,
	ANC_MIC_AMIC3,
	ANC_MIC_AMIC4,
	ANC_MIC_AMIC5,
	ANC_MIC_AMIC6,
};

enum {
	INTn_1_MIX_INP_SEL_ZERO = 0,
	INTn_1_MIX_INP_SEL_DEC0,
	INTn_1_MIX_INP_SEL_DEC1,
	INTn_1_MIX_INP_SEL_IIR0,
	INTn_1_MIX_INP_SEL_IIR1,
	INTn_1_MIX_INP_SEL_RX0,
	INTn_1_MIX_INP_SEL_RX1,
	INTn_1_MIX_INP_SEL_RX2,
	INTn_1_MIX_INP_SEL_RX3,
	INTn_1_MIX_INP_SEL_RX4,
	INTn_1_MIX_INP_SEL_RX5,
	INTn_1_MIX_INP_SEL_RX6,
	INTn_1_MIX_INP_SEL_RX7,

};

#define IS_VALID_NATIVE_FIFO_PORT(inp) \
	((inp >= INTn_1_MIX_INP_SEL_RX0) && \
	 (inp <= INTn_1_MIX_INP_SEL_RX3))

enum {
	INTn_2_INP_SEL_ZERO = 0,
	INTn_2_INP_SEL_RX0,
	INTn_2_INP_SEL_RX1,
	INTn_2_INP_SEL_RX2,
	INTn_2_INP_SEL_RX3,
	INTn_2_INP_SEL_RX4,
	INTn_2_INP_SEL_RX5,
	INTn_2_INP_SEL_RX6,
	INTn_2_INP_SEL_RX7,
	INTn_2_INP_SEL_PROXIMITY,
};

enum {
	INTERP_EAR = 0,
	INTERP_HPHL,
	INTERP_HPHR,
	INTERP_LO1,
	INTERP_LO2,
	INTERP_LO3,
	INTERP_LO4,
	INTERP_SPKR1,
	INTERP_SPKR2,
};

struct interp_sample_rate {
	int sample_rate;
	int rate_val;
};

static struct interp_sample_rate int_prim_sample_rate_val[] = {
	{8000, 0x0},	/* 8K */
	{16000, 0x1},	/* 16K */
	{24000, -EINVAL},/* 24K */
	{32000, 0x3},	/* 32K */
	{48000, 0x4},	/* 48K */
	{96000, 0x5},	/* 96K */
	{192000, 0x6},	/* 192K */
	{384000, 0x7},	/* 384K */
	{44100, 0x8}, /* 44.1K */
};

static struct interp_sample_rate int_mix_sample_rate_val[] = {
	{48000, 0x4},	/* 48K */
	{96000, 0x5},	/* 96K */
	{192000, 0x6},	/* 192K */
};

static const struct wcd_slim_ch wcd9335_rx_chs[WCD9335_RX_MAX] = {
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER, 0),	 /* 16 */
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 1, 1),	 /* 17 */
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 2, 2),   /* 18 */
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 3, 3),   /* 19 */
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 4, 4),   /* 20 */
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 5, 5),   /* 21 */
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 6, 6),
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 7, 7),
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 8, 8),
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 9, 9),
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 10, 10),
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 11, 11),
	WCD_SLIM_CH(WCD9335_RX_PORT_START_NUMBER + 12, 12),
};

enum {
	SRC_IN_HPHL,
	SRC_IN_LO1,
	SRC_IN_HPHR,
	SRC_IN_LO2,
	SRC_IN_SPKRL,
	SRC_IN_LO3,
	SRC_IN_SPKRR,
	SRC_IN_LO4,
};

static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 1, 0);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);

static int wcd9335_config_compander(struct snd_soc_component *, int, int);
static int wcd9335_codec_vote_max_bw(struct snd_soc_component *comp,
				   bool vote);

static const struct wcd9335_reg_mask_val wcd9335_spkr_default[] = {
	{WCD9335_CDC_COMPANDER7_CTL3, 0x80, 0x80},
	{WCD9335_CDC_COMPANDER8_CTL3, 0x80, 0x80},
	{WCD9335_CDC_COMPANDER7_CTL7, 0x01, 0x01},
	{WCD9335_CDC_COMPANDER8_CTL7, 0x01, 0x01},
	{WCD9335_CDC_BOOST0_BOOST_CTL, 0x7C, 0x50},
	{WCD9335_CDC_BOOST1_BOOST_CTL, 0x7C, 0x50},
};

static const struct wcd9335_reg_mask_val wcd9335_spkr_mode1[] = {
	{WCD9335_CDC_COMPANDER7_CTL3, 0x80, 0x00},
	{WCD9335_CDC_COMPANDER8_CTL3, 0x80, 0x00},
	{WCD9335_CDC_COMPANDER7_CTL7, 0x01, 0x00},
	{WCD9335_CDC_COMPANDER8_CTL7, 0x01, 0x00},
	{WCD9335_CDC_BOOST0_BOOST_CTL, 0x7C, 0x44},
	{WCD9335_CDC_BOOST1_BOOST_CTL, 0x7C, 0x44},
};

static void wcd9335_enable_sido_buck(struct snd_soc_component *component)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	snd_soc_component_update_bits(component, WCD9335_ANA_RCO, 0x80, 0x80);
	snd_soc_component_update_bits(component, WCD9335_ANA_BUCK_CTL, 0x02, 0x02);
	/* 100us sleep needed after IREF settings */
	usleep_range(100, 110);
	snd_soc_component_update_bits(component, WCD9335_ANA_BUCK_CTL, 0x04, 0x04);
	/* 100us sleep needed after VREF settings */
	usleep_range(100, 110);
	wcd->sido_input_src = SIDO_SOURCE_RCO_BG;
}

static void wcd9335_cdc_sido_ccl_enable(struct wcd9335_priv *wcd, bool ccl_flag)
{
	struct snd_soc_component *component = wcd->component;

	if (!component)
		return;

	if (!WCD9335_IS_2_0(wcd->version)) {
		dev_dbg(component->dev, "%s: wcd version < 2p0, return\n",
			__func__);
		return;
	}
	if (ccl_flag) {
		if (++wcd->sido_ccl_cnt == 1)
			snd_soc_component_update_bits(component,
				WCD9335_SIDO_SIDO_CCL_10, 0xFF, 0x6E);
	} else {
		if (wcd->sido_ccl_cnt == 0) {
			dev_dbg(component->dev, "%s: sido_ccl already disabled\n",
				__func__);
			return;
		}
		if (--wcd->sido_ccl_cnt == 0)
			snd_soc_component_update_bits(component,
				WCD9335_SIDO_SIDO_CCL_10, 0xFF, 0x02);
	}
}

static bool wcd9335_cdc_is_svs_enabled(struct wcd9335_priv *wcd)
{
	if (WCD9335_IS_2_0(wcd->version) &&
		svs_scaling_enabled)
		return true;

	return false;
}

int wcd9335_enable_master_bias(struct wcd9335_priv *wcd)
{
	mutex_lock(&wcd->master_bias_lock);

	wcd->master_bias_users++;
	if (wcd->master_bias_users == 1) {
		regmap_update_bits(wcd->regmap, WCD9335_ANA_BIAS, 0x80, 0x80);
		regmap_update_bits(wcd->regmap, WCD9335_ANA_BIAS, 0x40, 0x40);
		/*
		 * 1ms delay is required after pre-charge is enabled
		 * as per HW requirement
		 */
		usleep_range(1000, 1100);
		regmap_update_bits(wcd->regmap, WCD9335_ANA_BIAS, 0x40, 0x00);
		regmap_update_bits(wcd->regmap, WCD9335_ANA_BIAS, 0x20, 0x00);
	}

	mutex_unlock(&wcd->master_bias_lock);
	return 0;
}

static int wcd9335_enable_mclk(struct wcd9335_priv *wcd)
{
	/* Enable mclk requires master bias to be enabled first */
	if (wcd->master_bias_users <= 0) {
		dev_err(wcd->dev, "Cannot turn on MCLK, BG is not enabled\n");
		return -EINVAL;
	}

	if (((wcd->clk_mclk_users == 0) &&
	     (wcd->clk_type == WCD_CLK_MCLK)) ||
	    ((wcd->clk_mclk_users > 0) &&
	    (wcd->clk_type != WCD_CLK_MCLK))) {
		pr_err("%s: Error enabling MCLK, clk_type: %d\n",
			__func__,
			wcd->clk_type);
		return -EINVAL;
	}

	if (++wcd->clk_mclk_users == 1) {

		regmap_update_bits(wcd->regmap,
				   WCD9335_ANA_CLK_TOP, 0x80, 0x80);
		regmap_update_bits(wcd->regmap,
				   WCD9335_ANA_CLK_TOP, 0x08, 0x00);
		regmap_update_bits(wcd->regmap,
				   WCD9335_ANA_CLK_TOP, 0x04, 0x04);
		regmap_update_bits(wcd->regmap,
				   WCD9335_CDC_CLK_RST_CTRL_FS_CNT_CONTROL,
				   0x01, 0x01);
		regmap_update_bits(wcd->regmap,
				   WCD9335_CDC_CLK_RST_CTRL_MCLK_CONTROL,
				   0x01, 0x01);
		/*
		 * 10us sleep is required after clock is enabled
		 * as per HW requirement
		 */
		usleep_range(10, 15);
	}

	wcd->clk_type = WCD_CLK_MCLK;

	return 0;
}

static int wcd9335_disable_mclk(struct wcd9335_priv *wcd)
{
	if (wcd->clk_mclk_users <= 0) {
		dev_err(wcd->dev, "No mclk users, cannot disable mclk\n");
		return -EINVAL;
	}

	if (--wcd->clk_mclk_users == 0) {
		if (wcd->clk_rco_users > 0) {
			/* MCLK to RCO switch */
			regmap_update_bits(wcd->regmap,
					WCD9335_ANA_CLK_TOP,
					0x08, 0x08);
			wcd->clk_type = WCD_CLK_RCO;
		} else {
			regmap_update_bits(wcd->regmap,
					WCD9335_ANA_CLK_TOP,
					0x04, 0x00);
			wcd->clk_type = WCD_CLK_OFF;
		}

		regmap_update_bits(wcd->regmap, WCD9335_ANA_CLK_TOP,
						 0x80, 0x00);
	}

	return 0;
}

int wcd9335_disable_master_bias(struct wcd9335_priv *wcd)
{
	mutex_lock(&wcd->master_bias_lock);
	if (wcd->master_bias_users <= 0) {
		mutex_unlock(&wcd->master_bias_lock);
		return -EINVAL;
	}

	wcd->master_bias_users--;
	if (wcd->master_bias_users == 0) {
		regmap_update_bits(wcd->regmap, WCD9335_ANA_BIAS,
						 0x80, 0x00);
		regmap_update_bits(wcd->regmap, WCD9335_ANA_BIAS,
						 0x20, 0x00);
	}
	mutex_unlock(&wcd->master_bias_lock);
	return 0;
}

static int wcd9335_cdc_req_mclk_enable(struct wcd9335_priv *wcd,
				     bool enable)
{
	int ret = 0;

	if (enable) {
		wcd9335_cdc_sido_ccl_enable(wcd, true);
		ret = clk_prepare_enable(wcd->ext_clk);
		if (ret) {
			dev_err(wcd->dev, "%s: ext clk enable failed\n",
				__func__);
			goto err;
		}
		/* get BG */
		wcd9335_enable_master_bias(wcd);
		/* get MCLK */
		wcd9335_enable_mclk(wcd);

	} else {
		/* put MCLK */
		wcd9335_disable_mclk(wcd);
		/* put BG */
		wcd9335_disable_master_bias(wcd);
		clk_disable_unprepare(wcd->ext_clk);
		wcd9335_cdc_sido_ccl_enable(wcd, false);
	}
err:
	return ret;
}

static int wcd9335_cdc_check_sido_value(enum wcd9335_sido_voltage req_mv)
{
	if ((req_mv != SIDO_VOLTAGE_SVS_MV) &&
		(req_mv != SIDO_VOLTAGE_NOMINAL_MV))
		return -EINVAL;

	return 0;
}

static void wcd9335_codec_apply_sido_voltage(
				struct wcd9335_priv *wcd,
				enum wcd9335_sido_voltage req_mv)
{
	u32 vout_d_val;
	struct snd_soc_component *component = wcd->component;
	int ret;

	if (!component)
		return;

	if (!wcd9335_cdc_is_svs_enabled(wcd))
		return;

	if ((sido_buck_svs_voltage != SIDO_VOLTAGE_SVS_MV) &&
		(sido_buck_svs_voltage != SIDO_VOLTAGE_NOMINAL_MV))
		sido_buck_svs_voltage = SIDO_VOLTAGE_SVS_MV;

	ret = wcd9335_cdc_check_sido_value(req_mv);
	if (ret < 0) {
		dev_err(component->dev, "%s: requested mv=%d not in range\n",
			__func__, req_mv);
		return;
	}
	if (req_mv == wcd->sido_voltage) {
		dev_err(component->dev, "%s: Already at requested mv=%d\n",
			__func__, req_mv);
		return;
	}
	if (req_mv == sido_buck_svs_voltage) {
		if (test_bit(AUDIO_NOMINAL, &wcd->status_mask) ||
			test_bit(CPE_NOMINAL, &wcd->status_mask)) {
			dev_err(component->dev,
				"%s: nominal client running, status_mask=%lu\n",
				__func__, wcd->status_mask);
			return;
		}
	}
	/* compute the vout_d step value */
	vout_d_val = CALCULATE_VOUT_D(req_mv);
	snd_soc_component_write(component, WCD9335_ANA_BUCK_VOUT_D, vout_d_val & 0xFF);
	snd_soc_component_update_bits(component, WCD9335_ANA_BUCK_CTL, 0x80, 0x80);

	/* 1 msec sleep required after SIDO Vout_D voltage change */
	usleep_range(1000, 1100);
	wcd->sido_voltage = req_mv;
	snd_soc_component_update_bits(component, WCD9335_ANA_BUCK_CTL,
				0x80, 0x00);
}

static int wcd9335_codec_update_sido_voltage(
				struct wcd9335_priv *wcd,
				enum wcd9335_sido_voltage req_mv)
{
	int ret = 0;

	if (!wcd9335_cdc_is_svs_enabled(wcd))
		return ret;

	mutex_lock(&wcd->sido_lock);
	/* enable mclk before setting SIDO voltage */
	ret = wcd9335_cdc_req_mclk_enable(wcd, true);
	if (ret) {
		dev_err(wcd->dev, "%s: ext clk enable failed\n",
			__func__);
		goto err;
	}
	wcd9335_codec_apply_sido_voltage(wcd, req_mv);
	wcd9335_cdc_req_mclk_enable(wcd, false);

err:
	mutex_unlock(&wcd->sido_lock);
	return ret;
}

static int slim_rx_mux_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{

        struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	ucontrol->value.enumerated.item[0] = wcd->rx_port_value;

	return 0;
}

static const char *const slim_rx_mux_text[] = {
	"ZERO", "AIF1_PB", "AIF2_PB", "AIF3_PB", "AIF4_PB", "AIF_MIX1_PB"
};

static int wcd_slim_rx_vport_validation(u32 port_id,
				struct list_head *codec_dai_list)
{
	struct wcd_slim_ch *ch;
	int ret = 0;

	list_for_each_entry(ch,
		codec_dai_list, list) {
		if (ch->port == port_id) {
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}

static int slim_rx_mux_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_soc_dapm_kcontrol_widget(kcontrol);
        struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);

	struct snd_soc_component *component = snd_soc_dapm_kcontrol_component(kcontrol);//to_component(dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;
	u32 port_id = widget->shift;

	wcd->rx_port_value = ucontrol->value.enumerated.item[0];

	if (wcd->intf_type != WCD_INTERFACE_TYPE_SLIMBUS) {
		if (wcd->rx_port_value > 2) {
			dev_err(component->dev, "%s: invalid AIF for I2C mode\n",
				__func__);
			goto err;
		}
	}
	/* value need to match the Virtual port and AIF number */
	switch (wcd->rx_port_value) {
	case 0:

		list_del_init(&wcd->slim_data->rx_chs[port_id].list);
		break;
	case 1:
		if (wcd_slim_rx_vport_validation(port_id +
			WCD9335_RX_PORT_START_NUMBER,
			&wcd->dai[AIF1_PB].wcd_slim_ch_list)) {
			goto rtn;
		}
		list_add_tail(&wcd->slim_data->rx_chs[port_id].list,
			      &wcd->dai[AIF1_PB].wcd_slim_ch_list);
		break;
	case 2:
		if (wcd_slim_rx_vport_validation(port_id +
			WCD9335_RX_PORT_START_NUMBER,
			&wcd->dai[AIF2_PB].wcd_slim_ch_list)) {
			goto rtn;
		}
		list_add_tail(&wcd->slim_data->rx_chs[port_id].list,
			      &wcd->dai[AIF2_PB].wcd_slim_ch_list);
		break;
	case 3:
		if (wcd_slim_rx_vport_validation(port_id +
			WCD9335_RX_PORT_START_NUMBER,
			&wcd->dai[AIF3_PB].wcd_slim_ch_list)) {
			goto rtn;
		}
		list_add_tail(&wcd->slim_data->rx_chs[port_id].list,
			      &wcd->dai[AIF3_PB].wcd_slim_ch_list);
		break;
	case 4:
		if (wcd_slim_rx_vport_validation(port_id +
			WCD9335_RX_PORT_START_NUMBER,
			&wcd->dai[AIF4_PB].wcd_slim_ch_list)) {
			goto rtn;
		}
		list_add_tail(&wcd->slim_data->rx_chs[port_id].list,
			      &wcd->dai[AIF4_PB].wcd_slim_ch_list);
		break;
	case 5:
		if (wcd_slim_rx_vport_validation(port_id +
			WCD9335_RX_PORT_START_NUMBER,
			&wcd->dai[AIF_MIX1_PB].wcd_slim_ch_list)) {
			goto rtn;
		}
		list_add_tail(&wcd->slim_data->rx_chs[port_id].list,
			      &wcd->dai[AIF_MIX1_PB].wcd_slim_ch_list);
		break;
	default:
		dev_err(wcd->dev, "Unknown AIF %d\n", wcd->rx_port_value);
		goto err;
	}
rtn:
	snd_soc_dapm_mux_update_power(widget->dapm, kcontrol,
					wcd->rx_port_value, e, update);

	return 0;
err:
	return -EINVAL;
}

static const struct soc_enum slim_rx_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slim_rx_mux_text), slim_rx_mux_text);

static const struct snd_kcontrol_new slim_rx_mux[WCD9335_RX_MAX] = {
	SOC_DAPM_ENUM_EXT("SLIM RX0 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX1 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX2 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX3 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX4 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX5 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX6 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX7 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
};

static const struct snd_kcontrol_new rx_int1_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("HPHL Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("HPHL Native Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new rx_int2_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("HPHR Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("HPHR Native Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new rx_int3_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("LO1 Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("LO1 Native Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new rx_int4_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("LO2 Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("LO2 Native Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new rx_int5_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("LO3 Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new rx_int6_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("LO4 Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new rx_int7_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("SPKRL Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new rx_int8_spline_mix_switch[] = {
	SOC_DAPM_SINGLE("SPKRR Switch", SND_SOC_NOPM, 0, 1, 0)
};

static void wcd9335_codec_enable_int_port(struct wcd_slim_codec_dai_data *dai,
					struct snd_soc_component *component)
{
	struct wcd_slim_ch *ch;
	int port_num = 0;
	unsigned short reg = 0;
	unsigned int val = 0;
	struct wcd9335_priv *wcd;

	wcd = dev_get_drvdata(component->dev);
	list_for_each_entry(ch, &dai->wcd_slim_ch_list, list) {
		if (ch->port >= WCD9335_RX_PORT_START_NUMBER) {
			port_num = ch->port - WCD9335_RX_PORT_START_NUMBER;
			reg = WCD9335_SLIM_PGD_PORT_INT_EN0 + (port_num / 8);
			regmap_read(wcd->if_regmap,
				reg, &val);

			if (!(val & BYTE_BIT_MASK(port_num))) {
				val |= BYTE_BIT_MASK(port_num);
				regmap_write(wcd->if_regmap, reg, val);
				regmap_read(
					wcd->if_regmap, reg, &val);
			}
		} else {
			port_num = ch->port;
			reg = WCD9335_SLIM_PGD_PORT_INT_TX_EN0 + (port_num / 8);
			regmap_read(wcd->if_regmap,
				reg, &val);
			if (!(val & BYTE_BIT_MASK(port_num))) {
				val |= BYTE_BIT_MASK(port_num);
				regmap_write(wcd->if_regmap,
					reg, val);
				regmap_read(
					wcd->if_regmap, reg, &val);
			}
		}
	}
}

int wcd_slim_get_slave_port(unsigned int ch_num)
{
	int ret = 0;

	ret = (ch_num - BASE_CH_NUM);
	if (ret < 0) {
		pr_err("%s: Error:- Invalid slave port found = %d\n",
			__func__, ret);
		return -EINVAL;
	}
	return ret;
}

static int wcd9335_codec_enable_slim_chmask(struct wcd_slim_codec_dai_data *dai,
					  bool up)
{
	int ret = 0;
	struct wcd_slim_ch *ch;

	if (up) {
		list_for_each_entry(ch, &dai->wcd_slim_ch_list, list) {
			ret = wcd_slim_get_slave_port(ch->ch_num);
			if (ret < 0) {
				pr_err("%s: Invalid slave port ID: %d\n",
				       __func__, ret);
				ret = -EINVAL;
			} else {
				set_bit(ret, &dai->ch_mask);
			}
		}
	} else {
		ret = wait_event_timeout(dai->dai_wait, (dai->ch_mask == 0),
					 msecs_to_jiffies(
						WCD9335_SLIM_CLOSE_TIMEOUT));
		if (!ret) {
			pr_err("%s: Slim close tx/rx wait timeout, ch_mask:0x%lx\n",
				__func__, dai->ch_mask);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
		}
	}
	return ret;
}

static int wcd_slim_stream_enable(struct wcd_slim_data *wcd,
			struct wcd_slim_codec_dai_data *dai_data);
static int wcd_slim_stream_prepare(struct wcd9335_priv *wcd,
	struct wcd_slim_codec_dai_data *dai_data);

static int wcd9335_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int ret = 0;
	struct wcd_slim_codec_dai_data *dai;

	/* Execute the callback only if interface type is slimbus */
	if (wcd->intf_type != WCD_INTERFACE_TYPE_SLIMBUS)
		return 0;

	dai = &wcd->dai[w->shift];

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		wcd9335_codec_enable_int_port(dai, component);
		(void) wcd9335_codec_enable_slim_chmask(dai, true);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd9335_codec_vote_max_bw(component, true);

		wcd9335_codec_enable_slim_chmask(dai, false);
		wcd9335_codec_vote_max_bw(component, false);
		slim_stream_unprepare(dai->sruntime);
		slim_stream_disable(dai->sruntime);

		break;
	}
	return ret;
}

static int wcd9335_get_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	int comp = ((struct soc_mixer_control *)
		    kcontrol->private_value)->shift;
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	ucontrol->value.integer.value[0] = wcd->comp_enabled[comp];
	return 0;
}

static int wcd9335_set_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int comp = ((struct soc_mixer_control *)  kcontrol->private_value)->shift;
	int value = ucontrol->value.integer.value[0];

	wcd->comp_enabled[comp] = value;

	/* Any specific register configuration for compander */
	switch (comp) {
	case COMPANDER_1:
		/* Set Gain Source Select based on compander enable/disable */
		snd_soc_component_update_bits(component, WCD9335_HPH_L_EN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_2:
		snd_soc_component_update_bits(component, WCD9335_HPH_R_EN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_3:
		break;
	case COMPANDER_4:
		break;
	case COMPANDER_5:
		snd_soc_component_update_bits(component, WCD9335_SE_LO_LO3_GAIN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_6:
		snd_soc_component_update_bits(component, WCD9335_SE_LO_LO4_GAIN, 0x20,
				(value ? 0x00:0x20));
		break;
	case COMPANDER_7:
		break;
	case COMPANDER_8:
		break;
	default:
		/*
		 * if compander is not enabled for any interpolator,
		 * it does not cause any audio failure, so do not
		 * return error in this case, but just print a log
		 */
		dev_warn(component->dev, "%s: unknown compander: %d\n",
			__func__, comp);
	};
	return 0;
}

static void wcd9335_codec_init_flyback(struct snd_soc_component *component)
{
	snd_soc_component_update_bits(component, WCD9335_HPH_L_EN, 0xC0, 0x00);
	snd_soc_component_update_bits(component, WCD9335_HPH_R_EN, 0xC0, 0x00);
	snd_soc_component_update_bits(component, WCD9335_RX_BIAS_FLYB_BUFF, 0x0F, 0x00);
	snd_soc_component_update_bits(component, WCD9335_RX_BIAS_FLYB_BUFF, 0xF0, 0x00);
}

static int wcd9335_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd->rx_bias_count++;
		if (wcd->rx_bias_count == 1) {
			if (WCD9335_IS_2_0(wcd->version))
				wcd9335_codec_init_flyback(component);
			snd_soc_component_update_bits(component, WCD9335_ANA_RX_SUPPLIES,
					    0x01, 0x01);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd->rx_bias_count--;
		if (!wcd->rx_bias_count)
			snd_soc_component_update_bits(component, WCD9335_ANA_RX_SUPPLIES,
					    0x01, 0x00);
		break;
	};

	return 0;
}

static void wcd9335_codec_hph_post_pa_config(struct wcd9335_priv *wcd,
					   int mode, int event)
{
	u8 scale_val = 0;

	if (!WCD9335_IS_2_0(wcd->version))
		return;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		switch (mode) {
		case CLS_H_HIFI:
			scale_val = 0x3;
			break;
		case CLS_H_LOHIFI:
			scale_val = 0x1;
			break;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		scale_val = 0x6;
		break;
	}

	if (scale_val)
		snd_soc_component_update_bits(wcd->component, WCD9335_HPH_PA_CTL1, 0x0E,
				    scale_val << 1);
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (wcd->comp_enabled[COMPANDER_1] ||
		    wcd->comp_enabled[COMPANDER_2]) {
			/* GAIN Source Selection */
			snd_soc_component_update_bits(wcd->component, WCD9335_HPH_L_EN,
					    0x20, 0x00);
			snd_soc_component_update_bits(wcd->component, WCD9335_HPH_R_EN,
					    0x20, 0x00);
			snd_soc_component_update_bits(wcd->component, WCD9335_HPH_AUTO_CHOP,
					    0x20, 0x20);
		}
		snd_soc_component_update_bits(wcd->component, WCD9335_HPH_L_EN, 0x1F,
				    wcd->hph_l_gain);
		snd_soc_component_update_bits(wcd->component, WCD9335_HPH_R_EN, 0x1F,
				    wcd->hph_r_gain);
	}

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		snd_soc_component_update_bits(wcd->component, WCD9335_HPH_AUTO_CHOP, 0x20,
				    0x00);
	}
}

static int wcd9335_codec_enable_hphr_pa(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *kcontrol,
				      int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int hph_mode = wcd->hph_mode;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		set_bit(HPH_PA_DELAY, &wcd->status_mask);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * 7ms sleep is required after PA is enabled as per
		 * HW requirement
		 */
		if (test_bit(HPH_PA_DELAY, &wcd->status_mask)) {
			usleep_range(7000, 7100);
			clear_bit(HPH_PA_DELAY, &wcd->status_mask);
		}
		wcd9335_codec_hph_post_pa_config(wcd, hph_mode, event);
		snd_soc_component_update_bits(component, WCD9335_CDC_RX2_RX_PATH_CTL,
				    0x10, 0x00);
		/* Remove mix path mute if it is enabled */
		if ((snd_soc_component_read32(component, WCD9335_CDC_RX2_RX_PATH_MIX_CTL)) &
				  0x10)
			snd_soc_component_update_bits(component,
					    WCD9335_CDC_RX2_RX_PATH_MIX_CTL,
					    0x10, 0x00);

		break;

	case SND_SOC_DAPM_PRE_PMD:
		wcd9335_codec_hph_post_pa_config(wcd, hph_mode, event);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* 5ms sleep is required after PA is disabled as per
		 * HW requirement
		 */
		usleep_range(5000, 5500);
		break;
	};

	return ret;
}

static int wcd9335_codec_enable_hphl_pa(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *kcontrol,
				      int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int hph_mode = wcd->hph_mode;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		set_bit(HPH_PA_DELAY, &wcd->status_mask);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * 7ms sleep is required after PA is enabled as per
		 * HW requirement
		 */
		if (test_bit(HPH_PA_DELAY, &wcd->status_mask)) {
			usleep_range(7000, 7100);
			clear_bit(HPH_PA_DELAY, &wcd->status_mask);
		}

		wcd9335_codec_hph_post_pa_config(wcd, hph_mode, event);
		snd_soc_component_update_bits(component, WCD9335_CDC_RX1_RX_PATH_CTL,
				    0x10, 0x00);
		/* Remove mix path mute if it is enabled */
		if ((snd_soc_component_read32(component, WCD9335_CDC_RX1_RX_PATH_MIX_CTL)) &
				  0x10)
			snd_soc_component_update_bits(component,
					    WCD9335_CDC_RX1_RX_PATH_MIX_CTL,
					    0x10, 0x00);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		wcd9335_codec_hph_post_pa_config(wcd, hph_mode, event);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* 5ms sleep is required after PA is disabled as per
		 * HW requirement
		 */
		usleep_range(5000, 5500);
		break;
	};

	return ret;
}

static int wcd9335_codec_enable_lineout_pa(struct snd_soc_dapm_widget *w,
					 struct snd_kcontrol *kcontrol,
					 int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	u16 lineout_vol_reg = 0, lineout_mix_vol_reg = 0;
	int ret = 0;

	if (w->reg == WCD9335_ANA_LO_1_2) {
		if (w->shift == 7) {
			lineout_vol_reg = WCD9335_CDC_RX3_RX_PATH_CTL;
			lineout_mix_vol_reg = WCD9335_CDC_RX3_RX_PATH_MIX_CTL;
		} else if (w->shift == 6) {
			lineout_vol_reg = WCD9335_CDC_RX4_RX_PATH_CTL;
			lineout_mix_vol_reg = WCD9335_CDC_RX4_RX_PATH_MIX_CTL;
		}
	} else if (w->reg == WCD9335_ANA_LO_3_4) {
		if (w->shift == 7) {
			lineout_vol_reg = WCD9335_CDC_RX5_RX_PATH_CTL;
			lineout_mix_vol_reg = WCD9335_CDC_RX5_RX_PATH_MIX_CTL;
		} else if (w->shift == 6) {
			lineout_vol_reg = WCD9335_CDC_RX6_RX_PATH_CTL;
			lineout_mix_vol_reg = WCD9335_CDC_RX6_RX_PATH_MIX_CTL;
		}
	} else {
		dev_err(component->dev, "%s: Error enabling lineout PA\n",
			__func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* 5ms sleep is required after PA is enabled as per
		 * HW requirement
		 */
		usleep_range(5000, 5500);
		snd_soc_component_update_bits(component, lineout_vol_reg,
				    0x10, 0x00);
		/* Remove mix path mute if it is enabled */
		if ((snd_soc_component_read32(component, lineout_mix_vol_reg)) & 0x10)
			snd_soc_component_update_bits(component,
					    lineout_mix_vol_reg,
					    0x10, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* 5ms sleep is required after PA is disabled as per
		 * HW requirement
		 */
		usleep_range(5000, 5500);
		break;
	};

	return ret;
}

static int wcd9335_codec_enable_ear_pa(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* 5ms sleep is required after PA is enabled as per
		 * HW requirement
		 */
		usleep_range(5000, 5500);
		snd_soc_component_update_bits(component, WCD9335_CDC_RX0_RX_PATH_CTL,
				    0x10, 0x00);
		/* Remove mix path mute if it is enabled */
		if ((snd_soc_component_read32(component, WCD9335_CDC_RX0_RX_PATH_MIX_CTL)) &
		     0x10)
			snd_soc_component_update_bits(component,
					    WCD9335_CDC_RX0_RX_PATH_MIX_CTL,
					    0x10, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* 5ms sleep is required after PA is disabled as per
		 * HW requirement
		 */
		usleep_range(5000, 5500);

		break;
	};

	return ret;
}

static void wcd9335_codec_hph_mode_gain_opt(struct snd_soc_component *component,
					  u8 gain)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	u8 hph_l_en, hph_r_en;
	u8 l_val, r_val;
	u8 hph_pa_status;
	bool is_hphl_pa, is_hphr_pa;

	hph_pa_status = snd_soc_component_read32(component, WCD9335_ANA_HPH);
	is_hphl_pa = hph_pa_status >> 7;
	is_hphr_pa = (hph_pa_status & 0x40) >> 6;

	hph_l_en = snd_soc_component_read32(component, WCD9335_HPH_L_EN);
	hph_r_en = snd_soc_component_read32(component, WCD9335_HPH_R_EN);

	l_val = (hph_l_en & 0xC0) | 0x20 | gain;
	r_val = (hph_r_en & 0xC0) | 0x20 | gain;

	/*
	 * Set HPH_L & HPH_R gain source selection to REGISTER
	 * for better click and pop only if corresponding PAs are
	 * not enabled. Also cache the values of the HPHL/R
	 * PA gains to be applied after PAs are enabled
	 */
	if ((l_val != hph_l_en) && !is_hphl_pa) {
		snd_soc_component_write(component, WCD9335_HPH_L_EN, l_val);
		wcd->hph_l_gain = hph_l_en & 0x1F;
	}

	if ((r_val != hph_r_en) && !is_hphr_pa) {
		snd_soc_component_write(component, WCD9335_HPH_R_EN, r_val);
		wcd->hph_r_gain = hph_r_en & 0x1F;
	}
}

static void wcd9335_codec_hph_lohifi_config(struct snd_soc_component *component,
					  int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_component_update_bits(component, WCD9335_RX_BIAS_HPH_PA, 0x0F, 0x06);
		snd_soc_component_update_bits(component, WCD9335_RX_BIAS_HPH_RDACBUFF_CNP2,
				    0xF0, 0x40);
		snd_soc_component_update_bits(component, WCD9335_HPH_CNP_WG_CTL, 0x07, 0x03);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x08, 0x08);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL1, 0x0E, 0x0C);
		wcd9335_codec_hph_mode_gain_opt(component, 0x11);
	}

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x08, 0x00);
		snd_soc_component_update_bits(component, WCD9335_HPH_CNP_WG_CTL, 0x07, 0x02);
		snd_soc_component_write(component, WCD9335_RX_BIAS_HPH_RDACBUFF_CNP2, 0x8A);
		snd_soc_component_update_bits(component, WCD9335_RX_BIAS_HPH_PA, 0x0F, 0x0A);
	}
}

static void wcd9335_codec_hph_lp_config(struct snd_soc_component *component,
				      int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL1, 0x0E, 0x0C);
		wcd9335_codec_hph_mode_gain_opt(component, 0x10);
		snd_soc_component_update_bits(component, WCD9335_HPH_CNP_WG_CTL, 0x07, 0x03);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x08, 0x08);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x04, 0x04);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x20, 0x20);
		snd_soc_component_update_bits(component, WCD9335_HPH_RDAC_LDO_CTL, 0x07,
				    0x01);
		snd_soc_component_update_bits(component, WCD9335_HPH_RDAC_LDO_CTL, 0x70,
				    0x10);
		snd_soc_component_update_bits(component, WCD9335_RX_BIAS_HPH_RDAC_LDO,
				    0x0F, 0x01);
		snd_soc_component_update_bits(component, WCD9335_RX_BIAS_HPH_RDAC_LDO,
				    0xF0, 0x10);
	}

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		snd_soc_component_write(component, WCD9335_RX_BIAS_HPH_RDAC_LDO, 0x88);
		snd_soc_component_write(component, WCD9335_HPH_RDAC_LDO_CTL, 0x33);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x20, 0x00);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x04, 0x00);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x08, 0x00);
		snd_soc_component_update_bits(component, WCD9335_HPH_CNP_WG_CTL, 0x07, 0x02);
		snd_soc_component_update_bits(component, WCD9335_HPH_R_EN, 0xC0, 0x80);
		snd_soc_component_update_bits(component, WCD9335_HPH_L_EN, 0xC0, 0x80);
	}
}

static void wcd9335_codec_hph_hifi_config(struct snd_soc_component *component,
					int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_component_update_bits(component, WCD9335_HPH_CNP_WG_CTL, 0x07, 0x03);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x08, 0x08);
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL1, 0x0E, 0x0C);
		wcd9335_codec_hph_mode_gain_opt(component, 0x11);
	}

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		snd_soc_component_update_bits(component, WCD9335_HPH_PA_CTL2, 0x08, 0x00);
		snd_soc_component_update_bits(component, WCD9335_HPH_CNP_WG_CTL, 0x07, 0x02);
	}
}

static void wcd9335_codec_hph_mode_config(struct snd_soc_component *component,
					int event, int mode)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	if (!WCD9335_IS_2_0(wcd->version))
		return;

	switch (mode) {
	case CLS_H_LP:
		wcd9335_codec_hph_lp_config(component, event);
		break;
	case CLS_H_LOHIFI:
		wcd9335_codec_hph_lohifi_config(component, event);
		break;
	case CLS_H_HIFI:
		wcd9335_codec_hph_hifi_config(component, event);
		break;
	}
}

static int wcd9335_codec_hphr_dac_event(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *kcontrol,
				      int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int hph_mode = wcd->hph_mode;
	u8 dem_inp;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		/* Read DEM INP Select */
		dem_inp = snd_soc_component_read32(component, WCD9335_CDC_RX2_RX_PATH_SEC0) &
			  0x03;
		if (((hph_mode == CLS_H_HIFI) || (hph_mode == CLS_H_LOHIFI) ||
		     (hph_mode == CLS_H_LP)) && (dem_inp != 0x01)) {
			dev_err(component->dev, "%s: DEM Input not set correctly, hph_mode: %d\n",
					__func__, hph_mode);
			return -EINVAL;
		}

		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_HPHR,
			     ((hph_mode == CLS_H_LOHIFI) ?
			       CLS_H_HIFI : hph_mode));

		wcd9335_codec_hph_mode_config(component, event, hph_mode);

		break;
	case SND_SOC_DAPM_POST_PMU:
		/* 1000us required as per HW requirement */
		usleep_range(1000, 1100);
		if ((hph_mode == CLS_H_LP) &&
		   (WCD9335_IS_1_1(wcd->version))) {
			snd_soc_component_update_bits(component, WCD9335_HPH_L_DAC_CTL,
					    0x03, 0x03);
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if ((hph_mode == CLS_H_LP) &&
		   (WCD9335_IS_1_1(wcd->version))) {
			snd_soc_component_update_bits(component, WCD9335_HPH_L_DAC_CTL,
					    0x03, 0x00);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* 1000us required as per HW requirement */
		usleep_range(1000, 1100);

		if (!(wcd_clsh_get_clsh_state(&wcd->clsh_d) &
		     WCD_CLSH_STATE_HPHL))
			wcd9335_codec_hph_mode_config(component, event, hph_mode);

		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_POST_PA,
			     WCD_CLSH_STATE_HPHR,
			     ((hph_mode == CLS_H_LOHIFI) ?
			       CLS_H_HIFI : hph_mode));
		break;
	};

	return ret;
}

static int wcd9335_codec_hphl_dac_event(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *kcontrol,
				      int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int hph_mode = wcd->hph_mode;
	u8 dem_inp;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		/* Read DEM INP Select */
		dem_inp = snd_soc_component_read32(component, WCD9335_CDC_RX1_RX_PATH_SEC0) &
			  0x03;
		if (((hph_mode == CLS_H_HIFI) || (hph_mode == CLS_H_LOHIFI) ||
		     (hph_mode == CLS_H_LP)) && (dem_inp != 0x01)) {
			dev_err(component->dev, "%s: DEM Input not set correctly, hph_mode: %d\n",
					__func__, hph_mode);
			return -EINVAL;
		}
		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_HPHL,
			     ((hph_mode == CLS_H_LOHIFI) ?
			       CLS_H_HIFI : hph_mode));

		wcd9335_codec_hph_mode_config(component, event, hph_mode);

		break;
	case SND_SOC_DAPM_POST_PMU:
		/* 1000us required as per HW requirement */
		usleep_range(1000, 1100);
		if ((hph_mode == CLS_H_LP) &&
		   (WCD9335_IS_1_1(wcd->version))) {
			snd_soc_component_update_bits(component, WCD9335_HPH_L_DAC_CTL,
					    0x03, 0x03);
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if ((hph_mode == CLS_H_LP) &&
		   (WCD9335_IS_1_1(wcd->version))) {
			snd_soc_component_update_bits(component, WCD9335_HPH_L_DAC_CTL,
					    0x03, 0x00);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* 1000us required as per HW requirement */
		usleep_range(1000, 1100);

		if (!(wcd_clsh_get_clsh_state(&wcd->clsh_d) &
		     WCD_CLSH_STATE_HPHR))
			wcd9335_codec_hph_mode_config(component, event, hph_mode);
		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_POST_PA,
			     WCD_CLSH_STATE_HPHL,
			     ((hph_mode == CLS_H_LOHIFI) ?
			       CLS_H_HIFI : hph_mode));
		break;
	};

	return ret;
}

static int wcd9335_codec_lineout_dac_event(struct snd_soc_dapm_widget *w,
					 struct snd_kcontrol *kcontrol,
					 int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_LO,
			     CLS_AB);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_POST_PA,
			     WCD_CLSH_STATE_LO,
			     CLS_AB);
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget wcd9335_dapm_i2s_widgets[] = {
	SND_SOC_DAPM_SUPPLY("RX_I2S_CTL", WCD9335_DATA_HUB_DATA_HUB_RX_I2S_CTL,
	0, 0, NULL, 0),
};

static int wcd9335_codec_ear_dac_event(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_EAR,
			     CLS_H_NORMAL);

		break;
	case SND_SOC_DAPM_POST_PMU:
		break;
	case SND_SOC_DAPM_PRE_PMD:
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd_clsh_fsm(component, &wcd->clsh_d,
			     WCD_CLSH_EVENT_POST_PA,
			     WCD_CLSH_STATE_EAR,
			     CLS_H_NORMAL);
		break;
	};

	return ret;
}

static u16 wcd9335_interp_get_primary_reg(u16 reg, u16 *ind)
{
	u16 prim_int_reg;

	switch (reg) {
	case WCD9335_CDC_RX0_RX_PATH_CTL:
	case WCD9335_CDC_RX0_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX0_RX_PATH_CTL;
		*ind = 0;
		break;
	case WCD9335_CDC_RX1_RX_PATH_CTL:
	case WCD9335_CDC_RX1_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX1_RX_PATH_CTL;
		*ind = 1;
		break;
	case WCD9335_CDC_RX2_RX_PATH_CTL:
	case WCD9335_CDC_RX2_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX2_RX_PATH_CTL;
		*ind = 2;
		break;
	case WCD9335_CDC_RX3_RX_PATH_CTL:
	case WCD9335_CDC_RX3_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX3_RX_PATH_CTL;
		*ind = 3;
		break;
	case WCD9335_CDC_RX4_RX_PATH_CTL:
	case WCD9335_CDC_RX4_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX4_RX_PATH_CTL;
		*ind = 4;
		break;
	case WCD9335_CDC_RX5_RX_PATH_CTL:
	case WCD9335_CDC_RX5_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX5_RX_PATH_CTL;
		*ind = 5;
		break;
	case WCD9335_CDC_RX6_RX_PATH_CTL:
	case WCD9335_CDC_RX6_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX6_RX_PATH_CTL;
		*ind = 6;
		break;
	case WCD9335_CDC_RX7_RX_PATH_CTL:
	case WCD9335_CDC_RX7_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX7_RX_PATH_CTL;
		*ind = 7;
		break;
	case WCD9335_CDC_RX8_RX_PATH_CTL:
	case WCD9335_CDC_RX8_RX_PATH_MIX_CTL:
		prim_int_reg = WCD9335_CDC_RX8_RX_PATH_CTL;
		*ind = 8;
		break;
	};

	return prim_int_reg;
}

static void wcd9335_codec_hd2_control(struct snd_soc_component *component,
				    u16 prim_int_reg, int event)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	u16 hd2_scale_reg;
	u16 hd2_enable_reg = 0;

	if (!WCD9335_IS_2_0(wcd->version))
		return;

	if (prim_int_reg == WCD9335_CDC_RX1_RX_PATH_CTL) {
		hd2_scale_reg = WCD9335_CDC_RX1_RX_PATH_SEC3;
		hd2_enable_reg = WCD9335_CDC_RX1_RX_PATH_CFG0;
	}
	if (prim_int_reg == WCD9335_CDC_RX2_RX_PATH_CTL) {
		hd2_scale_reg = WCD9335_CDC_RX2_RX_PATH_SEC3;
		hd2_enable_reg = WCD9335_CDC_RX2_RX_PATH_CFG0;
	}

	if (hd2_enable_reg && SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_component_update_bits(component, hd2_scale_reg, 0x3C, 0x10);
		snd_soc_component_update_bits(component, hd2_scale_reg, 0x03, 0x01);
		snd_soc_component_update_bits(component, hd2_enable_reg, 0x04, 0x04);
	}

	if (hd2_enable_reg && SND_SOC_DAPM_EVENT_OFF(event)) {
		snd_soc_component_update_bits(component, hd2_enable_reg, 0x04, 0x00);
		snd_soc_component_update_bits(component, hd2_scale_reg, 0x03, 0x00);
		snd_soc_component_update_bits(component, hd2_scale_reg, 0x3C, 0x00);
	}
}

static int wcd9335_codec_enable_prim_interpolator(
				struct snd_soc_component *component,
				u16 reg, int event)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	u16 prim_int_reg, ind = 0;

	prim_int_reg = wcd9335_interp_get_primary_reg(reg, &ind);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd->prim_int_users[ind]++;
		if (wcd->prim_int_users[ind] == 1) {
			snd_soc_component_update_bits(component, prim_int_reg,
					    0x10, 0x10);
			wcd9335_codec_hd2_control(component, prim_int_reg, event);
			snd_soc_component_update_bits(component, prim_int_reg,
					    1 << 0x5, 1 << 0x5);
		}
		if ((reg != prim_int_reg) && ((snd_soc_component_read32(component, prim_int_reg)) & 0x10))
			snd_soc_component_update_bits(component, reg, 0x10, 0x10);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd->prim_int_users[ind]--;
		if (wcd->prim_int_users[ind] == 0) {
			snd_soc_component_update_bits(component, prim_int_reg,
					1 << 0x5, 0 << 0x5);
			snd_soc_component_update_bits(component, prim_int_reg,
					0x40, 0x40);
			snd_soc_component_update_bits(component, prim_int_reg,
					0x40, 0x00);
			wcd9335_codec_hd2_control(component, prim_int_reg, event);
		}
		break;
	};

	return 0;
}

static int wcd9335_codec_enable_mix_path(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	u16 gain_reg;
	int offset_val = 0;
	int val = 0;

	switch (w->reg) {
	case WCD9335_CDC_RX0_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX0_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX1_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX1_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX2_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX2_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX3_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX3_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX4_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX4_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX5_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX5_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX6_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX6_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX7_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX7_RX_VOL_MIX_CTL;
		break;
	case WCD9335_CDC_RX8_RX_PATH_MIX_CTL:
		gain_reg = WCD9335_CDC_RX8_RX_VOL_MIX_CTL;
		break;
	default:
		dev_err(component->dev, "%s: No gain register avail for %s\n",
			__func__, w->name);
		return 0;
	};

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		val = snd_soc_component_read32(component, gain_reg);
		val += offset_val;
		snd_soc_component_write(component, gain_reg, val);
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	};

	return 0;
}

static int __tasha_cdc_native_clk_enable(struct wcd9335_priv *wcd,
					 bool enable)
{
	int ret = 0;
	struct snd_soc_component *component = wcd->component;

	if (!wcd->native_clk) {
		dev_err(wcd->dev, "%s: wcd native clock is NULL\n", __func__);
		return -EINVAL;
	}

	if (enable) {
		ret = clk_prepare_enable(wcd->native_clk);
		if (ret) {
			dev_err(wcd->dev, "%s: native clk enable failed\n",
				__func__);
			goto err;
		}
		if (++wcd->native_clk_users == 1) {
			snd_soc_component_update_bits(component, WCD9335_CLOCK_TEST_CTL,
					    0x10, 0x10);
			snd_soc_component_update_bits(component, WCD9335_CLOCK_TEST_CTL,
					    0x80, 0x80);
			snd_soc_component_update_bits(component, WCD9335_CODEC_RPM_CLK_GATE,
					    0x04, 0x00);
			snd_soc_component_update_bits(component,
					WCD9335_CDC_CLK_RST_CTRL_MCLK_CONTROL,
					0x02, 0x02);
		}
	} else {
		if (wcd->native_clk_users &&
		    (--wcd->native_clk_users == 0)) {
			snd_soc_component_update_bits(component,
					WCD9335_CDC_CLK_RST_CTRL_MCLK_CONTROL,
					0x02, 0x00);
			snd_soc_component_update_bits(component, WCD9335_CODEC_RPM_CLK_GATE,
					    0x04, 0x04);
			snd_soc_component_update_bits(component, WCD9335_CLOCK_TEST_CTL,
					    0x80, 0x00);
			snd_soc_component_update_bits(component, WCD9335_CLOCK_TEST_CTL,
					    0x10, 0x00);
		}
		clk_disable_unprepare(wcd->native_clk);
	}

err:
	return ret;
}

static int wcd9335_codec_get_native_fifo_sync_mask(struct snd_soc_component *component,
						 int interp_n)
{
	int mask = 0;
	u16 reg;
	u8 val1, val2, inp0 = 0;
	u8 inp1 = 0, inp2 = 0;

	reg = WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG0 + (2 * interp_n) - 2;

	val1 = snd_soc_component_read32(component, reg);
	val2 = snd_soc_component_read32(component, reg + 1);

	inp0 = val1 & 0x0F;
	inp1 = (val1 >> 4) & 0x0F;
	inp2 = (val2 >> 4) & 0x0F;

	if (IS_VALID_NATIVE_FIFO_PORT(inp0))
		mask |= (1 << (inp0 - 5));
	if (IS_VALID_NATIVE_FIFO_PORT(inp1))
		mask |= (1 << (inp1 - 5));
	if (IS_VALID_NATIVE_FIFO_PORT(inp2))
		mask |= (1 << (inp2 - 5));

	if (!mask)
		dev_err(component->dev, "native fifo err,int:%d,inp0:%d,inp1:%d,inp2:%d\n",
			interp_n, inp0, inp1, inp2);
	return mask;
}

static int wcd9335_enable_native_supply(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *kcontrol, int event)
{
	int mask;
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	u16 interp_reg;

	if (w->shift < INTERP_HPHL || w->shift > INTERP_LO2)
		return -EINVAL;

	interp_reg = WCD9335_CDC_RX1_RX_PATH_CTL + 20 * (w->shift - 1);

	mask = wcd9335_codec_get_native_fifo_sync_mask(component, w->shift);
	if (!mask)
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Adjust interpolator rate to 44P1_NATIVE */
		snd_soc_component_update_bits(component, interp_reg, 0x0F, 0x09);
		__tasha_cdc_native_clk_enable(wcd, true);
		snd_soc_component_update_bits(component, WCD9335_DATA_HUB_NATIVE_FIFO_SYNC,
				    mask, mask);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, WCD9335_DATA_HUB_NATIVE_FIFO_SYNC,
				    mask, 0x0);
		__tasha_cdc_native_clk_enable(wcd, false);
		/* Adjust interpolator rate to default */
		snd_soc_component_update_bits(component, interp_reg, 0x0F, 0x04);
		break;
	}

	return 0;
}

static int wcd9335_codec_enable_interpolator(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	u16 gain_reg;
	u16 reg;
	int val;
	int offset_val = 0;

	if (!(strcmp(w->name, "RX INT0 INTERP"))) {
		reg = WCD9335_CDC_RX0_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX0_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT1 INTERP"))) {
		reg = WCD9335_CDC_RX1_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX1_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT2 INTERP"))) {
		reg = WCD9335_CDC_RX2_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX2_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT3 INTERP"))) {
		reg = WCD9335_CDC_RX3_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX3_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT4 INTERP"))) {
		reg = WCD9335_CDC_RX4_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX4_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT5 INTERP"))) {
		reg = WCD9335_CDC_RX5_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX5_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT6 INTERP"))) {
		reg = WCD9335_CDC_RX6_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX6_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT7 INTERP"))) {
		reg = WCD9335_CDC_RX7_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX7_RX_VOL_CTL;
	} else if (!(strcmp(w->name, "RX INT8 INTERP"))) {
		reg = WCD9335_CDC_RX8_RX_PATH_CTL;
		gain_reg = WCD9335_CDC_RX8_RX_VOL_CTL;
	} else {
		dev_err(component->dev, "%s: Interpolator reg not found\n",
			__func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (!test_bit(SB_CLK_GEAR, &wcd->status_mask)) {
			wcd9335_codec_vote_max_bw(component, true);
			set_bit(SB_CLK_GEAR, &wcd->status_mask);
		}
		/* Reset if needed */
		wcd9335_codec_enable_prim_interpolator(component, reg, event);
		break;
	case SND_SOC_DAPM_POST_PMU:
		wcd9335_config_compander(component, w->shift, event);
		val = snd_soc_component_read32(component, gain_reg);
		val += offset_val;
		snd_soc_component_write(component, gain_reg, val);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd9335_config_compander(component, w->shift, event);
		wcd9335_codec_enable_prim_interpolator(component, reg, event);
		break;
	};

	return 0;
}

static const char * const rx_cf_text[] = {
	"CF_NEG_3DB_4HZ", "CF_NEG_3DB_75HZ", "CF_NEG_3DB_150HZ",
	"CF_NEG_3DB_0P48HZ"
};

static const char * const wcd9335_ear_pa_gain_text[] = {
	"G_6_DB", "G_4P5_DB", "G_3_DB", "G_1P5_DB",
	"G_0_DB", "G_M2P5_DB", "UNDEFINED", "G_M12_DB"
};

static const struct soc_enum wcd9335_ear_pa_gain_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wcd9335_ear_pa_gain_text),
			wcd9335_ear_pa_gain_text);

static const char * const spl_src0_mux_text[] = {
	"ZERO", "SRC_IN_HPHL", "SRC_IN_LO1",
};

static const struct soc_enum cf_int0_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX0_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int0_2_enum, WCD9335_CDC_RX0_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int1_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX1_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int1_2_enum, WCD9335_CDC_RX1_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int2_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX2_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int2_2_enum, WCD9335_CDC_RX2_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int3_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX3_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int3_2_enum, WCD9335_CDC_RX3_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int4_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX4_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int4_2_enum, WCD9335_CDC_RX4_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int5_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX5_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int5_2_enum, WCD9335_CDC_RX5_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int6_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX6_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int6_2_enum, WCD9335_CDC_RX6_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int7_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX7_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int7_2_enum, WCD9335_CDC_RX7_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int8_1_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX8_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int8_2_enum, WCD9335_CDC_RX8_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct snd_soc_dapm_route audio_i2s_map[] = {
	{"SLIM RX0 MUX", NULL, "RX_I2S_CTL"},
	{"SLIM RX1 MUX", NULL, "RX_I2S_CTL"},
	{"SLIM RX2 MUX", NULL, "RX_I2S_CTL"},
	{"SLIM RX3 MUX", NULL, "RX_I2S_CTL"},
};

static const struct snd_soc_dapm_route wcd9335_audio_map[] = {
	/* SLIM_MUX("AIF1_PB", "AIF1 PB"),*/
	{"SLIM RX0 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX1 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX2 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX3 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX4 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX5 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX6 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX7 MUX", "AIF1_PB", "AIF1 PB"},
	/* SLIM_MUX("AIF2_PB", "AIF2 PB"),*/
	{"SLIM RX0 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX1 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX2 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX3 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX4 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX5 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX6 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX7 MUX", "AIF2_PB", "AIF2 PB"},
	/* SLIM_MUX("AIF3_PB", "AIF3 PB"),*/
	{"SLIM RX0 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX1 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX2 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX3 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX4 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX5 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX6 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX7 MUX", "AIF3_PB", "AIF3 PB"},
	/* SLIM_MUX("AIF4_PB", "AIF4 PB"),*/
	{"SLIM RX0 MUX", "AIF4_PB", "AIF4 PB"},
	{"SLIM RX1 MUX", "AIF4_PB", "AIF4 PB"},
	{"SLIM RX2 MUX", "AIF4_PB", "AIF4 PB"},
	{"SLIM RX3 MUX", "AIF4_PB", "AIF4 PB"},
	{"SLIM RX4 MUX", "AIF4_PB", "AIF4 PB"},
	{"SLIM RX5 MUX", "AIF4_PB", "AIF4 PB"},
	{"SLIM RX6 MUX", "AIF4_PB", "AIF4 PB"},
	{"SLIM RX7 MUX", "AIF4_PB", "AIF4 PB"},

	/* SLIM_MUX("AIF_MIX1_PB", "AIF MIX1 PB"),*/
	{"SLIM RX0 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},
	{"SLIM RX1 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},
	{"SLIM RX2 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},
	{"SLIM RX3 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},
	{"SLIM RX4 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},
	{"SLIM RX5 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},
	{"SLIM RX6 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},
	{"SLIM RX7 MUX", "AIF_MIX1_PB", "AIF MIX1 PB"},

	{"SLIM RX0", NULL, "SLIM RX0 MUX"},
	{"SLIM RX1", NULL, "SLIM RX1 MUX"},
	{"SLIM RX2", NULL, "SLIM RX2 MUX"},
	{"SLIM RX3", NULL, "SLIM RX3 MUX"},
	{"SLIM RX4", NULL, "SLIM RX4 MUX"},
	{"SLIM RX5", NULL, "SLIM RX5 MUX"},
	{"SLIM RX6", NULL, "SLIM RX6 MUX"},
	{"SLIM RX7", NULL, "SLIM RX7 MUX"},
	
	/* MIXing path INT1 */
	{"RX INT1_2 MUX", "RX0", "SLIM RX0"},
	{"RX INT1_2 MUX", "RX1", "SLIM RX1"},
	{"RX INT1_2 MUX", "RX2", "SLIM RX2"},
	{"RX INT1_2 MUX", "RX3", "SLIM RX3"},
	{"RX INT1_2 MUX", "RX4", "SLIM RX4"},
	{"RX INT1_2 MUX", "RX5", "SLIM RX5"},
	{"RX INT1_2 MUX", "RX6", "SLIM RX6"},
	{"RX INT1_2 MUX", "RX7", "SLIM RX7"},

	{"RX INT1_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT1_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT1_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT1_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT1_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT1_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT1_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT1_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT1_1 MIX1 INP0", "IIR0", "IIR0"},
	{"RX INT1_1 MIX1 INP0", "IIR1", "IIR1"},

	{"RX INT1_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT1_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT1_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT1_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT1_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT1_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT1_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT1_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT1_1 MIX1 INP1", "IIR0", "IIR0"},
	{"RX INT1_1 MIX1 INP1", "IIR1", "IIR1"},

	{"RX INT1_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT1_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT1_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT1_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT1_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT1_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT1_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT1_1 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX INT1_1 MIX1 INP2", "IIR0", "IIR0"},
	{"RX INT1_1 MIX1 INP2", "IIR1", "IIR1"},

	{"RX INT1_1 MIX1", NULL, "RX INT1_1 MIX1 INP0"},
	{"RX INT1_1 MIX1", NULL, "RX INT1_1 MIX1 INP1"},
	{"RX INT1_1 MIX1", NULL, "RX INT1_1 MIX1 INP2"},

	//{"SPL SRC0 MUX", "SRC_IN_HPHL", "RX INT1_1 MIX1"},
//	{"RX INT1 SPLINE MIX", "HPHL Switch", "SPL SRC0 MUX"},
	{"RX INT1 SPLINE MIX", "HPHL Native Switch", "RX INT1 NATIVE SUPPLY"},
	{"RX INT1 SPLINE MIX", NULL, "RX INT1_1 MIX1"},

	{"RX INT1 SEC MIX", NULL, "RX INT1_2 MUX"},
	{"RX INT1 SEC MIX", NULL, "RX INT1 SPLINE MIX"},

	{"RX INT1 MIX2", NULL, "RX INT1 SEC MIX"},

	{"RX INT1 INTERP", NULL, "RX INT1 MIX2"},	
	{"RX INT1 DEM MUX", "CLSH_DSM_OUT", "RX INT1 INTERP"},	
	{"RX INT1 DAC", NULL, "RX INT1 DEM MUX"},
	{"RX INT1 DAC", NULL, "RX_BIAS"},

	{"HPHL PA", NULL, "RX INT1 DAC"},
	{"HPHL", NULL, "HPHL PA"},

	/* MIXing path INT2 */
	{"RX INT2_2 MUX", "RX0", "SLIM RX0"},
	{"RX INT2_2 MUX", "RX1", "SLIM RX1"},
	{"RX INT2_2 MUX", "RX2", "SLIM RX2"},
	{"RX INT2_2 MUX", "RX3", "SLIM RX3"},
	{"RX INT2_2 MUX", "RX4", "SLIM RX4"},
	{"RX INT2_2 MUX", "RX5", "SLIM RX5"},
	{"RX INT2_2 MUX", "RX6", "SLIM RX6"},
	{"RX INT2_2 MUX", "RX7", "SLIM RX7"},

	{"RX INT2_1 MIX1 INP0", "RX0", "SLIM RX0"},
	{"RX INT2_1 MIX1 INP0", "RX1", "SLIM RX1"},
	{"RX INT2_1 MIX1 INP0", "RX2", "SLIM RX2"},
	{"RX INT2_1 MIX1 INP0", "RX3", "SLIM RX3"},
	{"RX INT2_1 MIX1 INP0", "RX4", "SLIM RX4"},
	{"RX INT2_1 MIX1 INP0", "RX5", "SLIM RX5"},
	{"RX INT2_1 MIX1 INP0", "RX6", "SLIM RX6"},
	{"RX INT2_1 MIX1 INP0", "RX7", "SLIM RX7"},
	{"RX INT2_1 MIX1 INP0", "IIR0", "IIR0"},
	{"RX INT2_1 MIX1 INP0", "IIR1", "IIR1"},
	{"RX INT2_1 MIX1 INP1", "RX0", "SLIM RX0"},
	{"RX INT2_1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX INT2_1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX INT2_1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX INT2_1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX INT2_1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX INT2_1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX INT2_1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX INT2_1 MIX1 INP1", "IIR0", "IIR0"},
	{"RX INT2_1 MIX1 INP1", "IIR1", "IIR1"},
	{"RX INT2_1 MIX1 INP2", "RX0", "SLIM RX0"},
	{"RX INT2_1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX INT2_1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX INT2_1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX INT2_1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX INT2_1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX INT2_1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX INT2_1 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX INT2_1 MIX1 INP2", "IIR0", "IIR0"},
	{"RX INT2_1 MIX1 INP2", "IIR1", "IIR1"},

	{"RX INT2_1 MIX1", NULL, "RX INT2_1 MIX1 INP0"},
	{"RX INT2_1 MIX1", NULL, "RX INT2_1 MIX1 INP1"},
	{"RX INT2_1 MIX1", NULL, "RX INT2_1 MIX1 INP2"},

//	{"SPL SRC1 MUX", "SRC_IN_HPHR", "RX INT2_1 MIX1"},
	{"RX INT2 SPLINE MIX", NULL, "RX INT2_1 MIX1"},
//	{"RX INT2 SPLINE MIX", "HPHR Switch", "SPL SRC1 MUX"},
	{"RX INT2 SPLINE MIX", "HPHR Native Switch", "RX INT2 NATIVE SUPPLY"},
	
	{"RX INT2 SEC MIX", NULL, "RX INT2_2 MUX"},
	{"RX INT2 SEC MIX", NULL, "RX INT2 SPLINE MIX"},
	{"RX INT2 MIX2", NULL, "RX INT2 SEC MIX"},
	{"RX INT2 INTERP", NULL, "RX INT2 MIX2"},
	{"RX INT2 DEM MUX", "CLSH_DSM_OUT", "RX INT2 INTERP"},
	{"RX INT2 DAC", NULL, "RX INT2 DEM MUX"},
	{"RX INT2 DAC", NULL, "RX_BIAS"},
	{"HPHR PA", NULL, "RX INT2 DAC"},
	{"HPHR", NULL, "HPHR PA"},
};

static int wcd9335_rx_hph_mode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	ucontrol->value.integer.value[0] = wcd->hph_mode;
	return 0;
}

static int wcd9335_rx_hph_mode_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	u32 mode_val;

	mode_val = ucontrol->value.enumerated.item[0];

	if (mode_val == 0) {
		dev_warn(component->dev, "%s:Invalid HPH Mode, default to Cls-H HiFi\n",
			__func__);
		mode_val = CLS_H_HIFI;
	}
	wcd->hph_mode = mode_val;
	return 0;
}

static int wcd9335_ear_pa_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain;
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);

	ear_pa_gain = snd_soc_component_read32(component, WCD9335_ANA_EAR);

	ear_pa_gain = (ear_pa_gain & 0x70) >> 4;

	ucontrol->value.integer.value[0] = ear_pa_gain;

	return 0;
}

static int wcd9335_ear_pa_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain;
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);

	ear_pa_gain =  ucontrol->value.integer.value[0] << 4;

	snd_soc_component_update_bits(component, WCD9335_ANA_EAR, 0x70, ear_pa_gain);
	return 0;
}
static const char * const rx_hph_mode_mux_text[] = {
	"CLS_H_INVALID", "CLS_H_HIFI", "CLS_H_LP", "CLS_AB", "CLS_H_LOHIFI"
};

static const struct soc_enum rx_hph_mode_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rx_hph_mode_mux_text),
			    rx_hph_mode_mux_text);

static const struct snd_kcontrol_new wcd9335_snd_controls[] = {
	/* -84dB min - 40dB max */
	SOC_SINGLE_SX_TLV("RX0 Digital Volume", WCD9335_CDC_RX0_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX1 Digital Volume", WCD9335_CDC_RX1_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX2 Digital Volume", WCD9335_CDC_RX2_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX3 Digital Volume", WCD9335_CDC_RX3_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX4 Digital Volume", WCD9335_CDC_RX4_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX5 Digital Volume", WCD9335_CDC_RX5_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX6 Digital Volume", WCD9335_CDC_RX6_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX7 Digital Volume", WCD9335_CDC_RX7_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX8 Digital Volume", WCD9335_CDC_RX8_RX_VOL_CTL,
		0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX0 Mix Digital Volume",
			  WCD9335_CDC_RX0_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX1 Mix Digital Volume",
			  WCD9335_CDC_RX1_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX2 Mix Digital Volume",
			  WCD9335_CDC_RX2_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX3 Mix Digital Volume",
			  WCD9335_CDC_RX3_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX4 Mix Digital Volume",
			  WCD9335_CDC_RX4_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX5 Mix Digital Volume",
			  WCD9335_CDC_RX5_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX6 Mix Digital Volume",
			  WCD9335_CDC_RX6_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX7 Mix Digital Volume",
			  WCD9335_CDC_RX7_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX8 Mix Digital Volume",
			  WCD9335_CDC_RX8_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_ENUM("RX INT0_1 HPF cut off", cf_int0_1_enum),
	SOC_ENUM("RX INT0_2 HPF cut off", cf_int0_2_enum),
	SOC_ENUM("RX INT1_1 HPF cut off", cf_int1_1_enum),
	SOC_ENUM("RX INT1_2 HPF cut off", cf_int1_2_enum),
	SOC_ENUM("RX INT2_1 HPF cut off", cf_int2_1_enum),
	SOC_ENUM("RX INT2_2 HPF cut off", cf_int2_2_enum),
	SOC_ENUM("RX INT3_1 HPF cut off", cf_int3_1_enum),
	SOC_ENUM("RX INT3_2 HPF cut off", cf_int3_2_enum),
	SOC_ENUM("RX INT4_1 HPF cut off", cf_int4_1_enum),
	SOC_ENUM("RX INT4_2 HPF cut off", cf_int4_2_enum),
	SOC_ENUM("RX INT5_1 HPF cut off", cf_int5_1_enum),
	SOC_ENUM("RX INT5_2 HPF cut off", cf_int5_2_enum),
	SOC_ENUM("RX INT6_1 HPF cut off", cf_int6_1_enum),
	SOC_ENUM("RX INT6_2 HPF cut off", cf_int6_2_enum),
	SOC_ENUM("RX INT7_1 HPF cut off", cf_int7_1_enum),
	SOC_ENUM("RX INT7_2 HPF cut off", cf_int7_2_enum),
	SOC_ENUM("RX INT8_1 HPF cut off", cf_int8_1_enum),
	SOC_ENUM("RX INT8_2 HPF cut off", cf_int8_2_enum),
	SOC_SINGLE_EXT("COMP1 Switch", SND_SOC_NOPM, COMPANDER_1, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_SINGLE_EXT("COMP2 Switch", SND_SOC_NOPM, COMPANDER_2, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_SINGLE_EXT("COMP3 Switch", SND_SOC_NOPM, COMPANDER_3, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_SINGLE_EXT("COMP4 Switch", SND_SOC_NOPM, COMPANDER_4, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_SINGLE_EXT("COMP5 Switch", SND_SOC_NOPM, COMPANDER_5, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_SINGLE_EXT("COMP6 Switch", SND_SOC_NOPM, COMPANDER_6, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_SINGLE_EXT("COMP7 Switch", SND_SOC_NOPM, COMPANDER_7, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_SINGLE_EXT("COMP8 Switch", SND_SOC_NOPM, COMPANDER_8, 1, 0,
		       wcd9335_get_compander, wcd9335_set_compander),
	SOC_ENUM_EXT("RX HPH Mode", rx_hph_mode_mux_enum,
		       wcd9335_rx_hph_mode_get, wcd9335_rx_hph_mode_put),

	/* Gain Controls */
	SOC_ENUM_EXT("EAR PA Gain", wcd9335_ear_pa_gain_enum,
		wcd9335_ear_pa_gain_get, wcd9335_ear_pa_gain_put),

	SOC_SINGLE_TLV("HPHL Volume", WCD9335_HPH_L_EN, 0, 20, 1,
		line_gain),
	SOC_SINGLE_TLV("HPHR Volume", WCD9335_HPH_R_EN, 0, 20, 1,
		line_gain),
	SOC_SINGLE_TLV("LINEOUT1 Volume", WCD9335_DIFF_LO_LO1_COMPANDER,
			3, 16, 1, line_gain),
	SOC_SINGLE_TLV("LINEOUT2 Volume", WCD9335_DIFF_LO_LO2_COMPANDER,
			3, 16, 1, line_gain),
	SOC_SINGLE_TLV("LINEOUT3 Volume", WCD9335_SE_LO_LO3_GAIN, 0, 20, 1,
			line_gain),
	SOC_SINGLE_TLV("LINEOUT4 Volume", WCD9335_SE_LO_LO4_GAIN, 0, 20, 1,
			line_gain),
};

static int wcd9335_int_dem_inp_mux_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_dapm_kcontrol_component(kcontrol);

	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;
	unsigned short look_ahead_dly_reg = 0;

	val = ucontrol->value.enumerated.item[0];
	if (val >= e->items)
		return -EINVAL;

	if (e->reg == WCD9335_CDC_RX0_RX_PATH_SEC0)
		look_ahead_dly_reg = WCD9335_CDC_RX0_RX_PATH_CFG0;
	else if (e->reg == WCD9335_CDC_RX1_RX_PATH_SEC0)
		look_ahead_dly_reg = WCD9335_CDC_RX1_RX_PATH_CFG0;
	else if (e->reg == WCD9335_CDC_RX2_RX_PATH_SEC0)
		look_ahead_dly_reg = WCD9335_CDC_RX2_RX_PATH_CFG0;

	/* Set Look Ahead Delay */
	snd_soc_component_update_bits(component, look_ahead_dly_reg,
			    0x08, (val ? 0x08 : 0x00));
	/* Set DEM INP Select */
	return snd_soc_dapm_put_enum_double(kcontrol, ucontrol);
}

static int wcd9335_config_compander(struct snd_soc_component *component, int interp_n,
				  int event)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int comp;
	u16 comp_ctl0_reg, rx_path_cfg0_reg;

	/* EAR does not have compander */
	if (!interp_n)
		return 0;

	comp = interp_n - 1;
	if (!wcd->comp_enabled[comp])
		return 0;

	comp_ctl0_reg = WCD9335_CDC_COMPANDER1_CTL0 + (comp * 8);
	rx_path_cfg0_reg = WCD9335_CDC_RX1_RX_PATH_CFG0 + (comp * 20);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		/* Enable Compander Clock */

		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x01, 0x01);
		/* Reset comander */
		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x02, 0x02);
		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x02, 0x00);
		/* Enables DRE in this path */
		snd_soc_component_update_bits(component, rx_path_cfg0_reg, 0x02, 0x02);
	}

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x04, 0x04);
		snd_soc_component_update_bits(component, rx_path_cfg0_reg, 0x02, 0x00);
		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x02, 0x02);
		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x02, 0x00);
		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x01, 0x00);
		snd_soc_component_update_bits(component, comp_ctl0_reg, 0x04, 0x00);
	}

	return 0;
}

static int wcd9335_codec_aif4_mixer_switch_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_dapm_kcontrol_component(kcontrol);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	if (test_bit(AIF4_SWITCH_VALUE, &wcd->status_mask))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int wcd9335_codec_aif4_mixer_switch_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	struct snd_soc_dapm_update *update = NULL;

	if (ucontrol->value.integer.value[0]) {
		snd_soc_dapm_mixer_update_power(dapm, kcontrol, 1, update);
		set_bit(AIF4_SWITCH_VALUE, &wcd->status_mask);
	} else {
		snd_soc_dapm_mixer_update_power(dapm, kcontrol, 0, update);
		clear_bit(AIF4_SWITCH_VALUE, &wcd->status_mask);
	}

	return 1;
}

static int _wcd9335_codec_enable_mclk(struct snd_soc_component *component, int enable)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int ret = 0;

	if (!wcd->ext_clk) {
		dev_err(wcd->dev, "%s: wcd ext clock is NULL\n", __func__);
		return -EINVAL;
	}

	if (enable) {
		ret = wcd9335_cdc_req_mclk_enable(wcd, true);
		if (ret)
			return ret;

		set_bit(AUDIO_NOMINAL, &wcd->status_mask);
		wcd9335_codec_apply_sido_voltage(wcd,
				SIDO_VOLTAGE_NOMINAL_MV);
	} else {
		clear_bit(AUDIO_NOMINAL, &wcd->status_mask);
		wcd9335_codec_update_sido_voltage(wcd,
					sido_buck_svs_voltage);
		wcd9335_cdc_req_mclk_enable(wcd, false);
	}

	return ret;
}

static int wcd9335_codec_enable_mclk(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return _wcd9335_codec_enable_mclk(component, true);
	case SND_SOC_DAPM_POST_PMD:
		return _wcd9335_codec_enable_mclk(component, false);
	}

	return 0;
}
static const char * const spl_src1_mux_text[] = {
	"ZERO", "SRC_IN_HPHR", "SRC_IN_LO2",
};

static const char * const spl_src2_mux_text[] = {
	"ZERO", "SRC_IN_LO3", "SRC_IN_SPKRL",
};

static const char * const spl_src3_mux_text[] = {
	"ZERO", "SRC_IN_LO4", "SRC_IN_SPKRR",
};

static const char * const rx_int0_7_mix_mux_text[] = {
	"ZERO", "RX0", "RX1", "RX2", "RX3", "RX4", "RX5",
	"RX6", "RX7", "PROXIMITY"
};

static const char * const rx_int_mix_mux_text[] = {
	"ZERO", "RX0", "RX1", "RX2", "RX3", "RX4", "RX5",
	"RX6", "RX7"
};

static const char * const rx_prim_mix_text[] = {
	"ZERO", "DEC0", "DEC1", "IIR0", "IIR1", "RX0", "RX1", "RX2",
	"RX3", "RX4", "RX5", "RX6", "RX7"
};

static const char * const rx_int_dem_inp_mux_text[] = {
	"NORMAL_DSM_OUT", "CLSH_DSM_OUT",
};

static const char * const rx_int0_interp_mux_text[] = {
	"ZERO", "RX INT0 MIX2",
};

static const char * const rx_int1_interp_mux_text[] = {
	"ZERO", "RX INT1 MIX2",
};

static const char * const rx_int2_interp_mux_text[] = {
	"ZERO", "RX INT2 MIX2",
};

static const char * const rx_int3_interp_mux_text[] = {
	"ZERO", "RX INT3 MIX2",
};

static const char * const rx_int4_interp_mux_text[] = {
	"ZERO", "RX INT4 MIX2",
};

static const char * const rx_int5_interp_mux_text[] = {
	"ZERO", "RX INT5 MIX2",
};

static const char * const rx_int6_interp_mux_text[] = {
	"ZERO", "RX INT6 MIX2",
};

static const char * const rx_int7_interp_mux_text[] = {
	"ZERO", "RX INT7 MIX2",
};

static const char * const rx_int8_interp_mux_text[] = {
	"ZERO", "RX INT8 SEC MIX"
};

static const char * const rx_echo_mux_text[] = {
	"ZERO", "RX_MIX0", "RX_MIX1", "RX_MIX2", "RX_MIX3", "RX_MIX4",
	"RX_MIX5", "RX_MIX6", "RX_MIX7", "RX_MIX8",
};

static const char * const anc0_fb_mux_text[] = {
	"ZERO", "ANC_IN_HPHL", "ANC_IN_EAR", "ANC_IN_EAR_SPKR",
	"ANC_IN_LO1"
};

static const char * const anc1_fb_mux_text[] = {
	"ZERO", "ANC_IN_HPHR", "ANC_IN_LO2"
};

static const struct soc_enum spl_src0_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SPLINE_SRC_CFG0, 0, 3,
			spl_src0_mux_text);

static const struct soc_enum spl_src1_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SPLINE_SRC_CFG0, 2, 3,
			spl_src1_mux_text);

static const struct soc_enum spl_src2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SPLINE_SRC_CFG0, 4, 3,
			spl_src2_mux_text);

static const struct soc_enum spl_src3_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_SPLINE_SRC_CFG0, 6, 3,
			spl_src3_mux_text);

static const struct soc_enum rx_int0_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG1, 0, 10,
			rx_int0_7_mix_mux_text);

static const struct soc_enum rx_int1_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG1, 0, 9,
			rx_int_mix_mux_text);

static const struct soc_enum rx_int2_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT2_CFG1, 0, 9,
			rx_int_mix_mux_text);

static const struct soc_enum rx_int3_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT3_CFG1, 0, 9,
			rx_int_mix_mux_text);

static const struct soc_enum rx_int4_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT4_CFG1, 0, 9,
			rx_int_mix_mux_text);

static const struct soc_enum rx_int5_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT5_CFG1, 0, 9,
			rx_int_mix_mux_text);

static const struct soc_enum rx_int6_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT6_CFG1, 0, 9,
			rx_int_mix_mux_text);

static const struct soc_enum rx_int7_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT7_CFG1, 0, 10,
			rx_int0_7_mix_mux_text);

static const struct soc_enum rx_int8_2_mux_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT8_CFG1, 0, 9,
			rx_int_mix_mux_text);

static const struct soc_enum rx_int0_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int0_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int0_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int1_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int1_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int1_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT1_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int2_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT2_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int2_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT2_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int2_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT2_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int3_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT3_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int3_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT3_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int3_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT3_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int4_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT4_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int4_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT4_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int4_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT4_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int5_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT5_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int5_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT5_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int5_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT5_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int6_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT6_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int6_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT6_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int6_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT6_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int7_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT7_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int7_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT7_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int7_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT7_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int8_1_mix_inp0_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT8_CFG0, 0, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int8_1_mix_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT8_CFG0, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int8_1_mix_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_RX_INT8_CFG1, 4, 13,
			rx_prim_mix_text);

static const struct soc_enum rx_int0_dem_inp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX0_RX_PATH_SEC0, 0,
			ARRAY_SIZE(rx_int_dem_inp_mux_text),
			rx_int_dem_inp_mux_text);

static const struct soc_enum rx_int1_dem_inp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX1_RX_PATH_SEC0, 0,
			ARRAY_SIZE(rx_int_dem_inp_mux_text),
			rx_int_dem_inp_mux_text);

static const struct soc_enum rx_int2_dem_inp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX2_RX_PATH_SEC0, 0,
			ARRAY_SIZE(rx_int_dem_inp_mux_text),
			rx_int_dem_inp_mux_text);

static const struct soc_enum rx_int0_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX0_RX_PATH_CTL, 5, 2,
			rx_int0_interp_mux_text);

static const struct soc_enum rx_int1_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX1_RX_PATH_CTL, 5, 2,
			rx_int1_interp_mux_text);

static const struct soc_enum rx_int2_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX2_RX_PATH_CTL, 5, 2,
			rx_int2_interp_mux_text);

static const struct soc_enum rx_int3_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX3_RX_PATH_CTL, 5, 2,
			rx_int3_interp_mux_text);

static const struct soc_enum rx_int4_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX4_RX_PATH_CTL, 5, 2,
			rx_int4_interp_mux_text);

static const struct soc_enum rx_int5_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX5_RX_PATH_CTL, 5, 2,
			rx_int5_interp_mux_text);

static const struct soc_enum rx_int6_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX6_RX_PATH_CTL, 5, 2,
			rx_int6_interp_mux_text);

static const struct soc_enum rx_int7_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX7_RX_PATH_CTL, 5, 2,
			rx_int7_interp_mux_text);

static const struct soc_enum rx_int8_interp_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX8_RX_PATH_CTL, 5, 2,
			rx_int8_interp_mux_text);

static const struct soc_enum anc0_fb_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_ANC_CFG0, 0, 5,
			anc0_fb_mux_text);

static const struct soc_enum anc1_fb_mux_enum =
	SOC_ENUM_SINGLE(WCD9335_CDC_RX_INP_MUX_ANC_CFG0, 3, 3,
			anc1_fb_mux_text);

static const struct snd_kcontrol_new rx_int0_dem_inp_mux =
	SOC_DAPM_ENUM_EXT("RX INT0 DEM MUX Mux", rx_int0_dem_inp_mux_enum,
			  snd_soc_dapm_get_enum_double,
			  wcd9335_int_dem_inp_mux_put);

static const struct snd_kcontrol_new rx_int1_dem_inp_mux =
	SOC_DAPM_ENUM_EXT("RX INT1 DEM MUX Mux", rx_int1_dem_inp_mux_enum,
			  snd_soc_dapm_get_enum_double,
			  wcd9335_int_dem_inp_mux_put);

static const struct snd_kcontrol_new rx_int2_dem_inp_mux =
	SOC_DAPM_ENUM_EXT("RX INT2 DEM MUX Mux", rx_int2_dem_inp_mux_enum,
			  snd_soc_dapm_get_enum_double,
			  wcd9335_int_dem_inp_mux_put);

static const struct snd_kcontrol_new spl_src0_mux =
	SOC_DAPM_ENUM("SPL SRC0 MUX Mux", spl_src0_mux_chain_enum);

static const struct snd_kcontrol_new spl_src1_mux =
	SOC_DAPM_ENUM("SPL SRC1 MUX Mux", spl_src1_mux_chain_enum);

static const struct snd_kcontrol_new spl_src2_mux =
	SOC_DAPM_ENUM("SPL SRC2 MUX Mux", spl_src2_mux_chain_enum);

static const struct snd_kcontrol_new spl_src3_mux =
	SOC_DAPM_ENUM("SPL SRC3 MUX Mux", spl_src3_mux_chain_enum);

static const struct snd_kcontrol_new rx_int0_2_mux =
	SOC_DAPM_ENUM("RX INT0_2 MUX Mux", rx_int0_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int1_2_mux =
	SOC_DAPM_ENUM("RX INT1_2 MUX Mux", rx_int1_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int2_2_mux =
	SOC_DAPM_ENUM("RX INT2_2 MUX Mux", rx_int2_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int3_2_mux =
	SOC_DAPM_ENUM("RX INT3_2 MUX Mux", rx_int3_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int4_2_mux =
	SOC_DAPM_ENUM("RX INT4_2 MUX Mux", rx_int4_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int5_2_mux =
	SOC_DAPM_ENUM("RX INT5_2 MUX Mux", rx_int5_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int6_2_mux =
	SOC_DAPM_ENUM("RX INT6_2 MUX Mux", rx_int6_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int7_2_mux =
	SOC_DAPM_ENUM("RX INT7_2 MUX Mux", rx_int7_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int8_2_mux =
	SOC_DAPM_ENUM("RX INT8_2 MUX Mux", rx_int8_2_mux_chain_enum);

static const struct snd_kcontrol_new rx_int0_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT0_1 MIX1 INP0 Mux", rx_int0_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int0_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT0_1 MIX1 INP1 Mux", rx_int0_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int0_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT0_1 MIX1 INP2 Mux", rx_int0_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int1_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT1_1 MIX1 INP0 Mux", rx_int1_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int1_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT1_1 MIX1 INP1 Mux", rx_int1_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int1_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT1_1 MIX1 INP2 Mux", rx_int1_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int2_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT2_1 MIX1 INP0 Mux", rx_int2_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int2_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT2_1 MIX1 INP1 Mux", rx_int2_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int2_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT2_1 MIX1 INP2 Mux", rx_int2_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int3_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT3_1 MIX1 INP0 Mux", rx_int3_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int3_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT3_1 MIX1 INP1 Mux", rx_int3_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int3_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT3_1 MIX1 INP2 Mux", rx_int3_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int4_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT4_1 MIX1 INP0 Mux", rx_int4_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int4_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT4_1 MIX1 INP1 Mux", rx_int4_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int4_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT4_1 MIX1 INP2 Mux", rx_int4_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int5_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT5_1 MIX1 INP0 Mux", rx_int5_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int5_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT5_1 MIX1 INP1 Mux", rx_int5_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int5_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT5_1 MIX1 INP2 Mux", rx_int5_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int6_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT6_1 MIX1 INP0 Mux", rx_int6_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int6_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT6_1 MIX1 INP1 Mux", rx_int6_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int6_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT6_1 MIX1 INP2 Mux", rx_int6_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int7_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT7_1 MIX1 INP0 Mux", rx_int7_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int7_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT7_1 MIX1 INP1 Mux", rx_int7_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int7_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT7_1 MIX1 INP2 Mux", rx_int7_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int8_1_mix_inp0_mux =
	SOC_DAPM_ENUM("RX INT8_1 MIX1 INP0 Mux", rx_int8_1_mix_inp0_chain_enum);

static const struct snd_kcontrol_new rx_int8_1_mix_inp1_mux =
	SOC_DAPM_ENUM("RX INT8_1 MIX1 INP1 Mux", rx_int8_1_mix_inp1_chain_enum);

static const struct snd_kcontrol_new rx_int8_1_mix_inp2_mux =
	SOC_DAPM_ENUM("RX INT8_1 MIX1 INP2 Mux", rx_int8_1_mix_inp2_chain_enum);

static const struct snd_kcontrol_new rx_int0_interp_mux =
	SOC_DAPM_ENUM("RX INT0 INTERP Mux", rx_int0_interp_mux_enum);

static const struct snd_kcontrol_new rx_int1_interp_mux =
	SOC_DAPM_ENUM("RX INT1 INTERP Mux", rx_int1_interp_mux_enum);

static const struct snd_kcontrol_new rx_int2_interp_mux =
	SOC_DAPM_ENUM("RX INT2 INTERP Mux", rx_int2_interp_mux_enum);

static const struct snd_kcontrol_new rx_int3_interp_mux =
	SOC_DAPM_ENUM("RX INT3 INTERP Mux", rx_int3_interp_mux_enum);

static const struct snd_kcontrol_new rx_int4_interp_mux =
	SOC_DAPM_ENUM("RX INT4 INTERP Mux", rx_int4_interp_mux_enum);

static const struct snd_kcontrol_new rx_int5_interp_mux =
	SOC_DAPM_ENUM("RX INT5 INTERP Mux", rx_int5_interp_mux_enum);

static const struct snd_kcontrol_new rx_int6_interp_mux =
	SOC_DAPM_ENUM("RX INT6 INTERP Mux", rx_int6_interp_mux_enum);

static const struct snd_kcontrol_new rx_int7_interp_mux =
	SOC_DAPM_ENUM("RX INT7 INTERP Mux", rx_int7_interp_mux_enum);

static const struct snd_kcontrol_new rx_int8_interp_mux =
	SOC_DAPM_ENUM("RX INT8 INTERP Mux", rx_int8_interp_mux_enum);

static const struct snd_kcontrol_new aif4_switch_mixer_controls =
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM,
			0, 1, 0, wcd9335_codec_aif4_mixer_switch_get,
			wcd9335_codec_aif4_mixer_switch_put);

static const struct snd_kcontrol_new anc_hphl_switch =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new anc_hphr_switch =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new anc_ear_switch =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new anc_lineout1_switch =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new anc_lineout2_switch =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux0_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux1_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux2_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux3_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux4_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux5_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux6_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux7_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new adc_us_mux8_switch =
	SOC_DAPM_SINGLE("US_Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new anc0_fb_mux =
	SOC_DAPM_ENUM("ANC0 FB MUX Mux", anc0_fb_mux_enum);

static const struct snd_kcontrol_new anc1_fb_mux =
	SOC_DAPM_ENUM("ANC1 FB MUX Mux", anc1_fb_mux_enum);

static const struct snd_soc_dapm_widget wcd9335_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("EAR"),
	SND_SOC_DAPM_OUTPUT("ANC EAR"),
	SND_SOC_DAPM_OUTPUT("HPHL"),
	SND_SOC_DAPM_OUTPUT("HPHR"),
	SND_SOC_DAPM_OUTPUT("SPK1 OUT"),
	SND_SOC_DAPM_OUTPUT("SPK2 OUT"),
	SND_SOC_DAPM_OUTPUT("LINEOUT1"),
	SND_SOC_DAPM_OUTPUT("LINEOUT2"),
	SND_SOC_DAPM_OUTPUT("LINEOUT3"),
	SND_SOC_DAPM_OUTPUT("LINEOUT4"),
	SND_SOC_DAPM_AIF_IN_E("AIF1 PB", "AIF1 Playback", 0, SND_SOC_NOPM,
				AIF1_PB, 0, wcd9335_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("AIF2 PB", "AIF2 Playback", 0, SND_SOC_NOPM,
				AIF2_PB, 0, wcd9335_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("AIF3 PB", "AIF3 Playback", 0, SND_SOC_NOPM,
				AIF3_PB, 0, wcd9335_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("AIF4 PB", "AIF4 Playback", 0, SND_SOC_NOPM,
				AIF4_PB, 0, wcd9335_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("SLIM RX0 MUX", SND_SOC_NOPM, WCD9335_RX0, 0,
				&slim_rx_mux[WCD9335_RX0]),
	SND_SOC_DAPM_MUX("SLIM RX1 MUX", SND_SOC_NOPM, WCD9335_RX1, 0,
				&slim_rx_mux[WCD9335_RX1]),
	SND_SOC_DAPM_MUX("SLIM RX2 MUX", SND_SOC_NOPM, WCD9335_RX2, 0,
				&slim_rx_mux[WCD9335_RX2]),
	SND_SOC_DAPM_MUX("SLIM RX3 MUX", SND_SOC_NOPM, WCD9335_RX3, 0,
				&slim_rx_mux[WCD9335_RX3]),
	SND_SOC_DAPM_MUX("SLIM RX4 MUX", SND_SOC_NOPM, WCD9335_RX4, 0,
				&slim_rx_mux[WCD9335_RX4]),
	SND_SOC_DAPM_MUX("SLIM RX5 MUX", SND_SOC_NOPM, WCD9335_RX5, 0,
				&slim_rx_mux[WCD9335_RX5]),
	SND_SOC_DAPM_MUX("SLIM RX6 MUX", SND_SOC_NOPM, WCD9335_RX6, 0,
				&slim_rx_mux[WCD9335_RX6]),
	SND_SOC_DAPM_MUX("SLIM RX7 MUX", SND_SOC_NOPM, WCD9335_RX7, 0,
				&slim_rx_mux[WCD9335_RX7]),
	SND_SOC_DAPM_MIXER("SLIM RX0", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX3", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX4", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX5", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX6", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX7", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX_E("RX INT0_2 MUX", WCD9335_CDC_RX0_RX_PATH_MIX_CTL,
			5, 0, &rx_int0_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT1_2 MUX", WCD9335_CDC_RX1_RX_PATH_MIX_CTL,
			5, 0, &rx_int1_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT2_2 MUX", WCD9335_CDC_RX2_RX_PATH_MIX_CTL,
			5, 0, &rx_int2_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT3_2 MUX", WCD9335_CDC_RX3_RX_PATH_MIX_CTL,
			5, 0, &rx_int3_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT4_2 MUX", WCD9335_CDC_RX4_RX_PATH_MIX_CTL,
			5, 0, &rx_int4_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT5_2 MUX", WCD9335_CDC_RX5_RX_PATH_MIX_CTL,
			5, 0, &rx_int5_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT6_2 MUX", WCD9335_CDC_RX6_RX_PATH_MIX_CTL,
			5, 0, &rx_int6_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT7_2 MUX", WCD9335_CDC_RX7_RX_PATH_MIX_CTL,
			5, 0, &rx_int7_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX_E("RX INT8_2 MUX", WCD9335_CDC_RX8_RX_PATH_MIX_CTL,
			5, 0, &rx_int8_2_mux, wcd9335_codec_enable_mix_path,
			SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MUX("RX INT0_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int0_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT0_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int0_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT0_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int0_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT1_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int1_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT1_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int1_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT1_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int1_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT2_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int2_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT2_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int2_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT2_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int2_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT3_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int3_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT3_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int3_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT3_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int3_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT4_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int4_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT4_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int4_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT4_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int4_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT5_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int5_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT5_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int5_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT5_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int5_1_mix_inp2_mux),
	SND_SOC_DAPM_MUX("RX INT6_1 MIX1 INP0", SND_SOC_NOPM, 0, 0,
		&rx_int6_1_mix_inp0_mux),
	SND_SOC_DAPM_MUX("RX INT6_1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_int6_1_mix_inp1_mux),
	SND_SOC_DAPM_MUX("RX INT6_1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_int6_1_mix_inp2_mux),
	SND_SOC_DAPM_MIXER("RX INT0_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT0 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT1_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT1 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int1_spline_mix_switch,
			ARRAY_SIZE(rx_int1_spline_mix_switch)),
	SND_SOC_DAPM_MIXER("RX INT1 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT2_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT2 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int2_spline_mix_switch,
			ARRAY_SIZE(rx_int2_spline_mix_switch)),
	SND_SOC_DAPM_MIXER("RX INT2 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT3_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT3 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int3_spline_mix_switch,
			ARRAY_SIZE(rx_int3_spline_mix_switch)),
	SND_SOC_DAPM_MIXER("RX INT3 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT4_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT4 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int4_spline_mix_switch,
			ARRAY_SIZE(rx_int4_spline_mix_switch)),
	SND_SOC_DAPM_MIXER("RX INT4 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT5_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT5 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int5_spline_mix_switch,
			ARRAY_SIZE(rx_int5_spline_mix_switch)),
	SND_SOC_DAPM_MIXER("RX INT5 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("RX INT6_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT6 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int6_spline_mix_switch,
			ARRAY_SIZE(rx_int6_spline_mix_switch)),
	SND_SOC_DAPM_MIXER("RX INT6 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("RX INT7_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT7 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT7 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int7_spline_mix_switch,
			ARRAY_SIZE(rx_int7_spline_mix_switch)),

	SND_SOC_DAPM_MIXER("RX INT8_1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT8 SEC MIX", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT8 SPLINE MIX", SND_SOC_NOPM, 0, 0,
			rx_int8_spline_mix_switch,
			ARRAY_SIZE(rx_int8_spline_mix_switch)),

	SND_SOC_DAPM_MIXER("RX INT0 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT1 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT2 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT3 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT4 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT5 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT6 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX INT7 MIX2", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("RX INT1 NATIVE SUPPLY", SND_SOC_NOPM,
			    INTERP_HPHL, 0, wcd9335_enable_native_supply,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("RX INT2 NATIVE SUPPLY", SND_SOC_NOPM,
			    INTERP_HPHR, 0, wcd9335_enable_native_supply,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("RX INT3 NATIVE SUPPLY", SND_SOC_NOPM,
			    INTERP_LO1, 0, wcd9335_enable_native_supply,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("RX INT4 NATIVE SUPPLY", SND_SOC_NOPM,
			    INTERP_LO2, 0, wcd9335_enable_native_supply,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MUX("RX INT0 DEM MUX", SND_SOC_NOPM, 0, 0,
		&rx_int0_dem_inp_mux),
	SND_SOC_DAPM_MUX("RX INT1 DEM MUX", SND_SOC_NOPM, 0, 0,
		&rx_int1_dem_inp_mux),
	SND_SOC_DAPM_MUX("RX INT2 DEM MUX", SND_SOC_NOPM, 0, 0,
		&rx_int2_dem_inp_mux),

	SND_SOC_DAPM_MUX_E("RX INT0 INTERP", SND_SOC_NOPM,
		INTERP_EAR, 0, &rx_int0_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT1 INTERP", SND_SOC_NOPM,
		INTERP_HPHL, 0, &rx_int1_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT2 INTERP", SND_SOC_NOPM,
		INTERP_HPHR, 0, &rx_int2_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT3 INTERP", SND_SOC_NOPM,
		INTERP_LO1, 0, &rx_int3_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT4 INTERP", SND_SOC_NOPM,
		INTERP_LO2, 0, &rx_int4_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT5 INTERP", SND_SOC_NOPM,
		INTERP_LO3, 0, &rx_int5_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT6 INTERP", SND_SOC_NOPM,
		INTERP_LO4, 0, &rx_int6_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT7 INTERP", SND_SOC_NOPM,
		INTERP_SPKR1, 0, &rx_int7_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX INT8 INTERP", SND_SOC_NOPM,
		INTERP_SPKR2, 0, &rx_int8_interp_mux,
		wcd9335_codec_enable_interpolator,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("RX INT0 DAC", NULL, SND_SOC_NOPM,
		0, 0, wcd9335_codec_ear_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RX INT1 DAC", NULL, WCD9335_ANA_HPH,
		5, 0, wcd9335_codec_hphl_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RX INT2 DAC", NULL, WCD9335_ANA_HPH,
		4, 0, wcd9335_codec_hphr_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RX INT3 DAC", NULL, SND_SOC_NOPM,
		0, 0, wcd9335_codec_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RX INT4 DAC", NULL, SND_SOC_NOPM,
		0, 0, wcd9335_codec_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RX INT5 DAC", NULL, SND_SOC_NOPM,
		0, 0, wcd9335_codec_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RX INT6 DAC", NULL, SND_SOC_NOPM,
		0, 0, wcd9335_codec_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHL PA", WCD9335_ANA_HPH, 7, 0, NULL, 0,
			   wcd9335_codec_enable_hphl_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHR PA", WCD9335_ANA_HPH, 6, 0, NULL, 0,
			   wcd9335_codec_enable_hphr_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("EAR PA", WCD9335_ANA_EAR, 7, 0, NULL, 0,
			   wcd9335_codec_enable_ear_pa,
			   SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT1 PA", WCD9335_ANA_LO_1_2, 7, 0, NULL, 0,
			   wcd9335_codec_enable_lineout_pa,
			   SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT2 PA", WCD9335_ANA_LO_1_2, 6, 0, NULL, 0,
			   wcd9335_codec_enable_lineout_pa,
			   SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT3 PA", WCD9335_ANA_LO_3_4, 7, 0, NULL, 0,
			   wcd9335_codec_enable_lineout_pa,
			   SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT4 PA", WCD9335_ANA_LO_3_4, 6, 0, NULL, 0,
			   wcd9335_codec_enable_lineout_pa,
			   SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0,
		wcd9335_codec_enable_rx_bias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MCLK",  SND_SOC_NOPM, 0, 0,
	wcd9335_codec_enable_mclk, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static int wcd9335_get_channel_map(struct snd_soc_dai *dai,
				 unsigned int *tx_num, unsigned int *tx_slot,
				 unsigned int *rx_num, unsigned int *rx_slot)
{
	struct wcd9335_priv *wcd = snd_soc_component_get_drvdata(dai->component);
	u32 i = 0;
	struct wcd_slim_ch *ch;
	switch (dai->id) {
	case AIF1_PB:
	case AIF2_PB:
	case AIF3_PB:
	case AIF4_PB:
	case AIF_MIX1_PB:
		if (!rx_slot || !rx_num) {
			dev_err(wcd->dev, "Invalid rx_slot %p or rx_num %p\n",
				rx_slot, rx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &wcd->dai[dai->id].wcd_slim_ch_list,
				    list) {
			dev_err(wcd->dev ,"slot_num %u ch->ch_num %d\n",
				i, ch->ch_num);
			rx_slot[i++] = ch->ch_num;
		}
		*rx_num = i;
		break;
	case AIF1_CAP:
	case AIF2_CAP:
	case AIF3_CAP:
	case AIF4_MAD_TX:
	case AIF4_VIFEED:
		if (!tx_slot || !tx_num) {
			dev_err(wcd->dev, "Invalid tx_slot %p or tx_num %p\n",
				tx_slot, tx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &wcd->dai[dai->id].wcd_slim_ch_list,
				    list) {
			tx_slot[i++] = ch->ch_num;
		}
		*tx_num = i;
		break;

	default:
		dev_err(wcd->dev, "Invalid DAI ID %x\n", dai->id);
		break;
	}

	return 0;
}

static int mywcd_slim_init_slimslave(struct wcd_slim_data *wcd, u8 wcd_slim_pgd_la,
			   unsigned int tx_num, unsigned int *tx_slot,
			   unsigned int rx_num, unsigned int *rx_slot)
{
	int ret = 0;
	int i;

	if (wcd->rx_chs) {
		wcd->num_rx_port = rx_num;
		for (i = 0; i < rx_num; i++) {
			wcd->rx_chs[i].ch_num = rx_slot[i];
			INIT_LIST_HEAD(&wcd->rx_chs[i].list);
		}
#if 0
		ret = wcd_slim_alloc_slim_sh_ch(wcd->slim, wcd_slim_pgd_la,
						wcd->num_rx_port,
						wcd->rx_chs,
						SLIM_SINK);
		if (ret) {
			pr_err("%s: Failed to alloc %d rx slimbus channels\n",
				__func__, wcd->num_rx_port);
			kfree(wcd->rx_chs);
			wcd->rx_chs = NULL;
			wcd->num_rx_port = 0;
		}
#endif
	} else {
		pr_err("Not able to allocate memory for %d slimbus rx ports\n",
			wcd->num_rx_port);
	}

	if (wcd->tx_chs) {
		wcd->num_tx_port = tx_num;
		for (i = 0; i < tx_num; i++) {
			wcd->tx_chs[i].ch_num = tx_slot[i];
			INIT_LIST_HEAD(&wcd->tx_chs[i].list);
		}
#if 0
		ret = wcd_slim_alloc_slim_sh_ch(wcd->slim, wcd_slim_pgd_la,
						wcd->num_tx_port,
						wcd->tx_chs,
						SLIM_SRC);
		if (ret) {
			pr_err("%s: Failed to alloc %d tx slimbus channels\n",
				__func__, wcd->num_tx_port);
			kfree(wcd->tx_chs);
			wcd->tx_chs = NULL;
			wcd->num_tx_port = 0;
		}
#endif
	} else {
		pr_err("Not able to allocate memory for %d slimbus tx ports\n",
			wcd->num_tx_port);
	}
}

static int wcd9335_set_channel_map(struct snd_soc_dai *dai,
				 unsigned int tx_num, unsigned int *tx_slot,
				 unsigned int rx_num, unsigned int *rx_slot)
{
	struct wcd9335_priv *wcd;

	wcd = snd_soc_component_get_drvdata(dai->component);

	if (!tx_slot || !rx_slot) {
		dev_err(wcd->dev, "Invalid tx_slot=%p, rx_slot=%p\n",
			tx_slot, rx_slot);
		return -EINVAL;
	}

	if (wcd->intf_type == WCD_INTERFACE_TYPE_SLIMBUS) {
//		//FIXME ..
		mywcd_slim_init_slimslave(wcd->slim_data, wcd->slim->laddr,
					   tx_num, tx_slot, rx_num, rx_slot);
	}
	return 0;
}

static int wcd9335_set_mix_interpolator_rate(struct snd_soc_dai *dai,
					   u8 int_mix_fs_rate_reg_val,
					   u32 sample_rate)
{
	u8 int_2_inp;
	u32 j;
	u16 int_mux_cfg1, int_fs_reg;
	u8 int_mux_cfg1_val;
	struct snd_soc_component *component = dai->component;
	struct wcd_slim_ch *ch;
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	list_for_each_entry(ch, &wcd->dai[dai->id].wcd_slim_ch_list, list) {
		int_2_inp = ch->port + INTn_2_INP_SEL_RX0 -
				  WCD9335_RX_PORT_START_NUMBER;
		if ((int_2_inp < INTn_2_INP_SEL_RX0) ||
		   (int_2_inp > INTn_2_INP_SEL_RX7)) {
			pr_err("%s: Invalid RX%u port, Dai ID is %d\n",
				__func__,
				(ch->port - WCD9335_RX_PORT_START_NUMBER),
				dai->id);
			return -EINVAL;
		}

		int_mux_cfg1 = WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG1;
		for (j = 0; j < WCD9335_NUM_INTERPOLATORS; j++) {
			int_mux_cfg1_val = snd_soc_component_read32(component, int_mux_cfg1) &
						0x0F;
			if (int_mux_cfg1_val == int_2_inp) {
				int_fs_reg = WCD9335_CDC_RX0_RX_PATH_MIX_CTL +
						20 * j;
				snd_soc_component_update_bits(component, int_fs_reg,
						0x0F, int_mix_fs_rate_reg_val);
			}
			int_mux_cfg1 += 2;
		}
	}
	return 0;
}

static int wcd9335_set_prim_interpolator_rate(struct snd_soc_dai *dai,
					    u8 int_prim_fs_rate_reg_val,
					    u32 sample_rate)
{
	u8 int_1_mix1_inp;
	u32 j;
	u16 int_mux_cfg0, int_mux_cfg1;
	u16 int_fs_reg;
	u8 int_mux_cfg0_val, int_mux_cfg1_val;
	u8 inp0_sel, inp1_sel, inp2_sel;
	struct snd_soc_component *component = dai->component;
	struct wcd_slim_ch *ch;
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	list_for_each_entry(ch, &wcd->dai[dai->id].wcd_slim_ch_list, list) {
		int_1_mix1_inp = ch->port + INTn_1_MIX_INP_SEL_RX0 -
				  WCD9335_RX_PORT_START_NUMBER;
		if ((int_1_mix1_inp < INTn_1_MIX_INP_SEL_RX0) ||
		   (int_1_mix1_inp > INTn_1_MIX_INP_SEL_RX7)) {
			pr_err("%s: Invalid RX%u port, Dai ID is %d\n",
				__func__,
				(ch->port - WCD9335_RX_PORT_START_NUMBER),
				dai->id);
			return -EINVAL;
		}

		int_mux_cfg0 = WCD9335_CDC_RX_INP_MUX_RX_INT0_CFG0;

		/*
		 * Loop through all interpolator MUX inputs and find out
		 * to which interpolator input, the slim rx port
		 * is connected
		 */
		for (j = 0; j < WCD9335_NUM_INTERPOLATORS; j++) {
			int_mux_cfg1 = int_mux_cfg0 + 1;

			int_mux_cfg0_val = snd_soc_component_read32(component, int_mux_cfg0);
			int_mux_cfg1_val = snd_soc_component_read32(component, int_mux_cfg1);
			inp0_sel = int_mux_cfg0_val & 0x0F;
			inp1_sel = (int_mux_cfg0_val >> 4) & 0x0F;
			inp2_sel = (int_mux_cfg1_val >> 4) & 0x0F;
			if ((inp0_sel == int_1_mix1_inp) ||
			    (inp1_sel == int_1_mix1_inp) ||
			    (inp2_sel == int_1_mix1_inp)) {
				int_fs_reg = WCD9335_CDC_RX0_RX_PATH_CTL +
					     20 * j;
				/* sample_rate is in Hz */
				if ((j == 0) && (sample_rate == 44100)) {
					pr_info("%s: Cannot set 44.1KHz on INT0\n",
						__func__);
				} else
					snd_soc_component_update_bits(component, int_fs_reg,
						0x0F, int_prim_fs_rate_reg_val);
			}
			int_mux_cfg0 += 2;
		}
	}

	return 0;
}

static int wcd9335_set_interpolator_rate(struct snd_soc_dai *dai,
				       u32 sample_rate)
{
	int rate_val = 0;
	int i, ret;

	/* set mixing path rate */
	for (i = 0; i < ARRAY_SIZE(int_mix_sample_rate_val); i++) {
		if (sample_rate ==
				int_mix_sample_rate_val[i].sample_rate) {
			rate_val =
				int_mix_sample_rate_val[i].rate_val;
			break;
		}
	}
	if ((i == ARRAY_SIZE(int_mix_sample_rate_val)) ||
			(rate_val < 0))
		goto prim_rate;
	ret = wcd9335_set_mix_interpolator_rate(dai,
			(u8) rate_val, sample_rate);
prim_rate:
	/* set primary path sample rate */
	for (i = 0; i < ARRAY_SIZE(int_prim_sample_rate_val); i++) {
		if (sample_rate ==
				int_prim_sample_rate_val[i].sample_rate) {
			rate_val =
				int_prim_sample_rate_val[i].rate_val;
			break;
		}
	}
	if ((i == ARRAY_SIZE(int_prim_sample_rate_val)) ||
			(rate_val < 0))
		return -EINVAL;
	ret = wcd9335_set_prim_interpolator_rate(dai,
			(u8) rate_val, sample_rate);
	return ret;
}

static int wcd_slim_stream_enable(struct wcd_slim_data *wcd,
			struct wcd_slim_codec_dai_data *dai_data)
{
	struct slim_stream_config *cfg = &dai_data->sconfig;
	struct list_head *wcd_slim_ch_list = &dai_data->wcd_slim_ch_list;
	unsigned int rate = dai_data->rate;
	unsigned int bit_width = dai_data->bit_width;
	u8 ch_cnt = 0;
	u16 ch_h[SLIM_MAX_RX_PORTS] = {0};
	u8  payload = 0;
	u16 codec_port = 0;
	int ret;
//	struct slim_ch prop;
	struct wcd_slim_ch *rx;
	int size = ARRAY_SIZE(ch_h);
	int count = 0;
	int i =0;

	cfg->ch_count = 0;
	/* Configure slave interface device */
	list_for_each_entry(rx, wcd_slim_ch_list, list) {
		payload |= 1 << rx->shift;
	}
#if 0
	list_for_each_entry(rx, wcd_slim_ch_list, list) {
		codec_port = rx->port;
		pr_debug("%s: codec_port %d rx 0x%p, payload %d\n"
			 "wcd->rx_port_ch_reg_base0 0x%x\n"
			 "wcd->port_rx_cfg_reg_base 0x%x\n",
			 __func__, codec_port, rx, payload,
			 wcd->rx_port_ch_reg_base,
			wcd->port_rx_cfg_reg_base);

		/* look for the valid port range and chose the
		 * payload accordingly
		 */
		/* write to interface device */
		ret = regmap_write(wcd->if_regmap,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				wcd->rx_port_ch_reg_base, codec_port),
				payload);

		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				wcd->rx_port_ch_reg_base, codec_port),
				payload, ret);
			return ret;
		}
		/* configure the slave port for water mark and enable*/
		ret = regmap_write(wcd->if_regmap,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				wcd->port_rx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}
	}
#endif
	if (!dai_data->sruntime)
		pr_err("SLIM runtime is NULL...............................\n");

	slim_stream_enable(dai_data->sruntime);

	return 0;

}

static int wcd_slim_stream_prepare(struct wcd9335_priv *wcd,
	struct wcd_slim_codec_dai_data *dai_data)

{
	struct slim_stream_config *cfg = &dai_data->sconfig;
	struct list_head *wcd_slim_ch_list = &dai_data->wcd_slim_ch_list;
	unsigned int rate = dai_data->rate;
	unsigned int bit_width = dai_data->bit_width;
	struct wcd_slim_data *wcd_sd = wcd->slim_data;
	u8 ch_cnt = 0;
	u16 ch_h[SLIM_MAX_RX_PORTS] = {0};
	u8  payload = 0;
	u16 codec_port = 0;
	int ret;
	struct wcd_slim_ch *rx;
	int size = ARRAY_SIZE(ch_h);
	int count = 0;
	int i =0;

	cfg->ch_count = 0;
	/* Configure slave interface device */
	list_for_each_entry(rx, wcd_slim_ch_list, list) {
		payload |= 1 << rx->shift;
		cfg->port_mask |= 1 << (rx->shift + 0x10);
		cfg->ch_count++;

	}

	cfg->chs = kcalloc(cfg->ch_count, sizeof(unsigned int), GFP_KERNEL);
	if (!cfg->chs)
		return -ENOMEM;

	/* slim_define_ch api */
	cfg->prot = SLIM_HARD_ISO;
	cfg->rate = rate;
	cfg->bps = bit_width;
	i = 0;
	list_for_each_entry(rx, wcd_slim_ch_list, list) {
		codec_port = rx->port;
		cfg->chs[i++] = rx->ch_num;

		/* look for the valid port range and chose the
		 * payload accordingly
		 */
		/* write to interface device */
		ret = regmap_write(wcd_sd->if_regmap,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				wcd_sd->rx_port_ch_reg_base, codec_port),
				payload);

		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				wcd_sd->rx_port_ch_reg_base, codec_port),
				payload, ret);
			return ret;
		}
		/* configure the slave port for water mark and enable*/
		ret = regmap_write(wcd_sd->if_regmap,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				wcd_sd->port_rx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}
	}
	dai_data->sruntime = slim_stream_allocate(wcd->slim, cfg);
	slim_stream_prepare(dai_data->sruntime);

	return 0;

}

static int wcd9335_prepare(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{

	struct wcd9335_priv *wcd = snd_soc_component_get_drvdata(dai->component);
	struct wcd_slim_codec_dai_data *dai_data = &wcd->dai[dai->id];

	if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
	    test_bit(SB_CLK_GEAR, &wcd->status_mask)) {
		wcd9335_codec_vote_max_bw(dai->component, false);
		clear_bit(SB_CLK_GEAR, &wcd->status_mask);
	}
	wcd_slim_stream_enable(wcd->slim_data, &wcd->dai[dai->id]);

	return 0;
}

static int wcd9335_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct wcd9335_priv *wcd = snd_soc_component_get_drvdata(dai->component);
	struct snd_soc_component *component = dai->component;
	int rx_fs_rate = -EINVAL;
	int i2s_bit_mode = 0;
	int ret;

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		ret = wcd9335_set_interpolator_rate(dai, params_rate(params));
		if (ret) {
			dev_err(wcd->dev, "cannot set sample rate: %u\n",
				params_rate(params));
			return ret;
		}
		switch (params_width(params)) {
		case 16:
			wcd->dai[dai->id].bit_width = 16;
			i2s_bit_mode = 0x01;
			break;
		case 24:
			wcd->dai[dai->id].bit_width = 24;
			i2s_bit_mode = 0x00;
			break;
		}
		wcd->dai[dai->id].rate = params_rate(params);
		if (wcd->intf_type == WCD_INTERFACE_TYPE_I2C) {
			switch (params_rate(params)) {
			case 8000:
				rx_fs_rate = 0;
				break;
			case 16000:
				rx_fs_rate = 1;
				break;
			case 32000:
				rx_fs_rate = 2;
				break;
			case 48000:
				rx_fs_rate = 3;
				break;
			case 96000:
				rx_fs_rate = 4;
				break;
			case 192000:
				rx_fs_rate = 5;
				break;
			default:
				dev_err(wcd->dev,
				"%s: Invalid RX sample rate: %d\n",
				__func__, params_rate(params));
				return -EINVAL;
			};
			snd_soc_component_update_bits(component,
					WCD9335_DATA_HUB_DATA_HUB_RX_I2S_CTL,
					0x20, i2s_bit_mode << 5);
			snd_soc_component_update_bits(component,
					WCD9335_DATA_HUB_DATA_HUB_RX_I2S_CTL,
					0x1c, (rx_fs_rate << 2));
		}
		break;
	default:
		dev_err(wcd->dev, "Invalid stream type %d\n",
			substream->stream);
		return -EINVAL;
	};
	if (dai->id == AIF4_VIFEED)
		wcd->dai[dai->id].bit_width = 32;

	wcd_slim_stream_prepare(wcd, &wcd->dai[dai->id]);

	return 0;
}

static int wcd9335_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct wcd9335_priv *wcd = snd_soc_component_get_drvdata(dai->component);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is master */
		if (wcd->intf_type == WCD_INTERFACE_TYPE_I2C) {
			if (dai->id == AIF1_CAP)
				snd_soc_component_update_bits(dai->component,
					WCD9335_DATA_HUB_DATA_HUB_TX_I2S_CTL,
					0x2, 0);
			else if (dai->id == AIF1_PB)
				snd_soc_component_update_bits(dai->component,
					WCD9335_DATA_HUB_DATA_HUB_RX_I2S_CTL,
					0x2, 0);
		}
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CPU is slave */
		if (wcd->intf_type == WCD_INTERFACE_TYPE_I2C) {
			if (dai->id == AIF1_CAP)
				snd_soc_component_update_bits(dai->component,
					WCD9335_DATA_HUB_DATA_HUB_TX_I2S_CTL,
					0x2, 0x2);
			else if (dai->id == AIF1_PB)
				snd_soc_component_update_bits(dai->component,
					WCD9335_DATA_HUB_DATA_HUB_RX_I2S_CTL,
					0x2, 0x2);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct snd_soc_dai_ops wcd9335_dai_ops = {
	.hw_params = wcd9335_hw_params,
	.prepare = wcd9335_prepare,
	.set_fmt = wcd9335_set_dai_fmt,
	.set_channel_map = wcd9335_set_channel_map,
	.get_channel_map = wcd9335_get_channel_map,
};

static struct snd_soc_dai_driver wcd9335_slim_dai[] = {
	[0] = {
		.name = "wcd9335_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9335_RATES_MASK | WCD9335_FRAC_RATES_MASK,
			.formats = wcd9335_FORMATS_S16_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9335_dai_ops,
	},
	[1] = {
		.name = "wcd9335_rx2",
		.id = AIF2_PB,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = WCD9335_RATES_MASK | WCD9335_FRAC_RATES_MASK,
			.formats = wcd9335_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9335_dai_ops,
	},
	[2] = {
		.name = "wcd9335_rx3",
		.id = AIF3_PB,
		.playback = {
			.stream_name = "AIF3 Playback",
			.rates = WCD9335_RATES_MASK | WCD9335_FRAC_RATES_MASK,
			.formats = wcd9335_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9335_dai_ops,
	},
	[3] = {
		.name = "wcd9335_rx4",
		.id = AIF4_PB,
		.playback = {
			.stream_name = "AIF4 Playback",
			.rates = WCD9335_RATES_MASK | WCD9335_FRAC_RATES_MASK,
			.formats = wcd9335_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9335_dai_ops,
	},
};

static struct snd_soc_dai_driver wcd9335_i2s_dai[] = {
	[AIF1_PB] = {
		.name = "wcd9335_i2s_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9335_RATES_MASK,
			.formats = wcd9335_FORMATS_S16_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9335_dai_ops,
	},
	[AIF2_PB] = {
		.name = "wcd9335_i2s_rx2",
		.id = AIF2_PB,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = WCD9335_RATES_MASK,
			.formats = wcd9335_FORMATS_S16_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9335_dai_ops,
	},
};

static const struct wcd9335_reg_mask_val wcd9335_reg_update_reset_val_1_1[] = {
	{WCD9335_RCO_CTRL_2, 0xFF, 0x47},
	{WCD9335_FLYBACK_VNEG_DAC_CTRL_4, 0xFF, 0x60},
};

static const struct wcd9335_reg_mask_val wcd9335_codec_reg_init_val_1_1[] = {
	{WCD9335_FLYBACK_VNEG_DAC_CTRL_1, 0xFF, 0x65},
	{WCD9335_FLYBACK_VNEG_DAC_CTRL_2, 0xFF, 0x52},
	{WCD9335_FLYBACK_VNEG_DAC_CTRL_3, 0xFF, 0xAF},
	{WCD9335_FLYBACK_VNEG_DAC_CTRL_4, 0xFF, 0x60},
	{WCD9335_FLYBACK_VNEG_CTRL_3, 0xFF, 0xF4},
	{WCD9335_FLYBACK_VNEG_CTRL_9, 0xFF, 0x40},
	{WCD9335_FLYBACK_VNEG_CTRL_2, 0xFF, 0x4F},
	{WCD9335_FLYBACK_EN, 0xFF, 0x6E},
	{WCD9335_CDC_RX2_RX_PATH_SEC0, 0xF8, 0xF8},
	{WCD9335_CDC_RX1_RX_PATH_SEC0, 0xF8, 0xF8},
};

static const struct wcd9335_reg_mask_val wcd9335_codec_reg_init_val_1_0[] = {
	{WCD9335_FLYBACK_VNEG_CTRL_3, 0xFF, 0x54},
	{WCD9335_CDC_RX2_RX_PATH_SEC0, 0xFC, 0xFC},
	{WCD9335_CDC_RX1_RX_PATH_SEC0, 0xFC, 0xFC},
};

static const struct wcd9335_reg_mask_val wcd9335_codec_reg_init_val_2_0[] = {
	{WCD9335_RCO_CTRL_2, 0x0F, 0x08},
	{WCD9335_RX_BIAS_FLYB_MID_RST, 0xF0, 0x10},
	{WCD9335_FLYBACK_CTRL_1, 0x20, 0x20},
	{WCD9335_HPH_OCP_CTL, 0xFF, 0x5A},
	{WCD9335_HPH_L_TEST, 0x01, 0x01},
	{WCD9335_HPH_R_TEST, 0x01, 0x01},
	{WCD9335_CDC_BOOST0_BOOST_CFG1, 0x3F, 0x12},
	{WCD9335_CDC_BOOST0_BOOST_CFG2, 0x1C, 0x08},
	{WCD9335_CDC_COMPANDER7_CTL7, 0x1E, 0x18},
	{WCD9335_CDC_BOOST1_BOOST_CFG1, 0x3F, 0x12},
	{WCD9335_CDC_BOOST1_BOOST_CFG2, 0x1C, 0x08},
	{WCD9335_CDC_COMPANDER8_CTL7, 0x1E, 0x18},
	{WCD9335_CDC_TX0_TX_PATH_SEC7, 0xFF, 0x45},
	{WCD9335_CDC_RX0_RX_PATH_SEC0, 0xFC, 0xF4},
	{WCD9335_HPH_REFBUFF_LP_CTL, 0x08, 0x08},
	{WCD9335_HPH_REFBUFF_LP_CTL, 0x06, 0x02},
};

static const struct wcd9335_reg_mask_val wcd9335_codec_reg_defaults[] = {
	{WCD9335_CODEC_RPM_CLK_GATE, 0x03, 0x00},
	{WCD9335_CODEC_RPM_CLK_MCLK_CFG, 0x03, 0x01},
	{WCD9335_CODEC_RPM_CLK_MCLK_CFG, 0x04, 0x04},
};

static const struct wcd9335_reg_mask_val wcd9335_codec_reg_i2c_defaults[] = {
	{WCD9335_ANA_CLK_TOP, 0x20, 0x20},
	{WCD9335_CODEC_RPM_CLK_GATE, 0x03, 0x01},
	{WCD9335_CODEC_RPM_CLK_MCLK_CFG, 0x03, 0x00},
	{WCD9335_CODEC_RPM_CLK_MCLK_CFG, 0x05, 0x05},
	{WCD9335_DATA_HUB_DATA_HUB_RX0_INP_CFG, 0x01, 0x01},
	{WCD9335_DATA_HUB_DATA_HUB_RX1_INP_CFG, 0x01, 0x01},
	{WCD9335_DATA_HUB_DATA_HUB_RX2_INP_CFG, 0x01, 0x01},
	{WCD9335_DATA_HUB_DATA_HUB_RX3_INP_CFG, 0x01, 0x01},
	{WCD9335_DATA_HUB_DATA_HUB_TX_I2S_SD0_L_CFG, 0x05, 0x05},
	{WCD9335_DATA_HUB_DATA_HUB_TX_I2S_SD0_R_CFG, 0x05, 0x05},
	{WCD9335_DATA_HUB_DATA_HUB_TX_I2S_SD1_L_CFG, 0x05, 0x05},
	{WCD9335_DATA_HUB_DATA_HUB_TX_I2S_SD1_R_CFG, 0x05, 0x05},
};

static const struct wcd9335_reg_mask_val wcd9335_codec_reg_init_common_val[] = {
	/* Rbuckfly/R_EAR(32) */
	{WCD9335_CDC_CLSH_K2_MSB, 0x0F, 0x00},
	{WCD9335_CDC_CLSH_K2_LSB, 0xFF, 0x60},
	{WCD9335_CPE_SS_DMIC_CFG, 0x80, 0x00},
	{WCD9335_CDC_BOOST0_BOOST_CTL, 0x70, 0x50},
	{WCD9335_CDC_BOOST1_BOOST_CTL, 0x70, 0x50},
	{WCD9335_CDC_RX7_RX_PATH_CFG1, 0x08, 0x08},
	{WCD9335_CDC_RX8_RX_PATH_CFG1, 0x08, 0x08},
	{WCD9335_ANA_LO_1_2, 0x3C, 0X3C},
	{WCD9335_DIFF_LO_COM_SWCAP_REFBUF_FREQ, 0x70, 0x00},
	{WCD9335_DIFF_LO_COM_PA_FREQ, 0x70, 0x40},
	{WCD9335_SOC_MAD_AUDIO_CTL_2, 0x03, 0x03},
	{WCD9335_CDC_TOP_TOP_CFG1, 0x02, 0x02},
	{WCD9335_CDC_TOP_TOP_CFG1, 0x01, 0x01},
	{WCD9335_EAR_CMBUFF, 0x08, 0x00},
	{WCD9335_CDC_TX9_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_TX10_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_TX11_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_TX12_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_COMPANDER7_CTL3, 0x80, 0x80},
	{WCD9335_CDC_COMPANDER8_CTL3, 0x80, 0x80},
	{WCD9335_CDC_COMPANDER7_CTL7, 0x01, 0x01},
	{WCD9335_CDC_COMPANDER8_CTL7, 0x01, 0x01},
	{WCD9335_CDC_RX0_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX1_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX2_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX3_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX4_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX5_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX6_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX7_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX8_RX_PATH_CFG0, 0x01, 0x01},
	{WCD9335_CDC_RX0_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX1_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX2_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX3_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX4_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX5_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX6_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX7_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_CDC_RX8_RX_PATH_MIX_CFG, 0x01, 0x01},
	{WCD9335_VBADC_IBIAS_FE, 0x0C, 0x08},
};

static const struct wcd9335_reg_mask_val wcd9335_codec_reg_init_1_x_val[] = {
	/* Enable TX HPF Filter & Linear Phase */
	{WCD9335_CDC_TX0_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX1_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX2_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX3_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX4_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX5_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX6_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX7_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_TX8_TX_PATH_CFG0, 0x11, 0x11},
	{WCD9335_CDC_RX0_RX_PATH_SEC0, 0xF8, 0xF8},
	{WCD9335_CDC_RX0_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX1_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX2_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX3_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX4_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX5_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX6_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX7_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX8_RX_PATH_SEC1, 0x08, 0x08},
	{WCD9335_CDC_RX0_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX1_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX2_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX3_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX4_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX5_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX6_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX7_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_RX8_RX_PATH_MIX_SEC0, 0x08, 0x08},
	{WCD9335_CDC_TX0_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX1_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX2_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX3_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX4_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX5_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX6_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX7_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_TX8_TX_PATH_SEC2, 0x01, 0x01},
	{WCD9335_CDC_RX3_RX_PATH_SEC0, 0xF8, 0xF0},
	{WCD9335_CDC_RX4_RX_PATH_SEC0, 0xF8, 0xF0},
	{WCD9335_CDC_RX5_RX_PATH_SEC0, 0xF8, 0xF8},
	{WCD9335_CDC_RX6_RX_PATH_SEC0, 0xF8, 0xF8},
	{WCD9335_RX_OCP_COUNT, 0xFF, 0xFF},
	{WCD9335_HPH_OCP_CTL, 0xF0, 0x70},
	{WCD9335_CPE_SS_CPAR_CFG, 0xFF, 0x00},
	{WCD9335_FLYBACK_VNEG_CTRL_1, 0xFF, 0x63},
	{WCD9335_FLYBACK_VNEG_CTRL_4, 0xFF, 0x7F},
	{WCD9335_CLASSH_CTRL_VCL_1, 0xFF, 0x60},
	{WCD9335_CLASSH_CTRL_CCL_5, 0xFF, 0x40},
	{WCD9335_RX_TIMER_DIV, 0xFF, 0x32},
	{WCD9335_SE_LO_COM2, 0xFF, 0x01},
	{WCD9335_MBHC_ZDET_ANA_CTL, 0x0F, 0x07},
	{WCD9335_RX_BIAS_HPH_PA, 0xF0, 0x60},
	{WCD9335_HPH_RDAC_LDO_CTL, 0x88, 0x88},
	{WCD9335_HPH_L_EN, 0x20, 0x20},
	{WCD9335_HPH_R_EN, 0x20, 0x20},
	{WCD9335_DIFF_LO_CORE_OUT_PROG, 0xFC, 0xD8},
	{WCD9335_CDC_RX5_RX_PATH_SEC3, 0xBD, 0xBD},
	{WCD9335_CDC_RX6_RX_PATH_SEC3, 0xBD, 0xBD},
};

static int wcd9335_enable_efuse_sensing(struct snd_soc_component *component)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	_wcd9335_codec_enable_mclk(component, true);

	if (!WCD9335_IS_2_0(wcd->version))
		snd_soc_component_update_bits(component, WCD9335_CHIP_TIER_CTRL_EFUSE_CTL,
				    0x1E, 0x02);
	snd_soc_component_update_bits(component, WCD9335_CHIP_TIER_CTRL_EFUSE_CTL,
			    0x01, 0x01);
	/*
	 * 5ms sleep required after enabling efuse control
	 * before checking the status.
	 */
	usleep_range(5000, 5500);
	if (!(snd_soc_component_read32(component, WCD9335_CHIP_TIER_CTRL_EFUSE_STATUS) & 0x01))
		WARN(1, "%s: Efuse sense is not complete\n", __func__);

	if (WCD9335_IS_2_0(wcd->version)) {
		if (!(snd_soc_component_read32(component, WCD9335_CHIP_TIER_CTRL_EFUSE_VAL_OUT0) & 0x40))
			snd_soc_component_update_bits(component, WCD9335_HPH_R_ATEST, 0x04, 0x00);
		wcd9335_enable_sido_buck(component);
	}

	_wcd9335_codec_enable_mclk(component, false);

	return 0;
}

static void wcd9335_codec_init(struct snd_soc_component *component)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(wcd9335_codec_reg_init_common_val); i++)
		snd_soc_component_update_bits(component,
				wcd9335_codec_reg_init_common_val[i].reg,
				wcd9335_codec_reg_init_common_val[i].mask,
				wcd9335_codec_reg_init_common_val[i].val);

	if (WCD9335_IS_1_1(wcd->version) ||
	    WCD9335_IS_1_0(wcd->version))
		for (i = 0; i < ARRAY_SIZE(wcd9335_codec_reg_init_1_x_val); i++)
			snd_soc_component_update_bits(component,
					wcd9335_codec_reg_init_1_x_val[i].reg,
					wcd9335_codec_reg_init_1_x_val[i].mask,
					wcd9335_codec_reg_init_1_x_val[i].val);

	if (WCD9335_IS_1_1(wcd->version)) {
		for (i = 0; i < ARRAY_SIZE(wcd9335_codec_reg_init_val_1_1); i++)
			snd_soc_component_update_bits(component,
					wcd9335_codec_reg_init_val_1_1[i].reg,
					wcd9335_codec_reg_init_val_1_1[i].mask,
					wcd9335_codec_reg_init_val_1_1[i].val);
	} else if (WCD9335_IS_1_0(wcd->version)) {
		for (i = 0; i < ARRAY_SIZE(wcd9335_codec_reg_init_val_1_0); i++)
			snd_soc_component_update_bits(component,
					wcd9335_codec_reg_init_val_1_0[i].reg,
					wcd9335_codec_reg_init_val_1_0[i].mask,
					wcd9335_codec_reg_init_val_1_0[i].val);
	} else if (WCD9335_IS_2_0(wcd->version)) {
		for (i = 0; i < ARRAY_SIZE(wcd9335_codec_reg_init_val_2_0); i++)
			snd_soc_component_update_bits(component,
					wcd9335_codec_reg_init_val_2_0[i].reg,
					wcd9335_codec_reg_init_val_2_0[i].mask,
					wcd9335_codec_reg_init_val_2_0[i].val);
	}

	wcd9335_enable_efuse_sensing(component);
}

static void wcd9335_update_reg_defaults(struct wcd9335_priv *wcd)
{
	u32 i;
	for (i = 0; i < ARRAY_SIZE(wcd9335_codec_reg_defaults); i++)
		regmap_update_bits(wcd->regmap,
					wcd9335_codec_reg_defaults[i].reg,
					wcd9335_codec_reg_defaults[i].mask,
					wcd9335_codec_reg_defaults[i].val);

	if (wcd->intf_type == WCD_INTERFACE_TYPE_I2C)
		for (i = 0; i < ARRAY_SIZE(wcd9335_codec_reg_i2c_defaults); i++)
			regmap_update_bits(wcd->regmap,
				wcd9335_codec_reg_i2c_defaults[i].reg,
				wcd9335_codec_reg_i2c_defaults[i].mask,
				wcd9335_codec_reg_i2c_defaults[i].val);

	return;
}

static irqreturn_t wcd9335_slimbus_irq(int irq, void *data)
{
	struct wcd9335_priv *wcd = data;
	unsigned long status = 0;
	int i, j, port_id, k;
	u32 bit;
	unsigned int val, int_val = 0;
	bool tx, cleared;
	unsigned short reg = 0;

	for (i = WCD9335_SLIM_PGD_PORT_INT_STATUS_RX_0, j = 0;
	     i <= WCD9335_SLIM_PGD_PORT_INT_STATUS_TX_1; i++, j++) {
		regmap_read(wcd->if_regmap, i, &val);
		status |= ((u32)val << (8 * j));
	}

	for_each_set_bit(j, &status, 32) {
		tx = (j >= 16 ? true : false);
		port_id = (tx ? j - 16 : j);
		regmap_read(wcd->if_regmap,
				WCD9335_SLIM_PGD_PORT_INT_RX_SOURCE0 + j, &val);
		if (val) {
			if (!tx)
				reg = WCD9335_SLIM_PGD_PORT_INT_EN0 +
					(port_id / 8);
			else
				reg = WCD9335_SLIM_PGD_PORT_INT_TX_EN0 +
					(port_id / 8);
			regmap_read(
				wcd->if_regmap, reg, &int_val);
			/*
			 * Ignore interrupts for ports for which the
			 * interrupts are not specifically enabled.
			 */
			if (!(int_val & (1 << (port_id % 8))))
				continue;
		}
		if (val & WCD9335_SLIM_IRQ_OVERFLOW)
			pr_err_ratelimited(
			   "%s: overflow error on %s port %d, value %x\n",
			   __func__, (tx ? "TX" : "RX"), port_id, val);
		if (val & WCD9335_SLIM_IRQ_UNDERFLOW)
			pr_err_ratelimited(
			   "%s: underflow error on %s port %d, value %x\n",
			   __func__, (tx ? "TX" : "RX"), port_id, val);
		if ((val & WCD9335_SLIM_IRQ_OVERFLOW) ||
			(val & WCD9335_SLIM_IRQ_UNDERFLOW)) {
			if (!tx)
				reg = WCD9335_SLIM_PGD_PORT_INT_EN0 +
					(port_id / 8);
			else
				reg = WCD9335_SLIM_PGD_PORT_INT_TX_EN0 +
					(port_id / 8);
			regmap_read(
				wcd->if_regmap, reg, &int_val);
			if (int_val & (1 << (port_id % 8))) {
				int_val = int_val ^ (1 << (port_id % 8));
				regmap_write(wcd->if_regmap,
					reg, int_val);
			}
		}
		if (val & WCD9335_SLIM_IRQ_PORT_CLOSED) {
			/*
			 * INT SOURCE register starts from RX to TX
			 * but port number in the ch_mask is in opposite way
			 */
			bit = (tx ? j - 16 : j + 16);
			for (k = 0, cleared = false; k < NUM_CODEC_DAIS; k++) {
				if (test_and_clear_bit(bit,
						       &wcd->dai[k].ch_mask)) {
					cleared = true;
					if (!wcd->dai[k].ch_mask)
						wake_up(&wcd->dai[k].dai_wait);
					/*
					 * There are cases when multiple DAIs
					 * might be using the same slimbus
					 * channel. Hence don't break here.
					 */
				}
			}
			WARN(!cleared,
			     "Couldn't find slimbus %s port %d for closing\n",
			     (tx ? "TX" : "RX"), port_id);
		}
		regmap_write(wcd->if_regmap,
					    WCD9335_SLIM_PGD_PORT_INT_CLR_RX_0 +
					    (j / 8),
					    1 << (j % 8));
	}

	return IRQ_HANDLED;
}

static inline int wcd9335_request_irq(struct wcd9335_priv *w, int irq,
                                     irq_handler_t handler, const char *name,
                                     void *data)
{
        return request_threaded_irq(regmap_irq_get_virq(w->irq_data, irq),
                                    NULL, handler, IRQF_TRIGGER_RISING, name,
                                    data);
}

static int wcd9335_setup_irqs(struct wcd9335_priv *wcd)
{
	int i, ret = 0;

	ret = wcd9335_request_irq(wcd, WCD9335_IRQ_SLIMBUS,
				  wcd9335_slimbus_irq, "SLIMBUS Slave", wcd);
	if (ret) {
		dev_err(wcd->dev, "Failed to request SLIMBUS irq \n");
		return ret;
	}

	for (i = 0; i < WCD9335_SLIM_NUM_PORT_REG; i++)
		regmap_write(wcd->if_regmap, WCD9335_SLIM_PGD_PORT_INT_EN0 + i,
			     0xFF);

	return ret;
}

static void wcd9335_cleanup_irqs(struct wcd9335_priv *w)
{
        free_irq(regmap_irq_get_virq(w->irq_data, WCD9335_IRQ_SLIMBUS), w);
}

static int wcd9335_codec_slim_reserve_bw(struct snd_soc_component *component,
		u32 bw_ops, bool commit)
{
//truct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	return 0;
//FIXME	return slim_reservemsg_bw(wcd->slim, bw_ops, commit);
}

static int wcd9335_codec_vote_max_bw(struct snd_soc_component *component,
			bool vote)
{
	return wcd9335_codec_slim_reserve_bw(component,
			vote ? SLIM_BW_CLK_GEAR_9 : SLIM_BW_UNVOTE, true);
}

static int wcd9335_codec_set_sysclk(struct snd_soc_component *comp,
				    int clk_id, int source,
				    unsigned int freq, int dir)
{
//	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);

	return 0;
}

static int wcd9335_codec_probe(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	struct wcd9335_priv *wcd = dev_get_drvdata(component->dev);
	int i;

	snd_soc_component_init_regmap(component, wcd->regmap);
	wcd->clsh_d.codec_version = wcd->version;
	/* Class-H Init*/
	wcd_clsh_init(&wcd->clsh_d);
	/* Default HPH Mode to Class-H HiFi */
	wcd->hph_mode = CLS_H_HIFI;
	wcd->component = component;

	if (wcd->mclk_rate == wcd9335_MCLK_CLK_12P288MHZ)
		snd_soc_component_update_bits(component, WCD9335_CODEC_RPM_CLK_MCLK_CFG,
				    0x03, 0x00);
	else if (wcd->mclk_rate == wcd9335_MCLK_CLK_9P6MHZ)
		snd_soc_component_update_bits(component, WCD9335_CODEC_RPM_CLK_MCLK_CFG,
				    0x03, 0x01);

	wcd9335_codec_init(component);

	if (wcd->intf_type == WCD_INTERFACE_TYPE_I2C) {
		snd_soc_dapm_new_controls(dapm, wcd9335_dapm_i2s_widgets,
			ARRAY_SIZE(wcd9335_dapm_i2s_widgets));
		snd_soc_dapm_add_routes(dapm, audio_i2s_map,
			ARRAY_SIZE(audio_i2s_map));
		for (i = 0; i < ARRAY_SIZE(wcd9335_i2s_dai); i++) {
			INIT_LIST_HEAD(&wcd->dai[i].wcd_slim_ch_list);
			init_waitqueue_head(&wcd->dai[i].dai_wait);
		}
	} else if (wcd->intf_type == WCD_INTERFACE_TYPE_SLIMBUS) {
		for (i = 0; i < NUM_CODEC_DAIS; i++) {
			INIT_LIST_HEAD(&wcd->dai[i].wcd_slim_ch_list);
			init_waitqueue_head(&wcd->dai[i].dai_wait);
		}
	}

	return wcd9335_setup_irqs(wcd);
}

static void wcd9335_codec_remove(struct snd_soc_component *comp)
{
	struct wcd9335_priv *wcd = dev_get_drvdata(comp->dev);

	wcd9335_cleanup_irqs(wcd);
}

static struct snd_soc_component_driver wcd9335_component_drv = {
	.probe = wcd9335_codec_probe,
	.remove = wcd9335_codec_remove,
	.set_sysclk = wcd9335_codec_set_sysclk,
	.controls = wcd9335_snd_controls,
	.num_controls = ARRAY_SIZE(wcd9335_snd_controls),
	.dapm_widgets = wcd9335_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wcd9335_dapm_widgets),
	.dapm_routes = wcd9335_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wcd9335_audio_map),
};

int wcd9335_probe(struct platform_device *pdev)
{
	struct wcd9335 *control = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	int ret = 0;
	struct wcd9335_priv *wcd;
	int clk1_gpio;

	//FIXME
//	clk1_gpio = of_get_named_gpio(control->dev->of_node, "qcom,clk1-gpio", 0);
//	gpio_request(clk1_gpio, "CLK1");
//	gpio_direction_output(clk1_gpio, 0);

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return -ENOMEM;

	wcd->slim_data = &control->slim_data;

	wcd->slim_data->rx_chs = devm_kzalloc(dev, sizeof(wcd9335_rx_chs), GFP_KERNEL);
	if (!wcd->slim_data->rx_chs)
		return -ENOMEM;

	memcpy(wcd->slim_data->rx_chs, wcd9335_rx_chs, sizeof(wcd9335_rx_chs));

	dev_set_drvdata(dev, wcd);

	wcd->mclk_rate = control->mclk_rate;
	wcd->regmap = control->regmap;
	wcd->if_regmap = control->ifd_regmap;
	wcd->slim = control->slim;
	wcd->slim_slave = control->slim_slave;
	wcd->irq_data = control->irq_data;
	wcd->version = control->version;
	wcd->intf_type = control->intf_type;
	wcd->dev = dev;
	wcd->ext_clk = control->ext_clk;
	wcd->native_clk = control->native_clk;

	mutex_init(&wcd->sido_lock);

	mutex_init(&wcd->codec_bg_clk_lock);
	mutex_init(&wcd->master_bias_lock);
	wcd->sido_input_src = SIDO_SOURCE_INTERNAL;

	wcd->sido_voltage = SIDO_VOLTAGE_NOMINAL_MV;
	set_bit(AUDIO_NOMINAL, &wcd->status_mask);

	if (wcd->intf_type == WCD_INTERFACE_TYPE_SLIMBUS)
		ret = snd_soc_register_component(dev, &wcd9335_component_drv,
					     wcd9335_slim_dai,
					     ARRAY_SIZE(wcd9335_slim_dai));
	else if (wcd->intf_type == WCD_INTERFACE_TYPE_I2C)
		ret = snd_soc_register_component(dev, &wcd9335_component_drv,
					     wcd9335_i2s_dai,
					     ARRAY_SIZE(wcd9335_i2s_dai));
	else
		ret = -EINVAL;

	if (ret) {
		dev_err(dev, "Codec registration failed\n");
		return ret;
	}

	/* Update codec register default values */
	//FIXME Need to go into the regmap defaults.
	wcd9335_update_reg_defaults(wcd);

	return ret;

}

static int wcd9335_remove(struct platform_device *pdev)
{
	struct wcd9335_priv *wcd;

	wcd = platform_get_drvdata(pdev);

	clk_put(wcd->ext_clk);
	if (wcd->native_clk)
		clk_put(wcd->native_clk);
	devm_kfree(&pdev->dev, wcd);
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id wcd9335_of_match[] = {
	{ .compatible = "qcom,wcd9335", } ,
	{}
};
static struct platform_driver wcd9335_codec_driver = {
	.probe = wcd9335_probe,
	.remove = wcd9335_remove,
	.driver = {
		.name = "wcd9335_codec",
		.owner = THIS_MODULE,
		.of_match_table = wcd9335_of_match,
	},
};

module_platform_driver(wcd9335_codec_driver);
MODULE_DESCRIPTION("WCD9335 Codec driver");
MODULE_LICENSE("GPL v2");
