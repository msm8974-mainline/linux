// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
// Copyright (c) 2017-2018, Linaro Limited

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/jack.h>
#include <linux/kernel.h>
#include <linux/slimbus.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/info.h>
#include "wcd9320.h"
#include "wcd-clsh.h"

#define WCD9320_RATES_MASK (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			    SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)
#define WCD9320_FORMATS_S16_S24_LE (SNDRV_PCM_FMTBIT_S16_LE | \
				  SNDRV_PCM_FMTBIT_S24_LE)

#define WCD9320_NUM_INTERPOLATORS 7
#define WCD9320_RX_START 16

#define DAPM_MICBIAS2_EXTERNAL_STANDALONE "MIC BIAS2 External Standalone"

#define WCD9320_SLIM_RX_CH(p) \
	{.port = p + WCD9320_RX_START, .shift = p,}

#define WCD9320_SLIM_TX_CH(p) \
	{.port = p, .shift = p,}

enum {
	WCD9320_RX1 = 0,
	WCD9320_RX2,
	WCD9320_RX3,
	WCD9320_RX4,
	WCD9320_RX5,
	WCD9320_RX6,
	WCD9320_RX7,
	WCD9320_RX8,
	WCD9320_RX9,
	WCD9320_RX10,
	WCD9320_RX11,
	WCD9320_RX12,
	WCD9320_RX13,
	WCD9320_RX_MAX,
};

enum {
	WCD9320_TX1 = 0,
	WCD9320_TX2,
	WCD9320_TX3,
	WCD9320_TX4,
	WCD9320_TX5,
	WCD9320_TX6,
	WCD9320_TX7,
	WCD9320_TX8,
	WCD9320_TX9,
	WCD9320_TX10,
	WCD9320_TX11,
	WCD9320_TX12,
	WCD9320_TX13,
	WCD9320_TX14,
	WCD9320_TX15,
	WCD9320_TX16,
	WCD9320_TX_MAX,
};

enum {
	AIF1_PB = 0,
	AIF1_CAP,
	AIF2_PB,
	AIF2_CAP,
	AIF3_PB,
	AIF3_CAP,
	AIF4_VIFEED,
	AIF4_MAD_TX,
	NUM_CODEC_DAIS,
};

enum {
	COMPANDER_0,
	COMPANDER_1,
	COMPANDER_2,
	COMPANDER_MAX,
};

enum {
	COMPANDER_FS_8KHZ = 0,
	COMPANDER_FS_16KHZ,
	COMPANDER_FS_32KHZ,
	COMPANDER_FS_48KHZ,
	COMPANDER_FS_96KHZ,
	COMPANDER_FS_192KHZ,
	COMPANDER_FS_MAX,
};

enum {
	RX_MIX1_INP_SEL_ZERO = 0,
	RX_MIX1_INP_SEL_SRC1,
	RX_MIX1_INP_SEL_SRC2,
	RX_MIX1_INP_SEL_IIR1,
	RX_MIX1_INP_SEL_IIR2,
	RX_MIX1_INP_SEL_RX1,
	RX_MIX1_INP_SEL_RX2,
	RX_MIX1_INP_SEL_RX3,
	RX_MIX1_INP_SEL_RX4,
	RX_MIX1_INP_SEL_RX5,
	RX_MIX1_INP_SEL_RX6,
	RX_MIX1_INP_SEL_RX7,
	RX_MIX1_INP_SEL_AUXRX,
};

struct wcd9320_slim_ch {
	u32 ch_num;
	u16 port;
	u16 shift;
	struct list_head list;
};

struct wcd_slim_codec_dai_data {
	struct list_head slim_ch_list;
	struct slim_stream_config sconfig;
	struct slim_stream_runtime *sruntime;
};

struct wcd9320_codec {
	struct device *dev;
	struct clk *mclk;
	struct clk *native_clk;
	u32 mclk_rate;
	u8 version;

	struct slim_device *slim;
	struct slim_device *slim_ifc_dev;
	struct regmap *regmap;
	struct regmap *if_regmap;
	struct regmap_irq_chip_data *irq_data;

	struct wcd9320_slim_ch rx_chs[WCD9320_RX_MAX];
	struct wcd9320_slim_ch tx_chs[WCD9320_TX_MAX];
	u32 num_rx_port;
	u32 num_tx_port;

	u32 anc_slot;
	bool anc_func;

	//

	struct snd_soc_jack *jack;
	bool hphl_jack_type_normally_open;
	bool gnd_jack_type_normally_open;
	bool	mbhc_btn_enabled;
	int	mbhc_btn0_released;
	bool	detect_accessory_type;
	int	accessory_type;
	/* Voltage threshold for button detection */
	//u32 vref_btn[WCD9320_MBHC_MAX_BUTTONS];

	int sido_input_src;
	//enum wcd9320_sido_voltage sido_voltage;

	struct wcd_slim_codec_dai_data dai[NUM_CODEC_DAIS];
	struct snd_soc_component *component;

	int master_bias_users;
	int clk_mclk_users;
	int clk_rco_users;
	int sido_ccl_cnt;
	//enum wcd_clock_type clk_type;

	struct wcd_clsh_ctrl *clsh_ctrl;
	u32 hph_mode;
	int prim_int_users[WCD9320_NUM_INTERPOLATORS];

	int comp_enabled[COMPANDER_MAX];
	u32 comp_fs[COMPANDER_MAX];

	int intr1;
	int reset_gpio;
	struct regulator_bulk_data supplies[WCD9320_MAX_SUPPLY];

	unsigned int rx_port_value;
	unsigned int tx_port_value;
	int hph_l_gain;
	int hph_r_gain;
	u32 rx_bias_count;

	/*TX*/
	//int micb_ref[WCD9320_MAX_MICBIAS];
	//int pullup_ref[WCD9320_MAX_MICBIAS];

	int dmic_0_1_clk_cnt;
	int dmic_2_3_clk_cnt;
	int dmic_4_5_clk_cnt;
	int dmic_sample_rate;
	int mad_dmic_sample_rate;

	int native_clk_users;
};

struct wcd9320_irq {
	int irq;
	irqreturn_t (*handler)(int irq, void *data);
	char *name;
};

static const struct wcd9320_slim_ch wcd9320_tx_chs[WCD9320_TX_MAX] = {
	WCD9320_SLIM_TX_CH(0),
	WCD9320_SLIM_TX_CH(1),
	WCD9320_SLIM_TX_CH(2),
	WCD9320_SLIM_TX_CH(3),
	WCD9320_SLIM_TX_CH(4),
	WCD9320_SLIM_TX_CH(5),
	WCD9320_SLIM_TX_CH(6),
	WCD9320_SLIM_TX_CH(7),
	WCD9320_SLIM_TX_CH(8),
	WCD9320_SLIM_TX_CH(9),
	WCD9320_SLIM_TX_CH(10),
	WCD9320_SLIM_TX_CH(11),
	WCD9320_SLIM_TX_CH(12),
	WCD9320_SLIM_TX_CH(13),
	WCD9320_SLIM_TX_CH(14),
	WCD9320_SLIM_TX_CH(15),
};

static const struct wcd9320_slim_ch wcd9320_rx_chs[WCD9320_RX_MAX] = {
	WCD9320_SLIM_RX_CH(0),	 /* 16 */
	WCD9320_SLIM_RX_CH(1),	 /* 17 */
	WCD9320_SLIM_RX_CH(2),
	WCD9320_SLIM_RX_CH(3),
	WCD9320_SLIM_RX_CH(4),
	WCD9320_SLIM_RX_CH(5),
	WCD9320_SLIM_RX_CH(6),
	WCD9320_SLIM_RX_CH(7),
	WCD9320_SLIM_RX_CH(8),
	WCD9320_SLIM_RX_CH(9),
	WCD9320_SLIM_RX_CH(10),
	WCD9320_SLIM_RX_CH(11),
	WCD9320_SLIM_RX_CH(12),
};

struct wcd9320_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

static const struct wcd9320_reg_mask_val wcd9320_codec_reg_init[] = {
	/* set MCLk to 9.6 */
	{WCD9320_CHIP_CTL, 0xff, 0x02},
	{WCD9320_CDC_CLK_POWER_CTL, 0xff, 0x03},

	/* EAR PA deafults  */
	{WCD9320_RX_EAR_CMBUFF, 0xff, 0x05},

	/* RX deafults */
	{WCD9320_CDC_RX1_B5_CTL, 0xff, 0x78},
	{WCD9320_CDC_RX2_B5_CTL, 0xff, 0x78},
	{WCD9320_CDC_RX3_B5_CTL, 0xff, 0x78},
	{WCD9320_CDC_RX4_B5_CTL, 0xff, 0x78},
	{WCD9320_CDC_RX5_B5_CTL, 0xff, 0x78},
	{WCD9320_CDC_RX6_B5_CTL, 0xff, 0x78},
	{WCD9320_CDC_RX7_B5_CTL, 0xff, 0x78},

	/* RX1 and RX2 defaults */
	{WCD9320_CDC_RX1_B6_CTL, 0xff, 0xA0},
	{WCD9320_CDC_RX2_B6_CTL, 0xff, 0xA0},

	/* RX3 to RX7 defaults */
	{WCD9320_CDC_RX3_B6_CTL, 0xff, 0x80},
	{WCD9320_CDC_RX4_B6_CTL, 0xff, 0x80},
	{WCD9320_CDC_RX5_B6_CTL, 0xff, 0x80},
	{WCD9320_CDC_RX6_B6_CTL, 0xff, 0x80},
	{WCD9320_CDC_RX7_B6_CTL, 0xff, 0x80},

	/* MAD registers */
	{WCD9320_MAD_ANA_CTRL, 0xff, 0xF1},
	{WCD9320_CDC_MAD_MAIN_CTL_1, 0xff, 0x00},
	{WCD9320_CDC_MAD_MAIN_CTL_2, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_CTL_1, 0xff, 0x00},
	/* Set SAMPLE_TX_EN bit */
	{WCD9320_CDC_MAD_AUDIO_CTL_2, 0xff, 0x03},
	{WCD9320_CDC_MAD_AUDIO_CTL_3, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_CTL_4, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_CTL_5, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_CTL_6, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_CTL_7, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_CTL_8, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_IIR_CTL_PTR, 0xff, 0x00},
	{WCD9320_CDC_MAD_AUDIO_IIR_CTL_VAL, 0xff, 0x40},
	{WCD9320_CDC_DEBUG_B7_CTL, 0xff, 0x00},
	{WCD9320_CDC_CLK_OTHR_RESET_B1_CTL, 0xff, 0x00},
	{WCD9320_CDC_CLK_OTHR_CTL, 0xff, 0x00},
	{WCD9320_CDC_CONN_MAD, 0xff, 0x01},

#if 0 // TAIKO_V1
	/*
	 * The following only need to be written for Taiko 1.0 parts.
	 * Taiko 2.0 will have appropriate defaults for these registers.
	 */

	/* BUCK default */
	{WCD9320_BUCK_CTRL_CCL_4, 0xff, 0x50},

	/* Required defaults for class H operation */
	{WCD9320_RX_HPH_CHOP_CTL, 0xff, 0xF4},
	{WCD9320_BIAS_CURR_CTL_2, 0xff, 0x08},
	{WCD9320_BUCK_CTRL_CCL_1, 0xff, 0x5B},
	{WCD9320_BUCK_CTRL_CCL_3, 0xff, 0x60},

	/* Choose max non-overlap time for NCP */
	{WCD9320_NCP_CLK, 0xff, 0xFC},
	/* Use 25mV/50mV for deltap/m to reduce ripple */
	{WCD9320_BUCK_CTRL_VCL_1, 0xff, 0x08},
	/*
	 * Set DISABLE_MODE_SEL<1:0> to 0b10 (disable PWM in auto mode).
	 * Note that the other bits of this register will be changed during
	 * Rx PA bring up.
	 */
	{WCD9320_BUCK_MODE_3, 0xff, 0xCE},
	/* Reduce HPH DAC bias to 70% */
	{WCD9320_RX_HPH_BIAS_PA, 0xff, 0x7A},
	/*Reduce EAR DAC bias to 70% */
	{WCD9320_RX_EAR_BIAS_PA, 0xff, 0x76},
	/* Reduce LINE DAC bias to 70% */
	{WCD9320_RX_LINE_BIAS_PA, 0xff, 0x78},

	/*
	 * There is a diode to pull down the micbias while doing
	 * insertion detection.  This diode can cause leakage.
	 * Set bit 0 to 1 to prevent leakage.
	 * Setting this bit of micbias 2 prevents leakage for all other micbias.
	 */
	{WCD9320_MICB_2_MBHC, 0xff, 0x41},

	/* Disable TX7 internal biasing path which can cause leakage */
	{WCD9320_TX_SUP_SWITCH_CTRL_1, 0xff, 0xBF},

	/* Close leakage on the spkdrv */
	{WCD9320_SPKR_DRV_DBG_PWRSTG, 0xff, 0x24},
#else
	{WCD9320_CDC_TX_1_GAIN, 0xff, 0x2},
	{WCD9320_CDC_TX_2_GAIN, 0xff, 0x2},
	{WCD9320_CDC_TX_1_2_ADC_IB, 0xff, 0x44},
	{WCD9320_CDC_TX_3_GAIN, 0xff, 0x2},
	{WCD9320_CDC_TX_4_GAIN, 0xff, 0x2},
	{WCD9320_CDC_TX_3_4_ADC_IB, 0xff, 0x44},
	{WCD9320_CDC_TX_5_GAIN, 0xff, 0x2},
	{WCD9320_CDC_TX_6_GAIN, 0xff, 0x2},
	{WCD9320_CDC_TX_5_6_ADC_IB, 0xff, 0x44},
	{WCD9320_BUCK_MODE_3, 0xff, 0xCE},
	{WCD9320_BUCK_CTRL_VCL_1, 0xff, 0x8},
	{WCD9320_BUCK_CTRL_CCL_4, 0xff, 0x51},
	{WCD9320_NCP_DTEST, 0xff, 0x10},
	{WCD9320_RX_HPH_CHOP_CTL, 0xff, 0xA4},
	{WCD9320_RX_HPH_BIAS_PA, 0xff, 0x7A},
	{WCD9320_RX_HPH_OCP_CTL, 0xff, 0x69},
	{WCD9320_RX_HPH_CNP_WG_CTL, 0xff, 0xDA},
	{WCD9320_RX_HPH_CNP_WG_TIME, 0xff, 0x15},
	{WCD9320_RX_EAR_BIAS_PA, 0xff, 0x76},
	{WCD9320_RX_EAR_CNP, 0xff, 0xC0},
	{WCD9320_RX_LINE_BIAS_PA, 0xff, 0x78},
	{WCD9320_RX_LINE_1_TEST, 0xff, 0x2},
	{WCD9320_RX_LINE_2_TEST, 0xff, 0x2},
	{WCD9320_RX_LINE_3_TEST, 0xff, 0x2},
	{WCD9320_RX_LINE_4_TEST, 0xff, 0x2},
	{WCD9320_SPKR_DRV_OCP_CTL, 0xff, 0x97},
	{WCD9320_SPKR_DRV_CLIP_DET, 0xff, 0x1},
	{WCD9320_SPKR_DRV_IEC, 0xff, 0x0},
	{WCD9320_CDC_TX1_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX2_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX3_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX4_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX5_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX6_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX7_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX8_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX9_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_TX10_MUX_CTL, 0xff, 0x48},
	{WCD9320_CDC_RX1_B4_CTL, 0xff, 0x8},
	{WCD9320_CDC_RX2_B4_CTL, 0xff, 0x8},
	{WCD9320_CDC_RX3_B4_CTL, 0xff, 0x8},
	{WCD9320_CDC_RX4_B4_CTL, 0xff, 0x8},
	{WCD9320_CDC_RX5_B4_CTL, 0xff, 0x8},
	{WCD9320_CDC_RX6_B4_CTL, 0xff, 0x8},
	{WCD9320_CDC_RX7_B4_CTL, 0xff, 0x8},
	{WCD9320_CDC_VBAT_GAIN_UPD_MON, 0xff, 0x0},
	{WCD9320_CDC_PA_RAMP_B1_CTL, 0xff, 0x0},
	{WCD9320_CDC_PA_RAMP_B2_CTL, 0xff, 0x0},
	{WCD9320_CDC_PA_RAMP_B3_CTL, 0xff, 0x0},
	{WCD9320_CDC_PA_RAMP_B4_CTL, 0xff, 0x0},
	{WCD9320_CDC_SPKR_CLIPDET_B1_CTL, 0xff, 0x0},
	{WCD9320_CDC_COMP0_B4_CTL, 0xff, 0x37},
	{WCD9320_CDC_COMP0_B5_CTL, 0xff, 0x7f},
#endif
};

static int slim_rx_mux_get(struct snd_kcontrol *kc,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(dapm->dev);

	ucontrol->value.enumerated.item[0] = wcd->rx_port_value;

	return 0;
}

static int slim_rx_mux_put(struct snd_kcontrol *kc,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(w->dapm->dev);
	struct soc_enum *e = (struct soc_enum *)kc->private_value;
	struct snd_soc_dapm_update *update = NULL;
	u32 port_id = w->shift;

	wcd->rx_port_value = ucontrol->value.enumerated.item[0];

	switch (wcd->rx_port_value) {
	case 0:
		list_del_init(&wcd->rx_chs[port_id].list);
		break;
	case 1:
		list_add_tail(&wcd->rx_chs[port_id].list,
			      &wcd->dai[AIF1_PB].slim_ch_list);
		break;
	case 2:
		list_add_tail(&wcd->rx_chs[port_id].list,
			      &wcd->dai[AIF2_PB].slim_ch_list);
		break;
	case 3:
		list_add_tail(&wcd->rx_chs[port_id].list,
			      &wcd->dai[AIF3_PB].slim_ch_list);
		break;
	default:
		dev_err(wcd->dev, "Unknown AIF %d\n", wcd->rx_port_value);
		goto err;
	}

	snd_soc_dapm_mux_update_power(w->dapm, kc, wcd->rx_port_value,
				      e, update);

	return 0;
err:
	return -EINVAL;
}

static int slim_tx_mixer_get(struct snd_kcontrol *kc,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(dapm->dev);

	ucontrol->value.integer.value[0] = wcd->tx_port_value;

	return 0;
}

static int slim_tx_mixer_put(struct snd_kcontrol *kc,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_soc_dapm_kcontrol_widget(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(widget->dapm->dev);
	struct snd_soc_dapm_update *update = NULL;
	struct soc_mixer_control *mixer =
			(struct soc_mixer_control *)kc->private_value;
	int enable = ucontrol->value.integer.value[0];
	int dai_id = widget->shift;
	int port_id = mixer->shift;

	switch (dai_id) {
	case AIF1_CAP:
	case AIF2_CAP:
	case AIF3_CAP:
		/* only add to the list if value not set */
		if (enable && !(wcd->tx_port_value & BIT(port_id))) {
			wcd->tx_port_value |= BIT(port_id);
			list_add_tail(&wcd->tx_chs[port_id].list,
					&wcd->dai[dai_id].slim_ch_list);
		} else if (!enable && (wcd->tx_port_value & BIT(port_id))) {
			wcd->tx_port_value &= ~BIT(port_id);
			list_del_init(&wcd->tx_chs[port_id].list);
		}
		break;
	default:
		dev_err(wcd->dev, "Unknown AIF %d\n", dai_id);
		return -EINVAL;
	}

	snd_soc_dapm_mixer_update_power(widget->dapm, kc, enable, update);

	return 0;
}

static int wcd9320_put_dec_enum(struct snd_kcontrol *kc,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kc);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	unsigned int val, reg, sel;

	val = ucontrol->value.enumerated.item[0];

	// XXX get reg/sel
	{
		unsigned decimator;

		struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kc);
		struct snd_soc_dapm_widget *w = wlist->widgets[0];

		if (w->name[4] == ' ')
			decimator = w->name[3] - '0';
		else
			decimator = 10;

		printk("put_dec_num %s decimator=%d\n", w->name, decimator);

		sel = (decimator <= 6 && val == 1) || (decimator >= 6 && (val == 1 || val == 2));
		reg = WCD9320_CDC_TX1_MUX_CTL + 8 * (decimator - 1);
	}

	snd_soc_component_update_bits(component, reg, 1, sel);

	return snd_soc_dapm_put_enum_double(kc, ucontrol);
}

static int taiko_get_iir_band_audio_mixer(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int taiko_put_iir_band_audio_mixer(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int taiko_get_compander(struct snd_kcontrol *kc, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	int comp = ((struct soc_mixer_control *)kc->private_value)->shift;
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);

	ucontrol->value.integer.value[0] = wcd->comp_enabled[comp];
	return 0;
}

static int taiko_set_compander(struct snd_kcontrol *kc, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	int comp = ((struct soc_mixer_control *)kc->private_value)->shift;
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);

	wcd->comp_enabled[comp] = ucontrol->value.integer.value[0];

	if (comp != COMPANDER_1)
		return 0;

	if (wcd->comp_enabled[comp]) {
		/* Wavegen to 5 msec */
		snd_soc_component_write(component, WCD9320_RX_HPH_CNP_WG_CTL, 0xDB);
		snd_soc_component_write(component, WCD9320_RX_HPH_CNP_WG_TIME, 0x2A);
		snd_soc_component_write(component, WCD9320_RX_HPH_BIAS_WG_OCP, 0x2A);

		/* Enable Chopper */
		snd_soc_component_update_bits(component,
			WCD9320_RX_HPH_CHOP_CTL, 0x80, 0x80);

		snd_soc_component_write(component, WCD9320_NCP_DTEST, 0x20);
	} else {
		/* Wavegen to 20 msec */
		snd_soc_component_write(component, WCD9320_RX_HPH_CNP_WG_CTL, 0xDB);
		snd_soc_component_write(component, WCD9320_RX_HPH_CNP_WG_TIME, 0x58);
		snd_soc_component_write(component, WCD9320_RX_HPH_BIAS_WG_OCP, 0x1A);

		/* Disable CHOPPER block */
		snd_soc_component_update_bits(component,
			WCD9320_RX_HPH_CHOP_CTL, 0x80, 0x00);

		snd_soc_component_write(component, WCD9320_NCP_DTEST, 0x10);
	}

	return 0;
}

static int taiko_mad_input_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int taiko_mad_input_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_get_anc_slot(struct snd_kcontrol *kc, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);
	ucontrol->value.integer.value[0] = wcd->anc_slot;
	return 0;
}

static int taiko_put_anc_slot(struct snd_kcontrol *kc, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);
	wcd->anc_slot = ucontrol->value.integer.value[0];
	return 0;
}

static int taiko_get_anc_func(struct snd_kcontrol *kc, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);
	ucontrol->value.integer.value[0] = wcd->anc_func;
	return 0;
}

static int taiko_put_anc_func(struct snd_kcontrol *kc, struct snd_ctl_elem_value *ucontrol)
{
	printk("taiko_put_anc_func\n");

	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kc);
	struct wcd9320_codec *wcd = dev_get_drvdata(dapm->dev);

	wcd->anc_func = ucontrol->value.integer.value[0];

	if (wcd->anc_func) {
		snd_soc_dapm_enable_pin(dapm, "ANC HPHR");
		snd_soc_dapm_enable_pin(dapm, "ANC HPHL");
		snd_soc_dapm_enable_pin(dapm, "ANC HEADPHONE");
		snd_soc_dapm_enable_pin(dapm, "ANC EAR PA");
		snd_soc_dapm_enable_pin(dapm, "ANC EAR");
		snd_soc_dapm_disable_pin(dapm, "HPHR");
		snd_soc_dapm_disable_pin(dapm, "HPHL");
		snd_soc_dapm_disable_pin(dapm, "HEADPHONE");
		snd_soc_dapm_disable_pin(dapm, "EAR PA");
		snd_soc_dapm_disable_pin(dapm, "EAR");
	} else {
		snd_soc_dapm_disable_pin(dapm, "ANC HPHR");
		snd_soc_dapm_disable_pin(dapm, "ANC HPHL");
		snd_soc_dapm_disable_pin(dapm, "ANC HEADPHONE");
		snd_soc_dapm_disable_pin(dapm, "ANC EAR PA");
		snd_soc_dapm_disable_pin(dapm, "ANC EAR");
		snd_soc_dapm_enable_pin(dapm, "HPHR");
		snd_soc_dapm_enable_pin(dapm, "HPHL");
		snd_soc_dapm_enable_pin(dapm, "HEADPHONE");
		snd_soc_dapm_enable_pin(dapm, "EAR PA");
		snd_soc_dapm_enable_pin(dapm, "EAR");
	}
	snd_soc_dapm_sync(dapm);
	return 0;
}

static int taiko_codec_enable_adc(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_ear_pa(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_ear_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

#if 0
// interrupts are all always enabled
static void wcd9320_codec_enable_int_port(struct wcd_slim_codec_dai_data *dai,
					struct snd_soc_component *component)
{
	int port_num = 0;
	unsigned short reg = 0;
	unsigned int val = 0;
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);
	struct wcd9320_slim_ch *ch;

	list_for_each_entry(ch, &dai->slim_ch_list, list) {
		port_num = ch->port;
		reg = WCD9320_SLIM_PGD_PORT_INT_EN0 + (port_num / 8);

		regmap_read(wcd->if_regmap, reg, &val);
		if (!(val & BIT(port_num % 8)))
			regmap_write(wcd->if_regmap, reg,
					val | BIT(port_num % 8));
	}
}
#endif

static int taiko_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wcd9320_codec *wcd = snd_soc_component_get_drvdata(comp);
	struct wcd_slim_codec_dai_data *dai = &wcd->dai[w->shift];

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		;// wcd9320_codec_enable_int_port(dai, comp);
		break;
	case SND_SOC_DAPM_POST_PMD:
		kfree(dai->sconfig.chs);

		break;
	}

	return 0;
}

static int taiko_hph_pa_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_hphl_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wcd9320_codec *wcd = dev_get_drvdata(comp->dev);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLK_RDAC_CLK_EN_CTL,
							0x02, 0x02);
		wcd_clsh_ctrl_set_state(wcd->clsh_ctrl, WCD_CLSH_EVENT_PRE_DAC, WCD_CLSH_STATE_HPHL);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLK_RDAC_CLK_EN_CTL,
							0x02, 0x00);
	}

	return 0;
}

static int taiko_hphr_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wcd9320_codec *wcd = dev_get_drvdata(comp->dev);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLK_RDAC_CLK_EN_CTL,
							0x04, 0x04);
		snd_soc_component_update_bits(comp, w->reg, 0x40, 0x40);
		wcd_clsh_ctrl_set_state(wcd->clsh_ctrl, WCD_CLSH_EVENT_PRE_DAC, WCD_CLSH_STATE_HPHR);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLK_RDAC_CLK_EN_CTL,
							0x04, 0x00);
		snd_soc_component_update_bits(comp, w->reg, 0x40, 0x00);
		break;
	}

	return 0;
}

static int taiko_codec_enable_lineout(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_spk_pa(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_dmic(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_lineout_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_spk_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_vdd_spkr(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}

static unsigned short rx_digital_gain_reg[] = {
	WCD9320_CDC_RX1_VOL_CTL_B2_CTL,
	WCD9320_CDC_RX2_VOL_CTL_B2_CTL,
	WCD9320_CDC_RX3_VOL_CTL_B2_CTL,
	WCD9320_CDC_RX4_VOL_CTL_B2_CTL,
	WCD9320_CDC_RX5_VOL_CTL_B2_CTL,
	WCD9320_CDC_RX6_VOL_CTL_B2_CTL,
	WCD9320_CDC_RX7_VOL_CTL_B2_CTL,
};

static unsigned short tx_digital_gain_reg[] = {
	WCD9320_CDC_TX1_VOL_CTL_GAIN,
	WCD9320_CDC_TX2_VOL_CTL_GAIN,
	WCD9320_CDC_TX3_VOL_CTL_GAIN,
	WCD9320_CDC_TX4_VOL_CTL_GAIN,
	WCD9320_CDC_TX5_VOL_CTL_GAIN,
	WCD9320_CDC_TX6_VOL_CTL_GAIN,
	WCD9320_CDC_TX7_VOL_CTL_GAIN,
	WCD9320_CDC_TX8_VOL_CTL_GAIN,
	WCD9320_CDC_TX9_VOL_CTL_GAIN,
	WCD9320_CDC_TX10_VOL_CTL_GAIN,
};

static int taiko_codec_enable_interpolator(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	printk("taiko_codec_enable_interpolator %d\n", w->shift);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 1 << w->shift);
		snd_soc_component_update_bits(comp, WCD9320_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 0x0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* apply the digital gain after the interpolator is enabled*/
		printk("gain=%.2X\n", snd_soc_component_read32(comp, rx_digital_gain_reg[w->shift]));
		snd_soc_component_write(comp,
				rx_digital_gain_reg[w->shift],
				snd_soc_component_read32(comp, rx_digital_gain_reg[w->shift]));
		break;
	}
	return 0;
}

static int taiko_codec_dsm_mux_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	u8 reg_val, zoh_mux_val = 0x00;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		reg_val = snd_soc_component_read32(comp, WCD9320_CDC_CONN_CLSH_CTL);

		if ((reg_val & 0x30) == 0x10)
			zoh_mux_val = 0x04;
		else if ((reg_val & 0x30) == 0x20)
			zoh_mux_val = 0x08;

		if (zoh_mux_val != 0x00)
			snd_soc_component_update_bits(comp,
					WCD9320_CDC_CONN_CLSH_CTL,
					0x0C, zoh_mux_val);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(comp, WCD9320_CDC_CONN_CLSH_CTL,
							0x0C, 0x00);
		break;
	}
	return 0;
}

static int taiko_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, WCD9320_RX_COM_BIAS, 0x80, 0x80);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(comp, WCD9320_RX_COM_BIAS, 0x80, 0x00);
		break;
	}
	return 0;
}

static int taiko_codec_enable_ldo_h(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int __taiko_codec_enable_ldo_h(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_config_gain_compander(struct snd_soc_component *c,
				       int comp, bool enable)
{
	int ret = 0;

	switch (comp) {
	case COMPANDER_0:
		snd_soc_component_update_bits(c, WCD9320_SPKR_DRV_GAIN,
				    1 << 2, !enable << 2);
		break;
	case COMPANDER_1:
		snd_soc_component_update_bits(c, WCD9320_RX_HPH_L_GAIN,
				    1 << 5, !enable << 5);
		snd_soc_component_update_bits(c, WCD9320_RX_HPH_R_GAIN,
				    1 << 5, !enable << 5);
		break;
	case COMPANDER_2:
		snd_soc_component_update_bits(c, WCD9320_RX_LINE_1_GAIN,
				    1 << 5, !enable << 5);
		snd_soc_component_update_bits(c, WCD9320_RX_LINE_3_GAIN,
				    1 << 5, !enable << 5);
		snd_soc_component_update_bits(c, WCD9320_RX_LINE_2_GAIN,
				    1 << 5, !enable << 5);
		snd_soc_component_update_bits(c, WCD9320_RX_LINE_4_GAIN,
				    1 << 5, !enable << 5);
		break;
	default:
		WARN_ON(1);
		ret = -EINVAL;
	}

	return ret;
}

static void taiko_discharge_comp(struct snd_soc_component *c, int comp)
{
	/* Level meter DIV Factor to 5*/
	snd_soc_component_update_bits(c, WCD9320_CDC_COMP0_B2_CTL + (comp * 8), 0xF0,
			    0x05 << 4);
	/* RMS meter Sampling to 0x01 */
	snd_soc_component_write(c, WCD9320_CDC_COMP0_B3_CTL + (comp * 8), 0x01);

	/* Worst case timeout for compander CnP sleep timeout */
	usleep_range(3000, 3000);
}

static const u32 comp_shift[] = {
	4, /* Compander 0's clock source is on interpolator 7 */
	0,
	2,
};

static const int comp_rx_path[] = {
	COMPANDER_1,
	COMPANDER_1,
	COMPANDER_2,
	COMPANDER_2,
	COMPANDER_2,
	COMPANDER_2,
	COMPANDER_0,
	COMPANDER_MAX,
};

struct comp_sample_dependent_params {
	u32 peak_det_timeout;
	u32 rms_meter_div_fact;
	u32 rms_meter_resamp_fact;
};

static const struct comp_sample_dependent_params comp_samp_params[] = {
	{
		/* 8 Khz */
		.peak_det_timeout = 0x06,
		.rms_meter_div_fact = 0x09,
		.rms_meter_resamp_fact = 0x06,
	},
	{
		/* 16 Khz */
		.peak_det_timeout = 0x07,
		.rms_meter_div_fact = 0x0A,
		.rms_meter_resamp_fact = 0x0C,
	},
	{
		/* 32 Khz */
		.peak_det_timeout = 0x08,
		.rms_meter_div_fact = 0x0B,
		.rms_meter_resamp_fact = 0x1E,
	},
	{
		/* 48 Khz */
		.peak_det_timeout = 0x09,
		.rms_meter_div_fact = 0x0B,
		.rms_meter_resamp_fact = 0x28,
	},
	{
		/* 96 Khz */
		.peak_det_timeout = 0x0A,
		.rms_meter_div_fact = 0x0C,
		.rms_meter_resamp_fact = 0x50,
	},
	{
		/* 192 Khz */
		.peak_det_timeout = 0x0B,
		.rms_meter_div_fact = 0xC,
		.rms_meter_resamp_fact = 0x50,
	},
};

static int taiko_config_compander(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct wcd9320_codec *wcd = dev_get_drvdata(w->dapm->dev);
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

	int mask, enable_mask;
	const int comp = w->shift;
	const u32 rate = wcd->comp_fs[comp];
	const struct comp_sample_dependent_params *comp_params =
	    &comp_samp_params[rate];
	unsigned buck_mv;

	if (!wcd->comp_enabled[comp])
		return 0;

	/* Compander 0 has single channel */
	mask = (comp == COMPANDER_0 ? 0x01 : 0x03);
	enable_mask = (comp == COMPANDER_0 ? 0x02 : 0x03);
	buck_mv = 2150000;//taiko_codec_get_buck_mv(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Set compander Sample rate */
		snd_soc_component_update_bits(c,
				    WCD9320_CDC_COMP0_FS_CFG + (comp * 8),
				    0x07, rate);
		/* Set the static gain offset */
		if (comp == COMPANDER_1 && buck_mv == 1800000) {
			snd_soc_component_update_bits(c,
					WCD9320_CDC_COMP0_B4_CTL + (comp * 8),
					0x80, 0x80);
		} else {
			snd_soc_component_update_bits(c,
					WCD9320_CDC_COMP0_B4_CTL + (comp * 8),
					0x80, 0x00);
		}
		/* Enable RX interpolation path compander clocks */
		snd_soc_component_update_bits(c, WCD9320_CDC_CLK_RX_B2_CTL,
				    mask << comp_shift[comp],
				    mask << comp_shift[comp]);
		/* Toggle compander reset bits */
		snd_soc_component_update_bits(c, WCD9320_CDC_CLK_OTHR_RESET_B2_CTL,
				    mask << comp_shift[comp],
				    mask << comp_shift[comp]);
		snd_soc_component_update_bits(c, WCD9320_CDC_CLK_OTHR_RESET_B2_CTL,
				    mask << comp_shift[comp], 0);

		/* Set gain source to compander */
		taiko_config_gain_compander(c, comp, true);

		/* Compander enable */
		snd_soc_component_update_bits(c, WCD9320_CDC_COMP0_B1_CTL +
				    (comp * 8), enable_mask, enable_mask);

		taiko_discharge_comp(c, comp);

		/* Set sample rate dependent paramater */
		snd_soc_component_write(c, WCD9320_CDC_COMP0_B3_CTL + (comp * 8),
			      comp_params->rms_meter_resamp_fact);
		snd_soc_component_update_bits(c,
				    WCD9320_CDC_COMP0_B2_CTL + (comp * 8),
				    0xF0, comp_params->rms_meter_div_fact << 4);
		snd_soc_component_update_bits(c,
					WCD9320_CDC_COMP0_B2_CTL + (comp * 8),
					0x0F, comp_params->peak_det_timeout);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* Disable compander */
		snd_soc_component_update_bits(c,
				    WCD9320_CDC_COMP0_B1_CTL + (comp * 8),
				    enable_mask, 0x00);

		/* Toggle compander reset bits */
		snd_soc_component_update_bits(c, WCD9320_CDC_CLK_OTHR_RESET_B2_CTL,
				    mask << comp_shift[comp],
				    mask << comp_shift[comp]);
		snd_soc_component_update_bits(c, WCD9320_CDC_CLK_OTHR_RESET_B2_CTL,
				    mask << comp_shift[comp], 0);

		/* Turn off the clock for compander in pair */
		snd_soc_component_update_bits(c, WCD9320_CDC_CLK_RX_B2_CTL,
				    mask << comp_shift[comp], 0);

		/* Set gain source to register */
		taiko_config_gain_compander(c, comp, false);
		break;
	}
	return 0;
}

static int taiko_codec_enable_micbias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_dec(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_anc_hph(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;

	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = taiko_hph_pa_event(w, kcontrol, event);
		if (w->shift == 4) {
			//ret |= taiko_codec_enable_anc(w, kcontrol, event);
			printk("%s:%d unimplemented\n", __FUNCTION__, __LINE__);
			msleep(50);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (w->shift == 4) {
			snd_soc_component_update_bits(comp,
					WCD9320_RX_HPH_CNP_EN, 0x30, 0x30);
			msleep(30);
		}
		ret = taiko_hph_pa_event(w, kcontrol, event);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (w->shift == 5) {
			snd_soc_component_update_bits(comp,
					WCD9320_RX_HPH_CNP_EN, 0x30, 0x00);
			msleep(40);
			snd_soc_component_update_bits(comp,
					WCD9320_TX_7_MBHC_EN, 0x80, 00);
			printk("%s:%d unimplemented\n", __FUNCTION__, __LINE__);
			// ret |= taiko_codec_enable_anc(w, kcontrol, event);
		}
	case SND_SOC_DAPM_POST_PMD:
		ret = taiko_hph_pa_event(w, kcontrol, event);
		break;
	}
	return ret;
}

static int taiko_codec_enable_anc_ear(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_slimvi_feedback(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_mad(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int taiko_codec_enable_aux_pga(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static int wcd9320_codec_enable_mclk(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kc, int event)
{
	printk("%s unimplemented\n", __FUNCTION__);
	return 0;
}

static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 1, 0);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);
static const DECLARE_TLV_DB_SCALE(aux_pga_gain, 0, 2, 0);

static const char *const anc_func_text[] = {"OFF", "ON"};

static const char *const tabla_ear_pa_gain_text[] = {"POS_6_DB", "POS_2_DB"};

/*cut of frequency for high pass filter*/
static const char * const cf_text[] = {
	"MIN_3DB_4Hz", "MIN_3DB_75Hz", "MIN_3DB_150Hz"
};

static const char * const class_h_dsm_text[] = {
	"ZERO", "DSM_HPHL_RX1", "DSM_SPKR_RX7"
};

static const char *const taiko_conn_mad_text[] = {
	"ADC_MB", "ADC1", "ADC2", "ADC3", "ADC4", "ADC5", "ADC6", "NOTUSED1",
	"DMIC1", "DMIC2", "DMIC3", "DMIC4", "DMIC5", "DMIC6", "NOTUSED2",
	"NOTUSED3"};

static const char * const taiko_1_x_ear_pa_gain_text[] = {
	"POS_6_DB", "UNDEFINED_1", "UNDEFINED_2", "UNDEFINED_3", "POS_2_DB",
	"NEG_2P5_DB", "UNDEFINED_4", "NEG_12_DB"
};

static const char * const taiko_2_x_ear_pa_gain_text[] = {
	"POS_6_DB", "POS_4P5_DB", "POS_3_DB", "POS_1P5_DB",
	"POS_0_DB", "NEG_2P5_DB", "UNDEFINED", "NEG_12_DB"
};

static const char * const rx_mix1_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2", "RX1", "RX2", "RX3", "RX4",
		"RX5", "RX6", "RX7"
};

static const char * const rx_mix2_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2"
};

static const char * const rx_rdac5_text[] = {
	"DEM4", "DEM3_INV"
};

static const char * const rx_rdac7_text[] = {
	"DEM6", "DEM5_INV"
};


static const char * const sb_tx1_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC1"
};

static const char * const sb_tx2_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC2"
};

static const char * const sb_tx3_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC3"
};

static const char * const sb_tx4_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC4"
};

static const char * const sb_tx5_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC5"
};

static const char * const sb_tx6_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC6"
};

static const char * const sb_tx7_to_tx10_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC1", "DEC2", "DEC3", "DEC4", "DEC5", "DEC6", "DEC7", "DEC8",
		"DEC9", "DEC10"
};

static const char * const dec1_mux_text[] = {
	"ZERO", "DMIC1", "ADC6",
};

static const char * const dec2_mux_text[] = {
	"ZERO", "DMIC2", "ADC5",
};

static const char * const dec3_mux_text[] = {
	"ZERO", "DMIC3", "ADC4",
};

static const char * const dec4_mux_text[] = {
	"ZERO", "DMIC4", "ADC3",
};

static const char * const dec5_mux_text[] = {
	"ZERO", "DMIC5", "ADC2",
};

static const char * const dec6_mux_text[] = {
	"ZERO", "DMIC6", "ADC1",
};

static const char * const dec7_mux_text[] = {
	"ZERO", "DMIC1", "DMIC6", "ADC1", "ADC6", "ANC1_FB", "ANC2_FB",
};

static const char * const dec8_mux_text[] = {
	"ZERO", "DMIC2", "DMIC5", "ADC2", "ADC5",
};

static const char * const dec9_mux_text[] = {
	"ZERO", "DMIC4", "DMIC5", "ADC2", "ADC3", "ADCMB", "ANC1_FB", "ANC2_FB",
};

static const char * const dec10_mux_text[] = {
	"ZERO", "DMIC3", "DMIC6", "ADC1", "ADC4", "ADCMB", "ANC1_FB", "ANC2_FB",
};

static const char * const anc_mux_text[] = {
	"ZERO", "ADC1", "ADC2", "ADC3", "ADC4", "ADC5", "ADC6", "ADC_MB",
		"RSVD_1", "DMIC1", "DMIC2", "DMIC3", "DMIC4", "DMIC5", "DMIC6"
};

static const char * const anc1_fb_mux_text[] = {
	"ZERO", "EAR_HPH_L", "EAR_LINE_1",
};

static const char * const iir_inp1_text[] = {
	"ZERO", "DEC1", "DEC2", "DEC3", "DEC4", "DEC5", "DEC6", "DEC7", "DEC8",
	"DEC9", "DEC10", "RX1", "RX2", "RX3", "RX4", "RX5", "RX6", "RX7"
};

static const char *const slim_rx_mux_text[] = {
	"ZERO", "AIF1_PB", "AIF2_PB", "AIF3_PB"
};

static const struct soc_enum anc_func_enum =
		SOC_ENUM_SINGLE_EXT(2, anc_func_text);

static const struct soc_enum tabla_ear_pa_gain_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, tabla_ear_pa_gain_text),
};

static const struct soc_enum cf_dec1_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX1_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec2_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX2_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec3_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX3_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec4_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX4_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec5_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX5_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec6_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX6_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec7_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX7_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec8_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX8_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec9_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX9_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec10_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_TX10_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_rxmix1_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_RX1_B4_CTL, 0, 3, cf_text);

static const struct soc_enum cf_rxmix2_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_RX2_B4_CTL, 0, 3, cf_text);

static const struct soc_enum cf_rxmix3_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_RX3_B4_CTL, 0, 3, cf_text);

static const struct soc_enum cf_rxmix4_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_RX4_B4_CTL, 0, 3, cf_text);

static const struct soc_enum cf_rxmix5_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_RX5_B4_CTL, 0, 3, cf_text)
;
static const struct soc_enum cf_rxmix6_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_RX6_B4_CTL, 0, 3, cf_text);

static const struct soc_enum cf_rxmix7_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_RX7_B4_CTL, 0, 3, cf_text);

static const struct soc_enum class_h_dsm_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_CLSH_CTL, 4, 3, class_h_dsm_text);

static const struct soc_enum taiko_conn_mad_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(taiko_conn_mad_text),
			taiko_conn_mad_text);

static const struct soc_enum taiko_1_x_ear_pa_gain_enum =
	SOC_ENUM_SINGLE(WCD9320_RX_EAR_GAIN, 5, 8, taiko_1_x_ear_pa_gain_text);

static const struct soc_enum taiko_2_x_ear_pa_gain_enum =
	SOC_ENUM_SINGLE(WCD9320_RX_EAR_GAIN, 5, 8, taiko_2_x_ear_pa_gain_text);

static const struct soc_enum rx_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX1_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX1_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx_mix1_inp3_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX1_B2_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx2_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX2_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx2_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX2_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx3_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX3_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx3_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX3_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx4_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX4_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx4_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX4_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx5_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX5_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx5_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX5_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx6_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX6_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx6_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX6_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx7_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX7_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx7_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX7_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx1_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX1_B3_CTL, 0, 5, rx_mix2_text);

static const struct soc_enum rx1_mix2_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX1_B3_CTL, 3, 5, rx_mix2_text);

static const struct soc_enum rx2_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX2_B3_CTL, 0, 5, rx_mix2_text);

static const struct soc_enum rx2_mix2_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX2_B3_CTL, 3, 5, rx_mix2_text);

static const struct soc_enum rx7_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX7_B3_CTL, 0, 5, rx_mix2_text);

static const struct soc_enum rx7_mix2_inp2_chain_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_RX7_B3_CTL, 3, 5, rx_mix2_text);

static const struct soc_enum rx_rdac5_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_MISC, 2, 2, rx_rdac5_text);

static const struct soc_enum rx_rdac7_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_MISC, 1, 2, rx_rdac7_text);

static const struct soc_enum sb_tx1_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B1_CTL, 0, 9, sb_tx1_mux_text);

static const struct soc_enum sb_tx2_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B2_CTL, 0, 9, sb_tx2_mux_text);

static const struct soc_enum sb_tx3_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B3_CTL, 0, 9, sb_tx3_mux_text);

static const struct soc_enum sb_tx4_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B4_CTL, 0, 9, sb_tx4_mux_text);

static const struct soc_enum sb_tx5_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B5_CTL, 0, 9, sb_tx5_mux_text);

static const struct soc_enum sb_tx6_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B6_CTL, 0, 9, sb_tx6_mux_text);

static const struct soc_enum sb_tx7_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B7_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum sb_tx8_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B8_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum sb_tx9_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B9_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum sb_tx10_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_SB_B10_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum dec1_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B1_CTL, 0, 3, dec1_mux_text);

static const struct soc_enum dec2_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B1_CTL, 2, 3, dec2_mux_text);

static const struct soc_enum dec3_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B1_CTL, 4, 3, dec3_mux_text);

static const struct soc_enum dec4_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B1_CTL, 6, 3, dec4_mux_text);

static const struct soc_enum dec5_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B2_CTL, 0, 3, dec5_mux_text);

static const struct soc_enum dec6_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B2_CTL, 2, 3, dec6_mux_text);

static const struct soc_enum dec7_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B2_CTL, 4, 7, dec7_mux_text);

static const struct soc_enum dec8_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B3_CTL, 0, 7, dec8_mux_text);

static const struct soc_enum dec9_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B3_CTL, 3, 8, dec9_mux_text);

static const struct soc_enum dec10_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_TX_B4_CTL, 0, 8, dec10_mux_text);

static const struct soc_enum anc1_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_ANC_B1_CTL, 0, 16, anc_mux_text);

static const struct soc_enum anc2_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_ANC_B1_CTL, 4, 16, anc_mux_text);

static const struct soc_enum anc1_fb_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_ANC_B2_CTL, 0, 3, anc1_fb_mux_text);

static const struct soc_enum iir1_inp1_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_EQ1_B1_CTL, 0, 18, iir_inp1_text);

static const struct soc_enum iir2_inp1_mux_enum =
	SOC_ENUM_SINGLE(WCD9320_CDC_CONN_EQ2_B1_CTL, 0, 18, iir_inp1_text);

static const struct soc_enum slim_rx_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slim_rx_mux_text), slim_rx_mux_text);


static const struct snd_kcontrol_new class_h_dsm_mux =
	SOC_DAPM_ENUM("CLASS_H_DSM MUX Mux", class_h_dsm_enum);

static const struct snd_kcontrol_new wcd9320_snd_controls[] = {

	SOC_SINGLE_S8_TLV("RX1 Digital Volume", WCD9320_CDC_RX1_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX2 Digital Volume", WCD9320_CDC_RX2_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX3 Digital Volume", WCD9320_CDC_RX3_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX4 Digital Volume", WCD9320_CDC_RX4_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX5 Digital Volume", WCD9320_CDC_RX5_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX6 Digital Volume", WCD9320_CDC_RX6_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX7 Digital Volume", WCD9320_CDC_RX7_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),

	SOC_SINGLE_S8_TLV("DEC1 Volume", WCD9320_CDC_TX1_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC2 Volume", WCD9320_CDC_TX2_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC3 Volume", WCD9320_CDC_TX3_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC4 Volume", WCD9320_CDC_TX4_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC5 Volume", WCD9320_CDC_TX5_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC6 Volume", WCD9320_CDC_TX6_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC7 Volume", WCD9320_CDC_TX7_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC8 Volume", WCD9320_CDC_TX8_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC9 Volume", WCD9320_CDC_TX9_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC10 Volume", WCD9320_CDC_TX10_VOL_CTL_GAIN, -84,
		40, digital_gain),

	SOC_SINGLE_S8_TLV("IIR1 INP1 Volume", WCD9320_CDC_IIR1_GAIN_B1_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP2 Volume", WCD9320_CDC_IIR1_GAIN_B2_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP3 Volume", WCD9320_CDC_IIR1_GAIN_B3_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP4 Volume", WCD9320_CDC_IIR1_GAIN_B4_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR2 INP1 Volume", WCD9320_CDC_IIR2_GAIN_B1_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR2 INP2 Volume", WCD9320_CDC_IIR2_GAIN_B2_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR2 INP3 Volume", WCD9320_CDC_IIR2_GAIN_B3_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR2 INP4 Volume", WCD9320_CDC_IIR2_GAIN_B4_CTL, -84,
		40, digital_gain),

	SOC_SINGLE_EXT("ANC Slot", SND_SOC_NOPM, 0, 100, 0, taiko_get_anc_slot,
		taiko_put_anc_slot),
	SOC_ENUM_EXT("ANC Function", anc_func_enum, taiko_get_anc_func,
		taiko_put_anc_func),

	SOC_ENUM("TX1 HPF cut off", cf_dec1_enum),
	SOC_ENUM("TX2 HPF cut off", cf_dec2_enum),
	SOC_ENUM("TX3 HPF cut off", cf_dec3_enum),
	SOC_ENUM("TX4 HPF cut off", cf_dec4_enum),
	SOC_ENUM("TX5 HPF cut off", cf_dec5_enum),
	SOC_ENUM("TX6 HPF cut off", cf_dec6_enum),
	SOC_ENUM("TX7 HPF cut off", cf_dec7_enum),
	SOC_ENUM("TX8 HPF cut off", cf_dec8_enum),
	SOC_ENUM("TX9 HPF cut off", cf_dec9_enum),
	SOC_ENUM("TX10 HPF cut off", cf_dec10_enum),

	SOC_SINGLE("TX1 HPF Switch", WCD9320_CDC_TX1_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX2 HPF Switch", WCD9320_CDC_TX2_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX3 HPF Switch", WCD9320_CDC_TX3_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX4 HPF Switch", WCD9320_CDC_TX4_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX5 HPF Switch", WCD9320_CDC_TX5_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX6 HPF Switch", WCD9320_CDC_TX6_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX7 HPF Switch", WCD9320_CDC_TX7_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX8 HPF Switch", WCD9320_CDC_TX8_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX9 HPF Switch", WCD9320_CDC_TX9_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX10 HPF Switch", WCD9320_CDC_TX10_MUX_CTL, 3, 1, 0),

	SOC_SINGLE("RX1 HPF Switch", WCD9320_CDC_RX1_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX2 HPF Switch", WCD9320_CDC_RX2_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX3 HPF Switch", WCD9320_CDC_RX3_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX4 HPF Switch", WCD9320_CDC_RX4_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX5 HPF Switch", WCD9320_CDC_RX5_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX6 HPF Switch", WCD9320_CDC_RX6_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX7 HPF Switch", WCD9320_CDC_RX7_B5_CTL, 2, 1, 0),

	SOC_ENUM("RX1 HPF cut off", cf_rxmix1_enum),
	SOC_ENUM("RX2 HPF cut off", cf_rxmix2_enum),
	SOC_ENUM("RX3 HPF cut off", cf_rxmix3_enum),
	SOC_ENUM("RX4 HPF cut off", cf_rxmix4_enum),
	SOC_ENUM("RX5 HPF cut off", cf_rxmix5_enum),
	SOC_ENUM("RX6 HPF cut off", cf_rxmix6_enum),
	SOC_ENUM("RX7 HPF cut off", cf_rxmix7_enum),

	SOC_SINGLE("IIR1 Enable Band1", WCD9320_CDC_IIR1_CTL, 0, 1, 0),
	SOC_SINGLE("IIR1 Enable Band2", WCD9320_CDC_IIR1_CTL, 1, 1, 0),
	SOC_SINGLE("IIR1 Enable Band3", WCD9320_CDC_IIR1_CTL, 2, 1, 0),
	SOC_SINGLE("IIR1 Enable Band4", WCD9320_CDC_IIR1_CTL, 3, 1, 0),
	SOC_SINGLE("IIR1 Enable Band5", WCD9320_CDC_IIR1_CTL, 4, 1, 0),
	SOC_SINGLE("IIR2 Enable Band1", WCD9320_CDC_IIR2_CTL, 0, 1, 0),
	SOC_SINGLE("IIR2 Enable Band2", WCD9320_CDC_IIR2_CTL, 1, 1, 0),
	SOC_SINGLE("IIR2 Enable Band3", WCD9320_CDC_IIR2_CTL, 2, 1, 0),
	SOC_SINGLE("IIR2 Enable Band4", WCD9320_CDC_IIR2_CTL, 3, 1, 0),
	SOC_SINGLE("IIR2 Enable Band5", WCD9320_CDC_IIR2_CTL, 4, 1, 0),

#if 0 // SOC_SINGLE_MULTI_EXT is gone..
	SOC_SINGLE_MULTI_EXT("IIR1 Band1", IIR1, BAND1, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band2", IIR1, BAND2, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band3", IIR1, BAND3, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band4", IIR1, BAND4, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band5", IIR1, BAND5, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band1", IIR2, BAND1, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band2", IIR2, BAND2, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band3", IIR2, BAND3, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band4", IIR2, BAND4, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band5", IIR2, BAND5, 255, 0, 5,
	taiko_get_iir_band_audio_mixer, taiko_put_iir_band_audio_mixer),
#endif

	SOC_SINGLE_EXT("COMP0 Switch", SND_SOC_NOPM, COMPANDER_0, 1, 0,
		       taiko_get_compander, taiko_set_compander),
	SOC_SINGLE_EXT("COMP1 Switch", SND_SOC_NOPM, COMPANDER_1, 1, 0,
		       taiko_get_compander, taiko_set_compander),
	SOC_SINGLE_EXT("COMP2 Switch", SND_SOC_NOPM, COMPANDER_2, 1, 0,
		       taiko_get_compander, taiko_set_compander),

	SOC_ENUM_EXT("MAD Input", taiko_conn_mad_enum,
			taiko_mad_input_get, taiko_mad_input_put),

	/* analog gain controls */

	SOC_SINGLE_TLV("HPHL Volume", WCD9320_RX_HPH_L_GAIN, 0, 20, 1, line_gain),
	SOC_SINGLE_TLV("HPHR Volume", WCD9320_RX_HPH_R_GAIN, 0, 20, 1, line_gain),

	SOC_SINGLE_TLV("LINEOUT1 Volume", WCD9320_RX_LINE_1_GAIN, 0, 20, 1, line_gain),
	SOC_SINGLE_TLV("LINEOUT2 Volume", WCD9320_RX_LINE_2_GAIN, 0, 20, 1, line_gain),
	SOC_SINGLE_TLV("LINEOUT3 Volume", WCD9320_RX_LINE_3_GAIN, 0, 20, 1, line_gain),
	SOC_SINGLE_TLV("LINEOUT4 Volume", WCD9320_RX_LINE_4_GAIN, 0, 20, 1, line_gain),
#if 0 // WCD9320_V1
 	SOC_ENUM("EAR PA Gain", taiko_1_x_ear_pa_gain_enum),

	SOC_SINGLE_TLV("SPK DRV Volume", WCD9320_SPKR_DRV_GAIN, 3, 7, 1, line_gain),

	SOC_SINGLE_TLV("ADC1 Volume", WCD9320_TX_1_2_EN, 5, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", WCD9320_TX_1_2_EN, 1, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", WCD9320_TX_3_4_EN, 5, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC4 Volume", WCD9320_TX_3_4_EN, 1, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC5 Volume", WCD9320_TX_5_6_EN, 5, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC6 Volume", WCD9320_TX_5_6_EN, 1, 3, 0, analog_gain),
#else // WCD9320_V2
	SOC_ENUM("EAR PA Gain", taiko_2_x_ear_pa_gain_enum),

	SOC_SINGLE_TLV("SPK DRV Volume", WCD9320_SPKR_DRV_GAIN, 3, 8, 1, line_gain),

	SOC_SINGLE_TLV("ADC1 Volume", WCD9320_CDC_TX_1_GAIN, 2, 19, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", WCD9320_CDC_TX_2_GAIN, 2, 19, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", WCD9320_CDC_TX_3_GAIN, 2, 19, 0, analog_gain),
	SOC_SINGLE_TLV("ADC4 Volume", WCD9320_CDC_TX_4_GAIN, 2, 19, 0, analog_gain),
	SOC_SINGLE_TLV("ADC5 Volume", WCD9320_CDC_TX_5_GAIN, 2, 19, 0, analog_gain),
	SOC_SINGLE_TLV("ADC6 Volume", WCD9320_CDC_TX_6_GAIN, 2, 19, 0, analog_gain),
#endif
};

static const struct snd_kcontrol_new rx_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP2 Mux", rx_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx_mix1_inp3_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP3 Mux", rx_mix1_inp3_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP2 Mux", rx2_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP2 Mux", rx3_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx4_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX4 MIX1 INP1 Mux", rx4_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx4_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX4 MIX1 INP2 Mux", rx4_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx5_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX5 MIX1 INP1 Mux", rx5_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx5_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX5 MIX1 INP2 Mux", rx5_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx6_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX6 MIX1 INP1 Mux", rx6_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx6_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX6 MIX1 INP2 Mux", rx6_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx7_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX7 MIX1 INP1 Mux", rx7_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx7_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX7 MIX1 INP2 Mux", rx7_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx1_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX2 INP1 Mux", rx1_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new rx1_mix2_inp2_mux =
	SOC_DAPM_ENUM("RX1 MIX2 INP2 Mux", rx1_mix2_inp2_chain_enum);

static const struct snd_kcontrol_new rx2_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX2 INP1 Mux", rx2_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new rx2_mix2_inp2_mux =
	SOC_DAPM_ENUM("RX2 MIX2 INP2 Mux", rx2_mix2_inp2_chain_enum);

static const struct snd_kcontrol_new rx7_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX7 MIX2 INP1 Mux", rx7_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new rx7_mix2_inp2_mux =
	SOC_DAPM_ENUM("RX7 MIX2 INP2 Mux", rx7_mix2_inp2_chain_enum);

static const struct snd_kcontrol_new rx_dac5_mux =
	SOC_DAPM_ENUM("RDAC5 MUX Mux", rx_rdac5_enum);

static const struct snd_kcontrol_new rx_dac7_mux =
	SOC_DAPM_ENUM("RDAC7 MUX Mux", rx_rdac7_enum);

static const struct snd_kcontrol_new sb_tx1_mux =
	SOC_DAPM_ENUM("SLIM TX1 MUX Mux", sb_tx1_mux_enum);

static const struct snd_kcontrol_new sb_tx2_mux =
	SOC_DAPM_ENUM("SLIM TX2 MUX Mux", sb_tx2_mux_enum);

static const struct snd_kcontrol_new sb_tx3_mux =
	SOC_DAPM_ENUM("SLIM TX3 MUX Mux", sb_tx3_mux_enum);

static const struct snd_kcontrol_new sb_tx4_mux =
	SOC_DAPM_ENUM("SLIM TX4 MUX Mux", sb_tx4_mux_enum);

static const struct snd_kcontrol_new sb_tx5_mux =
	SOC_DAPM_ENUM("SLIM TX5 MUX Mux", sb_tx5_mux_enum);

static const struct snd_kcontrol_new sb_tx6_mux =
	SOC_DAPM_ENUM("SLIM TX6 MUX Mux", sb_tx6_mux_enum);

static const struct snd_kcontrol_new sb_tx7_mux =
	SOC_DAPM_ENUM("SLIM TX7 MUX Mux", sb_tx7_mux_enum);

static const struct snd_kcontrol_new sb_tx8_mux =
	SOC_DAPM_ENUM("SLIM TX8 MUX Mux", sb_tx8_mux_enum);

static const struct snd_kcontrol_new sb_tx9_mux =
	SOC_DAPM_ENUM("SLIM TX9 MUX Mux", sb_tx9_mux_enum);

static const struct snd_kcontrol_new sb_tx10_mux =
	SOC_DAPM_ENUM("SLIM TX10 MUX Mux", sb_tx10_mux_enum);

#define WCD9320_DEC_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_dapm_get_enum_double, \
	.put = wcd9320_put_dec_enum, \
	.private_value = (unsigned long)&xenum }

static const struct snd_kcontrol_new dec1_mux =
	WCD9320_DEC_ENUM("DEC1 MUX Mux", dec1_mux_enum);

static const struct snd_kcontrol_new dec2_mux =
	WCD9320_DEC_ENUM("DEC2 MUX Mux", dec2_mux_enum);

static const struct snd_kcontrol_new dec3_mux =
	WCD9320_DEC_ENUM("DEC3 MUX Mux", dec3_mux_enum);

static const struct snd_kcontrol_new dec4_mux =
	WCD9320_DEC_ENUM("DEC4 MUX Mux", dec4_mux_enum);

static const struct snd_kcontrol_new dec5_mux =
	WCD9320_DEC_ENUM("DEC5 MUX Mux", dec5_mux_enum);

static const struct snd_kcontrol_new dec6_mux =
	WCD9320_DEC_ENUM("DEC6 MUX Mux", dec6_mux_enum);

static const struct snd_kcontrol_new dec7_mux =
	WCD9320_DEC_ENUM("DEC7 MUX Mux", dec7_mux_enum);

static const struct snd_kcontrol_new dec8_mux =
	WCD9320_DEC_ENUM("DEC8 MUX Mux", dec8_mux_enum);

static const struct snd_kcontrol_new dec9_mux =
	WCD9320_DEC_ENUM("DEC9 MUX Mux", dec9_mux_enum);

static const struct snd_kcontrol_new dec10_mux =
	WCD9320_DEC_ENUM("DEC10 MUX Mux", dec10_mux_enum);

static const struct snd_kcontrol_new iir1_inp1_mux =
	SOC_DAPM_ENUM("IIR1 INP1 Mux", iir1_inp1_mux_enum);

static const struct snd_kcontrol_new iir2_inp1_mux =
	SOC_DAPM_ENUM("IIR2 INP1 Mux", iir2_inp1_mux_enum);

static const struct snd_kcontrol_new anc1_mux =
	SOC_DAPM_ENUM("ANC1 MUX Mux", anc1_mux_enum);

static const struct snd_kcontrol_new anc2_mux =
	SOC_DAPM_ENUM("ANC2 MUX Mux", anc2_mux_enum);

static const struct snd_kcontrol_new anc1_fb_mux =
	SOC_DAPM_ENUM("ANC1 FB MUX Mux", anc1_fb_mux_enum);

static const struct snd_kcontrol_new dac1_switch[] = {
	SOC_DAPM_SINGLE("Switch", WCD9320_RX_EAR_EN, 5, 1, 0)
};
static const struct snd_kcontrol_new hphl_switch[] = {
	SOC_DAPM_SINGLE("Switch", WCD9320_RX_HPH_L_DAC_CTL, 6, 1, 0)
};

static const struct snd_kcontrol_new hphl_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", WCD9320_RX_PA_AUX_IN_CONN,
					7, 1, 0),
};

static const struct snd_kcontrol_new hphr_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", WCD9320_RX_PA_AUX_IN_CONN,
					6, 1, 0),
};

static const struct snd_kcontrol_new ear_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", WCD9320_RX_PA_AUX_IN_CONN,
					5, 1, 0),
};
static const struct snd_kcontrol_new lineout1_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", WCD9320_RX_PA_AUX_IN_CONN,
					4, 1, 0),
};

static const struct snd_kcontrol_new lineout2_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", WCD9320_RX_PA_AUX_IN_CONN,
					3, 1, 0),
};

static const struct snd_kcontrol_new lineout3_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", WCD9320_RX_PA_AUX_IN_CONN,
					2, 1, 0),
};

static const struct snd_kcontrol_new lineout4_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", WCD9320_RX_PA_AUX_IN_CONN,
					1, 1, 0),
};

static const struct snd_kcontrol_new lineout3_ground_switch =
	SOC_DAPM_SINGLE("Switch", WCD9320_RX_LINE_3_DAC_CTL, 6, 1, 0);

static const struct snd_kcontrol_new lineout4_ground_switch =
	SOC_DAPM_SINGLE("Switch", WCD9320_RX_LINE_4_DAC_CTL, 6, 1, 0);

static const struct snd_kcontrol_new aif4_mad_switch =
	SOC_DAPM_SINGLE("Switch", WCD9320_CDC_CLK_OTHR_CTL, 4, 1, 0);

static const struct snd_kcontrol_new slim_rx_mux[WCD9320_RX_MAX] = {
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

static const struct snd_kcontrol_new aif_cap_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, WCD9320_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, WCD9320_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, WCD9320_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, WCD9320_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, WCD9320_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, WCD9320_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, WCD9320_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, WCD9320_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, WCD9320_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, WCD9320_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static int wcd9320_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	unsigned rates[] = {8000, 16000, 32000, 48000, 96000, 192000};
	u8 rate_index = 0;
	unsigned rate = params_rate(params);
	struct wcd9320_codec *wcd = snd_soc_component_get_drvdata(dai->component);
	struct snd_soc_component *component = dai->component;
	struct wcd9320_slim_ch *ch;
	u32 i, j;
	u8 rx_mix1_inp;
	u16 rx_mix_1_reg_1, rx_mix_1_reg_2;
	u16 rx_fs_reg;
	u8 rx_mix_1_reg_1_val, rx_mix_1_reg_2_val;

	printk("taiko_hw_params\n");

	do {
		if (rate_index == ARRAY_SIZE(rates))
			return -EINVAL;
	} while (rate != rates[rate_index] && ++rate_index);

	printk("taiko_hw_params rate_index=%d\n", rate_index);

	list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list) {
		rx_mix1_inp = ch->port + RX_MIX1_INP_SEL_RX1 - 16; //TAIKO_TX_PORT_NUMBER;
		WARN_ON(rx_mix1_inp < RX_MIX1_INP_SEL_RX1 || rx_mix1_inp > RX_MIX1_INP_SEL_RX7);

		rx_mix_1_reg_1 = WCD9320_CDC_CONN_RX1_B1_CTL;

		for (j = 0; j < WCD9320_NUM_INTERPOLATORS; j++) {
			rx_mix_1_reg_2 = rx_mix_1_reg_1 + 1;

			rx_mix_1_reg_1_val = snd_soc_component_read32(component, rx_mix_1_reg_1);
			rx_mix_1_reg_2_val = snd_soc_component_read32(component, rx_mix_1_reg_2);

			printk("%.2X %.2X %.2X\n", rx_mix_1_reg_1_val, rx_mix_1_reg_2_val, rx_mix1_inp);

			if (((rx_mix_1_reg_1_val & 0x0F) == rx_mix1_inp) ||
				(((rx_mix_1_reg_1_val >> 4) & 0x0F) == rx_mix1_inp) ||
				((rx_mix_1_reg_2_val & 0x0F) == rx_mix1_inp)) {

				rx_fs_reg = WCD9320_CDC_RX1_B5_CTL + 8 * j;

				printk("%s: AIF_PB DAI(%d) connected to RX%u\n", __func__, dai->id, j + 1);
				printk("%s: set RX%u sample rate to %u\n", __func__, j + 1, rate);

				snd_soc_component_update_bits(component, rx_fs_reg, 0xE0, rate_index << 5);
			}
			if (j < 2)
				rx_mix_1_reg_1 += 3;
			else
				rx_mix_1_reg_1 += 2;
		}
	}
	return 0;
}

/* slave port water mark level
 *   (0: 6bytes, 1: 9bytes, 2: 12 bytes, 3: 15 bytes)
 */
#define SLAVE_PORT_WATER_MARK_6BYTES  0
#define SLAVE_PORT_WATER_MARK_9BYTES  1
#define SLAVE_PORT_WATER_MARK_12BYTES 2
#define SLAVE_PORT_WATER_MARK_15BYTES 3
#define SLAVE_PORT_WATER_MARK_SHIFT 1
#define SLAVE_PORT_ENABLE           1
#define SLAVE_PORT_DISABLE          0
#define WCD9320_SLIM_WATER_MARK_VAL \
	((SLAVE_PORT_WATER_MARK_12BYTES << SLAVE_PORT_WATER_MARK_SHIFT) | \
	 (SLAVE_PORT_ENABLE))

static int wcd9320_slim_set_hw_params(struct wcd9320_codec *wcd,
				 struct wcd_slim_codec_dai_data *dai_data,
				 int direction)
{
	struct list_head *slim_ch_list = &dai_data->slim_ch_list;
	struct slim_stream_config *cfg = &dai_data->sconfig;
	struct wcd9320_slim_ch *ch;
	u16 payload = 0;
	int ret, i;

	cfg->ch_count = 0;
	cfg->direction = direction;
	cfg->port_mask = 0;

	/* Configure slave interface device */
	list_for_each_entry(ch, slim_ch_list, list) {
		cfg->ch_count++;
		payload |= 1 << ch->shift;
		cfg->port_mask |= BIT(ch->port);
	}

	cfg->chs = kcalloc(cfg->ch_count, sizeof(unsigned int), GFP_KERNEL);
	if (!cfg->chs)
		return -ENOMEM;

	i = 0;
	list_for_each_entry(ch, slim_ch_list, list) {
		cfg->chs[i++] = ch->ch_num;

		printk("ch %d %d %d\n", ch->ch_num, ch->port, ch->shift);

		if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
			/* write to interface device */
			ret = regmap_write(wcd->if_regmap,
				WCD9320_SLIM_PGD_RX_PORT_MULTI_CHNL_0(ch->port),
				payload);

			if (ret < 0)
				goto err;

			/* configure the slave port for water mark and enable*/
			ret = regmap_write(wcd->if_regmap,
					WCD9320_SLIM_PGD_RX_PORT_CFG(ch->port),
					WCD9320_SLIM_WATER_MARK_VAL);
			if (ret < 0)
				goto err;
		} else {
			ret = regmap_write(wcd->if_regmap,
				WCD9320_SLIM_PGD_TX_PORT_MULTI_CHNL_0(ch->port),
				payload & 0x00FF);
			if (ret < 0)
				goto err;

			/* ports 8,9 */
			ret = regmap_write(wcd->if_regmap,
				WCD9320_SLIM_PGD_TX_PORT_MULTI_CHNL_1(ch->port),
				(payload & 0xFF00)>>8);
			if (ret < 0)
				goto err;

			/* configure the slave port for water mark and enable*/
			ret = regmap_write(wcd->if_regmap,
					WCD9320_SLIM_PGD_TX_PORT_CFG(ch->port),
					WCD9320_SLIM_WATER_MARK_VAL);

			if (ret < 0)
				goto err;
		}
	}

	dai_data->sruntime = slim_stream_allocate(wcd->slim, "WCD9335-SLIM");

	return 0;

err:
	dev_err(wcd->dev, "Error Setting slim hw params\n");
	kfree(cfg->chs);
	cfg->chs = NULL;

	return ret;
}

static int wcd9320_trigger(struct snd_pcm_substream *substream, //int cmd,
			   struct snd_soc_dai *dai)
{
	struct wcd_slim_codec_dai_data *dai_data;
	struct wcd9320_codec *wcd;
	struct slim_stream_config *cfg;

	wcd = snd_soc_component_get_drvdata(dai->component);

	static int prepared = 0;

	{
	struct snd_soc_component *component = dai->component;
	printk("taiko trigger\n");

	snd_soc_component_update_bits(component, WCD9320_CDC_CONN_RX_SB_B1_CTL, 0xff, 0x02);
	snd_soc_component_update_bits(component, WCD9320_CDC_CONN_RX_SB_B1_CTL, 0xff, 0x0a);

if (!prepared) {
	snd_soc_component_write(component, WCD9320_BIAS_CENTRAL_BG_CTL, 0xD4);
	snd_soc_component_write(component, WCD9320_BIAS_CENTRAL_BG_CTL, 0xD5);
	snd_soc_component_write(component, WCD9320_BIAS_CENTRAL_BG_CTL, 0x55);

	snd_soc_component_write(component, WCD9320_CLK_BUFF_EN1, 4);
	snd_soc_component_write(component, WCD9320_CLK_BUFF_EN1, 5);
	snd_soc_component_write(component, WCD9320_CLK_BUFF_EN2, 0);
	snd_soc_component_write(component, WCD9320_CLK_BUFF_EN2, 4);
}

	//snd_soc_component_write(component, WCD9320_CDC_CLSH_V_PA_HD_HPH, 0x0d);
	//snd_soc_component_write(component, WCD9320_CDC_CLSH_V_PA_MIN_HPH, 0x1d);
	//snd_soc_component_write(component, WCD9320_CDC_CLSH_IDLE_HPH_THSD, 0x13);

// XXX
snd_soc_component_write(component, WCD9320_CDC_CLK_RX_B1_CTL, 0x3);

wcd->dai[dai->id].sconfig.bps = 16;
wcd->dai[dai->id].sconfig.rate = 48000;
wcd9320_slim_set_hw_params(wcd, &wcd->dai[dai->id], substream->stream);

	}

	dai_data = &wcd->dai[dai->id];

	cfg = &dai_data->sconfig;
	slim_stream_prepare(dai_data->sruntime, cfg);
	slim_stream_enable(dai_data->sruntime);

	prepared = 1;
#if 0

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		cfg = &dai_data->sconfig;
		slim_stream_prepare(dai_data->sruntime, cfg);
		slim_stream_enable(dai_data->sruntime);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		slim_stream_unprepare(dai_data->sruntime);
		slim_stream_disable(dai_data->sruntime);
		break;
	default:
		break;
	}
#endif

	return 0;
}

static int wcd9320_set_channel_map(struct snd_soc_dai *dai,
				   unsigned int tx_num, unsigned int *tx_slot,
				   unsigned int rx_num, unsigned int *rx_slot)
{
	struct wcd9320_codec *wcd;
	int i;

	wcd = snd_soc_component_get_drvdata(dai->component);

	if (!tx_slot || !rx_slot) {
		dev_err(wcd->dev, "Invalid tx_slot=%p, rx_slot=%p\n",
			tx_slot, rx_slot);
		return -EINVAL;
	}

	wcd->num_rx_port = rx_num;
	for (i = 0; i < rx_num; i++) {
		wcd->rx_chs[i].ch_num = rx_slot[i];
		INIT_LIST_HEAD(&wcd->rx_chs[i].list);
	}

	wcd->num_tx_port = tx_num;
	for (i = 0; i < tx_num; i++) {
		wcd->tx_chs[i].ch_num = tx_slot[i];
		INIT_LIST_HEAD(&wcd->tx_chs[i].list);
	}

	return 0;
}

static int wcd9320_get_channel_map(struct snd_soc_dai *dai,
				   unsigned int *tx_num, unsigned int *tx_slot,
				   unsigned int *rx_num, unsigned int *rx_slot)
{
	struct wcd9320_slim_ch *ch;
	struct wcd9320_codec *wcd;
	int i = 0;

	wcd = snd_soc_component_get_drvdata(dai->component);

	switch (dai->id) {
	case AIF1_PB:
	case AIF2_PB:
	case AIF3_PB:
		if (!rx_slot || !rx_num) {
			dev_err(wcd->dev, "Invalid rx_slot %p or rx_num %p\n",
				rx_slot, rx_num);
			return -EINVAL;
		}

		list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list)
			rx_slot[i++] = ch->ch_num;

		*rx_num = i;
		break;
	case AIF1_CAP:
	case AIF2_CAP:
	case AIF3_CAP:
		if (!tx_slot || !tx_num) {
			dev_err(wcd->dev, "Invalid tx_slot %p or tx_num %p\n",
				tx_slot, tx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list)
			tx_slot[i++] = ch->ch_num;

		*tx_num = i;
		break;
	default:
		dev_err(wcd->dev, "Invalid DAI ID %x\n", dai->id);
		break;
	}

	return 0;
}

static struct snd_soc_dai_ops wcd9320_dai_ops = {
	.hw_params = wcd9320_hw_params,
	.prepare = wcd9320_trigger,
	.set_channel_map = wcd9320_set_channel_map,
	.get_channel_map = wcd9320_get_channel_map,
};

static struct snd_soc_dai_ops dummy_dai_ops = {
};

static struct snd_soc_dai_driver wcd9320_slim_dais[] = {
	[0] = {
		.name = "wcd9320_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9320_RATES_MASK,
			.formats = WCD9320_FORMATS_S16_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9320_dai_ops,
	},
	[1] = {
		.name = "wcd9320_tx1",
		.id = AIF1_CAP,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = WCD9320_RATES_MASK,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd9320_dai_ops,
	},
	[2] = {
		.name = "wcd9320_rx2",
		.id = AIF2_PB,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = WCD9320_RATES_MASK,
			.formats = WCD9320_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9320_dai_ops,
	},
	[3] = {
		.name = "wcd9320_tx2",
		.id = AIF2_CAP,
		.capture = {
			.stream_name = "AIF2 Capture",
			.rates = WCD9320_RATES_MASK,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd9320_dai_ops,
	},
	[4] = {
		.name = "wcd9320_rx3",
		.id = AIF3_PB,
		.playback = {
			.stream_name = "AIF3 Playback",
			.rates = WCD9320_RATES_MASK,
			.formats = WCD9320_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd9320_dai_ops,
	},
	[5] = {
		.name = "wcd9320_tx3",
		.id = AIF3_CAP,
		.capture = {
			.stream_name = "AIF3 Capture",
			.rates = WCD9320_RATES_MASK,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd9320_dai_ops,
	},
	// XXX taiko_vifeedback/taiko_mad1


	[6] = {
		.name = "modem_dummy",
		.id = 6,
		.capture = {
			.stream_name = "AIF3 Capture",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				 SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min = 8000,
			.rate_max = 48000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &dummy_dai_ops,
	},
};

static const struct snd_soc_dapm_route wcd9320_audio_map[] = {
	/* SLIMBUS Connections */
	{"AIF1 CAP", NULL, "AIF1_CAP Mixer"},
	{"AIF2 CAP", NULL, "AIF2_CAP Mixer"},
	{"AIF3 CAP", NULL, "AIF3_CAP Mixer"},
	{"AIF4 VI", NULL, "SPK_OUT"},

	/* MAD */
	{"AIF4 MAD", NULL, "CDC_CONN"},
	{"MADONOFF", "Switch", "MADINPUT"},
	{"AIF4 MAD", NULL, "MADONOFF"},

	/* SLIM_MIXER("AIF1_CAP Mixer"),*/
	{"AIF1_CAP Mixer", "SLIM TX1", "SLIM TX1 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX2", "SLIM TX2 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX3", "SLIM TX3 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX4", "SLIM TX4 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX5", "SLIM TX5 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX6", "SLIM TX6 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX7", "SLIM TX7 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX8", "SLIM TX8 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX9", "SLIM TX9 MUX"},
	{"AIF1_CAP Mixer", "SLIM TX10", "SLIM TX10 MUX"},
	/* SLIM_MIXER("AIF2_CAP Mixer"),*/
	{"AIF2_CAP Mixer", "SLIM TX1", "SLIM TX1 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX2", "SLIM TX2 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX3", "SLIM TX3 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX4", "SLIM TX4 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX5", "SLIM TX5 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX6", "SLIM TX6 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX7", "SLIM TX7 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX8", "SLIM TX8 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX9", "SLIM TX9 MUX"},
	{"AIF2_CAP Mixer", "SLIM TX10", "SLIM TX10 MUX"},
	/* SLIM_MIXER("AIF3_CAP Mixer"),*/
	{"AIF3_CAP Mixer", "SLIM TX1", "SLIM TX1 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX2", "SLIM TX2 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX3", "SLIM TX3 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX4", "SLIM TX4 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX5", "SLIM TX5 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX6", "SLIM TX6 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX7", "SLIM TX7 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX8", "SLIM TX8 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX9", "SLIM TX9 MUX"},
	{"AIF3_CAP Mixer", "SLIM TX10", "SLIM TX10 MUX"},

	{"SLIM TX1 MUX", "DEC1", "DEC1 MUX"},

	{"SLIM TX2 MUX", "DEC2", "DEC2 MUX"},

	{"SLIM TX3 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX3 MUX", "RMIX1", "RX1 MIX1"},
	{"SLIM TX3 MUX", "RMIX2", "RX2 MIX1"},
	{"SLIM TX3 MUX", "RMIX3", "RX3 MIX1"},
	{"SLIM TX3 MUX", "RMIX4", "RX4 MIX1"},
	{"SLIM TX3 MUX", "RMIX5", "RX5 MIX1"},
	{"SLIM TX3 MUX", "RMIX6", "RX6 MIX1"},
	{"SLIM TX3 MUX", "RMIX7", "RX7 MIX1"},

	{"SLIM TX4 MUX", "DEC4", "DEC4 MUX"},

	{"SLIM TX5 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX5 MUX", "RMIX1", "RX1 MIX1"},
	{"SLIM TX5 MUX", "RMIX2", "RX2 MIX1"},
	{"SLIM TX5 MUX", "RMIX3", "RX3 MIX1"},
	{"SLIM TX5 MUX", "RMIX4", "RX4 MIX1"},
	{"SLIM TX5 MUX", "RMIX5", "RX5 MIX1"},
	{"SLIM TX5 MUX", "RMIX6", "RX6 MIX1"},
	{"SLIM TX5 MUX", "RMIX7", "RX7 MIX1"},

	{"SLIM TX6 MUX", "DEC6", "DEC6 MUX"},

	{"SLIM TX7 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX7 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX7 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX7 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX7 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX7 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX7 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX7 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX7 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX7 MUX", "DEC10", "DEC10 MUX"},
	{"SLIM TX7 MUX", "RMIX1", "RX1 MIX1"},
	{"SLIM TX7 MUX", "RMIX2", "RX2 MIX1"},
	{"SLIM TX7 MUX", "RMIX3", "RX3 MIX1"},
	{"SLIM TX7 MUX", "RMIX4", "RX4 MIX1"},
	{"SLIM TX7 MUX", "RMIX5", "RX5 MIX1"},
	{"SLIM TX7 MUX", "RMIX6", "RX6 MIX1"},
	{"SLIM TX7 MUX", "RMIX7", "RX7 MIX1"},

	{"SLIM TX8 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX8 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX8 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX8 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX8 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX8 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX8 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX8 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX8 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX8 MUX", "DEC10", "DEC10 MUX"},

	{"SLIM TX9 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX9 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX9 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX9 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX9 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX9 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX9 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX9 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX9 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX9 MUX", "DEC10", "DEC10 MUX"},

	{"SLIM TX10 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX10 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX10 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX10 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX10 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX10 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX10 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX10 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX10 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX10 MUX", "DEC10", "DEC10 MUX"},

	/* Earpiece (RX MIX1) */
	{"EAR", NULL, "EAR PA"},
	{"EAR PA", NULL, "EAR_PA_MIXER"},
	{"EAR_PA_MIXER", NULL, "DAC1"},
	{"DAC1", NULL, "RX_BIAS"},

	{"ANC EAR", NULL, "ANC EAR PA"},
	{"ANC EAR PA", NULL, "EAR_PA_MIXER"},
	{"ANC1 FB MUX", "EAR_HPH_L", "RX1 MIX2"},
	{"ANC1 FB MUX", "EAR_LINE_1", "RX2 MIX2"},

	/* Headset (RX MIX1 and RX MIX2) */
	{"HEADPHONE", NULL, "HPHL"},
	{"HEADPHONE", NULL, "HPHR"},

	{"HPHL", NULL, "HPHL_PA_MIXER"},
	{"HPHL_PA_MIXER", NULL, "HPHL DAC"},
	{"HPHL DAC", NULL, "RX_BIAS"},

	{"HPHR", NULL, "HPHR_PA_MIXER"},
	{"HPHR_PA_MIXER", NULL, "HPHR DAC"},
	{"HPHR DAC", NULL, "RX_BIAS"},

	{"ANC HEADPHONE", NULL, "ANC HPHL"},
	{"ANC HEADPHONE", NULL, "ANC HPHR"},

	{"ANC HPHL", NULL, "HPHL_PA_MIXER"},
	{"ANC HPHR", NULL, "HPHR_PA_MIXER"},

	{"ANC1 MUX", "ADC1", "ADC1"},
	{"ANC1 MUX", "ADC2", "ADC2"},
	{"ANC1 MUX", "ADC3", "ADC3"},
	{"ANC1 MUX", "ADC4", "ADC4"},
	{"ANC1 MUX", "DMIC1", "DMIC1"},
	{"ANC1 MUX", "DMIC2", "DMIC2"},
	{"ANC1 MUX", "DMIC3", "DMIC3"},
	{"ANC1 MUX", "DMIC4", "DMIC4"},
	{"ANC1 MUX", "DMIC5", "DMIC5"},
	{"ANC1 MUX", "DMIC6", "DMIC6"},
	{"ANC2 MUX", "ADC1", "ADC1"},
	{"ANC2 MUX", "ADC2", "ADC2"},
	{"ANC2 MUX", "ADC3", "ADC3"},
	{"ANC2 MUX", "ADC4", "ADC4"},

	{"ANC HPHR", NULL, "CDC_CONN"},

	{"DAC1", "Switch", "CLASS_H_DSM MUX"},
	{"HPHL DAC", "Switch", "CLASS_H_DSM MUX"},
	{"HPHR DAC", NULL, "RX2 CHAIN"},

	{"LINEOUT1", NULL, "LINEOUT1 PA"},
	{"LINEOUT2", NULL, "LINEOUT2 PA"},
	{"LINEOUT3", NULL, "LINEOUT3 PA"},
	{"LINEOUT4", NULL, "LINEOUT4 PA"},
	{"SPK_OUT", NULL, "SPK PA"},

	{"LINEOUT1 PA", NULL, "LINEOUT1_PA_MIXER"},
	{"LINEOUT1_PA_MIXER", NULL, "LINEOUT1 DAC"},

	{"LINEOUT2 PA", NULL, "LINEOUT2_PA_MIXER"},
	{"LINEOUT2_PA_MIXER", NULL, "LINEOUT2 DAC"},

	{"LINEOUT3 PA", NULL, "LINEOUT3_PA_MIXER"},
	{"LINEOUT3_PA_MIXER", NULL, "LINEOUT3 DAC"},

	{"LINEOUT4 PA", NULL, "LINEOUT4_PA_MIXER"},
	{"LINEOUT4_PA_MIXER", NULL, "LINEOUT4 DAC"},

	{"LINEOUT1 DAC", NULL, "RX3 MIX1"},

	{"RDAC5 MUX", "DEM3_INV", "RX3 MIX1"},
	{"RDAC5 MUX", "DEM4", "RX4 MIX1"},

	{"LINEOUT3 DAC", NULL, "RDAC5 MUX"},

	{"LINEOUT2 DAC", NULL, "RX5 MIX1"},

	{"RDAC7 MUX", "DEM5_INV", "RX5 MIX1"},
	{"RDAC7 MUX", "DEM6", "RX6 MIX1"},

	{"LINEOUT4 DAC", NULL, "RDAC7 MUX"},

	{"SPK PA", NULL, "SPK DAC"},
	{"SPK DAC", NULL, "RX7 MIX2"},
	{"SPK DAC", NULL, "VDD_SPKDRV"},

	{"CLASS_H_DSM MUX", "DSM_HPHL_RX1", "RX1 CHAIN"},

	{"RX1 CHAIN", NULL, "RX1 MIX2"},
	{"RX2 CHAIN", NULL, "RX2 MIX2"},
	{"RX1 MIX2", NULL, "ANC1 MUX"},
	{"RX2 MIX2", NULL, "ANC2 MUX"},

	{"LINEOUT1 DAC", NULL, "RX_BIAS"},
	{"LINEOUT2 DAC", NULL, "RX_BIAS"},
	{"LINEOUT3 DAC", NULL, "RX_BIAS"},
	{"LINEOUT4 DAC", NULL, "RX_BIAS"},
	{"SPK DAC", NULL, "RX_BIAS"},

	{"RX7 MIX1", NULL, "COMP0_CLK"},
	{"RX1 MIX1", NULL, "COMP1_CLK"},
	{"RX2 MIX1", NULL, "COMP1_CLK"},
	{"RX3 MIX1", NULL, "COMP2_CLK"},
	{"RX5 MIX1", NULL, "COMP2_CLK"},

	{"RX1 MIX1", NULL, "RX1 MIX1 INP1"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP2"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP3"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP1"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP2"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP1"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP2"},
	{"RX4 MIX1", NULL, "RX4 MIX1 INP1"},
	{"RX4 MIX1", NULL, "RX4 MIX1 INP2"},
	{"RX5 MIX1", NULL, "RX5 MIX1 INP1"},
	{"RX5 MIX1", NULL, "RX5 MIX1 INP2"},
	{"RX6 MIX1", NULL, "RX6 MIX1 INP1"},
	{"RX6 MIX1", NULL, "RX6 MIX1 INP2"},
	{"RX7 MIX1", NULL, "RX7 MIX1 INP1"},
	{"RX7 MIX1", NULL, "RX7 MIX1 INP2"},
	{"RX1 MIX2", NULL, "RX1 MIX1"},
	{"RX1 MIX2", NULL, "RX1 MIX2 INP1"},
	{"RX1 MIX2", NULL, "RX1 MIX2 INP2"},
	{"RX2 MIX2", NULL, "RX2 MIX1"},
	{"RX2 MIX2", NULL, "RX2 MIX2 INP1"},
	{"RX2 MIX2", NULL, "RX2 MIX2 INP2"},
	{"RX7 MIX2", NULL, "RX7 MIX1"},
	{"RX7 MIX2", NULL, "RX7 MIX2 INP1"},
	{"RX7 MIX2", NULL, "RX7 MIX2 INP2"},

	/* SLIM_MUX("AIF1_PB", "AIF1 PB"),*/
	{"SLIM RX1 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX2 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX3 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX4 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX5 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX6 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX7 MUX", "AIF1_PB", "AIF1 PB"},
	/* SLIM_MUX("AIF2_PB", "AIF2 PB"),*/
	{"SLIM RX1 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX2 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX3 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX4 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX5 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX6 MUX", "AIF2_PB", "AIF2 PB"},
	{"SLIM RX7 MUX", "AIF2_PB", "AIF2 PB"},
	/* SLIM_MUX("AIF3_PB", "AIF3 PB"),*/
	{"SLIM RX1 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX2 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX3 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX4 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX5 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX6 MUX", "AIF3_PB", "AIF3 PB"},
	{"SLIM RX7 MUX", "AIF3_PB", "AIF3 PB"},

	{"SLIM RX1", NULL, "SLIM RX1 MUX"},
	{"SLIM RX2", NULL, "SLIM RX2 MUX"},
	{"SLIM RX3", NULL, "SLIM RX3 MUX"},
	{"SLIM RX4", NULL, "SLIM RX4 MUX"},
	{"SLIM RX5", NULL, "SLIM RX5 MUX"},
	{"SLIM RX6", NULL, "SLIM RX6 MUX"},
	{"SLIM RX7", NULL, "SLIM RX7 MUX"},

	{"RX1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX1 MIX1 INP1", "IIR1", "IIR1"},
	{"RX1 MIX1 INP1", "IIR2", "IIR2"},
	{"RX1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX1 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX1 MIX1 INP2", "IIR1", "IIR1"},
	{"RX1 MIX1 INP2", "IIR2", "IIR2"},
	{"RX1 MIX1 INP3", "RX1", "SLIM RX1"},
	{"RX1 MIX1 INP3", "RX2", "SLIM RX2"},
	{"RX1 MIX1 INP3", "RX3", "SLIM RX3"},
	{"RX1 MIX1 INP3", "RX4", "SLIM RX4"},
	{"RX1 MIX1 INP3", "RX5", "SLIM RX5"},
	{"RX1 MIX1 INP3", "RX6", "SLIM RX6"},
	{"RX1 MIX1 INP3", "RX7", "SLIM RX7"},
	{"RX2 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX2 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX2 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX2 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX2 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX2 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX2 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX2 MIX1 INP1", "IIR1", "IIR1"},
	{"RX2 MIX1 INP1", "IIR2", "IIR2"},
	{"RX2 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX2 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX2 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX2 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX2 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX2 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX2 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX2 MIX1 INP2", "IIR1", "IIR1"},
	{"RX2 MIX1 INP2", "IIR2", "IIR2"},
	{"RX3 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX3 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX3 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX3 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX3 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX3 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX3 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX3 MIX1 INP1", "IIR1", "IIR1"},
	{"RX3 MIX1 INP1", "IIR2", "IIR2"},
	{"RX3 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX3 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX3 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX3 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX3 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX3 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX3 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX3 MIX1 INP2", "IIR1", "IIR1"},
	{"RX3 MIX1 INP2", "IIR2", "IIR2"},
	{"RX4 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX4 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX4 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX4 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX4 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX4 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX4 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX4 MIX1 INP1", "IIR1", "IIR1"},
	{"RX4 MIX1 INP1", "IIR2", "IIR2"},
	{"RX4 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX4 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX4 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX4 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX4 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX4 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX4 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX4 MIX1 INP2", "IIR1", "IIR1"},
	{"RX4 MIX1 INP2", "IIR2", "IIR2"},
	{"RX5 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX5 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX5 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX5 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX5 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX5 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX5 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX5 MIX1 INP1", "IIR1", "IIR1"},
	{"RX5 MIX1 INP1", "IIR2", "IIR2"},
	{"RX5 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX5 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX5 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX5 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX5 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX5 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX5 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX5 MIX1 INP2", "IIR1", "IIR1"},
	{"RX5 MIX1 INP2", "IIR2", "IIR2"},
	{"RX6 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX6 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX6 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX6 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX6 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX6 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX6 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX6 MIX1 INP1", "IIR1", "IIR1"},
	{"RX6 MIX1 INP1", "IIR2", "IIR2"},
	{"RX6 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX6 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX6 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX6 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX6 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX6 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX6 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX6 MIX1 INP2", "IIR1", "IIR1"},
	{"RX6 MIX1 INP2", "IIR2", "IIR2"},
	{"RX7 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX7 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX7 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX7 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX7 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX7 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX7 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX7 MIX1 INP1", "IIR1", "IIR1"},
	{"RX7 MIX1 INP1", "IIR2", "IIR2"},
	{"RX7 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX7 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX7 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX7 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX7 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX7 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX7 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX7 MIX1 INP2", "IIR1", "IIR1"},
	{"RX7 MIX1 INP2", "IIR2", "IIR2"},

	/* IIR1, IIR2 inputs to Second RX Mixer on RX1, RX2 and RX7 chains. */
	{"RX1 MIX2 INP1", "IIR1", "IIR1"},
	{"RX1 MIX2 INP2", "IIR1", "IIR1"},
	{"RX2 MIX2 INP1", "IIR1", "IIR1"},
	{"RX2 MIX2 INP2", "IIR1", "IIR1"},
	{"RX7 MIX2 INP1", "IIR1", "IIR1"},
	{"RX7 MIX2 INP2", "IIR1", "IIR1"},
	{"RX1 MIX2 INP1", "IIR2", "IIR2"},
	{"RX1 MIX2 INP2", "IIR2", "IIR2"},
	{"RX2 MIX2 INP1", "IIR2", "IIR2"},
	{"RX2 MIX2 INP2", "IIR2", "IIR2"},
	{"RX7 MIX2 INP1", "IIR2", "IIR2"},
	{"RX7 MIX2 INP2", "IIR2", "IIR2"},

	/* Decimator Inputs */
	{"DEC1 MUX", "DMIC1", "DMIC1"},
	{"DEC1 MUX", "ADC6", "ADC6"},
	{"DEC1 MUX", NULL, "CDC_CONN"},
	{"DEC2 MUX", "DMIC2", "DMIC2"},
	{"DEC2 MUX", "ADC5", "ADC5"},
	{"DEC2 MUX", NULL, "CDC_CONN"},
	{"DEC3 MUX", "DMIC3", "DMIC3"},
	{"DEC3 MUX", "ADC4", "ADC4"},
	{"DEC3 MUX", NULL, "CDC_CONN"},
	{"DEC4 MUX", "DMIC4", "DMIC4"},
	{"DEC4 MUX", "ADC3", "ADC3"},
	{"DEC4 MUX", NULL, "CDC_CONN"},
	{"DEC5 MUX", "DMIC5", "DMIC5"},
	{"DEC5 MUX", "ADC2", "ADC2"},
	{"DEC5 MUX", NULL, "CDC_CONN"},
	{"DEC6 MUX", "DMIC6", "DMIC6"},
	{"DEC6 MUX", "ADC1", "ADC1"},
	{"DEC6 MUX", NULL, "CDC_CONN"},
	{"DEC7 MUX", "DMIC1", "DMIC1"},
	{"DEC7 MUX", "DMIC6", "DMIC6"},
	{"DEC7 MUX", "ADC1", "ADC1"},
	{"DEC7 MUX", "ADC6", "ADC6"},
	{"DEC7 MUX", NULL, "CDC_CONN"},
	{"DEC8 MUX", "DMIC2", "DMIC2"},
	{"DEC8 MUX", "DMIC5", "DMIC5"},
	{"DEC8 MUX", "ADC2", "ADC2"},
	{"DEC8 MUX", "ADC5", "ADC5"},
	{"DEC8 MUX", NULL, "CDC_CONN"},
	{"DEC9 MUX", "DMIC4", "DMIC4"},
	{"DEC9 MUX", "DMIC5", "DMIC5"},
	{"DEC9 MUX", "ADC2", "ADC2"},
	{"DEC9 MUX", "ADC3", "ADC3"},
	{"DEC9 MUX", NULL, "CDC_CONN"},
	{"DEC10 MUX", "DMIC3", "DMIC3"},
	{"DEC10 MUX", "DMIC6", "DMIC6"},
	{"DEC10 MUX", "ADC1", "ADC1"},
	{"DEC10 MUX", "ADC4", "ADC4"},
	{"DEC10 MUX", NULL, "CDC_CONN"},

	/* ADC Connections */
	{"ADC1", NULL, "AMIC1"},
	{"ADC2", NULL, "AMIC2"},
	{"ADC3", NULL, "AMIC3"},
	{"ADC4", NULL, "AMIC4"},
	{"ADC5", NULL, "AMIC5"},
	{"ADC6", NULL, "AMIC6"},

	/* AUX PGA Connections */
	{"EAR_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"HPHL_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"HPHR_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"LINEOUT1_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"LINEOUT2_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"LINEOUT3_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"LINEOUT4_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"AUX_PGA_Left", NULL, "AMIC5"},
	{"AUX_PGA_Right", NULL, "AMIC6"},

	{"IIR1", NULL, "IIR1 INP1 MUX"},
	{"IIR1 INP1 MUX", "DEC1", "DEC1 MUX"},
	{"IIR1 INP1 MUX", "DEC2", "DEC2 MUX"},
	{"IIR1 INP1 MUX", "DEC3", "DEC3 MUX"},
	{"IIR1 INP1 MUX", "DEC4", "DEC4 MUX"},
	{"IIR1 INP1 MUX", "DEC5", "DEC5 MUX"},
	{"IIR1 INP1 MUX", "DEC6", "DEC6 MUX"},
	{"IIR1 INP1 MUX", "DEC7", "DEC7 MUX"},
	{"IIR1 INP1 MUX", "DEC8", "DEC8 MUX"},
	{"IIR1 INP1 MUX", "DEC9", "DEC9 MUX"},
	{"IIR1 INP1 MUX", "DEC10", "DEC10 MUX"},
	{"IIR1 INP1 MUX", "RX1", "SLIM RX1"},
	{"IIR1 INP1 MUX", "RX2", "SLIM RX2"},
	{"IIR1 INP1 MUX", "RX3", "SLIM RX3"},
	{"IIR1 INP1 MUX", "RX4", "SLIM RX4"},
	{"IIR1 INP1 MUX", "RX5", "SLIM RX5"},
	{"IIR1 INP1 MUX", "RX6", "SLIM RX6"},
	{"IIR1 INP1 MUX", "RX7", "SLIM RX7"},

	{"IIR2", NULL, "IIR2 INP1 MUX"},
	{"IIR2 INP1 MUX", "DEC1", "DEC1 MUX"},
	{"IIR2 INP1 MUX", "DEC2", "DEC2 MUX"},
	{"IIR2 INP1 MUX", "DEC3", "DEC3 MUX"},
	{"IIR2 INP1 MUX", "DEC4", "DEC4 MUX"},
	{"IIR2 INP1 MUX", "DEC5", "DEC5 MUX"},
	{"IIR2 INP1 MUX", "DEC6", "DEC6 MUX"},
	{"IIR2 INP1 MUX", "DEC7", "DEC7 MUX"},
	{"IIR2 INP1 MUX", "DEC8", "DEC8 MUX"},
	{"IIR2 INP1 MUX", "DEC9", "DEC9 MUX"},
	{"IIR2 INP1 MUX", "DEC10", "DEC10 MUX"},
	{"IIR2 INP1 MUX", "RX1", "SLIM RX1"},
	{"IIR2 INP1 MUX", "RX2", "SLIM RX2"},
	{"IIR2 INP1 MUX", "RX3", "SLIM RX3"},
	{"IIR2 INP1 MUX", "RX4", "SLIM RX4"},
	{"IIR2 INP1 MUX", "RX5", "SLIM RX5"},
	{"IIR2 INP1 MUX", "RX6", "SLIM RX6"},
	{"IIR2 INP1 MUX", "RX7", "SLIM RX7"},

	{"MIC BIAS1 Internal1", NULL, "LDO_H"},
	{"MIC BIAS1 Internal2", NULL, "LDO_H"},
	{"MIC BIAS1 External", NULL, "LDO_H"},
	{"MIC BIAS2 Internal1", NULL, "LDO_H"},
	{"MIC BIAS2 Internal2", NULL, "LDO_H"},
	{"MIC BIAS2 Internal3", NULL, "LDO_H"},
	{"MIC BIAS2 External", NULL, "LDO_H"},
	{"MIC BIAS3 Internal1", NULL, "LDO_H"},
	{"MIC BIAS3 Internal2", NULL, "LDO_H"},
	{"MIC BIAS3 External", NULL, "LDO_H"},
	{"MIC BIAS4 External", NULL, "LDO_H"},
	{DAPM_MICBIAS2_EXTERNAL_STANDALONE, NULL, "LDO_H Standalone"},
};

static const struct snd_soc_dapm_widget wcd9320_dapm_widgets[] = {
	/* RX stuff */
	SND_SOC_DAPM_OUTPUT("EAR"),

	SND_SOC_DAPM_PGA_E("EAR PA", WCD9320_RX_EAR_EN, 4, 0, NULL, 0,
			taiko_codec_enable_ear_pa, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("DAC1", WCD9320_RX_EAR_EN, 6, 0, dac1_switch,
		ARRAY_SIZE(dac1_switch), taiko_codec_ear_dac_event,
		SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_AIF_IN_E("AIF1 PB", "AIF1 Playback", 0, SND_SOC_NOPM,
				AIF1_PB, 0, taiko_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("AIF2 PB", "AIF2 Playback", 0, SND_SOC_NOPM,
				AIF2_PB, 0, taiko_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("AIF3 PB", "AIF3 Playback", 0, SND_SOC_NOPM,
				AIF3_PB, 0, taiko_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("SLIM RX1 MUX", SND_SOC_NOPM, WCD9320_RX1, 0,
				&slim_rx_mux[WCD9320_RX1]),
	SND_SOC_DAPM_MUX("SLIM RX2 MUX", SND_SOC_NOPM, WCD9320_RX2, 0,
				&slim_rx_mux[WCD9320_RX2]),
	SND_SOC_DAPM_MUX("SLIM RX3 MUX", SND_SOC_NOPM, WCD9320_RX3, 0,
				&slim_rx_mux[WCD9320_RX3]),
	SND_SOC_DAPM_MUX("SLIM RX4 MUX", SND_SOC_NOPM, WCD9320_RX4, 0,
				&slim_rx_mux[WCD9320_RX4]),
	SND_SOC_DAPM_MUX("SLIM RX5 MUX", SND_SOC_NOPM, WCD9320_RX5, 0,
				&slim_rx_mux[WCD9320_RX5]),
	SND_SOC_DAPM_MUX("SLIM RX6 MUX", SND_SOC_NOPM, WCD9320_RX6, 0,
				&slim_rx_mux[WCD9320_RX6]),
	SND_SOC_DAPM_MUX("SLIM RX7 MUX", SND_SOC_NOPM, WCD9320_RX7, 0,
				&slim_rx_mux[WCD9320_RX7]),

	SND_SOC_DAPM_MIXER("SLIM RX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX3", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX4", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX5", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX6", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX7", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Headphone */
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_PGA_E("HPHL", WCD9320_RX_HPH_CNP_EN, 5, 0, NULL, 0,
		taiko_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("HPHL DAC", WCD9320_RX_HPH_L_DAC_CTL, 7, 0,
		hphl_switch, ARRAY_SIZE(hphl_switch), taiko_hphl_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("HPHR", WCD9320_RX_HPH_CNP_EN, 4, 0, NULL, 0,
		taiko_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU |	SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("HPHR DAC", NULL, WCD9320_RX_HPH_R_DAC_CTL, 7, 0,
		taiko_hphr_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* Speaker */
	SND_SOC_DAPM_OUTPUT("LINEOUT1"),
	SND_SOC_DAPM_OUTPUT("LINEOUT2"),
	SND_SOC_DAPM_OUTPUT("LINEOUT3"),
	SND_SOC_DAPM_OUTPUT("LINEOUT4"),
	SND_SOC_DAPM_OUTPUT("SPK_OUT"),

	SND_SOC_DAPM_PGA_E("LINEOUT1 PA", WCD9320_RX_LINE_CNP_EN, 0, 0, NULL,
			0, taiko_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT2 PA", WCD9320_RX_LINE_CNP_EN, 1, 0, NULL,
			0, taiko_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT3 PA", WCD9320_RX_LINE_CNP_EN, 2, 0, NULL,
			0, taiko_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT4 PA", WCD9320_RX_LINE_CNP_EN, 3, 0, NULL,
			0, taiko_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("SPK PA", SND_SOC_NOPM, 0, 0 , NULL,
			   0, taiko_codec_enable_spk_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("LINEOUT1 DAC", NULL, WCD9320_RX_LINE_1_DAC_CTL, 7, 0
		, taiko_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("LINEOUT2 DAC", NULL, WCD9320_RX_LINE_2_DAC_CTL, 7, 0
		, taiko_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("LINEOUT3 DAC", NULL, WCD9320_RX_LINE_3_DAC_CTL, 7, 0
		, taiko_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH("LINEOUT3 DAC GROUND", SND_SOC_NOPM, 0, 0,
				&lineout3_ground_switch),
	SND_SOC_DAPM_DAC_E("LINEOUT4 DAC", NULL, WCD9320_RX_LINE_4_DAC_CTL, 7, 0
		, taiko_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH("LINEOUT4 DAC GROUND", SND_SOC_NOPM, 0, 0,
				&lineout4_ground_switch),

	SND_SOC_DAPM_DAC_E("SPK DAC", NULL, SND_SOC_NOPM, 0, 0,
			   taiko_spk_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("VDD_SPKDRV", SND_SOC_NOPM, 0, 0,
			    taiko_codec_enable_vdd_spkr,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX7 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER_E("RX1 MIX2", WCD9320_CDC_CLK_RX_B1_CTL, 0, 0, NULL,
		0, taiko_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX2 MIX2", WCD9320_CDC_CLK_RX_B1_CTL, 1, 0, NULL,
		0, taiko_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX3 MIX1", WCD9320_CDC_CLK_RX_B1_CTL, 2, 0, NULL,
		0, taiko_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX4 MIX1", WCD9320_CDC_CLK_RX_B1_CTL, 3, 0, NULL,
		0, taiko_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX5 MIX1", WCD9320_CDC_CLK_RX_B1_CTL, 4, 0, NULL,
		0, taiko_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX6 MIX1", WCD9320_CDC_CLK_RX_B1_CTL, 5, 0, NULL,
		0, taiko_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX7 MIX2", WCD9320_CDC_CLK_RX_B1_CTL, 6, 0, NULL,
		0, taiko_codec_enable_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),


	SND_SOC_DAPM_MIXER("RX1 CHAIN", WCD9320_CDC_RX1_B6_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 CHAIN", WCD9320_CDC_RX2_B6_CTL, 5, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP3", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp3_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx3_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx3_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX4 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx4_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX4 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx4_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX5 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx5_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX5 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx5_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX6 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx6_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX6 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx6_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX7 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx7_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX7 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx7_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx1_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX2 INP2", SND_SOC_NOPM, 0, 0,
		&rx1_mix2_inp2_mux),
	SND_SOC_DAPM_MUX("RX2 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx2_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX2 INP2", SND_SOC_NOPM, 0, 0,
		&rx2_mix2_inp2_mux),
	SND_SOC_DAPM_MUX("RX7 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx7_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX7 MIX2 INP2", SND_SOC_NOPM, 0, 0,
		&rx7_mix2_inp2_mux),

	SND_SOC_DAPM_MUX("RDAC5 MUX", SND_SOC_NOPM, 0, 0,
		&rx_dac5_mux),
	SND_SOC_DAPM_MUX("RDAC7 MUX", SND_SOC_NOPM, 0, 0,
		&rx_dac7_mux),

	SND_SOC_DAPM_MUX_E("CLASS_H_DSM MUX", SND_SOC_NOPM, 0, 0,
		&class_h_dsm_mux, taiko_codec_dsm_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_rx_bias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("CDC_I2S_RX_CONN", WCD9320_CDC_CLK_OTHR_CTL, 5, 0,
			    NULL, 0),

	/* TX */

	SND_SOC_DAPM_SUPPLY("CDC_CONN", WCD9320_CDC_CLK_OTHR_CTL, 2, 0, NULL,
		0),

	SND_SOC_DAPM_SUPPLY("LDO_H", SND_SOC_NOPM, 7, 0,
			    taiko_codec_enable_ldo_h,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	/*
	 * DAPM 'LDO_H Standalone' is to be powered by mbhc driver after
	 * acquring codec_resource lock.
	 * So call __taiko_codec_enable_ldo_h instead and avoid deadlock.
	 */
	SND_SOC_DAPM_SUPPLY("LDO_H Standalone", SND_SOC_NOPM, 7, 0,
			    __taiko_codec_enable_ldo_h,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("COMP0_CLK", SND_SOC_NOPM, 0, 0,
		taiko_config_compander, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("COMP1_CLK", SND_SOC_NOPM, 1, 0,
		taiko_config_compander, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("COMP2_CLK", SND_SOC_NOPM, 2, 0,
		taiko_config_compander, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_SUPPLY("MIC BIAS1 External", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS1 Internal1", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS1 Internal2", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_INPUT("AMIC3"),

	SND_SOC_DAPM_INPUT("AMIC4"),

	SND_SOC_DAPM_INPUT("AMIC5"),

	SND_SOC_DAPM_INPUT("AMIC6"),

	SND_SOC_DAPM_MUX_E("DEC1 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 0, 0,
		&dec1_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC2 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 1, 0,
		&dec2_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC3 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 2, 0,
		&dec3_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC4 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 3, 0,
		&dec4_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC5 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 4, 0,
		&dec5_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC6 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 5, 0,
		&dec6_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC7 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 6, 0,
		&dec7_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC8 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B1_CTL, 7, 0,
		&dec8_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC9 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B2_CTL, 0, 0,
		&dec9_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC10 MUX", WCD9320_CDC_CLK_TX_CLK_EN_B2_CTL, 1, 0,
		&dec10_mux, taiko_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("ANC1 MUX", SND_SOC_NOPM, 0, 0, &anc1_mux),
	SND_SOC_DAPM_MUX("ANC2 MUX", SND_SOC_NOPM, 0, 0, &anc2_mux),

	SND_SOC_DAPM_OUTPUT("ANC HEADPHONE"),
	SND_SOC_DAPM_PGA_E("ANC HPHL", SND_SOC_NOPM, 5, 0, NULL, 0,
		taiko_codec_enable_anc_hph,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_E("ANC HPHR", SND_SOC_NOPM, 4, 0, NULL, 0,
		taiko_codec_enable_anc_hph, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_OUTPUT("ANC EAR"),
	SND_SOC_DAPM_PGA_E("ANC EAR PA", SND_SOC_NOPM, 0, 0, NULL, 0,
		taiko_codec_enable_anc_ear,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("ANC1 FB MUX", SND_SOC_NOPM, 0, 0, &anc1_fb_mux),

	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_SUPPLY(DAPM_MICBIAS2_EXTERNAL_STANDALONE, SND_SOC_NOPM,
			       7, 0, taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2 External", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2 Internal1", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU |
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2 Internal2", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2 Internal3", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS3 External", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS3 Internal1", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU |
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS3 Internal2", SND_SOC_NOPM, 7, 0,
			       taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU |
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS4 External", SND_SOC_NOPM, 7,
			       0, taiko_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("AIF1 CAP", "AIF1 Capture", 0, SND_SOC_NOPM,
		AIF1_CAP, 0, taiko_codec_enable_slimtx,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("AIF2 CAP", "AIF2 Capture", 0, SND_SOC_NOPM,
		AIF2_CAP, 0, taiko_codec_enable_slimtx,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("AIF3 CAP", "AIF3 Capture", 0, SND_SOC_NOPM,
		AIF3_CAP, 0, taiko_codec_enable_slimtx,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("AIF4 VI", "VIfeed", 0, SND_SOC_NOPM,
		AIF4_VIFEED, 0, taiko_codec_enable_slimvi_feedback,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("AIF4 MAD", "AIF4 MAD TX", 0,
			       SND_SOC_NOPM, 0, 0,
			       taiko_codec_enable_mad, SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_SWITCH("MADONOFF", SND_SOC_NOPM, 0, 0,
			    &aif4_mad_switch),
	SND_SOC_DAPM_INPUT("MADINPUT"),

	SND_SOC_DAPM_MIXER("AIF1_CAP Mixer", SND_SOC_NOPM, AIF1_CAP, 0,
		aif_cap_mixer, ARRAY_SIZE(aif_cap_mixer)),

	SND_SOC_DAPM_MIXER("AIF2_CAP Mixer", SND_SOC_NOPM, AIF2_CAP, 0,
		aif_cap_mixer, ARRAY_SIZE(aif_cap_mixer)),

	SND_SOC_DAPM_MIXER("AIF3_CAP Mixer", SND_SOC_NOPM, AIF3_CAP, 0,
		aif_cap_mixer, ARRAY_SIZE(aif_cap_mixer)),

	SND_SOC_DAPM_MUX("SLIM TX1 MUX", SND_SOC_NOPM, WCD9320_TX1, 0,
		&sb_tx1_mux),
	SND_SOC_DAPM_MUX("SLIM TX2 MUX", SND_SOC_NOPM, WCD9320_TX2, 0,
		&sb_tx2_mux),
	SND_SOC_DAPM_MUX("SLIM TX3 MUX", SND_SOC_NOPM, WCD9320_TX3, 0,
		&sb_tx3_mux),
	SND_SOC_DAPM_MUX("SLIM TX4 MUX", SND_SOC_NOPM, WCD9320_TX4, 0,
		&sb_tx4_mux),
	SND_SOC_DAPM_MUX("SLIM TX5 MUX", SND_SOC_NOPM, WCD9320_TX5, 0,
		&sb_tx5_mux),
	SND_SOC_DAPM_MUX("SLIM TX6 MUX", SND_SOC_NOPM, WCD9320_TX6, 0,
		&sb_tx6_mux),
	SND_SOC_DAPM_MUX("SLIM TX7 MUX", SND_SOC_NOPM, WCD9320_TX7, 0,
		&sb_tx7_mux),
	SND_SOC_DAPM_MUX("SLIM TX8 MUX", SND_SOC_NOPM, WCD9320_TX8, 0,
		&sb_tx8_mux),
	SND_SOC_DAPM_MUX("SLIM TX9 MUX", SND_SOC_NOPM, WCD9320_TX9, 0,
		&sb_tx9_mux),
	SND_SOC_DAPM_MUX("SLIM TX10 MUX", SND_SOC_NOPM, WCD9320_TX10, 0,
		&sb_tx10_mux),

	/* Digital Mic Inputs */
	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC3", NULL, SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC4", NULL, SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC5", NULL, SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC6", NULL, SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	/* Sidetone */
	SND_SOC_DAPM_MUX("IIR1 INP1 MUX", SND_SOC_NOPM, 0, 0, &iir1_inp1_mux),
	SND_SOC_DAPM_MIXER("IIR1", WCD9320_CDC_CLK_SD_CTL, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("IIR2 INP1 MUX", SND_SOC_NOPM, 0, 0, &iir2_inp1_mux),
	SND_SOC_DAPM_MIXER("IIR2", WCD9320_CDC_CLK_SD_CTL, 1, 0, NULL, 0),

	/* AUX PGA */
	SND_SOC_DAPM_ADC_E("AUX_PGA_Left", NULL, WCD9320_RX_AUX_SW_CTL, 7, 0,
		taiko_codec_enable_aux_pga, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("AUX_PGA_Right", NULL, WCD9320_RX_AUX_SW_CTL, 6, 0,
		taiko_codec_enable_aux_pga, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	/* Lineout, ear and HPH PA Mixers */

	SND_SOC_DAPM_MIXER("EAR_PA_MIXER", SND_SOC_NOPM, 0, 0,
		ear_pa_mix, ARRAY_SIZE(ear_pa_mix)),

	SND_SOC_DAPM_MIXER("HPHL_PA_MIXER", SND_SOC_NOPM, 0, 0,
		hphl_pa_mix, ARRAY_SIZE(hphl_pa_mix)),

	SND_SOC_DAPM_MIXER("HPHR_PA_MIXER", SND_SOC_NOPM, 0, 0,
		hphr_pa_mix, ARRAY_SIZE(hphr_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT1_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout1_pa_mix, ARRAY_SIZE(lineout1_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT2_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout2_pa_mix, ARRAY_SIZE(lineout2_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT3_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout3_pa_mix, ARRAY_SIZE(lineout3_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT4_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout4_pa_mix, ARRAY_SIZE(lineout4_pa_mix)),

	/* taiko 1 ONLY (different for taiko 2) */
#if 0 // TAIKO_V1
	SND_SOC_DAPM_ADC_E("ADC1", NULL, WCD9320_TX_1_2_EN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2", NULL, WCD9320_TX_1_2_EN, 3, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC3", NULL, WCD9320_TX_3_4_EN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC4", NULL, WCD9320_TX_3_4_EN, 3, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC5", NULL, WCD9320_TX_5_6_EN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_ADC_E("ADC6", NULL, WCD9320_TX_5_6_EN, 3, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_POST_PMU),
#else
	SND_SOC_DAPM_ADC_E("ADC1", NULL, WCD9320_CDC_TX_1_GAIN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2", NULL, WCD9320_CDC_TX_2_GAIN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC3", NULL, WCD9320_CDC_TX_3_GAIN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC4", NULL, WCD9320_CDC_TX_4_GAIN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC5", NULL, WCD9320_CDC_TX_5_GAIN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_ADC_E("ADC6", NULL, WCD9320_CDC_TX_6_GAIN, 7, 0,
			   taiko_codec_enable_adc,
			   SND_SOC_DAPM_POST_PMU),
#endif

	/* not in downstream driver */
	SND_SOC_DAPM_SUPPLY("MCLK",  SND_SOC_NOPM, 0, 0,
		wcd9320_codec_enable_mclk, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

#if 0
	SND_SOC_DAPM_SUPPLY("MIC BIAS1", SND_SOC_NOPM, 0, 0,
			       wcd9320_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2", SND_SOC_NOPM, 0, 0,
			       wcd9320_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS3", SND_SOC_NOPM, 0, 0,
			       wcd9320_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS4", SND_SOC_NOPM, 0, 0,
			       wcd9320_codec_enable_micbias,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
#endif
};

static void wcd9320_codec_init(struct snd_soc_component *component)
{
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(wcd9320_codec_reg_init); i++)
		snd_soc_component_update_bits(component,
					wcd9320_codec_reg_init[i].reg,
					wcd9320_codec_reg_init[i].mask,
					wcd9320_codec_reg_init[i].val);
}

static int wcd9320_codec_probe(struct snd_soc_component *component)
{
	struct wcd9320_codec *wcd = dev_get_drvdata(component->dev);
	int i;

	printk("wcd9320_codec_probe\n");

	snd_soc_component_init_regmap(component, wcd->regmap);
	/* Class-H Init*/
	wcd->clsh_ctrl = wcd_clsh_ctrl_alloc(component, wcd->version);
	if (IS_ERR(wcd->clsh_ctrl))
		return PTR_ERR(wcd->clsh_ctrl);

	wcd->component = component;

	wcd9320_codec_init(component);

	regmap_write(wcd->regmap, WCD9320_LEAKAGE_CTL, 0x4);
	regmap_write(wcd->regmap, WCD9320_CDC_CTL, 0);
	usleep_range(5000, 5000);
	regmap_write(wcd->regmap, WCD9320_CDC_CTL, 3);
	regmap_write(wcd->regmap, WCD9320_LEAKAGE_CTL, 3);

	// irq init
	regmap_write(wcd->regmap, WCD9320_INTR_LEVEL0 + 0, 1);
	regmap_write(wcd->regmap, WCD9320_INTR_MASK0 + 0 , 0xff);

	regmap_write(wcd->regmap, WCD9320_INTR_LEVEL0 + 1, 0);
	regmap_write(wcd->regmap, WCD9320_INTR_MASK0 + 1 , 0xff);

	regmap_write(wcd->regmap, WCD9320_INTR_LEVEL0 + 2, 0);
	regmap_write(wcd->regmap, WCD9320_INTR_MASK0 + 2 , 0xff);

	regmap_write(wcd->regmap, WCD9320_INTR_LEVEL0 + 3, 0);
	regmap_write(wcd->regmap, WCD9320_INTR_MASK0 + 3 , 0x7f);

	// stuff happens...
	regmap_write(wcd->regmap, WCD9320_INTR_MASK0 + 2 , 0xff);

	// init base
	regmap_write(wcd->regmap, WCD9320_CHIP_CTL, 2);
	regmap_write(wcd->regmap, WCD9320_CDC_CLK_POWER_CTL, 3);


	/* replay of writes from taiko_handle_pdata */
	regmap_update_bits(wcd->regmap, WCD9320_LDO_H_MODE_1, 0x0C, 0xc);
	regmap_update_bits(wcd->regmap, WCD9320_MICB_CFILT_1_VAL, 0xFC, 0x60);
	regmap_update_bits(wcd->regmap, WCD9320_MICB_CFILT_2_VAL, 0xFC, 0x9c);
	regmap_update_bits(wcd->regmap, WCD9320_MICB_CFILT_3_VAL, 0xFC, 0x60);
	regmap_update_bits(wcd->regmap, WCD9320_MICB_2_CTL, 0x60, 0x20);
	regmap_write(wcd->regmap, WCD9320_CDC_ANC1_B2_CTL, 1);

	/* setup_irqs */
	regmap_write(wcd->regmap, WCD9320_INTR_MASK0, 0xfe);
	regmap_write(wcd->if_regmap, WCD9320_SLIM_PGD_PORT_INT_EN0 + 0, 0xff);
	regmap_write(wcd->if_regmap, WCD9320_SLIM_PGD_PORT_INT_EN0 + 1, 0xff);
	regmap_write(wcd->if_regmap, WCD9320_SLIM_PGD_PORT_INT_EN0 + 2, 0xff);

	/* mclk ? */
	regmap_write(wcd->regmap, WCD9320_CDC_CLK_MCLK_CTL, 1);
	for (i = 0; i < NUM_CODEC_DAIS; i++)
		INIT_LIST_HEAD(&wcd->dai[i].slim_ch_list);

	for (i = 0; i < COMPANDER_MAX; i++) {
		wcd->comp_enabled[i] = 0;
		wcd->comp_fs[i] = COMPANDER_FS_48KHZ;
	}

	return 0;//wcd9320_setup_irqs(wcd);
}

static void wcd9320_codec_remove(struct snd_soc_component *comp)
{
	//struct wcd9320_codec *wcd = dev_get_drvdata(comp->dev);

	//wcd_clsh_ctrl_free(wcd->clsh_ctrl);
	//free_irq(regmap_irq_get_virq(wcd->irq_data, WCD9320_IRQ_SLIMBUS), wcd);
}

static int wcd9320_codec_set_sysclk(struct snd_soc_component *comp,
				    int clk_id, int source,
				    unsigned int freq, int dir)
{
	struct wcd9320_codec *wcd = dev_get_drvdata(comp->dev);

	wcd->mclk_rate = freq;

#if 0
	if (wcd->mclk_rate == WCD9320_MCLK_CLK_12P288MHZ)
		snd_soc_component_update_bits(comp,
				WCD9320_CODEC_RPM_CLK_MCLK_CFG,
				WCD9320_CODEC_RPM_CLK_MCLK_CFG_MCLK_MASK,
				WCD9320_CODEC_RPM_CLK_MCLK_CFG_12P288MHZ);
	else if (wcd->mclk_rate == WCD9320_MCLK_CLK_9P6MHZ)
		snd_soc_component_update_bits(comp,
				WCD9320_CODEC_RPM_CLK_MCLK_CFG,
				WCD9320_CODEC_RPM_CLK_MCLK_CFG_MCLK_MASK,
				WCD9320_CODEC_RPM_CLK_MCLK_CFG_9P6MHZ);
#endif

	return clk_set_rate(wcd->mclk, freq);
}

static int wcd9320_codec_set_jack(struct snd_soc_component *comp,
				  struct snd_soc_jack *jack, void *data)
{
	struct wcd9320_codec *wcd = dev_get_drvdata(comp->dev);

	wcd->jack = jack;

	return 0;
}

static const struct snd_soc_component_driver wcd9320_component_drv = {
	.probe = wcd9320_codec_probe,
	.remove = wcd9320_codec_remove,
	.set_sysclk = wcd9320_codec_set_sysclk,
	.set_jack = wcd9320_codec_set_jack,
	.controls = wcd9320_snd_controls,
	.num_controls = ARRAY_SIZE(wcd9320_snd_controls),
	.dapm_widgets = wcd9320_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wcd9320_dapm_widgets),
	.dapm_routes = wcd9320_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wcd9320_audio_map),
};

static int wcd9320_probe(struct wcd9320_codec *wcd)
{
	struct device *dev = wcd->dev;

	memcpy(wcd->rx_chs, wcd9320_rx_chs, sizeof(wcd9320_rx_chs));
	memcpy(wcd->tx_chs, wcd9320_tx_chs, sizeof(wcd9320_tx_chs));

	int ret = devm_snd_soc_register_component(dev, &wcd9320_component_drv,
					       wcd9320_slim_dais,
					       ARRAY_SIZE(wcd9320_slim_dais));
	printk("ret=%i\n", ret);
	return ret;
}

static bool wcd9320_is_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0 ... 0x100:
	case WCD9320_MBHC_INSERT_DET_STATUS:
	case WCD9320_RX_HPH_L_STATUS:
	case WCD9320_RX_HPH_R_STATUS:
	case WCD9320_CDC_ANC1_IIR_B1_CTL...WCD9320_CDC_ANC1_LPF_B2_CTL:
	case WCD9320_CDC_ANC2_IIR_B1_CTL...WCD9320_CDC_ANC2_LPF_B2_CTL:
	case WCD9320_CDC_SPKR_CLIPDET_VAL0...WCD9320_CDC_SPKR_CLIPDET_VAL7:
	case WCD9320_CDC_VBAT_GAIN_MON_VAL:
	case WCD9320_CDC_RX1_VOL_CTL_B2_CTL:
	case WCD9320_CDC_RX2_VOL_CTL_B2_CTL:
	case WCD9320_CDC_RX3_VOL_CTL_B2_CTL:
	case WCD9320_CDC_RX4_VOL_CTL_B2_CTL:
	case WCD9320_CDC_RX5_VOL_CTL_B2_CTL:
	case WCD9320_CDC_RX6_VOL_CTL_B2_CTL:
	case WCD9320_CDC_RX7_VOL_CTL_B2_CTL:
	case WCD9320_CDC_TX1_VOL_CTL_GAIN:
	case WCD9320_CDC_TX2_VOL_CTL_GAIN:
	case WCD9320_CDC_TX3_VOL_CTL_GAIN:
	case WCD9320_CDC_TX4_VOL_CTL_GAIN:
	case WCD9320_CDC_TX5_VOL_CTL_GAIN:
	case WCD9320_CDC_TX6_VOL_CTL_GAIN:
	case WCD9320_CDC_TX7_VOL_CTL_GAIN:
	case WCD9320_CDC_TX8_VOL_CTL_GAIN:
	case WCD9320_CDC_TX9_VOL_CTL_GAIN:
	case WCD9320_CDC_TX10_VOL_CTL_GAIN:
	case WCD9320_CDC_IIR1_COEF_B1_CTL...WCD9320_CDC_IIR2_COEF_B2_CTL:
	case WCD9320_CDC_MBHC_EN_CTL...WCD9320_MAX_REGISTER:
	case WCD9320_CDC_ANC1_GAIN_CTL: /* XXX */
		return true;
	default:
		return false;
	}
}

static struct regmap_config wcd9320_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = WCD9320_MAX_REGISTER,
	.can_multi_write = true,
	.volatile_reg = wcd9320_is_volatile_register,
};

static struct regmap_config wcd9320_ifc_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.can_multi_write = true,
	.max_register = WCD9320_REG(0, 0x7FF), // XXX
};

static const struct regmap_irq wcd9320_irqs[] = {
	/* INTR_REG 0 */
	REGMAP_IRQ_REG(WCD9320_IRQ_SLIMBUS, 0, BIT(0)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MBHC_REMOVAL, 0, BIT(1)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MBHC_SHORT_TERM, 0, BIT(2)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MBHC_PRESS, 0, BIT(3)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MBHC_RELEASE, 0, BIT(4)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MBHC_POTENTIAL, 0, BIT(5)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MBHC_INSERTION, 0, BIT(6)),
	REGMAP_IRQ_REG(WCD9320_IRQ_BG_PRECHARGE, 0, BIT(7)),
	/* INTR_REG 1 */
	REGMAP_IRQ_REG(WCD9320_IRQ_PA1_STARTUP, 1, BIT(0)),
	REGMAP_IRQ_REG(WCD9320_IRQ_PA2_STARTUP, 1, BIT(1)),
	REGMAP_IRQ_REG(WCD9320_IRQ_PA3_STARTUP, 1, BIT(2)),
	REGMAP_IRQ_REG(WCD9320_IRQ_PA4_STARTUP, 1, BIT(3)),
	REGMAP_IRQ_REG(WCD9320_IRQ_PA5_STARTUP, 1, BIT(4)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MICBIAS1_PRECHARGE, 1, BIT(5)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MICBIAS2_PRECHARGE, 1, BIT(6)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MICBIAS3_PRECHARGE, 1, BIT(7)),
	/* INTR_REG 2 */
	REGMAP_IRQ_REG(WCD9320_IRQ_HPH_PA_OCPL_FAULT, 2, BIT(0)),
	REGMAP_IRQ_REG(WCD9320_IRQ_HPH_PA_OCPR_FAULT, 2, BIT(1)),
	REGMAP_IRQ_REG(WCD9320_IRQ_EAR_PA_OCPL_FAULT, 2, BIT(2)),
	REGMAP_IRQ_REG(WCD9320_IRQ_HPH_L_PA_STARTUP, 2, BIT(3)),
	REGMAP_IRQ_REG(WCD9320_IRQ_HPH_R_PA_STARTUP, 2, BIT(4)),
	REGMAP_IRQ_REG(WCD9320_IRQ_EAR_PA_STARTUP, 2, BIT(5)),
	REGMAP_IRQ_REG(WCD9320_IRQ_RESERVED_0, 2, BIT(6)),
	REGMAP_IRQ_REG(WCD9320_IRQ_RESERVED_1, 2, BIT(7)),
	/* INTR_REG 3 */
	REGMAP_IRQ_REG(WCD9320_IRQ_MAD_AUDIO, 3, BIT(0)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MAD_BEACON, 3, BIT(1)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MAD_ULTRASOUND, 3, BIT(2)),
	REGMAP_IRQ_REG(WCD9320_IRQ_SPEAKER_CLIPPING, 3, BIT(3)),
	REGMAP_IRQ_REG(WCD9320_IRQ_MBHC_JACK_SWITCH, 3, BIT(4)),
	REGMAP_IRQ_REG(WCD9320_IRQ_VBAT_MONITOR_ATTACK, 3, BIT(5)),
	REGMAP_IRQ_REG(WCD9320_IRQ_VBAT_MONITOR_RELEASE, 3, BIT(6)),
};

static const struct regmap_irq_chip wcd9320_regmap_irq1_chip = {
	.name = "wcd9320_pin1_irq",
	.status_base = WCD9320_INTR_STATUS0,
	.mask_base = WCD9320_INTR_MASK0,
	.ack_base = WCD9320_INTR_CLEAR0,
	.type_base = WCD9320_INTR_LEVEL0,
	.num_regs = 4,
	.irqs = wcd9320_irqs,
	.num_irqs = ARRAY_SIZE(wcd9320_irqs),
};

static int wcd9320_parse_dt(struct wcd9320_codec *wcd)
{
	struct device *dev = wcd->dev;
	struct device_node *np = dev->of_node;
	int ret;

	wcd->reset_gpio = of_get_named_gpio(np,	"reset-gpios", 0);
	if (wcd->reset_gpio < 0) {
		dev_err(dev, "Reset GPIO missing from DT\n");
		return wcd->reset_gpio;
	}

	wcd->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(wcd->mclk)) {
		dev_err(dev, "mclk not found\n");
		return PTR_ERR(wcd->mclk);
	}

	clk_set_rate(wcd->mclk, 9600000);
	clk_prepare_enable(wcd->mclk);

	wcd->native_clk = devm_clk_get(dev, "slimbus");
	if (IS_ERR(wcd->native_clk)) {
		dev_err(dev, "slimbus clock not found\n");
		return PTR_ERR(wcd->native_clk);
	}

	wcd->supplies[0].supply = "vdd-buck";
	wcd->supplies[1].supply = "vdd-buck-sido";
	wcd->supplies[2].supply = "vdd-tx";
	wcd->supplies[3].supply = "vdd-rx";
	wcd->supplies[4].supply = "vdd-io";

	ret = regulator_bulk_get(dev, WCD9320_MAX_SUPPLY, wcd->supplies);
	if (ret) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	return 0;
}

static int wcd9320_power_on_reset(struct wcd9320_codec *wcd)
{
	struct device *dev = wcd->dev;
	int ret;

	ret = regulator_bulk_enable(WCD9320_MAX_SUPPLY, wcd->supplies);
	if (ret) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	/*
	 * For WCD9320, it takes about 600us for the Vout_A and
	 * Vout_D to be ready after BUCK_SIDO is powered up.
	 * SYS_RST_N shouldn't be pulled high during this time
	 * Toggle the reset line to make sure the reset pulse is
	 * correctly applied
	 */
	usleep_range(600, 650);

	gpio_direction_output(wcd->reset_gpio, 0);
	msleep(20);
	gpio_set_value(wcd->reset_gpio, 1);
	msleep(20);

	return 0;
}

static int wcd9320_bring_up(struct wcd9320_codec *wcd)
{
	struct regmap *rm = wcd->regmap;
	int ctl, status, byte0, byte1, byte2, byte3;

	regmap_read(rm, WCD9320_CHIP_CTL, &ctl);
	regmap_read(rm, WCD9320_CHIP_STATUS, &status);

	regmap_read(rm, WCD9320_CHIP_ID_BYTE_0, &byte0);
	regmap_read(rm, WCD9320_CHIP_ID_BYTE_1, &byte1);
	regmap_read(rm, WCD9320_CHIP_ID_BYTE_2, &byte2);
	regmap_read(rm, WCD9320_CHIP_ID_BYTE_3, &byte3);

	printk("WCD9320 version %u %u, %u.%u.%u.%u\n", ctl, status, byte0, byte1, byte2, byte3);

	return 0;
}

static int wcd9320_irq_init(struct wcd9320_codec *wcd)
{
	int ret;

	/*
	 * INTR1 consists of all possible interrupt sources Ear OCP,
	 * HPH OCP, MBHC, MAD, VBAT, and SVA
	 */
	wcd->intr1 = of_irq_get_byname(wcd->dev->of_node, "intr1");
	if (wcd->intr1 < 0) {
		if (wcd->intr1 != -EPROBE_DEFER)
			dev_err(wcd->dev, "Unable to configure IRQ\n");

		return wcd->intr1;
	}

	ret = devm_regmap_add_irq_chip(wcd->dev, wcd->regmap, wcd->intr1,
				 IRQF_TRIGGER_HIGH, 0,
				 &wcd9320_regmap_irq1_chip, &wcd->irq_data);
	if (ret) {
		dev_err(wcd->dev, "Failed to register IRQ chip: %d\n", ret);
		ret = 0; // XXX
	}

	return ret;
}

static int wcd9320_slim_probe(struct slim_device *slim)
{
	struct device *dev = &slim->dev;
	struct wcd9320_codec *wcd;
	int ret;

	/* Interface device */
	if (slim->e_addr.dev_index == WCD9320_SLIM_INTERFACE_DEVICE_INDEX)
		return 0;

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return	-ENOMEM;

	wcd->dev = dev;
	ret = wcd9320_parse_dt(wcd);
	if (ret) {
		dev_err(dev, "Error parsing DT: %d\n", ret);
		return ret;
	}

	ret = wcd9320_power_on_reset(wcd);
	if (ret)
		return ret;

	dev_set_drvdata(dev, wcd);

	return 0;
}

static int wcd9320_slim_status(struct slim_device *sdev,
			       enum slim_device_status status)
{
	struct device_node *ifc_dev_np;
	struct wcd9320_codec *wcd;
	int ret;

	printk("status: %u dev_index=%u\n", status, sdev->e_addr.dev_index);

	if (sdev->e_addr.dev_index == WCD9320_SLIM_INTERFACE_DEVICE_INDEX)
		return 0;

	wcd = dev_get_drvdata(&sdev->dev);

	ifc_dev_np = of_parse_phandle(wcd->dev->of_node, "slim-ifc-dev", 0);
	if (!ifc_dev_np) {
		dev_err(wcd->dev, "No Interface device found\n");
		return -EINVAL;
	}

	wcd->slim = sdev;
	wcd->slim_ifc_dev = of_slim_get_device(sdev->ctrl, ifc_dev_np);
	if (!wcd->slim_ifc_dev) {
		dev_err(wcd->dev, "Unable to get SLIM Interface device\n");
		return -EINVAL;
	}

	wcd->regmap = regmap_init_slimbus(sdev, &wcd9320_regmap_config);
	if (IS_ERR(wcd->regmap)) {
		dev_err(wcd->dev, "Failed to allocate slim register map\n");
		return PTR_ERR(wcd->regmap);
	}

	wcd->if_regmap = regmap_init_slimbus(wcd->slim_ifc_dev,
						  &wcd9320_ifc_regmap_config);
	if (IS_ERR(wcd->if_regmap)) {
		dev_err(wcd->dev, "Failed to allocate ifc register map\n");
		return PTR_ERR(wcd->if_regmap);
	}

	ret = wcd9320_bring_up(wcd);
	if (ret) {
		dev_err(wcd->dev, "Failed to bringup WCD9320\n");
		return ret;
	}

	ret = wcd9320_irq_init(wcd);
	if (ret)
		return ret;

	wcd9320_probe(wcd);

	return ret;
}

static const struct slim_device_id wcd9320_slim_id[] = {
	{SLIM_MANF_ID_QCOM, SLIM_PROD_CODE_WCD9320, 0x1, 0x0},
	{SLIM_MANF_ID_QCOM, SLIM_PROD_CODE_WCD9320, 0x0, 0x0},
	{}
};
MODULE_DEVICE_TABLE(slim, wcd9320_slim_id);

static struct slim_driver wcd9320_slim_driver = {
	.driver = {
		.name = "wcd9320-slim",
	},
	.probe = wcd9320_slim_probe,
	.device_status = wcd9320_slim_status,
	.id_table = wcd9320_slim_id,
};

module_slim_driver(wcd9320_slim_driver);
MODULE_DESCRIPTION("WCD9320 slim driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("slim:217:0a0:*");
